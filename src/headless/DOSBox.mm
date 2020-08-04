#include "dosbox.h"
#import "DOSBox+Private.h"
#import "control.h"
#import "render.h"
#import "timer.h"
#import <libco.h>

@implementation DOSBox {
    BOOL _initialized;

    NSSize _frameSize;
    NSSize _frameScale;
    void *_frameBuffer;
    NSInteger _framePitch;

    BOOL _audioOpen;
    SDL_AudioSpec _audioSpec;
    int16_t *_audioBuffer;
    int _audioBufferLenBytes;
    int _audioSamplesPerVideoFrame;

    cothread_t _main_thread;
    cothread_t _game_thread;

    CFRunLoopRef _coreRunLoop;

    NSMutableArray<id<MIDIDevice>> *_midiDevices;
    std::vector<std::unique_ptr<MidiHandlerAdapter>> _midiAdapters;
}

static DOSBox *g_dosbox;

+ (DOSBox *)shared
{
    static dispatch_once_t once;
    dispatch_once(&once, ^{
      g_dosbox = [DOSBox new];
    });

    return g_dosbox;
}

- (void)dealloc
{
    NSLog(@"DOSBox: dealloc");
}

- (instancetype)init
{
    if ((self = [super init])) {
        _midiDevices = [NSMutableArray new];
    }
    return self;
}

#pragma mark - initialization

- (void)yield
{
    co_switch(_main_thread);
}

- (void)startup
{
    if (_initialized) {
        NSAssert(!_initialized, @"startup should be called only once");
    }

    if ([self.delegate respondsToSelector:@selector(dosBoxWillStartup)]) {
        [self.delegate dosBoxWillStartup];
    }

    initialize_timer();

    _main_thread = co_active();

    char const *argv[0];
    CommandLine commandLine(0, argv);
    control = new Config(&commandLine);
    DOSBOX_Init();

    for (NSURL *configURL in self.delegate.configurationURLs) {
        control->ParseConfigFile(configURL.fileSystemRepresentation);
    }

    // delegate other initialization
    if ([self.delegate respondsToSelector:@selector(registerMidiDevicesWithRegistrar:)]) {
        [self.delegate registerMidiDevicesWithRegistrar:self];
    }

    control->Init();
    RENDER_SetForceUpdate(true);

    Section *section = control->GetSection("cpu");
    auto *secprop = static_cast<Section_prop *>(section);
    if (secprop) {
        section->ExecuteDestroy(false);
        section->HandleInputline("cycles=fixed 3000");
        section->ExecuteInit(false);
    }

    _initialized = YES;
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    auto runner = []() {
        control->StartUp();

        // this loop prevents calling libco's crash function
        while (true) {
            puts("Running dead emulator");
            [g_dosbox yield];
        }
    };
#pragma clang diagnostic pop

    _game_thread = co_create(512'000 * sizeof(uintptr_t), runner);

    if ([self.delegate respondsToSelector:@selector(dosBoxDidStartup)]) {
        [self.delegate dosBoxDidStartup];
    }
}

#pragma mark - properties

- (CGFloat)framesPerSecond
{
    return static_cast<CGFloat>(render.src.fps);
}

- (NSSize)videoSize
{
    return NSMakeSize(render.src.width, render.src.height);
}

#pragma mark - execution

- (BOOL)execute
{
    co_switch(_game_thread);
    return YES;
}

- (void)processEvents
{
    return;
    CFRunLoopRunResult res;
    do {
        res = CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.0, YES);
    } while (res == kCFRunLoopRunHandledSource);
}

#pragma mark - Video APIs

- (void)setVideoBuffer:(void *)buffer size:(NSSize)size pitch:(NSInteger)pitch
{
    _frameBuffer = buffer;
    _frameSize = size;
    _framePitch = pitch;
}

- (void)videoSetWidth:(Bitu)width height:(Bitu)height scaleX:(double)scaleX scaleY:(double)scaleY pixelAspect:(double)pixelAspect
{
    [self updateAudioBuffers];
    [_delegate videoWillResizeWidth:width height:height pixelAspect:pixelAspect];
}

- (BOOL)videoBeginFrame:(uint8_t **)buffer pitch:(int *)pitch
{
    if (_frameBuffer == nullptr || _framePitch <= 0) {
        return NO;
    }
    *buffer = static_cast<uint8_t *>(_frameBuffer);
    *pitch = static_cast<int>(_framePitch);
    return YES;
}

- (void)videoEndFrameWithChangedLines:(const Bit16u *)lines
{
    if (lines != nullptr || YES) {
        if (_audioSpec.callback && _audioBuffer) {
            _audioSpec.callback(_audioSpec.userdata, reinterpret_cast<Uint8 *>(_audioBuffer), _audioBufferLenBytes);
            [_delegate updateAudioBuffer:_audioBuffer samples:_audioSamplesPerVideoFrame];
        }
    }
    [self yield];
}

#pragma mark - Audio APIs

- (void)updateAudioBuffers
{
    if (_audioBuffer) {
        free(_audioBuffer);
        _audioBufferLenBytes = 0;
    }

    if (self.framesPerSecond == 0) {
        return;
    }

    _audioSamplesPerVideoFrame = static_cast<int>(_audioSpec.freq / self.framesPerSecond) * _audioSpec.channels;
    _audioBufferLenBytes = _audioSamplesPerVideoFrame * sizeof(int16_t);
    _audioBuffer = static_cast<int16_t *>(malloc(static_cast<size_t>(_audioBufferLenBytes)));
}

- (SDL_AudioDeviceID)openAudioDevice:(const char *)device
                           isCapture:(int)isCapture
                         desiredSpec:(const SDL_AudioSpec *)desired
                        obtainedSpec:(SDL_AudioSpec *)obtained
                        allowChanges:(int)allowed_changes

{
    NSParameterAssert(isCapture == 0);
    if (_audioOpen) {
        return 0;
    }

    memcpy(obtained, desired, sizeof(*obtained));
    memcpy(&_audioSpec, desired, sizeof(_audioSpec));

    _audioOpen = YES;
    [self updateAudioBuffers];

    return 1;
}

- (void)closeAudioDevice:(SDL_AudioDeviceID)deviceID
{
    if (!_audioOpen) {
        return;
    }

    memset(&_audioSpec, 0, sizeof(_audioSpec));
    if (_audioBuffer) {
        free(_audioBuffer);
        _audioBufferLenBytes = 0;
    }
    _audioOpen = NO;
}

#pragma mark - MIDIRegistrar

- (void)addDevice:(id<MIDIDevice>)device
{
    _midiAdapters.push_back(std::make_unique<MidiHandlerAdapter>(device));
}

@end

// region GFX APIs

void GFX_Events()
{
    [g_dosbox processEvents];
}

Bitu GFX_GetRGB(Bit8u red, Bit8u green, Bit8u blue)
{
    return (blue | (green << 8u) | (red << 16u) | (255u << 24u));
}

Bitu GFX_SetSize(Bitu width, Bitu height, Bitu flags, double scalex, double scaley, GFX_CallBack_t callback, double pixel_aspect)
{
    [g_dosbox videoSetWidth:width height:height scaleX:scalex scaleY:scaley pixelAspect:pixel_aspect];
    return GFX_CAN_32 | GFX_SCALING;
}

Bitu GFX_GetBestMode(Bitu flags)
{
    return GFX_CAN_32 | GFX_SCALING;
}

bool GFX_StartUpdate(uint8_t *&pixels, int &pitch)
{
    return [g_dosbox videoBeginFrame:&pixels pitch:&pitch];
}

void GFX_EndUpdate(const Bit16u *changedLines)
{
    [g_dosbox videoEndFrameWithChangedLines:changedLines];
}

void GFX_ShowMsg(char const *format, ...)
{
    char buf[512];

    va_list msg;
    va_start(msg, format);
    vsnprintf(buf, sizeof(buf), format, msg);
    va_end(msg);

    buf[sizeof(buf) - 1] = '\0';
    puts(buf);
}

void GFX_SetTitle(Bit32s cycles, int /*frameskip*/, bool paused)
{}

// endregion

// region SDL audio APIs

SDL_AudioDeviceID SDLCALL SDL_OpenAudioDevice(const char *device, int iscapture, const SDL_AudioSpec *desired,
                                              SDL_AudioSpec *obtained, int allowed_changes)
{
    return [g_dosbox openAudioDevice:device
                           isCapture:iscapture
                         desiredSpec:desired
                        obtainedSpec:obtained
                        allowChanges:allowed_changes];
}

void SDLCALL SDL_PauseAudioDevice(SDL_AudioDeviceID dev, int pause_on)
{}

void SDLCALL SDL_LockAudioDevice(SDL_AudioDeviceID dev)
{}

void SDLCALL SDL_UnlockAudioDevice(SDL_AudioDeviceID dev)
{}

void SDLCALL SDL_CloseAudioDevice(SDL_AudioDeviceID dev)
{
    [g_dosbox closeAudioDevice:dev];
}

// endregion
