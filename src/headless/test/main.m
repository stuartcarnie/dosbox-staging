#import "Headless.h"

@interface TestDevice: NSObject<MIDIDevice>
@end

@interface TestDelegate: NSObject<DOSBoxDelegate>
@end

int main(int argc, char **argv)
{
    DOSBox *emu = DOSBox.shared;
    __auto_type del = [TestDelegate new];

    emu.delegate = del;

    [emu startup];
    [emu execute];
    [emu execute];

    NSLog(@"Start");

    for (int i=0; i<1000; i++)
    {
        [emu execute];
        if (i%60 == 0)
        {
            NSLog(@"Finished %d frames", i);
        }
    }

    NSLog(@"Done");

    return 0;
}

@implementation TestDelegate {
    void * _buffer;
}

@synthesize configurationURLs;

- (void)registerMidiDevicesWithRegistrar:(id<MIDIRegistrar>)registrar
{
    NSLog(@"%@", NSStringFromSelector(_cmd));
    [registrar addDevice:[TestDevice new]];
}

- (void)updateAudioBuffer:(const int16_t *)buffer samples:(NSInteger)samples
{
}

- (void)videoWillResizeWidth:(NSInteger)width height:(NSInteger)height pixelAspect:(CGFloat)pixelAspect
{
    NSLog(@"%@", NSStringFromSelector(_cmd));
}

- (void)dosBoxDidStartup
{
    _buffer = malloc(640*400*4);
    [DOSBox.shared setVideoBuffer:_buffer size:NSMakeSize(640, 400) pitch:640*4];
    NSLog(@"%@", NSStringFromSelector(_cmd));
}

@end

@implementation TestDevice

- (void)dealloc
{
    NSLog(@"dealloc TestDevice");
}

- (NSString *)name
{
    return @"foo_device";
}

- (BOOL)openWithConfiguration:(NSString *)conf
{
    return NO;
}

- (void)close
{}

- (void)playMessage:(uint8_t const *)message
{}

- (void)playSysExMessage:(uint8_t *)message length:(NSInteger)len
{}

@end