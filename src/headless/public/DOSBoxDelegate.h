#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@protocol MIDIRegistrar;

@protocol DOSBoxDelegate <NSObject>

/// @brief configurationURLs provides a list of files to configure DOSBox
@property (nonatomic, readonly, nullable) NSArray<NSURL *> *configurationURLs;

- (void)updateAudioBuffer:(int16_t const *)buffer samples:(NSInteger)samples;
- (void)videoWillResizeWidth:(NSInteger)width height:(NSInteger)height pixelAspect:(CGFloat)pixelAspect;

@optional
- (void)dosBoxWillStartup;
- (void)dosBoxDidStartup;
- (void)registerMidiDevicesWithRegistrar:(id<MIDIRegistrar>)registrar;
@end

NS_ASSUME_NONNULL_END