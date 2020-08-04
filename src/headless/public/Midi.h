#import <Foundation/Foundation.h>
#import "Types.h"

@protocol MIDIDevice;

@protocol MIDIRegistrar <NSObject>
- (void)addDevice:(id<MIDIDevice>)device;
@end

@protocol MIDIDevice <NSObject>

@property (readonly) NSString *name;

- (BOOL)openWithConfiguration:(NSString *)conf;
- (void)close;
- (void)playMessage:(uint8_t const *)message;
- (void)playSysExMessage:(uint8_t *)message length:(NSInteger)len;

@end