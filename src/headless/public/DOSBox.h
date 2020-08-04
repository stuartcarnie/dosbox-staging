#import "DOSBoxDelegate.h"
#import "Midi.h"
#import "oecommon.h"
#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

OE_EXPORTED_CLASS
@interface DOSBox : NSObject <MIDIRegistrar>

@property (class, nonatomic, readonly) DOSBox *shared;

@property (nonatomic, assign, nullable) id<DOSBoxDelegate> delegate;

- (instancetype)init NS_UNAVAILABLE;
- (void)startup;

#pragma mark - execution

- (BOOL)execute;

#pragma mark - video

@property (nonatomic, readonly) CGFloat framesPerSecond;
@property (nonatomic, readonly) NSSize videoSize;
- (void)setVideoBuffer:(void *)buffer size:(NSSize)size pitch:(NSInteger)pitch;

@end

NS_ASSUME_NONNULL_END
