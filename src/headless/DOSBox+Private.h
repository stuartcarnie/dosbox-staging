#import "public/Midi.h"
#import "public/DOSBox.h"
#import "Midi+Private.h"

#pragma mark - MIDIRegistrar category

@interface DOSBox (MIDIRegistrarProtocol) <MIDIRegistrar>
@end
