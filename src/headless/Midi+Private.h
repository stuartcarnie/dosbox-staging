#include "midi_handler.h"
#include "public/Midi.h"

class MidiHandlerAdapter : public MidiHandler {
  public:
    explicit MidiHandlerAdapter(id<MIDIDevice> device): device_(device){};

    const char *GetName() const override
    {
        return device_.name.UTF8String;
    }

    bool Open(const char *conf) override
    {
        return [device_ openWithConfiguration:[NSString stringWithUTF8String:conf]];
    }

    void Close() override
    {
        [device_ close];
    }

    void PlayMsg(const uint8_t *msg) override
    {
        [device_ playMessage:msg];
    }

    void PlaySysex(uint8_t *msg, Bitu len) override
    {
        [device_ playSysExMessage:msg length:len];
    }

  private:
    id<MIDIDevice> device_;
};