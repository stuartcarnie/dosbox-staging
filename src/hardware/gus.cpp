/*
 *  Copyright (C) 2020-2020  The dosbox-staging team
 *  Copyright (C) 2002-2020  The DOSBox Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include "dosbox.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string.h>

#include "inout.h"
#include "mixer.h"
#include "dma.h"
#include "pic.h"
#include "setup.h"
#include "shell.h"
#include "math.h"
#include "regs.h"

// Extra bits of precision over normal gus
#define WAVE_FRACT      9
#define WAVE_MSWMASK    ((1 << 16) - 1)
#define WAVE_LSWMASK    (0xffffffff ^ WAVE_MSWMASK)

#define GUS_ADDRESSES        8u
#define GUS_MIN_CHANNELS     14u
#define GUS_MAX_CHANNELS     32u
#define GUS_BUFFER_FRAMES    64u
#define GUS_PAN_POSITIONS    16u // 0 face-left, 7 face-forward, and 15 face-right
#define GUS_VOLUME_POSITIONS 4096u
#define GUS_VOLUME_SCALE_DIV 1.002709201 // 0.0235 dB increments
#define GUS_RAM_SIZE         1048576u    // 1 MB
#define GUS_READ_HANDLERS    8u
#define GUS_WRITE_HANDLERS   9u
#define LOG_GUS              0

#define WCTRL_STOPPED       0x01
#define WCTRL_STOP          0x02
#define WCTRL_16BIT         0x04
#define WCTRL_LOOP          0x08
#define WCTRL_BIDIRECTIONAL 0x10
#define WCTRL_IRQENABLED    0x20
#define WCTRL_DECREASING    0x40
#define WCTRL_IRQPENDING    0x80

class GUSChannels;

uint8_t adlib_commandreg = 0u;
struct Frame {
	float left = 0.0f;
	float right = 0.0f;
};

class Gus {
	struct Timer {
		float delay = 0.0f;
		uint8_t value = 0u;
		bool masked = false;
		bool raiseirq = false;
		bool reached = false;
		bool running = false;
	};

public:
	struct VoiceIrqs {
		const std::function<void()> check = nullptr;
		uint32_t ramp = 0u;
		uint32_t wave = 0u;
	};

	Gus(uint16_t port, uint8_t dma, uint8_t irq, const std::string &dir);
	~Gus();
	Gus() = delete;
	Gus(const Gus &) = delete;            // prevent copying
	Gus &operator=(const Gus &) = delete; // prevent assignment

	void GUS_CheckIRQ();
	void CheckVoiceIrq();
	bool SoftLimit(float (&)[64][2], int16_t (&)[64][2], uint16_t);
	void PopulateAutoExec(uint16_t port, const std::string &dir);
	void PopulateVolScalars();
	void PopulatePanScalars();
	void GUS_CallBack(uint16_t len);
	void ExecuteGlobRegister();
	uint16_t ExecuteReadRegister();
	void PrintStats();
	void GUSReset();
	bool CheckTimer(size_t t);
	Bitu ReadFromPort(const Bitu port, const Bitu iolen);
	void WriteToPort(Bitu port, Bitu val, Bitu iolen);

	Timer timers[2] = {};
	AutoexecObject autoexecline[2] = {};
	std::array<Frame, GUS_PAN_POSITIONS> pan_scalars = {};
	std::array<float, GUS_VOLUME_POSITIONS> vol_scalars = {{0.0f}};
	std::array<GUSChannels *, GUS_MAX_CHANNELS> guschan = {{nullptr}};
	std::array<IO_ReadHandleObject, GUS_READ_HANDLERS> read_handlers = {};
	std::array<IO_WriteHandleObject, GUS_WRITE_HANDLERS> write_handlers = {};

	VoiceIrqs voice_irqs = {std::bind(&Gus::CheckVoiceIrq, this), 0u, 0u};
	Frame peak_amplitude = {1.0f, 1.0f};
	MixerObject MixerChan = {};

	MixerChannel *gus_chan = nullptr;
	GUSChannels *curchan = nullptr;
	size_t portbase = 0u;

	uint32_t basefreq = 0u;
	uint32_t rate = 0u;
	uint32_t gDramAddr = 0u;
	uint32_t ActiveMask = 0u;

	uint16_t gRegData = 0u;
	uint16_t gCurChannel = 0u;
	uint16_t dmaAddr = 0u;

	const uint8_t irq_addresses[GUS_ADDRESSES] = {0, 2,  5,  3,
	                                              7, 11, 12, 15};
	const uint8_t dma_addresses[GUS_ADDRESSES] = {0, 1, 3, 5, 6, 7, 0, 0};
	uint8_t ram[GUS_RAM_SIZE] = {0u};

	uint8_t gRegSelect = 0u;
	uint8_t DMAControl = 0u;
	uint8_t TimerControl = 0u;
	uint8_t SampControl = 0u;
	uint8_t mixControl = 0u;
	uint8_t ActiveChannels = 0u;
	uint8_t dma1 = 0u;
	uint8_t dma2 = 0u;
	uint8_t irq1 = 0u;
	uint8_t irq2 = 0u;
	uint8_t IRQStatus = 0u;
	uint8_t IRQChan = 0u;
	uint8_t &adlib_command_reg = adlib_commandreg;

	bool irqenabled = false;
	bool ChangeIRQDMA = false;
};

static Gus *myGUS = nullptr;

Bitu DEBUG_EnableDebugger();

static void GUS_DMA_Callback(DmaChannel *chan, DMAEvent event);

class GUSChannels {
public:
	GUSChannels(uint8_t num, Gus::VoiceIrqs &irqs);
	GUSChannels() = delete;
	GUSChannels(const GUSChannels &) = delete;            // prevent copying
	GUSChannels &operator=(const GUSChannels &) = delete; // prevent assignment

	void WriteWaveCtrl(uint8_t val);

	typedef std::function<float(const uint8_t *const, const uint32_t)> get_sample_f;
	get_sample_f GetSample = std::bind(&GUSChannels::GetSample8,
	                                   this,
	                                   std::placeholders::_1,
	                                   std::placeholders::_2);

	Gus::VoiceIrqs &voice_irqs;
	uint32_t WaveStart = 0u;
	uint32_t WaveEnd = 0u;
	uint32_t WaveAddr = 0u;
	uint32_t WaveAdd = 0u;
	uint8_t WaveCtrl = 3u;
	uint16_t WaveFreq = 0u;

	uint32_t StartVolIndex = 0u;
	uint32_t EndVolIndex = 0u;
	uint32_t CurrentVolIndex = 0u;
	uint32_t IncrVolIndex = 0u;

	uint8_t RampRate = 0u;
	uint8_t RampCtrl = 3u;

	uint8_t PanPot = 7u;
	uint8_t channum = 0u;
	uint32_t irqmask = 0u;

	uint32_t generated_8bit_ms = 0u;
	uint32_t generated_16bit_ms = 0u;
	uint32_t *generated_ms = &generated_8bit_ms;

	void ClearStats()
	{
		generated_8bit_ms = 0u;
		generated_16bit_ms = 0u;
	}

	// Read an 8-bit sample scaled into the 16-bit range, returned as a float
	inline float GetSample8(const uint8_t *const ram, const uint32_t addr) const
	{
		constexpr float to_16bit_range =
		        1u << (std::numeric_limits<int16_t>::digits -
		               std::numeric_limits<int8_t>::digits);
		return static_cast<int8_t>(ram[addr & 0xFFFFFu]) * to_16bit_range;
	}

	// Read a 16-bit sample returned as a float
	inline float GetSample16(const uint8_t *const ram, const uint32_t addr) const
	{
		// Calculate offset of the 16-bit sample
		const uint32_t adjaddr = (addr & 0xC0000u) | ((addr & 0x1FFFFu) << 1u);
		return static_cast<int16_t>(host_readw(ram + adjaddr));
	}

	void WriteWaveFreq(uint16_t val)
	{
		WaveFreq = val;
		WaveAdd = ceil_udivide(val, 2u);
	}

	inline uint8_t ReadWaveCtrl() const
	{
		uint8_t ret = WaveCtrl;
		if (voice_irqs.wave & irqmask)
			ret |= 0x80;
		return ret;
	}
	void UpdateWaveRamp()
	{
		WriteWaveFreq(WaveFreq);
		WriteRampRate(RampRate);
	}

	void WritePanPot(uint8_t pos)
	{
		constexpr uint8_t max_pos = GUS_PAN_POSITIONS - 1;
		PanPot = std::min(pos, max_pos);
	}

	uint8_t ReadPanPot() const { return PanPot; }
	void WriteRampCtrl(uint8_t val)
	{
		const uint32_t old = voice_irqs.ramp;
		RampCtrl = val & 0x7f;
		// Manually set the irq
		if ((val & 0xa0) == 0xa0)
			voice_irqs.ramp |= irqmask;
		else
			voice_irqs.ramp &= ~irqmask;
		if (old != voice_irqs.ramp)
			voice_irqs.check();
	}
	inline uint8_t ReadRampCtrl() const
	{
		uint8_t ret = RampCtrl;
		if (voice_irqs.ramp & irqmask)
			ret |= 0x80;
		return ret;
	}
	void WriteRampRate(uint8_t val)
	{
		RampRate = val;
		const uint8_t scale = val & 63;
		const uint8_t divider = 1 << (3 * (val >> 6));
		IncrVolIndex = (!scale || !divider) ? 1u :ceil_udivide(scale, divider);
	}
	inline void WaveUpdate()
	{
		if (WaveCtrl & (WCTRL_STOP | WCTRL_STOPPED))
			return;
		int32_t WaveLeft;
		if (WaveCtrl & WCTRL_DECREASING) {
			WaveAddr -= WaveAdd;
			WaveLeft = WaveStart - WaveAddr;
		} else {
			WaveAddr += WaveAdd;
			WaveLeft = WaveAddr - WaveEnd;
		}
		// Not yet reaching a boundary
		if (WaveLeft < 0)
			return;
		/* Generate an IRQ if needed */
		if (WaveCtrl & 0x20) {
			voice_irqs.wave |= irqmask;
		}
		/* Check for not being in PCM operation */
		if (RampCtrl & 0x04)
			return;
		/* Check for looping */
		if (WaveCtrl & WCTRL_LOOP) {
			/* Bi-directional looping */
			if (WaveCtrl & WCTRL_BIDIRECTIONAL)
				WaveCtrl ^= WCTRL_DECREASING;
			WaveAddr = (WaveCtrl & WCTRL_DECREASING)
			                   ? (WaveEnd - WaveLeft)
			                   : (WaveStart + WaveLeft);
		} else {
			WaveCtrl |= 1; // Stop the channel
			WaveAddr = (WaveCtrl & WCTRL_DECREASING) ? WaveStart
			                                         : WaveEnd;
		}
	}

	inline void RampUpdate()
	{
		/* Check if ramping enabled */
		if (RampCtrl & 0x3)
			return;
		int32_t RemainingVolIndexes;
		if (RampCtrl & 0x40) {
			CurrentVolIndex -= IncrVolIndex;
			RemainingVolIndexes = StartVolIndex - CurrentVolIndex;
		} else {
			CurrentVolIndex += IncrVolIndex;
			RemainingVolIndexes = CurrentVolIndex - EndVolIndex;
		}
		if (RemainingVolIndexes < 0) {
			return;
		}
		/* Generate an IRQ if needed */
		if (RampCtrl & 0x20) {
			voice_irqs.ramp |= irqmask;
		}
		/* Check for looping */
		if (RampCtrl & 0x08) {
			/* Bi-directional looping */
			if (RampCtrl & 0x10)
				RampCtrl ^= 0x40;
			CurrentVolIndex = (RampCtrl & 0x40)
			                          ? (EndVolIndex - RemainingVolIndexes)
			                          : (StartVolIndex +
			                             RemainingVolIndexes);
		} else {
			RampCtrl |= 1; // Stop the channel
			CurrentVolIndex = (RampCtrl & 0x40) ? StartVolIndex
			                                    : EndVolIndex;
		}
	}

	void generateSamples(float *stream,
	                     const uint8_t *const ram,
	                     const std::array<float, GUS_VOLUME_POSITIONS> &vol_scalars,
	                     const std::array<Frame, GUS_PAN_POSITIONS> &pan_scalars,
	                     Frame &peak,
	                     uint16_t len)
	{
		if (RampCtrl & WaveCtrl & 3) // Channel is disabled
			return;

		while (len-- > 0) {
			// Get the sample
			const uint32_t sample_addr = WaveAddr >> WAVE_FRACT;
			float sample = GetSample(ram, sample_addr);

			// Add any fractional inter-wave portion
			constexpr uint32_t wave_width = 1 << WAVE_FRACT;
			if (WaveAdd < wave_width) {
				const float next_sample = GetSample(ram, sample_addr + 1u);
				const uint32_t wave_fraction = WaveAddr & (wave_width - 1u);
				sample += (next_sample - sample) * wave_fraction / wave_width;

				// Confirm the sample plus inter-wave portion is in-bounds
				assert(sample <= std::numeric_limits<int16_t>::max() &&
				       sample >= std::numeric_limits<int16_t>::min());
			}
			// Apply any selected volume reduction
			sample *= vol_scalars[CurrentVolIndex];

			// Add the sample to the stream, angled in L-R space
			*(stream++) += sample * pan_scalars[PanPot].left;
			*(stream++) += sample * pan_scalars[PanPot].right;

			// Keep tabs on the accumulated stream amplitudes
			peak.left = std::max(peak.left, fabsf(stream[-2]));
			peak.right = std::max(peak.right, fabsf(stream[-1]));

			// Move onto the the next memory address and volume reduction
			WaveUpdate();
			RampUpdate();
		}
		// Tally the number of generated sets so far
		(*generated_ms)++;
	}
};

GUSChannels::GUSChannels(uint8_t num, Gus::VoiceIrqs &irqs)
        : voice_irqs(irqs),
          channum(num),
          irqmask(1 << num)
{}

void GUSChannels::WriteWaveCtrl(uint8_t val)
{
	const uint32_t oldirq = voice_irqs.wave;
	WaveCtrl = val & 0x7f;
	if (WaveCtrl & WCTRL_16BIT) {
		GetSample = std::bind(&GUSChannels::GetSample16, this,
		                      std::placeholders::_1, std::placeholders::_2);
		generated_ms = &generated_16bit_ms;
	} else {
		GetSample = std::bind(&GUSChannels::GetSample8, this,
		                      std::placeholders::_1, std::placeholders::_2);
		generated_ms = &generated_8bit_ms;
	}

	if ((val & 0xa0) == 0xa0)
		voice_irqs.wave |= irqmask;
	else
		voice_irqs.wave &= ~irqmask;
	if (oldirq != voice_irqs.wave)
		voice_irqs.check();
}

Gus::Gus(uint16_t port, uint8_t dma, uint8_t irq, const std::string &ultradir)
        : portbase(port - 0x200),
          dma1(dma),
          dma2(dma),
          irq1(irq),
          irq2(irq)
{
	// Create the internal voice channels
	for (uint8_t chan_ct = 0; chan_ct < GUS_MAX_CHANNELS; chan_ct++) {
		guschan.at(chan_ct) = new GUSChannels(chan_ct, voice_irqs);
	}

	// Register the IO read addresses
	const auto read_from = std::bind(&Gus::ReadFromPort, this,
	                                 std::placeholders::_1,
	                                 std::placeholders::_2);
	read_handlers[0].Install(0x302 + portbase, read_from, IO_MB);
	read_handlers[1].Install(0x303 + portbase, read_from, IO_MB);
	read_handlers[2].Install(0x304 + portbase, read_from, IO_MB | IO_MW);
	read_handlers[3].Install(0x305 + portbase, read_from, IO_MB);
	read_handlers[4].Install(0x206 + portbase, read_from, IO_MB);
	read_handlers[5].Install(0x208 + portbase, read_from, IO_MB);
	read_handlers[6].Install(0x307 + portbase, read_from, IO_MB);
	// Board Only
	read_handlers[7].Install(0x20A + portbase, read_from, IO_MB);

	// Register the IO write addresses
	// We'll leave the MIDI interface to the MPU-401
	// Ditto for the Joystick
	// GF1 Synthesizer
	const auto write_to = std::bind(&Gus::WriteToPort, this,
	                                std::placeholders::_1, std::placeholders::_2,
	                                std::placeholders::_3);
	write_handlers[0].Install(0x302 + portbase, write_to, IO_MB);
	write_handlers[1].Install(0x303 + portbase, write_to, IO_MB);
	write_handlers[2].Install(0x304 + portbase, write_to, IO_MB | IO_MW);
	write_handlers[3].Install(0x305 + portbase, write_to, IO_MB);
	write_handlers[4].Install(0x208 + portbase, write_to, IO_MB);
	write_handlers[5].Install(0x209 + portbase, write_to, IO_MB);
	write_handlers[6].Install(0x307 + portbase, write_to, IO_MB);
	// Board Only
	write_handlers[7].Install(0x200 + portbase, write_to, IO_MB);
	write_handlers[8].Install(0x20B + portbase, write_to, IO_MB);

	// Register the Mixer CallBack
	gus_chan = MixerChan.Install(std::bind(&Gus::GUS_CallBack, this,
	                                       std::placeholders::_1),
	                             1, "GUS");

	// Populate the volume and pan tables
	PopulateVolScalars();
	PopulatePanScalars();

	// Reset the DSP
	gRegData = 0x1;
	GUSReset();
	gRegData = 0x0;

	PopulateAutoExec(port, ultradir);
}

Gus::~Gus()
{
	gRegData = 0x1;
	GUSReset();
	gRegData = 0x0;
	for (auto voice : guschan)
		delete voice;
}

void Gus::PopulateAutoExec(uint16_t port, const std::string &ultradir)
{
	// ULTRASND=Port,(rec)DMA1,(pcm)DMA2,(play)IRQ1,(midi)IRQ2
	std::ostringstream sndline;
	sndline << "SET ULTRASND=" << std::hex << std::setw(3)
	        << portbase + 0x200 << "," << std::dec << static_cast<int>(dma1)
	        << "," << static_cast<int>(dma2) << "," << static_cast<int>(irq1)
	        << "," << static_cast<int>(irq2) << std::ends;
	LOG_MSG("GUS: %s", sndline.str().c_str());
	autoexecline[0].Install(sndline.str());

	// ULTRADIR=full path to directory containing "midi"
	std::string dirline = "SET ULTRADIR=" + ultradir;
	autoexecline[1].Install(dirline);
}

void Gus::PrintStats()
{
	// Aggregate stats from all channels
	uint32_t combined_8bit_ms = 0u;
	uint32_t combined_16bit_ms = 0u;
	uint32_t used_8bit_voices = 0u;
	uint32_t used_16bit_voices = 0u;
	for (const auto voice : guschan) {
		if (voice->generated_8bit_ms) {
			combined_8bit_ms += voice->generated_8bit_ms;
			used_8bit_voices++;
		}
		if (voice->generated_16bit_ms) {
			combined_16bit_ms += voice->generated_16bit_ms;
			used_16bit_voices++;
		}
	}
	const uint32_t combined_ms = combined_8bit_ms + combined_16bit_ms;

	// Is there enough information to be meaningful?
	if (combined_ms < 10000u ||
	    (peak_amplitude.left + peak_amplitude.right) < 10 ||
	    !(used_8bit_voices + used_16bit_voices))
		return;

	// Print info about the type of audio and voices used
	if (used_16bit_voices == 0u)
		LOG_MSG("GUS: Audio was made up of 8-bit samples from %u "
		        "voices",
		        used_8bit_voices);
	else if (used_8bit_voices == 0u)
		LOG_MSG("GUS: Audio was made up of 16-bit samples from %u "
		        "voices",
		        used_16bit_voices);
	else {
		const uint8_t ratio_8bit = ceil_udivide(100u * combined_8bit_ms,
		                                        combined_ms);
		const uint8_t ratio_16bit = ceil_udivide(100u * combined_16bit_ms,
		                                         combined_ms);
		LOG_MSG("GUS: Audio was made up of %u%% 8-bit %u-voice and "
		        "%u%% 16-bit %u-voice samples",
		        ratio_8bit, used_8bit_voices, ratio_16bit,
		        used_16bit_voices);
	}

	// Calculate and print info about the volume
	const float mixer_scalar = std::max(gus_chan->volmain[0],
	                                    gus_chan->volmain[1]);
	double peak_ratio = mixer_scalar *
	                    std::max(peak_amplitude.left, peak_amplitude.right) /
	                    std::numeric_limits<int16_t>::max();

	// It's expected and normal for multi-channel audio to periodically
	// accumulate beyond the max, which is gracefully scaled without
	// distortion, so there is no need to recommend that users scale-down
	// their GUS channel.
	peak_ratio = std::min(peak_ratio, 1.0);
	LOG_MSG("GUS: Peak amplitude reached %.0f%% of max", 100 * peak_ratio);

	// Make a suggestion if the peak volume was well below 3 dB
	if (peak_ratio < 0.6) {
		const auto multiplier = static_cast<uint16_t>(
		        100.0 / (peak_ratio * static_cast<double>(mixer_scalar)));
		LOG_MSG("GUS: Consider %s: 'mixer gus %u' to normalize the "
		        "source's samples",
		        fabsf(mixer_scalar - 1.0f) > 0.01f ? "adjusting" : "using",
		        multiplier);
	}
}

void Gus::GUSReset()
{
	if ((gRegData & 0x1) == 0x1) {
		// Characterize playback before resettings
		PrintStats();

		// Reset
		adlib_command_reg = 85;
		IRQStatus = 0;
		timers[0].raiseirq = false;
		timers[1].raiseirq = false;
		timers[0].reached = false;
		timers[1].reached = false;
		timers[0].running = false;
		timers[1].running = false;

		timers[0].value = 0xff;
		timers[1].value = 0xff;
		timers[0].delay = 0.080f;
		timers[1].delay = 0.320f;

		ChangeIRQDMA = false;
		mixControl = 0x0b; // latches enabled, LINEs disabled
		// Stop all channels
		for (const auto channel : guschan) {
			channel->CurrentVolIndex = 0u;
			channel->WriteWaveCtrl(0x1);
			channel->WriteRampCtrl(0x1);
			channel->WritePanPot(0x7);
			channel->ClearStats();
		}
		IRQChan = 0;
		peak_amplitude = {1.0f, 1.0f};
	}
	if ((gRegData & 0x4) != 0) {
		irqenabled = true;
	} else {
		irqenabled = false;
	}
}

inline void Gus::GUS_CheckIRQ()
{
	if (IRQStatus && (mixControl & 0x08))
		PIC_ActivateIRQ(irq1);
}

void Gus::CheckVoiceIrq()
{
	IRQStatus &= 0x9f;
	const Bitu totalmask = (voice_irqs.ramp | voice_irqs.wave) & ActiveMask;
	if (!totalmask)
		return;
	if (voice_irqs.ramp)
		IRQStatus |= 0x40;
	if (voice_irqs.wave)
		IRQStatus |= 0x20;
	GUS_CheckIRQ();
	for (;;) {
		uint32_t check = (1 << IRQChan);
		if (totalmask & check)
			return;
		IRQChan++;
		if (IRQChan >= ActiveChannels)
			IRQChan = 0;
	}
}

uint16_t Gus::ExecuteReadRegister()
{
	uint8_t tmpreg;
	//	LOG_MSG("Read global reg %x",gRegSelect);
	switch (gRegSelect) {
	case 0x41: // Dma control register - read acknowledges DMA IRQ
		tmpreg = DMAControl & 0xbf;
		tmpreg |= (IRQStatus & 0x80) >> 1;
		IRQStatus &= 0x7f;
		return static_cast<uint16_t>(tmpreg << 8);
	case 0x42: // Dma address register
		return dmaAddr;
	case 0x45: // Timer control register matches Adlib's behavior
		return static_cast<uint16_t>(TimerControl << 8);
		break;
	case 0x49: // Dma sample register
		tmpreg = DMAControl & 0xbf;
		tmpreg |= (IRQStatus & 0x80) >> 1;
		return static_cast<uint16_t>(tmpreg << 8);
	case 0x80: // Channel voice control read register
		if (curchan)
			return curchan->ReadWaveCtrl() << 8;
		else
			return 0x0300;

	case 0x82: // Channel MSB start address register
		if (curchan)
			return static_cast<uint16_t>(curchan->WaveStart >> 16);
		else
			return 0x0000;
	case 0x83: // Channel LSW start address register
		if (curchan)
			return static_cast<uint16_t>(curchan->WaveStart);
		else
			return 0x0000;

	case 0x89: // Channel volume register
		if (curchan)
			return static_cast<uint16_t>(curchan->CurrentVolIndex << 4);
		else
			return 0x0000;
	case 0x8a: // Channel MSB current address register
		if (curchan)
			return static_cast<uint16_t>(curchan->WaveAddr >> 16);
		else
			return 0x0000;
	case 0x8b: // Channel LSW current address register
		if (curchan)
			return static_cast<uint16_t>(curchan->WaveAddr);
		else
			return 0x0000;

	case 0x8d: // Channel volume control register
		if (curchan)
			return curchan->ReadRampCtrl() << 8;
		else
			return 0x0300;
	case 0x8f: // General channel IRQ status register
		tmpreg = IRQChan | 0x20;
		uint32_t mask;
		mask = 1 << IRQChan;
		if (!(voice_irqs.ramp & mask))
			tmpreg |= 0x40;
		if (!(voice_irqs.wave & mask))
			tmpreg |= 0x80;
		voice_irqs.ramp &= ~mask;
		voice_irqs.wave &= ~mask;
		voice_irqs.check();
		return static_cast<uint16_t>(tmpreg << 8);
	default:
#if LOG_GUS
		LOG_MSG("Read Register num 0x%x", gRegSelect);
#endif
		return gRegData;
	}
}

bool Gus::CheckTimer(const size_t t)
{
	if (!timers[t].masked)
		timers[t].reached = true;
	if (timers[t].raiseirq) {
		IRQStatus |= 0x4 << t;
		GUS_CheckIRQ();
	}
	return timers[t].running;
}

static void GUS_TimerEvent(Bitu t)
{
	if (myGUS->CheckTimer(t))
		PIC_AddEvent(GUS_TimerEvent, myGUS->timers[t].delay, t);
}

void Gus::ExecuteGlobRegister()
{
	//	if (gRegSelect|1!=0x44) LOG_MSG("write global register %x
	// with %x", gRegSelect, gRegData);
	switch (gRegSelect) {
	case 0x0: // Channel voice control register
		if (curchan)
			curchan->WriteWaveCtrl(gRegData >> 8);
		break;
	case 0x1: // Channel frequency control register
		if (curchan)
			curchan->WriteWaveFreq(gRegData);
		break;
	case 0x2: // Channel MSW start address register
		if (curchan) {
			uint32_t tmpaddr = static_cast<uint32_t>(
			        (gRegData & 0x1fff) << 16);
			curchan->WaveStart = (curchan->WaveStart & WAVE_MSWMASK) |
			                     tmpaddr;
		}
		break;
	case 0x3: // Channel LSW start address register
		if (curchan) {
			uint32_t tmpaddr = static_cast<uint32_t>(gRegData);
			curchan->WaveStart = (curchan->WaveStart & WAVE_LSWMASK) |
			                     tmpaddr;
		}
		break;
	case 0x4: // Channel MSW end address register
		if (curchan) {
			uint32_t tmpaddr = static_cast<uint32_t>(gRegData & 0x1fff)
			                   << 16;
			curchan->WaveEnd = (curchan->WaveEnd & WAVE_MSWMASK) | tmpaddr;
		}
		break;
	case 0x5: // Channel MSW end address register
		if (curchan) {
			uint32_t tmpaddr = static_cast<uint32_t>(gRegData);
			curchan->WaveEnd = (curchan->WaveEnd & WAVE_LSWMASK) | tmpaddr;
		}
		break;
	case 0x6: // Channel volume ramp rate register
		if (curchan) {
			uint8_t tmpdata = gRegData >> 8;
			curchan->WriteRampRate(tmpdata);
		}
		break;
	case 0x7: // Channel volume ramp start register  EEEEMMMM
		if (curchan) {
			uint8_t tmpdata = gRegData >> 8;
			curchan->StartVolIndex = tmpdata << 4;
		}
		break;
	case 0x8: // Channel volume ramp end register  EEEEMMMM
		if (curchan) {
			uint8_t tmpdata = gRegData >> 8;
			curchan->EndVolIndex = tmpdata << 4;
		}
		break;
	case 0x9: // Channel current volume register
		if (curchan) {
			uint16_t tmpdata = gRegData >> 4;
			curchan->CurrentVolIndex = tmpdata;
		}
		break;
	case 0xA: // Channel MSW current address register
		if (curchan) {
			uint32_t tmpaddr = static_cast<uint32_t>(gRegData & 0x1fff)
			                   << 16;
			curchan->WaveAddr = (curchan->WaveAddr & WAVE_MSWMASK) |
			                    tmpaddr;
		}
		break;
	case 0xB: // Channel LSW current address register
		if (curchan) {
			uint32_t tmpaddr = static_cast<uint32_t>(gRegData);
			curchan->WaveAddr = (curchan->WaveAddr & WAVE_LSWMASK) |
			                    tmpaddr;
		}
		break;
	case 0xC: // Channel pan pot register
		if (curchan)
			curchan->WritePanPot(gRegData >> 8);
		break;
	case 0xD: // Channel volume control register
		if (curchan)
			curchan->WriteRampCtrl(gRegData >> 8);
		break;
	case 0xE: // Set active channel register
		gRegSelect = gRegData >> 8; // JAZZ Jackrabbit seems
		                            // to assume this?
		{
			unsigned requested = 1 + ((gRegData >> 8) & 63);
			requested = clamp(requested, GUS_MIN_CHANNELS,
			                  GUS_MAX_CHANNELS);
			if (requested != ActiveChannels) {
				ActiveChannels = requested;
				ActiveMask = 0xffffffffU >> (32 - ActiveChannels);
				basefreq = static_cast<uint32_t>(
				        0.5 + 1000000.0 / (1.619695497 *
				                           ActiveChannels));
				gus_chan->SetFreq(basefreq);
				LOG_MSG("GUS: Activated %u voices running at "
				        "%u Hz",
				        ActiveChannels, basefreq);
			}
			// Always re-apply the ramp as it can change elsewhere
			for (uint8_t i = 0; i < ActiveChannels; i++)
				guschan[i]->UpdateWaveRamp();
			gus_chan->Enable(true);
		}
		break;
	case 0x10: // Undocumented register used in Fast Tracker 2
		break;
	case 0x41: // Dma control register
		DMAControl = static_cast<uint8_t>(gRegData >> 8);
		GetDMAChannel(dma1)->Register_Callback(
		        (DMAControl & 0x1) ? GUS_DMA_Callback : 0);
		break;
	case 0x42: // Gravis DRAM DMA address register
		dmaAddr = gRegData;
		break;
	case 0x43: // MSB Peek/poke DRAM position
		gDramAddr = (0xff0000 & gDramAddr) |
		            (static_cast<uint32_t>(gRegData));
		break;
	case 0x44: // LSW Peek/poke DRAM position
		gDramAddr = (0xffff & gDramAddr) |
		            (static_cast<uint32_t>(gRegData >> 8)) << 16;
		break;
	case 0x45: // Timer control register.  Identical in operation to Adlib's
	           // timer
		TimerControl = static_cast<uint8_t>(gRegData >> 8);
		timers[0].raiseirq = (TimerControl & 0x04) > 0;
		if (!timers[0].raiseirq)
			IRQStatus &= ~0x04;
		timers[1].raiseirq = (TimerControl & 0x08) > 0;
		if (!timers[1].raiseirq)
			IRQStatus &= ~0x08;
		break;
	case 0x46: // Timer 1 control
		timers[0].value = static_cast<uint8_t>(gRegData >> 8);
		timers[0].delay = (0x100 - timers[0].value) * 0.080f;
		break;
	case 0x47: // Timer 2 control
		timers[1].value = static_cast<uint8_t>(gRegData >> 8);
		timers[1].delay = (0x100 - timers[1].value) * 0.320f;
		break;
	case 0x49: // DMA sampling control register
		SampControl = static_cast<uint8_t>(gRegData >> 8);
		GetDMAChannel(dma1)->Register_Callback(
		        (SampControl & 0x1) ? GUS_DMA_Callback : 0);
		break;
	case 0x4c: // GUS reset register
		GUSReset();
		break;
	default:
#if LOG_GUS
		LOG_MSG("Unimplemented global register %x -- %x", gRegSelect,
		        gRegData);
#endif
		break;
	}
	return;
}
Bitu Gus::ReadFromPort(const Bitu port, const Bitu iolen)
{
	//	LOG_MSG("read from gus port %x",port);
	switch (port - portbase) {
	case 0x206: return IRQStatus;
	case 0x208:
		uint8_t tmptime;
		tmptime = 0u;
		if (timers[0].reached)
			tmptime |= (1 << 6);
		if (timers[1].reached)
			tmptime |= (1 << 5);
		if (tmptime & 0x60)
			tmptime |= (1 << 7);
		if (IRQStatus & 0x04)
			tmptime |= (1 << 2);
		if (IRQStatus & 0x08)
			tmptime |= (1 << 1);
		return tmptime;
	case 0x20a: return adlib_command_reg;
	case 0x302: return static_cast<uint8_t>(gCurChannel);
	case 0x303: return gRegSelect;
	case 0x304:
		if (iolen == 2)
			return ExecuteReadRegister() & 0xffff;
		else
			return ExecuteReadRegister() & 0xff;
	case 0x305: return ExecuteReadRegister() >> 8;
	case 0x307:
		if (gDramAddr < GUS_RAM_SIZE) {
			return ram[gDramAddr];
		} else {
			return 0;
		}
	default:
#if LOG_GUS
		LOG_MSG("Read GUS at port 0x%x", port);
#endif
		break;
	}

	return 0xff;
}

void Gus::WriteToPort(Bitu port, Bitu val, Bitu iolen)
{
	//	LOG_MSG("Write gus port %x val %x",port,val);
	switch (port - portbase) {
	case 0x200:
		mixControl = static_cast<uint8_t>(val);
		ChangeIRQDMA = true;
		return;
	case 0x208: adlib_command_reg = static_cast<uint8_t>(val); break;
	case 0x209:
		// TODO adlib_command_reg should be 4 for this to work
		// else it should just latch the value
		if (val & 0x80) {
			timers[0].reached = false;
			timers[1].reached = false;
			return;
		}
		timers[0].masked = (val & 0x40) > 0;
		timers[1].masked = (val & 0x20) > 0;
		if (val & 0x1) {
			if (!timers[0].running) {
				PIC_AddEvent(GUS_TimerEvent, timers[0].delay, 0);
				timers[0].running = true;
			}
		} else
			timers[0].running = false;
		if (val & 0x2) {
			if (!timers[1].running) {
				PIC_AddEvent(GUS_TimerEvent, timers[1].delay, 1);
				timers[1].running = true;
			}
		} else
			timers[1].running = false;
		break;
		// TODO Check if 0x20a register is also available on the gus
		// like on the interwave
	case 0x20b:
		if (!ChangeIRQDMA)
			break;
		ChangeIRQDMA = false;
		if (mixControl & 0x40) {
			// IRQ configuration, only use low bits for irq 1
			if (irq_addresses[val & 0x7])
				irq1 = irq_addresses[val & 0x7];
#if LOG_GUS
			LOG_MSG("Assigned GUS to IRQ %d", irq1);
#endif
		} else {
			// DMA configuration, only use low bits for dma 1
			if (dma_addresses[val & 0x7])
				dma1 = dma_addresses[val & 0x7];
#if LOG_GUS
			LOG_MSG("Assigned GUS to DMA %d", dma1);
#endif
		}
		break;
	case 0x302:
		gCurChannel = val & 31;
		curchan = guschan[gCurChannel];
		break;
	case 0x303:
		gRegSelect = static_cast<uint8_t>(val);
		gRegData = 0;
		break;
	case 0x304:
		if (iolen == 2) {
			gRegData = static_cast<uint16_t>(val);
			ExecuteGlobRegister();
		} else
			gRegData = static_cast<uint16_t>(val);
		break;
	case 0x305:
		gRegData = static_cast<uint16_t>((0x00ff & gRegData) | val << 8);
		ExecuteGlobRegister();
		break;
	case 0x307:
		if (gDramAddr < GUS_RAM_SIZE)
			ram[gDramAddr] = static_cast<uint8_t>(val);
		break;
	default:
#if LOG_GUS
		LOG_MSG("Write GUS at port 0x%x with %x", port, val);
#endif
		break;
	}
}

static void GUS_DMA_Callback(DmaChannel *chan, DMAEvent event)
{
	if (event != DMA_UNMASKED)
		return;
	Bitu dmaaddr;
	// Calculate the dma address
	// DMA transfers can't cross 256k boundaries, so you should be safe to
	// just determine the start once and go from there Bit 2 - 0 = if DMA
	// channel is an 8 bit channel(0 - 3).
	if (myGUS->DMAControl & 0x4)
		dmaaddr = (((myGUS->dmaAddr & 0x1fff) << 1) |
		           (myGUS->dmaAddr & 0xc000))
		          << 4;
	else
		dmaaddr = myGUS->dmaAddr << 4;
	// Reading from dma?
	if ((myGUS->DMAControl & 0x2) == 0) {
		Bitu read = chan->Read(chan->currcnt + 1, &myGUS->ram[dmaaddr]);
		// Check for 16 or 8bit channel
		read *= (chan->DMA16 + 1);
		if ((myGUS->DMAControl & 0x80) != 0) {
			// Invert the MSB to convert twos compliment form
			const size_t dma_end = dmaaddr + read;
			if ((myGUS->DMAControl & 0x40) == 0) {
				// 8-bit data
				for (size_t i = dmaaddr; i < dma_end; ++i)
					myGUS->ram[i] ^= 0x80;
			} else {
				// 16-bit data
				for (size_t i = dmaaddr + 1; i < dma_end; i += 2)
					myGUS->ram[i] ^= 0x80;
			}
		}
		// Writing to dma
	} else {
		chan->Write(chan->currcnt + 1, &myGUS->ram[dmaaddr]);
	}
	/* Raise the TC irq if needed */
	if ((myGUS->DMAControl & 0x20) != 0) {
		myGUS->IRQStatus |= 0x80;
		myGUS->GUS_CheckIRQ();
	}
	chan->Register_Callback(0);
}

bool Gus::SoftLimit(float (&in)[GUS_BUFFER_FRAMES][2],
                    int16_t (&out)[GUS_BUFFER_FRAMES][2],
                    uint16_t len)
{
	constexpr float max_allowed = static_cast<float>(
	        std::numeric_limits<int16_t>::max() - 1);

	// If our peaks are under the max, then there's no need to limit
	if (peak_amplitude.left < max_allowed && peak_amplitude.right < max_allowed)
		return false;

	// Calculate the percent we need to scale down the volume.  In cases
	// where one side is less than the max, it's ratio is limited to 1.0.
	const Frame ratio = {std::min(1.0f, max_allowed / peak_amplitude.left),
	                     std::min(1.0f, max_allowed / peak_amplitude.right)};
	for (uint8_t i = 0; i < len; ++i) {
		out[i][0] = static_cast<int16_t>(in[i][0] * ratio.left);
		out[i][1] = static_cast<int16_t>(in[i][1] * ratio.right);
	}

	// Release the limit incrementally using our existing volume scale.
	constexpr float release_amount =
	        max_allowed * (static_cast<float>(GUS_VOLUME_SCALE_DIV) - 1.0f);

	if (peak_amplitude.left > max_allowed)
		peak_amplitude.left -= release_amount;
	if (peak_amplitude.right > max_allowed)
		peak_amplitude.right -= release_amount;
	// LOG_MSG("GUS: releasing peak_amplitude = %.2f | %.2f",
	//         static_cast<double>(peak_amplitude.left),
	//         static_cast<double>(peak_amplitude.right));
	return true;
}

void Gus::GUS_CallBack(uint16_t len)
{
	assert(len <= GUS_BUFFER_FRAMES);

	float accumulator[GUS_BUFFER_FRAMES][2] = {{0}};
	for (uint8_t i = 0; i < ActiveChannels; ++i)
		guschan[i]->generateSamples(*accumulator, ram, vol_scalars,
		                            pan_scalars, peak_amplitude, len);

	int16_t scaled[GUS_BUFFER_FRAMES][2];
	if (!SoftLimit(accumulator, scaled, len))
		for (uint8_t i = 0; i < len; ++i)
			for (uint8_t j = 0; j < 2; ++j)
				scaled[i][j] = static_cast<int16_t>(
				        accumulator[i][j]);

	gus_chan->AddSamples_s16(len, scaled[0]);
	CheckVoiceIrq();
}

// Generate logarithmic to linear volume conversion tables
void Gus::PopulateVolScalars()
{
	double out = 1.0;
	for (uint16_t i = GUS_VOLUME_POSITIONS - 1; i > 0; --i) {
		vol_scalars.at(i) = static_cast<float>(out);
		out /= GUS_VOLUME_SCALE_DIV;
	}
	vol_scalars.at(0) = 0.0f;
}

/*
Constant-Power Panning
-------------------------
The GUS SDK describes having 16 panning positions (0 through 15)
with 0 representing all full left rotation through to center or
mid-point at 7, to full-right rotation at 15.  The SDK also
describes that output power is held constant through this range.

	#!/usr/bin/env python3
	import math
	print(f'Left-scalar  Pot Norm.   Right-scalar | Power')
	print(f'-----------  --- -----   ------------ | -----')
	for pot in range(16):
		norm = (pot - 7.) / (7.0 if pot < 7 else 8.0)
		direction = math.pi * (norm + 1.0 ) / 4.0
		lscale = math.cos(direction)
		rscale = math.sin(direction)
		power = lscale * lscale + rscale * rscale
		print(f'{lscale:.5f} <~~~ {pot:2} ({norm:6.3f})'\
				f' ~~~> {rscale:.5f} | {power:.3f}')

	Left-scalar  Pot Norm.   Right-scalar | Power
	-----------  --- -----   ------------ | -----
	1.00000 <~~~  0 (-1.000) ~~~> 0.00000 | 1.000
	0.99371 <~~~  1 (-0.857) ~~~> 0.11196 | 1.000
	0.97493 <~~~  2 (-0.714) ~~~> 0.22252 | 1.000
	0.94388 <~~~  3 (-0.571) ~~~> 0.33028 | 1.000
	0.90097 <~~~  4 (-0.429) ~~~> 0.43388 | 1.000
	0.84672 <~~~  5 (-0.286) ~~~> 0.53203 | 1.000
	0.78183 <~~~  6 (-0.143) ~~~> 0.62349 | 1.000
	0.70711 <~~~  7 ( 0.000) ~~~> 0.70711 | 1.000
	0.63439 <~~~  8 ( 0.125) ~~~> 0.77301 | 1.000
	0.55557 <~~~  9 ( 0.250) ~~~> 0.83147 | 1.000
	0.47140 <~~~ 10 ( 0.375) ~~~> 0.88192 | 1.000
	0.38268 <~~~ 11 ( 0.500) ~~~> 0.92388 | 1.000
	0.29028 <~~~ 12 ( 0.625) ~~~> 0.95694 | 1.000
	0.19509 <~~~ 13 ( 0.750) ~~~> 0.98079 | 1.000
	0.09802 <~~~ 14 ( 0.875) ~~~> 0.99518 | 1.000
	0.00000 <~~~ 15 ( 1.000) ~~~> 1.00000 | 1.000
*/
void Gus::PopulatePanScalars()
{
	for (uint8_t pos = 0u; pos < GUS_PAN_POSITIONS; ++pos) {
		// Normalize absolute range [0, 15] to [-1.0, 1.0]
		const double norm = (pos - 7.0f) / (pos < 7u ? 7 : 8);
		// Convert to an angle between 0 and 90-degree, in radians
		const double angle = (norm + 1) * M_PI / 4;
		pan_scalars.at(pos).left = static_cast<float>(cos(angle));
		pan_scalars.at(pos).right = static_cast<float>(sin(angle));
		// DEBUG_LOG_MSG("GUS: pan_scalar[%u] = %f | %f", pos,
		// myGUS->myGUS->pan_scalars.at(pos).left,
		// myGUS->myGUS->pan_scalars.at(pos).right);
	}
}

void GUS_ShutDown(Section * /*sec*/)
{
	delete myGUS;
}

void GUS_Init(Section *sec)
{
	if (!IS_EGAVGA_ARCH)
		return;
	Section_prop *conf = dynamic_cast<Section_prop *>(sec);
	if (!conf->Get_bool("gus"))
		return;
	const uint16_t port = conf->Get_hex("gusbase");
	const uint8_t dma = clamp(conf->Get_int("gusdma"), 1, 255);
	const uint8_t irq = clamp(conf->Get_int("gusirq"), 1, 255);
	LOG_MSG("GUS init dma %u, irq %u, port %u", dma, irq, port);
	const std::string ultradir = conf->Get_string("ultradir");

	myGUS = new Gus(port, dma, irq, ultradir);
	sec->AddDestroyFunction(&GUS_ShutDown, true);
}
