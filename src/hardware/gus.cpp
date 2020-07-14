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
#define GUS_MIN_VOICES       14u
#define GUS_MAX_VOICES       32u
#define GUS_BUFFER_FRAMES    64u
#define GUS_PAN_POSITIONS    16u // 0 face-left, 7 face-forward, and 15 face-right
#define GUS_VOLUME_POSITIONS 4096u
#define GUS_VOLUME_SCALE_DIV 1.002709201 // 0.0235 dB increments
#define GUS_RAM_SIZE         1048576u    // 1 MB
#define GUS_READ_HANDLERS    8u
#define GUS_WRITE_HANDLERS   9u
#define LOG_GUS              0

uint8_t adlib_commandreg = 0u;
struct Frame {
	float left = 0.0f;
	float right = 0.0f;
};

struct VoiceIrqs {
	const std::function<void()> check = nullptr;
	uint32_t ramp = 0u;
	uint32_t wave = 0u;
};

class Voice {
public:
	Voice(uint8_t num, VoiceIrqs &irqs);
	void ClearStats();
	void GenerateSamples(float *stream,
	                     const uint8_t *ram,
	                     const std::array<float, GUS_VOLUME_POSITIONS> &,
	                     const std::array<Frame, GUS_PAN_POSITIONS> &,
	                     Frame &peak,
	                     uint16_t len);

	uint8_t ReadWaveCtrl() const;
	uint8_t ReadRampCtrl() const;
	void WritePanPot(uint8_t pos);
	void WriteRampCtrl(uint8_t val);
	void WriteRampRate(uint8_t val);
	void WriteWaveCtrl(uint8_t val);
	void Writewave_freq(uint16_t val);
	void UpdateWaveRamp();

	// bit-depth tracking
	uint32_t generated_8bit_ms = 0u;
	uint32_t generated_16bit_ms = 0u;
	uint32_t *generated_ms = &generated_8bit_ms;

	// volume scalar state
	uint32_t vol_index_start = 0u;
	uint32_t vol_index_end = 0u;
	uint32_t vol_index_current = 0u;

	// wave state
	uint32_t wave_start = 0u;
	uint32_t wave_end = 0u;
	uint32_t wave_addr = 0u;

private:
	enum WCtrl : uint8_t {
		Stopped = 0x01,
		Stop = 0x02,
		Bit16 = 0x04,
		Loop = 0x08,
		BiDirectional = 0x10,
		Decreasing = 0x40,
	};

	Voice() = delete;
	Voice(const Voice &) = delete;            // prevent copying
	Voice &operator=(const Voice &) = delete; // prevent assignment

	inline float GetSample8(const uint8_t *const ram, const uint32_t addr) const;
	inline float GetSample16(const uint8_t *const ram, const uint32_t addr) const;
	inline void RampUpdate();
	uint8_t ReadPanPot() const;
	inline void WaveUpdate();

	typedef std::function<float(const uint8_t *const, const uint32_t)> get_sample_f;
	get_sample_f GetSample = std::bind(&Voice::GetSample8,
	                                   this,
	                                   std::placeholders::_1,
	                                   std::placeholders::_2);
	// shared IRQs with the GUS DSP
	VoiceIrqs &voice_irqs;

	uint32_t irq_mask = 0u;

	// pan state
	uint8_t pan_position = 7u;

	// wave state
	uint8_t wave_ctrl = 3u;
	uint32_t wave_add = 0u;
	uint16_t wave_freq = 0u;

	// volume scalar state
	uint8_t vol_index_rate = 0u;
	uint8_t vol_ctrl = 3u;
	uint32_t vol_index_inc = 0u;

	uint8_t voice_num = 0u;
};

class Gus {
public:
	Gus(uint16_t port, uint8_t dma, uint8_t irq, const std::string &dir);
	~Gus();

	bool CheckTimer(size_t t);

	struct Timer {
		float delay = 0.0f;
		uint8_t value = 0u;
		bool is_masked = false;
		bool should_raise_irq = false;
		bool has_expired = false;
		bool is_counting_down = false;
	};
	Timer timers[2] = {};

private:
	Gus() = delete;
	Gus(const Gus &) = delete;            // prevent copying
	Gus &operator=(const Gus &) = delete; // prevent assignment

	void AudioCallback(uint16_t len);
	void CheckIrq();
	void CheckVoiceIrq();
	void DmaCallback(DmaChannel *dma_channel, DMAEvent event);
	void ExecuteGlobRegister();
	uint16_t ExecuteReadRegister();
	void PopulateAutoExec(uint16_t port, const std::string &dir);
	void PopulatePanScalars();
	void PopulateVolScalars();
	void PrintStats();
	Bitu ReadFromPort(const Bitu port, const Bitu iolen);
	void Reset();

	bool SoftLimit(float (&)[GUS_BUFFER_FRAMES][2],
	               int16_t (&)[GUS_BUFFER_FRAMES][2],
	               uint16_t);

	void WriteToPort(Bitu port, Bitu val, Bitu iolen);

	std::array<Frame, GUS_PAN_POSITIONS> pan_scalars = {};
	std::array<Voice *, GUS_MAX_VOICES> voices = {{nullptr}};
	std::array<float, GUS_VOLUME_POSITIONS> vol_scalars = {{0.0f}};
	std::array<IO_ReadHandleObject, GUS_READ_HANDLERS> read_handlers = {};
	std::array<IO_WriteHandleObject, GUS_WRITE_HANDLERS> write_handlers = {};

	AutoexecObject autoexec_lines[2] = {};
	VoiceIrqs voice_irqs = {std::bind(&Gus::CheckVoiceIrq, this), 0u, 0u};
	Frame peak_amplitude = {1.0f, 1.0f};
	MixerObject mixer_channel = {};

	MixerChannel *audio_channel = nullptr;
	Voice *current_voice = nullptr;

	size_t port_base = 0u;

	uint32_t base_freq = 0u;
	uint32_t rate = 0u;
	uint32_t dram_addr = 0u;
	uint32_t active_voice_mask = 0u;

	uint16_t register_data = 0u;
	uint16_t current_voice_index = 0u;
	uint16_t dma_addr = 0u;

	const uint8_t irq_addresses[GUS_ADDRESSES] = {0, 2,  5,  3,
	                                              7, 11, 12, 15};
	const uint8_t dma_addresses[GUS_ADDRESSES] = {0, 1, 3, 5, 6, 7, 0, 0};
	uint8_t ram[GUS_RAM_SIZE] = {0u};

	uint8_t register_select = 0u;
	uint8_t dma_ctrl = 0u;
	uint8_t timer_ctrl = 0u;
	uint8_t sample_ctrl = 0u;
	uint8_t mix_ctrl = 0u;
	uint8_t active_voices = 0u;
	uint8_t dma1 = 0u;
	uint8_t dma2 = 0u;
	uint8_t irq1 = 0u;
	uint8_t irq2 = 0u;
	uint8_t irq_status = 0u;
	uint8_t voice_irq_status = 0u;
	uint8_t &adlib_command_reg = adlib_commandreg;

	bool irq_enabled = false;
	bool should_change_irq_dma = false;
};

static Gus *myGUS = nullptr;

Bitu DEBUG_EnableDebugger();

// Read an 8-bit sample scaled into the 16-bit range, returned as a float
inline float Voice::GetSample8(const uint8_t *const ram, const uint32_t addr) const
{
	constexpr float to_16bit_range = 1u
	                                 << (std::numeric_limits<int16_t>::digits -
	                                     std::numeric_limits<int8_t>::digits);
	return static_cast<int8_t>(ram[addr & 0xFFFFFu]) * to_16bit_range;
}

// Read a 16-bit sample returned as a float
inline float Voice::GetSample16(const uint8_t *const ram, const uint32_t addr) const
{
	// Calculate offset of the 16-bit sample
	const uint32_t adjaddr = (addr & 0xC0000u) | ((addr & 0x1FFFFu) << 1u);
	return static_cast<int16_t>(host_readw(ram + adjaddr));
}

uint8_t Voice::ReadPanPot() const
{
	return pan_position;
}

inline void Voice::WaveUpdate()
{
	if (wave_ctrl & (WCtrl::Stop | WCtrl::Stopped))
		return;
	int32_t wave_remaining;
	if (wave_ctrl & WCtrl::Decreasing) {
		wave_addr -= wave_add;
		wave_remaining = wave_start - wave_addr;
	} else {
		wave_addr += wave_add;
		wave_remaining = wave_addr - wave_end;
	}
	// Not yet reaching a boundary
	if (wave_remaining < 0)
		return;
	/* Generate an IRQ if needed */
	if (wave_ctrl & 0x20) {
		voice_irqs.wave |= irq_mask;
	}
	/* Check for not being in PCM operation */
	if (vol_ctrl & 0x04)
		return;
	/* Check for looping */
	if (wave_ctrl & WCtrl::Loop) {
		/* Bi-directional looping */
		if (wave_ctrl & WCtrl::BiDirectional)
			wave_ctrl ^= WCtrl::Decreasing;
		wave_addr = (wave_ctrl & WCtrl::Decreasing)
		                    ? (wave_end - wave_remaining)
		                    : (wave_start + wave_remaining);
	} else {
		wave_ctrl |= 1; // Stop the voice
		wave_addr = (wave_ctrl & WCtrl::Decreasing) ? wave_start : wave_end;
	}
}

inline void Voice::RampUpdate()
{
	/* Check if ramping enabled */
	if (vol_ctrl & 0x3)
		return;
	int32_t RemainingVolIndexes;
	if (vol_ctrl & 0x40) {
		vol_index_current -= vol_index_inc;
		RemainingVolIndexes = vol_index_start - vol_index_current;
	} else {
		vol_index_current += vol_index_inc;
		RemainingVolIndexes = vol_index_current - vol_index_end;
	}
	if (RemainingVolIndexes < 0) {
		return;
	}
	/* Generate an IRQ if needed */
	if (vol_ctrl & 0x20) {
		voice_irqs.ramp |= irq_mask;
	}
	/* Check for looping */
	if (vol_ctrl & 0x08) {
		/* Bi-directional looping */
		if (vol_ctrl & 0x10)
			vol_ctrl ^= 0x40;
		vol_index_current = (vol_ctrl & 0x40)
		                            ? (vol_index_end - RemainingVolIndexes)
		                            : (vol_index_start + RemainingVolIndexes);
	} else {
		vol_ctrl |= 1; // Stop the voice
		vol_index_current = (vol_ctrl & 0x40) ? vol_index_start
		                                      : vol_index_end;
	}
}

inline uint8_t Voice::ReadRampCtrl() const
{
	uint8_t ret = vol_ctrl;
	if (voice_irqs.ramp & irq_mask)
		ret |= 0x80;
	return ret;
}

void Voice::WriteRampRate(uint8_t val)
{
	vol_index_rate = val;
	const uint8_t scale = val & 63;
	const uint8_t divider = 1 << (3 * (val >> 6));
	vol_index_inc = (!scale || !divider) ? 1u : ceil_udivide(scale, divider);
}

void Voice::GenerateSamples(float *stream,
                            const uint8_t *const ram,
                            const std::array<float, GUS_VOLUME_POSITIONS> &vol_scalars,
                            const std::array<Frame, GUS_PAN_POSITIONS> &pan_scalars,
                            Frame &peak,
                            uint16_t len)
{
	if (vol_ctrl & wave_ctrl & 3) // Channel is disabled
		return;

	while (len-- > 0) {
		// Get the sample
		const uint32_t sample_addr = wave_addr >> WAVE_FRACT;
		float sample = GetSample(ram, sample_addr);

		// Add any fractional inter-wave portion
		constexpr uint32_t wave_width = 1 << WAVE_FRACT;
		if (wave_add < wave_width) {
			const float next_sample = GetSample(ram, sample_addr + 1u);
			const uint32_t wave_fraction = wave_addr & (wave_width - 1u);
			sample += (next_sample - sample) * wave_fraction / wave_width;

			// Confirm the sample plus inter-wave portion is
			// in-bounds
			assert(sample <= std::numeric_limits<int16_t>::max() &&
			       sample >= std::numeric_limits<int16_t>::min());
		}
		// Apply any selected volume reduction
		sample *= vol_scalars[vol_index_current];

		// Add the sample to the stream, angled in L-R space
		*(stream++) += sample * pan_scalars[pan_position].left;
		*(stream++) += sample * pan_scalars[pan_position].right;

		// Keep tabs on the accumulated stream amplitudes
		peak.left = std::max(peak.left, fabsf(stream[-2]));
		peak.right = std::max(peak.right, fabsf(stream[-1]));

		// Move onto the the next memory address and volume
		// reduction
		WaveUpdate();
		RampUpdate();
	}
	// Tally the number of generated sets so far
	(*generated_ms)++;
}

void Voice::ClearStats()
{
	generated_8bit_ms = 0u;
	generated_16bit_ms = 0u;
}

inline uint8_t Voice::ReadWaveCtrl() const
{
	uint8_t ret = wave_ctrl;
	if (voice_irqs.wave & irq_mask)
		ret |= 0x80;
	return ret;
}

Voice::Voice(uint8_t num, VoiceIrqs &irqs)
        : voice_irqs(irqs),
          irq_mask(1 << num),
          voice_num(num)

{}

void Voice::Writewave_freq(uint16_t val)
{
	wave_freq = val;
	wave_add = ceil_udivide(val, 2u);
}

void Voice::UpdateWaveRamp()
{
	Writewave_freq(wave_freq);
	WriteRampRate(vol_index_rate);
}

void Voice::WritePanPot(uint8_t pos)
{
	constexpr uint8_t max_pos = GUS_PAN_POSITIONS - 1;
	pan_position = std::min(pos, max_pos);
}

void Voice::WriteWaveCtrl(uint8_t val)
{
	const uint32_t oldirq = voice_irqs.wave;
	wave_ctrl = val & 0x7f;
	using namespace std::placeholders;
	if (wave_ctrl & WCtrl::Bit16) {
		GetSample = std::bind(&Voice::GetSample16, this, _1, _2);
		generated_ms = &generated_16bit_ms;
	} else {
		GetSample = std::bind(&Voice::GetSample8, this, _1, _2);
		generated_ms = &generated_8bit_ms;
	}

	if ((val & 0xa0) == 0xa0)
		voice_irqs.wave |= irq_mask;
	else
		voice_irqs.wave &= ~irq_mask;
	if (oldirq != voice_irqs.wave)
		voice_irqs.check();
}

void Voice::WriteRampCtrl(uint8_t val)
{
	const uint32_t old = voice_irqs.ramp;
	vol_ctrl = val & 0x7f;
	// Manually set the irq
	if ((val & 0xa0) == 0xa0)
		voice_irqs.ramp |= irq_mask;
	else
		voice_irqs.ramp &= ~irq_mask;
	if (old != voice_irqs.ramp)
		voice_irqs.check();
}

Gus::Gus(uint16_t port, uint8_t dma, uint8_t irq, const std::string &ultradir)
        : port_base(port - 0x200),
          dma1(dma),
          dma2(dma),
          irq1(irq),
          irq2(irq)
{
		// Create the internal voice channels
	for (uint8_t i = 0; i < GUS_MAX_VOICES; ++i) {
		voices.at(i) = new Voice(i, voice_irqs);
	}
	// Register the IO read addresses
	using namespace std::placeholders;
	const auto read_from = std::bind(&Gus::ReadFromPort, this, _1, _2);
	read_handlers[0].Install(0x302 + port_base, read_from, IO_MB);
	read_handlers[1].Install(0x303 + port_base, read_from, IO_MB);
	read_handlers[2].Install(0x304 + port_base, read_from, IO_MB | IO_MW);
	read_handlers[3].Install(0x305 + port_base, read_from, IO_MB);
	read_handlers[4].Install(0x206 + port_base, read_from, IO_MB);
	read_handlers[5].Install(0x208 + port_base, read_from, IO_MB);
	read_handlers[6].Install(0x307 + port_base, read_from, IO_MB);
	// Board Only
	read_handlers[7].Install(0x20A + port_base, read_from, IO_MB);

	// Register the IO write addresses
	// We'll leave the MIDI interface to the MPU-401
	// Ditto for the Joystick
	// GF1 Synthesizer
	const auto write_to = std::bind(&Gus::WriteToPort, this, _1, _2, _3);
	write_handlers[0].Install(0x302 + port_base, write_to, IO_MB);
	write_handlers[1].Install(0x303 + port_base, write_to, IO_MB);
	write_handlers[2].Install(0x304 + port_base, write_to, IO_MB | IO_MW);
	write_handlers[3].Install(0x305 + port_base, write_to, IO_MB);
	write_handlers[4].Install(0x208 + port_base, write_to, IO_MB);
	write_handlers[5].Install(0x209 + port_base, write_to, IO_MB);
	write_handlers[6].Install(0x307 + port_base, write_to, IO_MB);
	// Board Only
	write_handlers[7].Install(0x200 + port_base, write_to, IO_MB);
	write_handlers[8].Install(0x20B + port_base, write_to, IO_MB);

	// Register the Mixer CallBack
	audio_channel = mixer_channel.Install(std::bind(&Gus::AudioCallback, this,
	                                                std::placeholders::_1),
	                                      1, "GUS");

	// Populate the volume and pan tables
	PopulateVolScalars();
	PopulatePanScalars();

	// Reset the DSP
	register_data = 0x1;
	Reset();
	register_data = 0x0;

	PopulateAutoExec(port, ultradir);
}

Gus::~Gus()
{
	register_data = 0x1;
	Reset();
	register_data = 0x0;
	for (auto voice : voices)
		delete voice;
}

void Gus::PopulateAutoExec(uint16_t port, const std::string &ultradir)
{
	// ULTRASND=Port,(rec)DMA1,(pcm)DMA2,(play)IRQ1,(midi)IRQ2
	std::ostringstream sndline;
	sndline << "SET ULTRASND=" << std::hex << std::setw(3) << port << ","
	        << std::dec << static_cast<int>(dma1) << ","
	        << static_cast<int>(dma2) << "," << static_cast<int>(irq1)
	        << "," << static_cast<int>(irq2) << std::ends;
	LOG_MSG("GUS: %s", sndline.str().c_str());
	autoexec_lines[0].Install(sndline.str());

	// ULTRADIR=full path to directory containing "midi"
	std::string dirline = "SET ULTRADIR=" + ultradir;
	autoexec_lines[1].Install(dirline);
}

void Gus::PrintStats()
{
	// Aggregate stats from all voices
	uint32_t combined_8bit_ms = 0u;
	uint32_t combined_16bit_ms = 0u;
	uint32_t used_8bit_voices = 0u;
	uint32_t used_16bit_voices = 0u;
	for (const auto voice : voices) {
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
	const float mixer_scalar = std::max(audio_channel->volmain[0],
	                                    audio_channel->volmain[1]);
	double peak_ratio = mixer_scalar *
	                    std::max(peak_amplitude.left, peak_amplitude.right) /
	                    std::numeric_limits<int16_t>::max();

	// It's expected and normal for multi-voice audio to periodically
	// accumulate beyond the max, which is gracefully scaled without
	// distortion, so there is no need to recommend that users scale-down
	// their GUS voice.
	peak_ratio = std::min(peak_ratio, 1.0);
	LOG_MSG("GUS: Peak amplitude has_expired %.0f%% of max", 100 * peak_ratio);

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

void Gus::Reset()
{
	if ((register_data & 0x1) == 0x1) {
		// Characterize playback before resettings
		PrintStats();

		// Reset
		adlib_command_reg = 85;
		irq_status = 0;
		timers[0].should_raise_irq = false;
		timers[1].should_raise_irq = false;
		timers[0].has_expired = false;
		timers[1].has_expired = false;
		timers[0].is_counting_down = false;
		timers[1].is_counting_down = false;

		timers[0].value = 0xff;
		timers[1].value = 0xff;
		timers[0].delay = 0.080f;
		timers[1].delay = 0.320f;

		should_change_irq_dma = false;
		mix_ctrl = 0x0b; // latches enabled, LINEs disabled
		// Stop all voices
		for (const auto voice : voices) {
			voice->vol_index_current = 0u;
			voice->WriteWaveCtrl(0x1);
			voice->WriteRampCtrl(0x1);
			voice->WritePanPot(0x7);
			voice->ClearStats();
		}
		voice_irq_status = 0;
		peak_amplitude = {1.0f, 1.0f};
	}
	if ((register_data & 0x4) != 0) {
		irq_enabled = true;
	} else {
		irq_enabled = false;
	}
}

inline void Gus::CheckIrq()
{
	if (irq_status && (mix_ctrl & 0x08))
		PIC_ActivateIRQ(irq1);
}

void Gus::CheckVoiceIrq()
{
	irq_status &= 0x9f;
	const Bitu totalmask = (voice_irqs.ramp | voice_irqs.wave) & active_voice_mask;
	if (!totalmask)
		return;
	if (voice_irqs.ramp)
		irq_status |= 0x40;
	if (voice_irqs.wave)
		irq_status |= 0x20;
	CheckIrq();
	for (;;) {
		uint32_t check = (1 << voice_irq_status);
		if (totalmask & check)
			return;
		voice_irq_status++;
		if (voice_irq_status >= active_voices)
			voice_irq_status = 0;
	}
}

uint16_t Gus::ExecuteReadRegister()
{
	uint8_t tmpreg;
	//	LOG_MSG("Read global reg %x",register_select);
	switch (register_select) {
	case 0x41: // Dma control register - read acknowledges DMA IRQ
		tmpreg = dma_ctrl & 0xbf;
		tmpreg |= (irq_status & 0x80) >> 1;
		irq_status &= 0x7f;
		return static_cast<uint16_t>(tmpreg << 8);
	case 0x42: // Dma address register
		return dma_addr;
	case 0x45: // Timer control register matches Adlib's behavior
		return static_cast<uint16_t>(timer_ctrl << 8);
		break;
	case 0x49: // Dma sample register
		tmpreg = dma_ctrl & 0xbf;
		tmpreg |= (irq_status & 0x80) >> 1;
		return static_cast<uint16_t>(tmpreg << 8);
	case 0x80: // Voice voice control read register
		if (current_voice)
			return current_voice->ReadWaveCtrl() << 8;
		else
			return 0x0300;

	case 0x82: // Voice MSB start address register
		if (current_voice)
			return static_cast<uint16_t>(current_voice->wave_start >> 16);
		else
			return 0x0000;
	case 0x83: // Voice LSW start address register
		if (current_voice)
			return static_cast<uint16_t>(current_voice->wave_start);
		else
			return 0x0000;

	case 0x89: // Voice volume register
		if (current_voice)
			return static_cast<uint16_t>(current_voice->vol_index_current
			                             << 4);
		else
			return 0x0000;
	case 0x8a: // Voice MSB current address register
		if (current_voice)
			return static_cast<uint16_t>(current_voice->wave_addr >> 16);
		else
			return 0x0000;
	case 0x8b: // Voice LSW current address register
		if (current_voice)
			return static_cast<uint16_t>(current_voice->wave_addr);
		else
			return 0x0000;

	case 0x8d: // Voice volume control register
		if (current_voice)
			return current_voice->ReadRampCtrl() << 8;
		else
			return 0x0300;
	case 0x8f: // General voice IRQ status register
		tmpreg = voice_irq_status | 0x20;
		uint32_t mask;
		mask = 1 << voice_irq_status;
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
		LOG_MSG("Read Register num 0x%x", register_select);
#endif
		return register_data;
	}
}

bool Gus::CheckTimer(const size_t t)
{
	if (!timers[t].is_masked)
		timers[t].has_expired = true;
	if (timers[t].should_raise_irq) {
		irq_status |= 0x4 << t;
		CheckIrq();
	}
	return timers[t].is_counting_down;
}

static void GUS_TimerEvent(Bitu t)
{
	if (myGUS->CheckTimer(t))
		PIC_AddEvent(GUS_TimerEvent, myGUS->timers[t].delay, t);
}

void Gus::ExecuteGlobRegister()
{
	//	if (register_select|1!=0x44) LOG_MSG("write global register %x
	// with %x", register_select, register_data);
	switch (register_select) {
	case 0x0: // Voice voice control register
		if (current_voice)
			current_voice->WriteWaveCtrl(register_data >> 8);
		break;
	case 0x1: // Voice frequency control register
		if (current_voice)
			current_voice->Writewave_freq(register_data);
		break;
	case 0x2: // Voice MSW start address register
		if (current_voice) {
			uint32_t tmp_addr = static_cast<uint32_t>(
			        (register_data & 0x1fff) << 16);
			current_voice->wave_start = (current_voice->wave_start &
			                             WAVE_MSWMASK) |
			                            tmp_addr;
		}
		break;
	case 0x3: // Voice LSW start address register
		if (current_voice) {
			uint32_t tmp_addr = static_cast<uint32_t>(register_data);
			current_voice->wave_start = (current_voice->wave_start &
			                             WAVE_LSWMASK) |
			                            tmp_addr;
		}
		break;
	case 0x4: // Voice MSW end address register
		if (current_voice) {
			uint32_t tmp_addr = static_cast<uint32_t>(register_data & 0x1fff)
			                    << 16;
			current_voice->wave_end = (current_voice->wave_end &
			                           WAVE_MSWMASK) |
			                          tmp_addr;
		}
		break;
	case 0x5: // Voice MSW end address register
		if (current_voice) {
			uint32_t tmp_addr = static_cast<uint32_t>(register_data);
			current_voice->wave_end = (current_voice->wave_end &
			                           WAVE_LSWMASK) |
			                          tmp_addr;
		}
		break;
	case 0x6: // ChanVoicenel volume ramp rate register
		if (current_voice) {
			uint8_t tmp_data = register_data >> 8;
			current_voice->WriteRampRate(tmp_data);
		}
		break;
	case 0x7: // Voice volume ramp start register  EEEEMMMM
		if (current_voice) {
			uint8_t tmp_data = register_data >> 8;
			current_voice->vol_index_start = tmp_data << 4;
		}
		break;
	case 0x8: // Voice volume ramp end register  EEEEMMMM
		if (current_voice) {
			uint8_t tmp_data = register_data >> 8;
			current_voice->vol_index_end = tmp_data << 4;
		}
		break;
	case 0x9: // Voice current volume register
		if (current_voice) {
			uint16_t tmp_data = register_data >> 4;
			current_voice->vol_index_current = tmp_data;
		}
		break;
	case 0xA: // Voice MSW current address register
		if (current_voice) {
			uint32_t tmp_addr = static_cast<uint32_t>(register_data & 0x1fff)
			                    << 16;
			current_voice->wave_addr = (current_voice->wave_addr &
			                            WAVE_MSWMASK) |
			                           tmp_addr;
		}
		break;
	case 0xB: // Voice LSW current address register
		if (current_voice) {
			uint32_t tmp_addr = static_cast<uint32_t>(register_data);
			current_voice->wave_addr = (current_voice->wave_addr &
			                            WAVE_LSWMASK) |
			                           tmp_addr;
		}
		break;
	case 0xC: // Voice pan pot register
		if (current_voice)
			current_voice->WritePanPot(register_data >> 8);
		break;
	case 0xD: // Voice volume control register
		if (current_voice)
			current_voice->WriteRampCtrl(register_data >> 8);
		break;
	case 0xE: // Set active voice register
		register_select = register_data >> 8; // JAZZ Jackrabbit seems
		                                      // to assume this?
		{
			unsigned requested = 1 + ((register_data >> 8) & 63);
			requested = clamp(requested, GUS_MIN_VOICES, GUS_MAX_VOICES);
			if (requested != active_voices) {
				active_voices = requested;
				active_voice_mask = 0xffffffffU >>
				                    (32 - active_voices);
				base_freq = static_cast<uint32_t>(
				        0.5 +
				        1000000.0 / (1.619695497 * active_voices));
				audio_channel->SetFreq(base_freq);
				LOG_MSG("GUS: Activated %u voices "
				        "is_counting_down at "
				        "%u Hz",
				        active_voices, base_freq);
			}
			// Always re-apply the ramp as it can change elsewhere
			for (uint8_t i = 0; i < active_voices; i++)
				voices[i]->UpdateWaveRamp();
			audio_channel->Enable(true);
		}
		break;
	case 0x10: // Undocumented register used in Fast Tracker 2
		break;
	case 0x41: // Dma control register
		dma_ctrl = static_cast<uint8_t>(register_data >> 8);
		{
			using namespace std::placeholders;
			auto dma_callback = std::bind(&Gus::DmaCallback, this,
			                              _1, _2);
			std::function<void(DmaChannel *, DMAEvent)> empty_callback = nullptr;
			GetDMAChannel(dma1)->Register_Callback(
			        (dma_ctrl & 0x1) ? dma_callback : empty_callback);
		}
		break;
	case 0x42: // Gravis DRAM DMA address register
		dma_addr = register_data;
		break;
	case 0x43: // MSB Peek/poke DRAM position
		dram_addr = (0xff0000 & dram_addr) |
		            (static_cast<uint32_t>(register_data));
		break;
	case 0x44: // LSW Peek/poke DRAM position
		dram_addr = (0xffff & dram_addr) |
		            (static_cast<uint32_t>(register_data >> 8)) << 16;
		break;
	case 0x45: // Timer control register.  Identical in operation to Adlib's
	           // timer
		timer_ctrl = static_cast<uint8_t>(register_data >> 8);
		timers[0].should_raise_irq = (timer_ctrl & 0x04) > 0;
		if (!timers[0].should_raise_irq)
			irq_status &= ~0x04;
		timers[1].should_raise_irq = (timer_ctrl & 0x08) > 0;
		if (!timers[1].should_raise_irq)
			irq_status &= ~0x08;
		break;
	case 0x46: // Timer 1 control
		timers[0].value = static_cast<uint8_t>(register_data >> 8);
		timers[0].delay = (0x100 - timers[0].value) * 0.080f;
		break;
	case 0x47: // Timer 2 control
		timers[1].value = static_cast<uint8_t>(register_data >> 8);
		timers[1].delay = (0x100 - timers[1].value) * 0.320f;
		break;
	case 0x49: // DMA sampling control register
		sample_ctrl = static_cast<uint8_t>(register_data >> 8);
		{
			using namespace std::placeholders;
			auto dma_callback = std::bind(&Gus::DmaCallback, this,
			                              _1, _2);
			std::function<void(DmaChannel *, DMAEvent)> empty_callback = nullptr;
			GetDMAChannel(dma1)->Register_Callback(
			        (sample_ctrl & 0x1) ? dma_callback : empty_callback);
		}
		break;
	case 0x4c: // GUS reset register
		Reset();
		break;
	default:
#if LOG_GUS
		LOG_MSG("Unimplemented global register %x -- %x",
		        register_select, register_data);
#endif
		break;
	}
	return;
}
Bitu Gus::ReadFromPort(const Bitu port, const Bitu iolen)
{
	//	LOG_MSG("read from gus port %x",port);
	switch (port - port_base) {
	case 0x206: return irq_status;
	case 0x208:
		uint8_t tmp_time;
		tmp_time = 0u;
		if (timers[0].has_expired)
			tmp_time |= (1 << 6);
		if (timers[1].has_expired)
			tmp_time |= (1 << 5);
		if (tmp_time & 0x60)
			tmp_time |= (1 << 7);
		if (irq_status & 0x04)
			tmp_time |= (1 << 2);
		if (irq_status & 0x08)
			tmp_time |= (1 << 1);
		return tmp_time;
	case 0x20a: return adlib_command_reg;
	case 0x302: return static_cast<uint8_t>(current_voice_index);
	case 0x303: return register_select;
	case 0x304:
		if (iolen == 2)
			return ExecuteReadRegister() & 0xffff;
		else
			return ExecuteReadRegister() & 0xff;
	case 0x305: return ExecuteReadRegister() >> 8;
	case 0x307:
		if (dram_addr < GUS_RAM_SIZE) {
			return ram[dram_addr];
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
	switch (port - port_base) {
	case 0x200:
		mix_ctrl = static_cast<uint8_t>(val);
		should_change_irq_dma = true;
		return;
	case 0x208: adlib_command_reg = static_cast<uint8_t>(val); break;
	case 0x209:
		// TODO adlib_command_reg should be 4 for this to work
		// else it should just latch the value
		if (val & 0x80) {
			timers[0].has_expired = false;
			timers[1].has_expired = false;
			return;
		}
		timers[0].is_masked = (val & 0x40) > 0;
		timers[1].is_masked = (val & 0x20) > 0;
		if (val & 0x1) {
			if (!timers[0].is_counting_down) {
				PIC_AddEvent(GUS_TimerEvent, timers[0].delay, 0);
				timers[0].is_counting_down = true;
			}
		} else
			timers[0].is_counting_down = false;
		if (val & 0x2) {
			if (!timers[1].is_counting_down) {
				PIC_AddEvent(GUS_TimerEvent, timers[1].delay, 1);
				timers[1].is_counting_down = true;
			}
		} else
			timers[1].is_counting_down = false;
		break;
		// TODO Check if 0x20a register is also available on the gus
		// like on the interwave
	case 0x20b:
		if (!should_change_irq_dma)
			break;
		should_change_irq_dma = false;
		if (mix_ctrl & 0x40) {
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
		current_voice_index = val & 31;
		current_voice = voices[current_voice_index];
		break;
	case 0x303:
		register_select = static_cast<uint8_t>(val);
		register_data = 0;
		break;
	case 0x304:
		if (iolen == 2) {
			register_data = static_cast<uint16_t>(val);
			ExecuteGlobRegister();
		} else
			register_data = static_cast<uint16_t>(val);
		break;
	case 0x305:
		register_data = static_cast<uint16_t>((0x00ff & register_data) |
		                                      val << 8);
		ExecuteGlobRegister();
		break;
	case 0x307:
		if (dram_addr < GUS_RAM_SIZE)
			ram[dram_addr] = static_cast<uint8_t>(val);
		break;
	default:
#if LOG_GUS
		LOG_MSG("Write GUS at port 0x%x with %x", port, val);
#endif
		break;
	}
}

void Gus::DmaCallback(DmaChannel *dma_channel, DMAEvent event)
{
	if (event != DMA_UNMASKED)
		return;
	Bitu addr;
	// Calculate the dma address
	// DMA transfers can't cross 256k boundaries, so you should be safe to
	// just determine the start once and go from there Bit 2 - 0 = if DMA
	// channel is an 8 bit channel(0 - 3).
	if (dma_ctrl & 0x4)
		addr = (((dma_addr & 0x1fff) << 1) | (dma_addr & 0xc000)) << 4;
	else
		addr = dma_addr << 4;
	// Reading from dma?
	if ((dma_ctrl & 0x2) == 0) {
		Bitu read = dma_channel->Read(dma_channel->currcnt + 1, &ram[addr]);
		// Check for 16 or 8bit channel
		read *= (dma_channel->DMA16 + 1);
		if ((dma_ctrl & 0x80) != 0) {
			// Invert the MSB to convert twos compliment form
			const size_t dma_end = addr + read;
			if ((dma_ctrl & 0x40) == 0) {
				// 8-bit data
				for (size_t i = addr; i < dma_end; ++i)
					ram[i] ^= 0x80;
			} else {
				// 16-bit data
				for (size_t i = addr + 1; i < dma_end; i += 2)
					ram[i] ^= 0x80;
			}
		}
		// Writing to dma
	} else {
		dma_channel->Write(dma_channel->currcnt + 1, &ram[addr]);
	}
	/* Raise the TC irq if needed */
	if ((dma_ctrl & 0x20) != 0) {
		irq_status |= 0x80;
		CheckIrq();
	}
	dma_channel->Register_Callback(0);
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

void Gus::AudioCallback(uint16_t len)
{
	assert(len <= GUS_BUFFER_FRAMES);

	float accumulator[GUS_BUFFER_FRAMES][2] = {{0}};
	for (uint8_t i = 0; i < active_voices; ++i)
		voices[i]->GenerateSamples(*accumulator, ram, vol_scalars,
		                           pan_scalars, peak_amplitude, len);

	int16_t scaled[GUS_BUFFER_FRAMES][2];
	if (!SoftLimit(accumulator, scaled, len))
		for (uint8_t i = 0; i < len; ++i)
			for (uint8_t j = 0; j < 2; ++j)
				scaled[i][j] = static_cast<int16_t>(
				        accumulator[i][j]);

	audio_channel->AddSamples_s16(len, scaled[0]);
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
		// pan_scalars.at(pos).left,
		// pan_scalars.at(pos).right);
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
