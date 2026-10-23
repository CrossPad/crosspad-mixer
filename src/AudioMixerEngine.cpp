/**
 * @file AudioMixerEngine.cpp
 * @brief Cross-platform audio routing engine implementation.
 */

#include "crosspad-mixer/AudioMixerEngine.hpp"

#include <crosspad/audio/IAudioNode.hpp>
#include <crosspad/audio/AudioFormatConvert.hpp>
#include <crosspad/dsp/AudioKernels.hpp>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>
#include <string>

/* Registration, setup and the profiler's report run at boot or from a verb,
 * never per block; a platform that builds this file for speed for render()
 * gets them built for size. GCC's `cold` changes nothing about what they do. */
#if defined(__GNUC__)
#define CP_COLD __attribute__((cold))
#else
#define CP_COLD
#endif

AudioMixerEngine::AudioMixerEngine() {
    for (auto& slot : channels_) {
        for (uint8_t o = 0; o < kMixerMaxOutputs; ++o) {
            slot.routeVol[o].store(1.0f, std::memory_order_relaxed);
            slot.routeEnabled[o].store(false, std::memory_order_relaxed);
        }
    }
}

AudioMixerEngine::~AudioMixerEngine() = default;

/* How long the limiter takes to let go: slow enough that a bass note is not
 * pumped into distortion by its own cycles, fast enough that a loud hit does
 * not duck the next bar. */
static constexpr float kLimReleaseS = 0.2f;
/* How long the gain stays where a peak put it before letting go at all. Per
 * block, a release between the crests of one loud note and a cut back at the
 * next crest moves the gain with the waveform -- the limiter turned into a
 * waveshaper, heard as drive. Modelled block for block (10.10.2026): a 40 Hz
 * sine 6 dB over the ceiling came out at THD -36 dB, -77 dB with this hold;
 * 60 Hz + 7 kHz IMD -40 -> -57 dB. Longer than a 20 Hz period. */
static constexpr float kLimHoldS = 0.05f;

// ── Lifecycle ──────────────────────────────────────────────────────────────

CP_COLD void AudioMixerEngine::setup(uint32_t frameCount, uint32_t sampleRate, uint8_t numOutputs) {
    for (auto& eq : outputEq_) eq.setRate(sampleRate);

    frameCount_ = frameCount;
    sampleRate_ = sampleRate;
    if (numOutputs == 0) numOutputs = 1;
    if (numOutputs > kMixerMaxOutputs) numOutputs = kMixerMaxOutputs;
    numOutputs_ = numOutputs;

    const uint32_t stereoSamples = frameCount * 2;
    for (auto& slot : channels_) {
        /* Only a registered channel gets its block: addChannel() sizes the
         * others' when they arrive. Sized up front, all kMixerMaxChannels of
         * them took a block each of internal RAM for nothing (AlignedVector). */
        if (!slot.active.load(std::memory_order_acquire)) continue;
        if (slot.scratch.size() < stereoSamples) slot.scratch.resize(stereoSamples);
        // Propagate the new buffer-size + SR to already-registered channel
        // nodes so they can pre-size their own scratch (RT-safe contract).
        if (slot.node) slot.node->onPrepare(frameCount, sampleRate);
    }
    for (auto& o : outputs_) {
        if (o.lookahead.load(std::memory_order_relaxed)) o.held.assign(stereoSamples, 0.0f);
    }

    limRelease_ = sampleRate ? 1.0f - std::exp(-(float)frameCount / ((float)sampleRate * kLimReleaseS))
                             : 1.0f;
    limHoldBlocks_ = frameCount ? (uint16_t)std::ceil(kLimHoldS * (float)sampleRate / (float)frameCount) : 0;
}

void AudioMixerEngine::resetChannelSlot(ChannelSlot& s) {
    // Volumes/mutes only; routing is owned separately so addChannel can
    // auto-route a fresh channel without setDefaults clobbering them.
    s.volume.store(1.0f, std::memory_order_relaxed);
    s.muted .store(false, std::memory_order_relaxed);
    s.soloed.store(false, std::memory_order_relaxed);
    s.pan   .store(0.0f, std::memory_order_relaxed);
    s.peakL .store(0.0f, std::memory_order_relaxed);
    s.peakR .store(0.0f, std::memory_order_relaxed);
}

CP_COLD void AudioMixerEngine::setDefaults() {
    std::lock_guard<std::mutex> lock(registrationMutex_);
    for (auto& slot : channels_) {
        resetChannelSlot(slot);
        // Clear routes — caller must explicitly enable any wanted routing.
        for (uint8_t o = 0; o < kMixerMaxOutputs; ++o) {
            slot.routeEnabled[o].store(false, std::memory_order_relaxed);
            slot.routeVol[o]    .store(1.0f,  std::memory_order_relaxed);
        }
    }
    for (auto& out : outputs_) {
        out.volume.store(1.0f, std::memory_order_relaxed);
        out.muted .store(false, std::memory_order_relaxed);
    }
}

// ── Channel registration ───────────────────────────────────────────────────

CP_COLD AudioMixerEngine::ChannelId
AudioMixerEngine::addChannel(crosspad::IAudioNode* node, const char* name) {
    if (!node) return kMixerInvalidChannelId;
    std::lock_guard<std::mutex> lock(registrationMutex_);

    for (uint8_t i = 0; i < kMixerMaxChannels; ++i) {
        auto& slot = channels_[i];
        if (slot.active.load(std::memory_order_acquire)) continue;

        // Slot is dead from render's POV; populate fields, then publish.
        slot.node = node;
        if (name) {
            std::strncpy(slot.name, name, kMixerNameLen - 1);
            slot.name[kMixerNameLen - 1] = 0;
        } else {
            slot.name[0] = 0;
        }
        resetChannelSlot(slot);
        // Routing is opt-in: caller / persisted state decides where the
        // channel goes. Fresh channels are silent until routed explicitly.
        for (uint8_t o = 0; o < kMixerMaxOutputs; ++o) {
            slot.routeEnabled[o].store(false, std::memory_order_relaxed);
            slot.routeVol[o]    .store(1.0f,  std::memory_order_relaxed);
        }

        const uint32_t stereoSamples = frameCount_ * 2;
        if (slot.scratch.size() < stereoSamples) slot.scratch.resize(stereoSamples);

        // Hand the node the buffer size + SR so it can pre-allocate before
        // the render thread ever calls mix(). If setup() hasn't run yet,
        // frameCount_ is 0 — render() can't fire either, so skip the call;
        // the node will be prepared on the next setup().
        if (frameCount_ > 0 && node) {
            node->onPrepare(frameCount_, sampleRate_);
        }

        slot.active.store(true, std::memory_order_release);
        return i;
    }
    return kMixerInvalidChannelId;
}

CP_COLD void AudioMixerEngine::removeChannel(ChannelId ch) {
    if (ch >= kMixerMaxChannels) return;
    std::lock_guard<std::mutex> lock(registrationMutex_);
    auto& slot = channels_[ch];
    slot.active.store(false, std::memory_order_release);
    slot.node = nullptr;
    slot.name[0] = 0;
    slot.peakL.store(0.0f, std::memory_order_relaxed);
    slot.peakR.store(0.0f, std::memory_order_relaxed);
}

uint8_t AudioMixerEngine::numActiveChannels() const {
    uint8_t n = 0;
    for (const auto& slot : channels_)
        if (slot.active.load(std::memory_order_acquire)) ++n;
    return n;
}

void AudioMixerEngine::setOutputName(OutputId out, const char* name) {
    if (out >= kMixerMaxOutputs) return;
    std::strncpy(outputs_[out].name, name ? name : "", kMixerNameLen - 1);
    outputs_[out].name[kMixerNameLen - 1] = 0;
}

const char* AudioMixerEngine::getOutputName(OutputId out) const {
    return out < kMixerMaxOutputs ? outputs_[out].name : "";
}

void AudioMixerEngine::setOutputHeard(OutputId out, bool heard) {
    if (out < kMixerMaxOutputs) outputs_[out].heard.store(heard, std::memory_order_relaxed);
}

CP_COLD void AudioMixerEngine::setOutputEq(OutputId out, const crosspad::EqSettings& eq) {
    if (out >= kMixerEqOutputs) return;
    outputEq_[out].set(eq);
    /* No notify: in this engine a setter changes the desk and whoever was
     * turning the knobs says when it is worth saving (MixerApp on its close,
     * the presets, a host command). Serialising it here meant a CDC verb
     * doing it on the main task's four kilobytes of stack, inside
     * ArduinoJson's allocator -- which is exactly where it ended. */
}

crosspad::EqSettings AudioMixerEngine::getOutputEq(OutputId out) const {
    if (out >= kMixerEqOutputs) return {};
    return outputEq_[out].settings();
}

bool AudioMixerEngine::isOutputEqActive(OutputId out) const {
    return out < kMixerEqOutputs && outputEq_[out].on();
}

bool AudioMixerEngine::isOutputHeard(OutputId out) const {
    return out < kMixerMaxOutputs && outputs_[out].heard.load(std::memory_order_relaxed);
}

void AudioMixerEngine::setChannelPan(ChannelId ch, float pan) {
    if (ch >= kMixerMaxChannels) return;
    if (pan < -1.0f) pan = -1.0f;
    if (pan >  1.0f) pan =  1.0f;
    channels_[ch].pan.store(pan, std::memory_order_relaxed);
}

float AudioMixerEngine::getChannelPan(ChannelId ch) const {
    return ch < kMixerMaxChannels ? channels_[ch].pan.load(std::memory_order_relaxed) : 0.0f;
}

const char* AudioMixerEngine::getChannelName(ChannelId ch) const {
    if (ch >= kMixerMaxChannels) return "";
    return channels_[ch].active.load(std::memory_order_acquire)
        ? channels_[ch].name : "";
}

bool AudioMixerEngine::isChannelActive(ChannelId ch) const {
    if (ch >= kMixerMaxChannels) return false;
    return channels_[ch].active.load(std::memory_order_acquire);
}

// ── Routing matrix ─────────────────────────────────────────────────────────

void AudioMixerEngine::setRouteEnabled(ChannelId ch, OutputId out, bool enabled) {
    if (ch >= kMixerMaxChannels || out >= kMixerMaxOutputs) return;
    channels_[ch].routeEnabled[out].store(enabled, std::memory_order_relaxed);
}

bool AudioMixerEngine::isRouteEnabled(ChannelId ch, OutputId out) const {
    if (ch >= kMixerMaxChannels || out >= kMixerMaxOutputs) return false;
    return channels_[ch].routeEnabled[out].load(std::memory_order_relaxed);
}

void AudioMixerEngine::setRouteVolume(ChannelId ch, OutputId out, float vol) {
    if (ch >= kMixerMaxChannels || out >= kMixerMaxOutputs) return;
    channels_[ch].routeVol[out].store(vol, std::memory_order_relaxed);
}

float AudioMixerEngine::getRouteVolume(ChannelId ch, OutputId out) const {
    if (ch >= kMixerMaxChannels || out >= kMixerMaxOutputs) return 0.0f;
    return channels_[ch].routeVol[out].load(std::memory_order_relaxed);
}

// ── Channel control ───────────────────────────────────────────────────────

void AudioMixerEngine::setChannelVolume(ChannelId ch, float vol) {
    if (ch >= kMixerMaxChannels) return;
    channels_[ch].volume.store(vol, std::memory_order_relaxed);
}

float AudioMixerEngine::getChannelVolume(ChannelId ch) const {
    if (ch >= kMixerMaxChannels) return 0.0f;
    return channels_[ch].volume.load(std::memory_order_relaxed);
}

void AudioMixerEngine::setChannelTrim(ChannelId ch, float gain) {
    if (ch >= kMixerMaxChannels) return;
    channels_[ch].trim.store(gain < 0.0f ? 0.0f : gain, std::memory_order_relaxed);
}

float AudioMixerEngine::getChannelTrim(ChannelId ch) const {
    if (ch >= kMixerMaxChannels) return 1.0f;
    return channels_[ch].trim.load(std::memory_order_relaxed);
}

void AudioMixerEngine::setChannelMute(ChannelId ch, bool muted) {
    if (ch >= kMixerMaxChannels) return;
    channels_[ch].muted.store(muted, std::memory_order_relaxed);
}

bool AudioMixerEngine::isChannelMuted(ChannelId ch) const {
    if (ch >= kMixerMaxChannels) return false;
    return channels_[ch].muted.load(std::memory_order_relaxed);
}

void AudioMixerEngine::setChannelSolo(ChannelId ch, bool soloed) {
    if (ch >= kMixerMaxChannels) return;
    channels_[ch].soloed.store(soloed, std::memory_order_relaxed);
}

bool AudioMixerEngine::isChannelSoloed(ChannelId ch) const {
    if (ch >= kMixerMaxChannels) return false;
    return channels_[ch].soloed.load(std::memory_order_relaxed);
}

bool AudioMixerEngine::isAnySoloed() const {
    for (const auto& slot : channels_) {
        if (!slot.active.load(std::memory_order_acquire)) continue;
        if (slot.soloed.load(std::memory_order_relaxed)) return true;
    }
    return false;
}

// ── Output bus control ────────────────────────────────────────────────────

void AudioMixerEngine::setOutputVolume(OutputId out, float vol) {
    if (out >= kMixerMaxOutputs || outputs_[out].unity.load(std::memory_order_relaxed)) return;
    outputs_[out].volume.store(vol, std::memory_order_relaxed);
}

float AudioMixerEngine::getOutputVolume(OutputId out) const {
    if (out >= kMixerMaxOutputs) return 0.0f;
    return outputs_[out].volume.load(std::memory_order_relaxed);
}

void AudioMixerEngine::setOutputMute(OutputId out, bool muted) {
    if (out >= kMixerMaxOutputs || outputs_[out].unity.load(std::memory_order_relaxed)) return;
    outputs_[out].muted.store(muted, std::memory_order_relaxed);
}

bool AudioMixerEngine::isOutputMuted(OutputId out) const {
    if (out >= kMixerMaxOutputs) return false;
    return outputs_[out].muted.load(std::memory_order_relaxed);
}

void AudioMixerEngine::setOutputUnity(OutputId out, bool unity) {
    if (out >= kMixerMaxOutputs) return;
    outputs_[out].unity.store(unity, std::memory_order_relaxed);
    if (!unity) return;
    outputs_[out].volume.store(1.0f, std::memory_order_relaxed);
    outputs_[out].muted.store(false, std::memory_order_relaxed);
}

bool AudioMixerEngine::isOutputUnity(OutputId out) const {
    if (out >= kMixerMaxOutputs) return false;
    return outputs_[out].unity.load(std::memory_order_relaxed);
}

// ── Metering ──────────────────────────────────────────────────────────────

void AudioMixerEngine::getChannelLevel(ChannelId ch, float& left, float& right) const {
    if (ch >= kMixerMaxChannels) { left = right = 0.0f; return; }
    left  = channels_[ch].peakL.load(std::memory_order_relaxed);
    right = channels_[ch].peakR.load(std::memory_order_relaxed);
}

void AudioMixerEngine::getOutputLevel(OutputId out, float& left, float& right) const {
    if (out >= kMixerMaxOutputs) { left = right = 0.0f; return; }
    left  = outputs_[out].peakL.load(std::memory_order_relaxed);
    right = outputs_[out].peakR.load(std::memory_order_relaxed);
}

static inline int16_t floatPeakToInt16(float v) {
    if (v < 0)    v = -v;
    if (v > 1.0f) v = 1.0f;
    return static_cast<int16_t>(v * 32767.0f);
}

void AudioMixerEngine::getChannelLevel(ChannelId ch, int16_t& left, int16_t& right) const {
    float l = 0.0f, r = 0.0f;
    getChannelLevel(ch, l, r);
    left  = floatPeakToInt16(l);
    right = floatPeakToInt16(r);
}

void AudioMixerEngine::getOutputLevel(OutputId out, int16_t& left, int16_t& right) const {
    float l = 0.0f, r = 0.0f;
    getOutputLevel(out, l, r);
    left  = floatPeakToInt16(l);
    right = floatPeakToInt16(r);
}

// ── Render (single audio thread) ──────────────────────────────────────────

void AudioMixerEngine::renderTap(const ChannelId* chs, uint8_t n, float* out, uint32_t frames) {
    const uint32_t stereo = frames * 2;
    std::memset(out, 0, stereo * sizeof(float));
    bool anySoloed = false;
    for (uint8_t ch = 0; ch < kMixerMaxChannels; ++ch) {
        if (channels_[ch].active.load(std::memory_order_acquire) &&
            channels_[ch].soloed.load(std::memory_order_relaxed)) { anySoloed = true; break; }
    }
    for (uint8_t i = 0; i < n; ++i) {
        if (chs[i] >= kMixerMaxChannels) continue;
        const auto& slot = channels_[chs[i]];
        if (!slot.active.load(std::memory_order_acquire) || !slot.node) continue;
        if (slot.muted.load(std::memory_order_relaxed)) continue;
        if (anySoloed && !slot.soloed.load(std::memory_order_relaxed)) continue;
        if (slot.scratch.size() < stereo) continue;
        /* render() left the channel's block in its scratch, panned -- or, for a
         * block it did not send (peak 0), unpanned: ±0 either way, the same
         * bits once summed. A block of NaN and zeros is summed as it is. */
        const float vol = slot.volume.load(std::memory_order_relaxed) * slot.trim.load(std::memory_order_relaxed);
        crosspad::dsp::mac(out, slot.scratch.data(), vol, stereo);
    }
    float pL = 0.0f, pR = 0.0f;
    limit(out, frames, kLimitCeiling, tapLimGain_, tapLimHold_, tapLimMin_, pL, pR);
}

void AudioMixerEngine::renderOnly(const ChannelId* chs, uint8_t n, uint32_t frames) {
    const uint32_t stereo = frames * 2;
    for (uint8_t ch = 0; ch < kMixerMaxChannels; ++ch) {
        bool wanted = false;
        for (uint8_t i = 0; i < n && !wanted; ++i) wanted = chs[i] == ch;
        if (!wanted) continue;
        auto& slot = channels_[ch];
        if (!slot.active.load(std::memory_order_acquire) || !slot.node) continue;
        if (slot.scratch.size() < stereo) continue;
        std::memset(slot.scratch.data(), 0, stereo * sizeof(float));
        slot.node->mix(slot.scratch.data(), frames);
        float pL = 0.0f, pR = 0.0f;
        crosspad::dsp::peakStereo(slot.scratch.data(), frames, pL, pR);
        slot.peakL.store(pL, std::memory_order_relaxed);
        slot.peakR.store(pR, std::memory_order_relaxed);
        /* As render() pans a channel that sounded; one that did not stays the
         * zeros it is, which renderTap() sums the same either way. */
        const float pan = slot.pan.load(std::memory_order_relaxed);
        if ((pL != 0.0f || pR != 0.0f) && pan != 0.0f) {
            const float gl = pan > 0.0f ? 1.0f - pan : 1.0f;
            const float gr = pan < 0.0f ? 1.0f + pan : 1.0f;
            crosspad::dsp::scaleStereo(slot.scratch.data(), frames, gl, gr);
        }
    }
}

void AudioMixerEngine::setOutputCeiling(OutputId out, float ceiling) {
    if (out >= kMixerMaxOutputs) return;
    if (ceiling > kLimitCeiling) ceiling = kLimitCeiling;
    if (ceiling < 0.01f) ceiling = 0.01f;
    outputs_[out].ceiling.store(ceiling, std::memory_order_relaxed);
}

float AudioMixerEngine::getOutputCeiling(OutputId out) const {
    if (out >= kMixerMaxOutputs) return kLimitCeiling;
    return outputs_[out].ceiling.load(std::memory_order_relaxed);
}

void AudioMixerEngine::setOutputLookahead(OutputId out, bool on) {
    if (out >= kMixerMaxOutputs) return;
    OutputSlot& o = outputs_[out];
    /* Sized before the flag is published, and only the first time: the
     * render reads `held` only behind an acquire of `lookahead`. */
    if (on && o.held.empty()) o.held.assign((size_t)frameCount_ * 2, 0.0f);
    o.lookahead.store(on, std::memory_order_release);
}

float AudioMixerEngine::takeOutputLimit(OutputId out) {
    if (out == kTapLimit) return tapLimMin_.exchange(1.0f, std::memory_order_relaxed);
    if (out >= numOutputs_) return 1.0f;
    return outputs_[out].limMin.exchange(1.0f, std::memory_order_relaxed);
}

/* A peak limiter in place of the hard clip at full scale.
 *
 * Nothing in the mix keeps headroom of its own: the sample engines sum their
 * voices at unity and a fader goes to +6 dB. Sixteen pads at once put 1.66 on
 * the Sampler2 channel, so with the output fader near 0 dB the clamp in
 * render() cut the tops off -- heard as distortion -- and the pads tap, with
 * no fader at all, wrote it into every bounce and take (09.10.2026).
 *
 * Per block, which the render has whole before any of it leaves: the gain
 * comes down to what the block's peak allows and goes back towards unity
 * over kLimReleaseS, once kLimHoldS has passed without a block that needed it
 * where it is. A block below the ceiling with the gain back at 1 is only
 * scanned. The clamp in render() stays; nothing should reach it now. */
bool AudioMixerEngine::limit(float* buf, uint32_t frames, float ceiling, float& gain, uint16_t& hold,
                             std::atomic<float>& deepest, float& pL, float& pR) {
    const float over = gain > 0.0f ? ceiling / gain : 0.0f;   // what the gain as it stands lets through
    /* The peak first, a side each and with no branch per frame; the frame
     * where the block first goes over only when it does, which is rare. A
     * frame whose right side is NaN counts its left side here, where the
     * one-pass scan this replaced skipped the frame. */
    crosspad::dsp::peakStereo(buf, frames, pL, pR);
    const float peak = crosspad::dsp::maxOf(pL, pR);
    uint32_t first = frames;
    if (peak > over) {
        for (uint32_t i = 0; i < frames; ++i) {
            const float a = crosspad::dsp::maxOf(std::fabs(buf[i * 2 + 0]), std::fabs(buf[i * 2 + 1]));
            if (a > over) { first = i; break; }
        }
    }
    const float want = peak > ceiling ? ceiling / peak : 1.0f;
    if (want <= gain) hold = limHoldBlocks_;
    if (gain >= 1.0f && want >= 1.0f) return false;

    /* Down by the first frame the present gain would let over the ceiling,
     * so none does; held there for the rest of the block. Up, or down to a
     * level nothing in the block exceeds, across the whole block. */
    float end = gain;
    if (hold) {
        --hold;
    } else {
        end = gain + (1.0f - gain) * limRelease_;
        if (end > 0.999f) end = 1.0f;
    }
    uint32_t span = frames;
    if (want < end) {
        end = want;
        if (want < gain) span = first + 1;
    }
    const float step = (end - gain) / (float)span;
    float g = gain;
    for (uint32_t i = 0; i < frames; ++i) {
        if (i < span) g += step;
        buf[i * 2 + 0] *= g;
        buf[i * 2 + 1] *= g;
    }
    gain = end;
    if (end < deepest.load(std::memory_order_relaxed)) deepest.store(end, std::memory_order_relaxed);
    return true;
}

/* limit() a block late. Coming down within the frames before a peak is a step
 * when the peak is near the start of a block -- and on the USB recording bus,
 * with no fader in front of it, a chord in 16 levels of pitch put one there on
 * every hit: lim 0.367, clicks in the take and none on the jacks (10.10.2026).
 *
 * Here what leaves is the block held back, and the one coming in is known, so
 * the gain reaches what both need by the end of the outgoing block, in one
 * ramp across it. Nothing gets over the ceiling: the gain this block starts
 * at was already low enough for it a block ago (it was the incoming one
 * then), a ramp down only goes lower, and a ramp up ends where both allow. */
void AudioMixerEngine::limitAhead(OutputSlot& o, float* buf, uint32_t frames, float ceiling) {
    float* held = o.held.data();
    float inL = 0.0f, inR = 0.0f;
    crosspad::dsp::peakStereo(buf, frames, inL, inR);
    const float peak = crosspad::dsp::maxOf(inL, inR);   // the incoming block's, which leaves next time
    /* Only a ceiling lowered since the last block -- a jack turned up -- can
     * leave the starting gain too high for the held block. */
    float g = o.limGain;
    if (o.heldPeak * g > ceiling) g = ceiling / o.heldPeak;
    const float worst = peak > o.heldPeak ? peak : o.heldPeak;
    if (worst * g >= ceiling) o.limHold = limHoldBlocks_;
    float end = g;
    if (o.limHold) {
        --o.limHold;
    } else {
        end = g + (1.0f - g) * limRelease_;
        if (end > 0.999f) end = 1.0f;
    }
    if (worst * end > ceiling) end = ceiling / worst;

    const float step = (end - g) / (float)frames;
    for (uint32_t i = 0; i < frames; ++i) {
        g += step;
        const float l = buf[i * 2 + 0], r = buf[i * 2 + 1];
        buf[i * 2 + 0] = held[i * 2 + 0] * g;
        buf[i * 2 + 1] = held[i * 2 + 1] * g;
        held[i * 2 + 0] = l;
        held[i * 2 + 1] = r;
    }
    o.heldPeak = peak;
    o.limGain = end;
    if (end < o.limMin.load(std::memory_order_relaxed)) o.limMin.store(end, std::memory_order_relaxed);
}

CP_COLD void AudioMixerEngine::profEnable(bool on)
{
    profPasses_.store(0, std::memory_order_relaxed);
    for (auto& n : profNode_) n.store(0, std::memory_order_relaxed);
    for (auto& n : profNodeMax_) n.store(0, std::memory_order_relaxed);
    profPeak_.store(0, std::memory_order_relaxed);
    profClear_.store(0, std::memory_order_relaxed);
    profRoute_.store(0, std::memory_order_relaxed);
    profOut_.store(0, std::memory_order_relaxed);
    prof_.store(on && cyc_ != nullptr, std::memory_order_relaxed);
}

CP_COLD void AudioMixerEngine::profRead(char* out, uint32_t cap, float cpuMhz)
{
    if (!out || cap == 0) return;
    const uint32_t n = profPasses_.exchange(0, std::memory_order_relaxed);
    /* Tenths of a microsecond in integers, no double through snprintf: on the
     * board this runs on a 4 kB task already deep in a verb handler, and
     * dtoa's frames with an interrupt on top overflowed it. */
    const uint64_t d = uint64_t(n ? n : 1) * uint64_t(cpuMhz > 0 ? cpuMhz + 0.5f : 240.0f);
    auto tenths = [d](uint32_t units) {
        return (unsigned)(((uint64_t(units) << kProfShift) * 10 + d / 2) / d);
    };
    const unsigned pk = tenths(profPeak_.exchange(0, std::memory_order_relaxed));
    const unsigned cl = tenths(profClear_.exchange(0, std::memory_order_relaxed));
    const unsigned ro = tenths(profRoute_.exchange(0, std::memory_order_relaxed));
    const unsigned ou = tenths(profOut_.exchange(0, std::memory_order_relaxed));
    int w = std::snprintf(out, cap, "MIXPROF: on=%d passes=%u peak_us=%u.%u clear_us=%u.%u "
                          "route_us=%u.%u out_us=%u.%u nodes=",
                          prof_.load(std::memory_order_relaxed) ? 1 : 0, (unsigned)n,
                          pk / 10, pk % 10, cl / 10, cl % 10, ro / 10, ro % 10, ou / 10, ou % 10);
    /* Only the channels that cost something: sixteen "ch3=0.0" would push the
     * interesting ones off a 192-byte reply. Each is its average pass and,
     * after the slash, its longest: a spike shows which node made it. */
    const uint64_t one = uint64_t(cpuMhz > 0 ? cpuMhz + 0.5f : 240.0f);
    for (uint8_t ch = 0; ch < kMixerMaxChannels && w > 0 && (uint32_t)w + 16 < cap; ++ch) {
        const unsigned us = (tenths(profNode_[ch].exchange(0, std::memory_order_relaxed)) + 5) / 10;
        const unsigned mx = (unsigned)((uint64_t(profNodeMax_[ch].exchange(0, std::memory_order_relaxed))
                                        << kProfShift) / one);
        if (us < 1) continue;
        w += std::snprintf(out + w, cap - (uint32_t)w, "%u:%u/%u ", (unsigned)ch, us, mx);
    }
    if (w > 0 && (uint32_t)w + 3 < cap) std::snprintf(out + w, cap - (uint32_t)w, "\r\n");
}

void AudioMixerEngine::render(float* const* outBuses, uint8_t numOut, uint32_t frames) {
    if (numOut > numOutputs_) numOut = numOutputs_;
    const uint32_t stereo = frames * 2;
    const bool prof = prof_.load(std::memory_order_relaxed);
    uint32_t peakAcc = 0;

    // 1. Pull each active channel via its IAudioNode::mix() into per-channel
    //    scratch (cleared first because mix() is additive). Update per-channel
    //    peak from the rendered scratch, and note which channels sounded.
    uint32_t sounded = 0;   // bit per channel: a peak above 0 this block
    for (uint8_t ch = 0; ch < kMixerMaxChannels; ++ch) {
        auto& slot = channels_[ch];
        if (!slot.active.load(std::memory_order_acquire) || !slot.node) continue;

        const uint32_t c0 = prof ? cyc_() : 0;
        std::memset(slot.scratch.data(), 0, stereo * sizeof(float));
        slot.node->mix(slot.scratch.data(), frames);
        const uint32_t c1 = prof ? cyc_() : 0;
        if (prof) {
            const uint32_t u = profUnits(c1 - c0);
            profNode_[ch].fetch_add(u, std::memory_order_relaxed);
            if (u > profNodeMax_[ch].load(std::memory_order_relaxed)) profNodeMax_[ch].store(u, std::memory_order_relaxed);
        }

        float pL = 0.0f, pR = 0.0f;
        crosspad::dsp::peakStereo(slot.scratch.data(), frames, pL, pR);
        slot.peakL.store(pL, std::memory_order_relaxed);
        slot.peakR.store(pR, std::memory_order_relaxed);
        if (pL != 0.0f || pR != 0.0f) sounded |= 1u << ch;
        if (prof) peakAcc += cyc_() - c1;
    }
    if (prof) profPeak_.fetch_add(profUnits(peakAcc), std::memory_order_relaxed);

    // 2. Solo arbitration
    bool anySoloed = false;
    for (uint8_t ch = 0; ch < kMixerMaxChannels; ++ch) {
        if (!channels_[ch].active.load(std::memory_order_acquire)) continue;
        if (channels_[ch].soloed.load(std::memory_order_relaxed)) { anySoloed = true; break; }
    }

    // 3. Clear all output buses
    const uint32_t cClear0 = prof ? cyc_() : 0;
    for (uint8_t o = 0; o < numOut; ++o) {
        std::memset(outBuses[o], 0, stereo * sizeof(float));
    }
    const uint32_t cClear1 = prof ? cyc_() : 0;
    if (prof) profClear_.fetch_add(profUnits(cClear1 - cClear0), std::memory_order_relaxed);

    // 4. Route + mix
    //
    // Per (ch, out): out[s] += slot.scratch[s] * gain   (stereo-interleaved)
    //
    // One pass of madd.s (dsp::mac). esp-dsp has no fused multiply-add for
    // float, and its two passes -- a product into a scratch, then a sum --
    // read and wrote every sample three times where this does it once.
    const uint32_t cRoute0 = prof ? cyc_() : 0;
    for (uint8_t ch = 0; ch < kMixerMaxChannels; ++ch) {
        auto& slot = channels_[ch];
        if (!slot.active.load(std::memory_order_acquire) || !slot.node) continue;
        if (slot.muted.load(std::memory_order_relaxed)) continue;
        const bool soloed = slot.soloed.load(std::memory_order_relaxed);
        if (anySoloed && !soloed) continue;

        /* A peak of 0 is a block of ±0: panned and sent at any finite gain
         * it adds ±0 to buses that start at +0, and every bus comes out bit
         * for bit as it was. So it is neither panned nor sent -- a silent
         * Click, Song, Preview or USB channel costs no pass per send. Its
         * node was pulled and its meter set above, and renderTap() reads the
         * same zeros unpanned. NaN slips past the peak, so a block of only
         * NaN and zeros is kept off the buses too instead of poisoning every
         * one (renderTap() still sums it as it is). */
        if (!(sounded & (1u << ch))) continue;

        /* Pan once, in the channel's own scratch, before it is sent anywhere:
         * every send then carries the same placement, as on a desk. After the
         * meter above, which stays pre-fader and pre-pan. */
        const float pan = slot.pan.load(std::memory_order_relaxed);
        if (pan != 0.0f) {
            const float gl = pan > 0.0f ? 1.0f - pan : 1.0f;
            const float gr = pan < 0.0f ? 1.0f + pan : 1.0f;
            crosspad::dsp::scaleStereo(slot.scratch.data(), frames, gl, gr);
        }

        const float chVol = slot.volume.load(std::memory_order_relaxed) * slot.trim.load(std::memory_order_relaxed);
        const float* src = slot.scratch.data();
        for (uint8_t o = 0; o < numOut; ++o) {
            if (!slot.routeEnabled[o].load(std::memory_order_relaxed)) continue;
            const float gain = chVol * slot.routeVol[o].load(std::memory_order_relaxed);
            crosspad::dsp::mac(outBuses[o], src, gain, stereo);
        }
    }

    const uint32_t cRoute1 = prof ? cyc_() : 0;
    if (prof) profRoute_.fetch_add(profUnits(cRoute1 - cRoute0), std::memory_order_relaxed);

    // 5. Per-output volume + limiter + clamp + meter
    //
    // The limiter scans the block for its peak anyway; when it leaves the
    // block as it was, that block is under its ceiling, so the clamp has
    // nothing to do and the limiter's peaks are the meter's.
    for (uint8_t o = 0; o < numOut; ++o) {
        const bool  oMuted = outputs_[o].muted.load(std::memory_order_relaxed);
        const float oVol   = outputs_[o].volume.load(std::memory_order_relaxed);
        float* dst = outBuses[o];
        float pL = 0.0f, pR = 0.0f;
        bool settled = false;   // pL/pR are the block's as it leaves, nothing over full scale

        if (oMuted) {
            std::memset(dst, 0, stereo * sizeof(float));
            /* The sections forget where they were: coming back from mute
             * should not play out what was in them a minute ago. */
            if (o < kMixerEqOutputs) outputEq_[o].reset();
            outputs_[o].limGain = 1.0f;
            outputs_[o].limHold = 0;
            if (outputs_[o].lookahead.load(std::memory_order_acquire)) {
                std::fill(outputs_[o].held.begin(), outputs_[o].held.end(), 0.0f);
                outputs_[o].heldPeak = 0.0f;
            }
            settled = true;
        } else {
            /* The correction, then the fader -- none at unity: an output
             * whose level the hardware sets after the DAC stays there, and
             * a pass of multiplying by one is time the render does not have. */
            if (o < kMixerEqOutputs) outputEq_[o].process(dst, frames);
            if (oVol != 1.0f) crosspad::dsp::scaleStereo(dst, frames, oVol, oVol);
            OutputSlot& os = outputs_[o];
            const float ceiling = os.ceiling.load(std::memory_order_relaxed);
            /* Held back only by exactly one block: a call of another length
             * would hand out frames that are not the ones before. */
            if (os.lookahead.load(std::memory_order_acquire) && os.held.size() == stereo) {
                limitAhead(os, dst, frames, ceiling);
            } else {
                settled = !limit(dst, frames, ceiling, os.limGain, os.limHold, os.limMin, pL, pR);
            }
        }

        if (!settled) crosspad::dsp::clampPeakStereo(dst, frames, pL, pR);
        outputs_[o].peakL.store(pL, std::memory_order_relaxed);
        outputs_[o].peakR.store(pR, std::memory_order_relaxed);
    }

    // 6. Tap (CI capture) — int16 conversion of the configured output
    auto* tap = tapBuffer_.load(std::memory_order_relaxed);
    if (tap) {
        const uint8_t tapO = tapOutput_.load(std::memory_order_relaxed);
        if (tapO < numOut) {
            if (tapBuf_.size() < stereo) tapBuf_.assign(stereo, 0);
            const float* src = outBuses[tapO];
            for (uint32_t i = 0; i < stereo; ++i) {
                float v = src[i] * 32767.0f;
                if (v >  32767.0f) v =  32767.0f;
                if (v < -32768.0f) v = -32768.0f;
                tapBuf_[i] = static_cast<int16_t>(v);
            }
            tap->write(tapBuf_.data(), stereo);
        }
    }

    if (prof) {
        profOut_.fetch_add(profUnits(cyc_() - cRoute1), std::memory_order_relaxed);
        profPasses_.fetch_add(1, std::memory_order_relaxed);
    }
}

// ── Tap ───────────────────────────────────────────────────────────────────

void AudioMixerEngine::setTapBuffer(crosspad::AudioRingBuffer<int16_t>* buf) {
    tapBuffer_.store(buf, std::memory_order_relaxed);
}

void AudioMixerEngine::setTapOutput(OutputId out) {
    tapOutput_.store(out, std::memory_order_relaxed);
}

// State persistence (stateJson, applyStateJson, saveState, loadState) is in
// MixerState.cpp: it runs on the LVGL thread and at boot, never per block,
// and the platform builds it for size where this file is built for speed.
