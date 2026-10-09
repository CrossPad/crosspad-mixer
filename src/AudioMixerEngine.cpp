/**
 * @file AudioMixerEngine.cpp
 * @brief Cross-platform audio routing engine implementation.
 */

#include "crosspad-mixer/AudioMixerEngine.hpp"

#include <crosspad/audio/IAudioNode.hpp>
#include <crosspad/audio/AudioFormatConvert.hpp>

#if defined(CROSSPAD_USE_ESP_DSP) && __has_include(<esp_dsp.h>)
  #include <esp_dsp.h>
  #define CROSSPAD_MIXER_USE_ESP_DSP 1
#else
  #define CROSSPAD_MIXER_USE_ESP_DSP 0
#endif

#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>
#include <string>

AudioMixerEngine::AudioMixerEngine() {
    for (auto& slot : channels_) {
        for (uint8_t o = 0; o < kMixerMaxOutputs; ++o) {
            slot.routeVol[o].store(1.0f, std::memory_order_relaxed);
            slot.routeEnabled[o].store(false, std::memory_order_relaxed);
        }
    }
}

AudioMixerEngine::~AudioMixerEngine() = default;

// ── Lifecycle ──────────────────────────────────────────────────────────────

void AudioMixerEngine::setup(uint32_t frameCount, uint32_t sampleRate, uint8_t numOutputs) {
    for (auto& eq : outputEq_) eq.setRate(sampleRate);

    frameCount_ = frameCount;
    sampleRate_ = sampleRate;
    if (numOutputs == 0) numOutputs = 1;
    if (numOutputs > kMixerMaxOutputs) numOutputs = kMixerMaxOutputs;
    numOutputs_ = numOutputs;

    const uint32_t stereoSamples = frameCount * 2;
    for (auto& slot : channels_) {
        if (slot.scratch.size() < stereoSamples) slot.scratch.resize(stereoSamples);
        // Propagate the new buffer-size + SR to already-registered channel
        // nodes so they can pre-size their own scratch (RT-safe contract).
        if (slot.active.load(std::memory_order_acquire) && slot.node) {
            slot.node->onPrepare(frameCount, sampleRate);
        }
    }
    // Render hot path uses this for the (channel × gain) product before
    // accumulating into the output bus (esp-dsp two-pass MAC). Sized once.
    if (mixScratch_.size() < stereoSamples) mixScratch_.assign(stereoSamples, 0.0f);
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

void AudioMixerEngine::setDefaults() {
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

AudioMixerEngine::ChannelId
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

void AudioMixerEngine::removeChannel(ChannelId ch) {
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

void AudioMixerEngine::setOutputEq(OutputId out, const crosspad::EqSettings& eq) {
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
    if (out >= kMixerMaxOutputs) return;
    outputs_[out].volume.store(vol, std::memory_order_relaxed);
}

float AudioMixerEngine::getOutputVolume(OutputId out) const {
    if (out >= kMixerMaxOutputs) return 0.0f;
    return outputs_[out].volume.load(std::memory_order_relaxed);
}

void AudioMixerEngine::setOutputMute(OutputId out, bool muted) {
    if (out >= kMixerMaxOutputs) return;
    outputs_[out].muted.store(muted, std::memory_order_relaxed);
}

bool AudioMixerEngine::isOutputMuted(OutputId out) const {
    if (out >= kMixerMaxOutputs) return false;
    return outputs_[out].muted.load(std::memory_order_relaxed);
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

void AudioMixerEngine::renderTap(const ChannelId* chs, uint8_t n, float* out, uint32_t frames) const {
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
        const float vol = slot.volume.load(std::memory_order_relaxed);
        const float* src = slot.scratch.data();
        for (uint32_t s = 0; s < stereo; ++s) out[s] += src[s] * vol;
    }
}

void AudioMixerEngine::profEnable(bool on)
{
    profPasses_.store(0, std::memory_order_relaxed);
    for (auto& n : profNode_) n.store(0, std::memory_order_relaxed);
    profPeak_.store(0, std::memory_order_relaxed);
    profClear_.store(0, std::memory_order_relaxed);
    profRoute_.store(0, std::memory_order_relaxed);
    profOut_.store(0, std::memory_order_relaxed);
    prof_.store(on && cyc_ != nullptr, std::memory_order_relaxed);
}

void AudioMixerEngine::profRead(char* out, uint32_t cap, float cpuMhz)
{
    if (!out || cap == 0) return;
    const uint32_t n = profPasses_.exchange(0, std::memory_order_relaxed);
    const double d = (n ? double(n) : 1.0) * double(cpuMhz > 0 ? cpuMhz : 240.0);
    int w = std::snprintf(out, cap, "MIXPROF: on=%d passes=%u peak_us=%.1f clear_us=%.1f "
                          "route_us=%.1f out_us=%.1f nodes=",
                          prof_.load(std::memory_order_relaxed) ? 1 : 0, (unsigned)n,
                          double(profPeak_.exchange(0, std::memory_order_relaxed)) / d,
                          double(profClear_.exchange(0, std::memory_order_relaxed)) / d,
                          double(profRoute_.exchange(0, std::memory_order_relaxed)) / d,
                          double(profOut_.exchange(0, std::memory_order_relaxed)) / d);
    /* Only the channels that cost something: sixteen "ch3=0.0" would push the
     * interesting ones off a 192-byte reply. */
    for (uint8_t ch = 0; ch < kMixerMaxChannels && w > 0 && (uint32_t)w + 16 < cap; ++ch) {
        const double us = double(profNode_[ch].exchange(0, std::memory_order_relaxed)) / d;
        if (us < 1.0) continue;
        w += std::snprintf(out + w, cap - (uint32_t)w, "%u:%.0f ", (unsigned)ch, us);
    }
    if (w > 0 && (uint32_t)w + 3 < cap) std::snprintf(out + w, cap - (uint32_t)w, "\r\n");
}

void AudioMixerEngine::render(float* const* outBuses, uint8_t numOut, uint32_t frames) {
    if (numOut > numOutputs_) numOut = numOutputs_;
    const uint32_t stereo = frames * 2;
    const bool prof = prof_.load(std::memory_order_relaxed);
    uint64_t peakAcc = 0;

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
        if (prof) profNode_[ch].fetch_add(c1 - c0, std::memory_order_relaxed);

        float pL = 0.0f, pR = 0.0f;
        for (uint32_t i = 0; i < frames; ++i) {
            float l = slot.scratch[i * 2 + 0]; if (l < 0) l = -l;
            float r = slot.scratch[i * 2 + 1]; if (r < 0) r = -r;
            if (l > pL) pL = l;
            if (r > pR) pR = r;
        }
        slot.peakL.store(pL, std::memory_order_relaxed);
        slot.peakR.store(pR, std::memory_order_relaxed);
        if (pL != 0.0f || pR != 0.0f) sounded |= 1u << ch;
        if (prof) peakAcc += cyc_() - c1;
    }
    if (prof) profPeak_.fetch_add(peakAcc, std::memory_order_relaxed);

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
    if (prof) profClear_.fetch_add(cClear1 - cClear0, std::memory_order_relaxed);

    // 4. Route + mix
    //
    // Per (ch, out): out[s] += slot.scratch[s] * gain   (stereo-interleaved)
    //
    // esp-dsp on Xtensa LX7 has no fused MAC for float, so the optimised
    // path is two passes over the buffer:
    //   dsps_mulc_f32: mixScratch_ = src * gain
    //   dsps_add_f32 : dst         = dst + mixScratch_
    // For sub-1-KiB stereo buffers (typical: 64..256 frames = 256..1024 B)
    // both passes stay in L1, so the win comes from the hand-tuned LX7
    // FPU loops vs the C compiler's auto-vectoriser.
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
            float* s = slot.scratch.data();
            for (uint32_t i = 0; i < frames; ++i) {
                s[i * 2 + 0] *= gl;
                s[i * 2 + 1] *= gr;
            }
        }

        const float chVol = slot.volume.load(std::memory_order_relaxed);
        const float* src = slot.scratch.data();
        for (uint8_t o = 0; o < numOut; ++o) {
            if (!slot.routeEnabled[o].load(std::memory_order_relaxed)) continue;
            const float gain = chVol * slot.routeVol[o].load(std::memory_order_relaxed);
            float* dst = outBuses[o];
#if CROSSPAD_MIXER_USE_ESP_DSP
            float* tmp = mixScratch_.data();
            dsps_mulc_f32(src, tmp, stereo, gain, 1, 1);
            dsps_add_f32 (dst, tmp, dst, stereo, 1, 1, 1);
#else
            for (uint32_t s = 0; s < stereo; ++s) dst[s] += src[s] * gain;
#endif
        }
    }

    const uint32_t cRoute1 = prof ? cyc_() : 0;
    if (prof) profRoute_.fetch_add(cRoute1 - cRoute0, std::memory_order_relaxed);

    // 5. Per-output volume + clamp + meter
    //
    // dsps_mulc_f32 runs in-place to apply the master output gain, then a
    // scalar pass handles clamp + peak tracking (esp-dsp has no clamp/abs-
    // max helper that fits this layout, and the scalar loop fuses fine).
    for (uint8_t o = 0; o < numOut; ++o) {
        const bool  oMuted = outputs_[o].muted.load(std::memory_order_relaxed);
        const float oVol   = outputs_[o].volume.load(std::memory_order_relaxed);
        float* dst = outBuses[o];

        if (oMuted) {
            std::memset(dst, 0, stereo * sizeof(float));
            /* The sections forget where they were: coming back from mute
             * should not play out what was in them a minute ago. */
            if (o < kMixerEqOutputs) outputEq_[o].reset();
        } else {
            /* The correction, then the fader. */
            if (o < kMixerEqOutputs) outputEq_[o].process(dst, frames);
#if CROSSPAD_MIXER_USE_ESP_DSP
            dsps_mulc_f32(dst, dst, stereo, oVol, 1, 1);
#else
            for (uint32_t i = 0; i < stereo; ++i) dst[i] *= oVol;
#endif
        }

        float pL = 0.0f, pR = 0.0f;
        for (uint32_t i = 0; i < frames; ++i) {
            float l = dst[i * 2 + 0];
            float r = dst[i * 2 + 1];
            if (l >  1.0f) l =  1.0f;
            if (l < -1.0f) l = -1.0f;
            if (r >  1.0f) r =  1.0f;
            if (r < -1.0f) r = -1.0f;
            dst[i * 2 + 0] = l;
            dst[i * 2 + 1] = r;
            float aL = l < 0 ? -l : l;
            float aR = r < 0 ? -r : r;
            if (aL > pL) pL = aL;
            if (aR > pR) pR = aR;
        }
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
        profOut_.fetch_add(cyc_() - cRoute1, std::memory_order_relaxed);
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
