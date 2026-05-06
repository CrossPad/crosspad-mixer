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

#include <ArduinoJson.h>

#include <cstdio>
#include <cstring>
#include <algorithm>
#include <fstream>

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

void AudioMixerEngine::render(float* const* outBuses, uint8_t numOut, uint32_t frames) {
    if (numOut > numOutputs_) numOut = numOutputs_;
    const uint32_t stereo = frames * 2;

    // 1. Pull each active channel via its IAudioNode::mix() into per-channel
    //    scratch (cleared first because mix() is additive). Update per-channel
    //    peak from the rendered scratch.
    for (uint8_t ch = 0; ch < kMixerMaxChannels; ++ch) {
        auto& slot = channels_[ch];
        if (!slot.active.load(std::memory_order_acquire) || !slot.node) continue;

        std::memset(slot.scratch.data(), 0, stereo * sizeof(float));
        slot.node->mix(slot.scratch.data(), frames);

        float pL = 0.0f, pR = 0.0f;
        for (uint32_t i = 0; i < frames; ++i) {
            float l = slot.scratch[i * 2 + 0]; if (l < 0) l = -l;
            float r = slot.scratch[i * 2 + 1]; if (r < 0) r = -r;
            if (l > pL) pL = l;
            if (r > pR) pR = r;
        }
        slot.peakL.store(pL, std::memory_order_relaxed);
        slot.peakR.store(pR, std::memory_order_relaxed);
    }

    // 2. Solo arbitration
    bool anySoloed = false;
    for (uint8_t ch = 0; ch < kMixerMaxChannels; ++ch) {
        if (!channels_[ch].active.load(std::memory_order_acquire)) continue;
        if (channels_[ch].soloed.load(std::memory_order_relaxed)) { anySoloed = true; break; }
    }

    // 3. Clear all output buses
    for (uint8_t o = 0; o < numOut; ++o) {
        std::memset(outBuses[o], 0, stereo * sizeof(float));
    }

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
    for (uint8_t ch = 0; ch < kMixerMaxChannels; ++ch) {
        auto& slot = channels_[ch];
        if (!slot.active.load(std::memory_order_acquire) || !slot.node) continue;
        if (slot.muted.load(std::memory_order_relaxed)) continue;
        const bool soloed = slot.soloed.load(std::memory_order_relaxed);
        if (anySoloed && !soloed) continue;

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
        } else {
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
}

// ── Tap ───────────────────────────────────────────────────────────────────

void AudioMixerEngine::setTapBuffer(crosspad::AudioRingBuffer<int16_t>* buf) {
    tapBuffer_.store(buf, std::memory_order_relaxed);
}

void AudioMixerEngine::setTapOutput(OutputId out) {
    tapOutput_.store(out, std::memory_order_relaxed);
}

// ── State persistence (filesystem JSON; PC; gated to non-embedded) ────────

void AudioMixerEngine::saveState(const std::string& path) const {
    std::lock_guard<std::mutex> lock(registrationMutex_);

    JsonDocument doc;
    doc["version"] = 2;
    doc["numOutputs"] = numOutputs_;

    JsonArray channelsArr = doc["channels"].to<JsonArray>();
    for (uint8_t ch = 0; ch < kMixerMaxChannels; ++ch) {
        const auto& slot = channels_[ch];
        if (!slot.active.load(std::memory_order_acquire)) continue;
        JsonObject c = channelsArr.add<JsonObject>();
        c["id"]     = ch;
        c["name"]   = slot.name;
        c["volume"] = slot.volume.load(std::memory_order_relaxed);
        c["muted"]  = slot.muted .load(std::memory_order_relaxed);
        c["soloed"] = slot.soloed.load(std::memory_order_relaxed);
        JsonArray routes = c["routes"].to<JsonArray>();
        for (uint8_t o = 0; o < numOutputs_; ++o) {
            JsonObject r = routes.add<JsonObject>();
            r["out"]     = o;
            r["enabled"] = slot.routeEnabled[o].load(std::memory_order_relaxed);
            r["volume"]  = slot.routeVol[o].load(std::memory_order_relaxed);
        }
    }

    JsonArray outArr = doc["outputs"].to<JsonArray>();
    for (uint8_t o = 0; o < numOutputs_; ++o) {
        JsonObject ob = outArr.add<JsonObject>();
        ob["volume"] = outputs_[o].volume.load(std::memory_order_relaxed);
        ob["muted"]  = outputs_[o].muted .load(std::memory_order_relaxed);
    }

    // Atomic file write: tmp + rename, prevents partial reads on kill.
    const std::string tmpPath = path + ".tmp";
    {
        std::ofstream f(tmpPath);
        if (!f.is_open()) {
            printf("[Mixer] Failed to save state to %s\n", tmpPath.c_str());
            return;
        }
        serializeJsonPretty(doc, f);
    }
    if (std::rename(tmpPath.c_str(), path.c_str()) != 0) {
        printf("[Mixer] Failed to commit %s → %s\n", tmpPath.c_str(), path.c_str());
        std::remove(tmpPath.c_str());
        return;
    }
    printf("[Mixer] State saved to %s\n", path.c_str());
}

void AudioMixerEngine::loadState(const std::string& path) {
    std::lock_guard<std::mutex> lock(registrationMutex_);

    std::ifstream f(path);
    if (!f.is_open()) {
        printf("[Mixer] No saved state at %s, leaving defaults\n", path.c_str());
        return;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, f);
    if (err) {
        printf("[Mixer] State parse error: %s, leaving defaults\n", err.c_str());
        return;
    }

    JsonArray channelsArr = doc["channels"];
    if (channelsArr) {
        for (JsonObject c : channelsArr) {
            int id = c["id"] | -1;
            if (id < 0 || id >= kMixerMaxChannels) continue;
            auto& slot = channels_[id];
            slot.volume.store(c["volume"] | 1.0f, std::memory_order_relaxed);
            slot.muted .store(c["muted"]  | false, std::memory_order_relaxed);
            slot.soloed.store(c["soloed"] | false, std::memory_order_relaxed);
            JsonArray routes = c["routes"];
            if (routes) {
                for (JsonObject r : routes) {
                    int o = r["out"] | -1;
                    if (o < 0 || o >= kMixerMaxOutputs) continue;
                    slot.routeEnabled[o].store(r["enabled"] | false, std::memory_order_relaxed);
                    slot.routeVol[o]    .store(r["volume"]  | 1.0f,  std::memory_order_relaxed);
                }
            }
        }
    }

    JsonArray outArr = doc["outputs"];
    if (outArr) {
        int idx = 0;
        for (JsonObject ob : outArr) {
            if (idx >= kMixerMaxOutputs) break;
            outputs_[idx].volume.store(ob["volume"] | 1.0f, std::memory_order_relaxed);
            outputs_[idx].muted .store(ob["muted"]  | false, std::memory_order_relaxed);
            ++idx;
        }
    }

    printf("[Mixer] State loaded from %s\n", path.c_str());
}
