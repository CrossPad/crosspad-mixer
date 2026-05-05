/**
 * @file AudioMixerEngine.cpp
 * @brief Audio mixer rendering — synchronous driver from IAudioModule pipeline.
 */

#include "AudioMixerEngine.hpp"

#include "pc_stubs/pc_platform.h"
#include "synth/MlPianoSynth.hpp"
#include "crosspad/synth/IAudioInput.hpp"
#include "crosspad/synth/ISynthEngine.hpp"

#include <ArduinoJson.h>

#include <cstdio>
#include <cstring>
#include <algorithm>
#include <vector>
#include <fstream>

AudioMixerEngine::~AudioMixerEngine() = default;

// ── Route matrix ──

void AudioMixerEngine::setRouteEnabled(MixerInput in, MixerOutput out, bool enabled)
{
    routes_[(int)in][(int)out].enabled.store(enabled, std::memory_order_relaxed);
}

bool AudioMixerEngine::isRouteEnabled(MixerInput in, MixerOutput out) const
{
    return routes_[(int)in][(int)out].enabled.load(std::memory_order_relaxed);
}

void AudioMixerEngine::setRouteVolume(MixerInput in, MixerOutput out, float vol)
{
    routes_[(int)in][(int)out].volume.store(vol, std::memory_order_relaxed);
}

float AudioMixerEngine::getRouteVolume(MixerInput in, MixerOutput out) const
{
    return routes_[(int)in][(int)out].volume.load(std::memory_order_relaxed);
}

// ── Channel control ──

void AudioMixerEngine::setChannelVolume(MixerInput in, float vol)
{
    channels_[(int)in].volume.store(vol, std::memory_order_relaxed);
}

float AudioMixerEngine::getChannelVolume(MixerInput in) const
{
    return channels_[(int)in].volume.load(std::memory_order_relaxed);
}

void AudioMixerEngine::setChannelMute(MixerInput in, bool muted)
{
    channels_[(int)in].muted.store(muted, std::memory_order_relaxed);
}

bool AudioMixerEngine::isChannelMuted(MixerInput in) const
{
    return channels_[(int)in].muted.load(std::memory_order_relaxed);
}

void AudioMixerEngine::setChannelSolo(MixerInput in, bool soloed)
{
    channels_[(int)in].soloed.store(soloed, std::memory_order_relaxed);
}

bool AudioMixerEngine::isChannelSoloed(MixerInput in) const
{
    return channels_[(int)in].soloed.load(std::memory_order_relaxed);
}

// ── Output bus ──

void AudioMixerEngine::setOutputVolume(MixerOutput out, float vol)
{
    outputs_[(int)out].volume.store(vol, std::memory_order_relaxed);
}

float AudioMixerEngine::getOutputVolume(MixerOutput out) const
{
    return outputs_[(int)out].volume.load(std::memory_order_relaxed);
}

void AudioMixerEngine::setOutputMute(MixerOutput out, bool muted)
{
    outputs_[(int)out].muted.store(muted, std::memory_order_relaxed);
}

bool AudioMixerEngine::isOutputMuted(MixerOutput out) const
{
    return outputs_[(int)out].muted.load(std::memory_order_relaxed);
}

// ── Metering ──

void AudioMixerEngine::getChannelLevel(MixerInput in, int16_t& left, int16_t& right) const
{
    left  = channels_[(int)in].peakL.load(std::memory_order_relaxed);
    right = channels_[(int)in].peakR.load(std::memory_order_relaxed);
}

void AudioMixerEngine::getOutputLevel(MixerOutput out, int16_t& left, int16_t& right) const
{
    left  = outputs_[(int)out].peakL.load(std::memory_order_relaxed);
    right = outputs_[(int)out].peakR.load(std::memory_order_relaxed);
}

bool AudioMixerEngine::isAnySoloed() const
{
    for (int i = 0; i < MIXER_NUM_INPUTS; i++) {
        if (channels_[i].soloed.load(std::memory_order_relaxed))
            return true;
    }
    return false;
}

// ── Helper: compute peak from interleaved stereo buffer ──

static void computePeak(const int16_t* buf, uint32_t frames,
                        std::atomic<int16_t>& peakL, std::atomic<int16_t>& peakR)
{
    int16_t maxL = 0, maxR = 0;
    for (uint32_t i = 0; i < frames; i++) {
        int16_t absL = buf[i * 2]     < 0 ? (int16_t)-buf[i * 2]     : buf[i * 2];
        int16_t absR = buf[i * 2 + 1] < 0 ? (int16_t)-buf[i * 2 + 1] : buf[i * 2 + 1];
        if (absL > maxL) maxL = absL;
        if (absR > maxR) maxR = absR;
    }
    peakL.store(maxL, std::memory_order_relaxed);
    peakR.store(maxR, std::memory_order_relaxed);
}

// ── Synchronous render API (driven by IAudioModule pipeline) ──

void AudioMixerEngine::render(float* out0, float* out1, uint32_t frames)
{
    const uint32_t stereo = frames * 2;

    // Lazy scratch alloc
    for (auto& b : inFloatBuf_) {
        if (b.size() < stereo) b.assign(stereo, 0.0f);
    }
    for (auto& b : inInt16Buf_) {
        if (b.size() < stereo) b.assign(stereo, 0);
    }

    // ── 1. Pull inputs ──
    auto pullInput = [&](uint8_t idx) {
        crosspad::IAudioInput* in = pc_platform_get_audio_input(idx);
        auto& i16 = inInt16Buf_[idx];
        auto& f   = inFloatBuf_[idx];
        if (!in) {
            std::memset(f.data(), 0, stereo * sizeof(float));
            return;
        }
        uint32_t got = in->read(i16.data(), frames);
        if (got < frames) {
            std::memset(i16.data() + got * 2, 0, (frames - got) * 2 * sizeof(int16_t));
        }
        constexpr float kInt16ToFloat = 1.0f / 32768.0f;
        for (uint32_t i = 0; i < stereo; ++i) f[i] = i16[i] * kInt16ToFloat;
    };
    pullInput(0); // IN1
    pullInput(1); // IN2

    // SYNTH — render directly as float
    {
        crosspad::ISynthEngine* synth = pc_platform_get_synth_engine();
        auto& f = inFloatBuf_[(int)MixerInput::SYNTH];
        if (synth) {
            synth->process(f.data(), frames);
        } else {
            std::memset(f.data(), 0, stereo * sizeof(float));
        }
    }

    // ── 2. Per-channel peaks (float → int16-equivalent for UI) ──
    for (int ch = 0; ch < MIXER_NUM_INPUTS; ++ch) {
        const auto& f = inFloatBuf_[ch];
        float maxL = 0.0f, maxR = 0.0f;
        for (uint32_t i = 0; i < frames; ++i) {
            float l = f[i * 2 + 0]; if (l < 0) l = -l;
            float r = f[i * 2 + 1]; if (r < 0) r = -r;
            if (l > maxL) maxL = l;
            if (r > maxR) maxR = r;
        }
        channels_[ch].peakL.store(static_cast<int16_t>(maxL * 32767.0f), std::memory_order_relaxed);
        channels_[ch].peakR.store(static_cast<int16_t>(maxR * 32767.0f), std::memory_order_relaxed);
    }

    // ── 3. Solo logic ──
    const bool anySoloed = isAnySoloed();

    // ── 4. Clear output buses ──
    std::memset(out0, 0, stereo * sizeof(float));
    std::memset(out1, 0, stereo * sizeof(float));
    float* outBuses[MIXER_NUM_OUTPUTS] = { out0, out1 };

    // ── 5. Route + mix ──
    for (int ch = 0; ch < MIXER_NUM_INPUTS; ++ch) {
        if (channels_[ch].muted.load(std::memory_order_relaxed)) continue;
        const bool solo = channels_[ch].soloed.load(std::memory_order_relaxed);
        if (anySoloed && !solo) continue;

        const float chVol = channels_[ch].volume.load(std::memory_order_relaxed);
        const auto& f = inFloatBuf_[ch];

        for (int o = 0; o < MIXER_NUM_OUTPUTS; ++o) {
            if (!routes_[ch][o].enabled.load(std::memory_order_relaxed)) continue;
            const float gain = chVol * routes_[ch][o].volume.load(std::memory_order_relaxed);
            float* dst = outBuses[o];
            for (uint32_t s = 0; s < stereo; ++s) dst[s] += f[s] * gain;
        }
    }

    // ── 6. Apply output volume + clamp + level meter + tap ──
    for (int o = 0; o < MIXER_NUM_OUTPUTS; ++o) {
        const bool muted = outputs_[o].muted.load(std::memory_order_relaxed);
        const float outVol = outputs_[o].volume.load(std::memory_order_relaxed);
        float* dst = outBuses[o];

        float maxL = 0.0f, maxR = 0.0f;
        for (uint32_t i = 0; i < frames; ++i) {
            float l = muted ? 0.0f : dst[i * 2 + 0] * outVol;
            float r = muted ? 0.0f : dst[i * 2 + 1] * outVol;
            if (l >  1.0f) l =  1.0f; if (l < -1.0f) l = -1.0f;
            if (r >  1.0f) r =  1.0f; if (r < -1.0f) r = -1.0f;
            dst[i * 2 + 0] = l;
            dst[i * 2 + 1] = r;
            float aL = l < 0 ? -l : l;
            float aR = r < 0 ? -r : r;
            if (aL > maxL) maxL = aL;
            if (aR > maxR) maxR = aR;
        }
        outputs_[o].peakL.store(static_cast<int16_t>(maxL * 32767.0f), std::memory_order_relaxed);
        outputs_[o].peakR.store(static_cast<int16_t>(maxR * 32767.0f), std::memory_order_relaxed);
    }

    // ── 7. Tap (CI capture) — int16 conversion of the configured output ──
    auto* tap = tapBuffer_.load(std::memory_order_relaxed);
    if (tap) {
        const uint8_t tapO = tapOutput_.load(std::memory_order_relaxed);
        if (tapO < MIXER_NUM_OUTPUTS) {
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


// ── Defaults ──

void AudioMixerEngine::setDefaults()
{
    // Default: SYNTH -> OUT1 enabled at full volume (matches legacy behavior)
    for (int i = 0; i < MIXER_NUM_INPUTS; i++) {
        for (int o = 0; o < MIXER_NUM_OUTPUTS; o++) {
            routes_[i][o].enabled.store(false, std::memory_order_relaxed);
            routes_[i][o].volume.store(1.0f, std::memory_order_relaxed);
        }
        channels_[i].volume.store(1.0f, std::memory_order_relaxed);
        channels_[i].muted.store(false, std::memory_order_relaxed);
        channels_[i].soloed.store(false, std::memory_order_relaxed);
    }
    for (int o = 0; o < MIXER_NUM_OUTPUTS; o++) {
        outputs_[o].volume.store(1.0f, std::memory_order_relaxed);
        outputs_[o].muted.store(false, std::memory_order_relaxed);
    }

    routes_[(int)MixerInput::SYNTH][(int)MixerOutput::OUT1].enabled.store(true, std::memory_order_relaxed);
}

// ── State persistence ──

void AudioMixerEngine::saveState(const std::string& path) const
{
    JsonDocument doc;

    // Route matrix
    JsonArray routesArr = doc["routes"].to<JsonArray>();
    for (int i = 0; i < MIXER_NUM_INPUTS; i++) {
        for (int o = 0; o < MIXER_NUM_OUTPUTS; o++) {
            JsonObject r = routesArr.add<JsonObject>();
            r["in"] = i;
            r["out"] = o;
            r["enabled"] = routes_[i][o].enabled.load(std::memory_order_relaxed);
            r["volume"] = routes_[i][o].volume.load(std::memory_order_relaxed);
        }
    }

    // Channel state
    JsonArray chArr = doc["channels"].to<JsonArray>();
    for (int i = 0; i < MIXER_NUM_INPUTS; i++) {
        JsonObject ch = chArr.add<JsonObject>();
        ch["volume"] = channels_[i].volume.load(std::memory_order_relaxed);
        ch["muted"] = channels_[i].muted.load(std::memory_order_relaxed);
        ch["soloed"] = channels_[i].soloed.load(std::memory_order_relaxed);
    }

    // Output bus state
    JsonArray outArr = doc["outputs"].to<JsonArray>();
    for (int o = 0; o < MIXER_NUM_OUTPUTS; o++) {
        JsonObject ob = outArr.add<JsonObject>();
        ob["volume"] = outputs_[o].volume.load(std::memory_order_relaxed);
        ob["muted"] = outputs_[o].muted.load(std::memory_order_relaxed);
    }

    std::ofstream f(path);
    if (!f.is_open()) {
        printf("[Mixer] Failed to save state to %s\n", path.c_str());
        return;
    }
    serializeJsonPretty(doc, f);
    printf("[Mixer] State saved to %s\n", path.c_str());
}

void AudioMixerEngine::loadState(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open()) {
        printf("[Mixer] No saved state at %s, using defaults\n", path.c_str());
        setDefaults();
        return;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, f);
    if (err) {
        printf("[Mixer] State parse error: %s, using defaults\n", err.c_str());
        setDefaults();
        return;
    }

    // Route matrix
    JsonArray routesArr = doc["routes"];
    if (routesArr) {
        for (JsonObject r : routesArr) {
            int i = r["in"] | 0;
            int o = r["out"] | 0;
            if (i >= 0 && i < MIXER_NUM_INPUTS && o >= 0 && o < MIXER_NUM_OUTPUTS) {
                routes_[i][o].enabled.store(r["enabled"] | false, std::memory_order_relaxed);
                routes_[i][o].volume.store(r["volume"] | 1.0f, std::memory_order_relaxed);
            }
        }
    }

    // Channel state
    JsonArray chArr = doc["channels"];
    if (chArr) {
        int idx = 0;
        for (JsonObject ch : chArr) {
            if (idx >= MIXER_NUM_INPUTS) break;
            channels_[idx].volume.store(ch["volume"] | 1.0f, std::memory_order_relaxed);
            channels_[idx].muted.store(ch["muted"] | false, std::memory_order_relaxed);
            channels_[idx].soloed.store(ch["soloed"] | false, std::memory_order_relaxed);
            idx++;
        }
    }

    // Output bus state
    JsonArray outArr = doc["outputs"];
    if (outArr) {
        int idx = 0;
        for (JsonObject ob : outArr) {
            if (idx >= MIXER_NUM_OUTPUTS) break;
            outputs_[idx].volume.store(ob["volume"] | 1.0f, std::memory_order_relaxed);
            outputs_[idx].muted.store(ob["muted"] | false, std::memory_order_relaxed);
            idx++;
        }
    }

    printf("[Mixer] State loaded from %s\n", path.c_str());
}
