#pragma once

/**
 * @file AudioMixerEngine.hpp
 * @brief Real-time audio mixing/routing engine for CrossPad PC.
 *
 * Routes 3 inputs (IN1, IN2, Synth) to 2 outputs (OUT1, OUT2) with per-route
 * volume, per-channel mute/solo, and peak level metering.
 *
 * Driven synchronously from the IAudioModule pipeline thread via render() —
 * single-writer guarantees the synth's FmSynth state is touched only from
 * one thread, eliminating the distortion seen with the legacy mixer thread.
 */

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>
#include <crosspad/audio/AudioRingBuffer.hpp>

static constexpr int MIXER_NUM_INPUTS  = 3;  // IN1, IN2, SYNTH
static constexpr int MIXER_NUM_OUTPUTS = 2;  // OUT1, OUT2
static constexpr uint32_t MIXER_CHUNK_FRAMES = 256;

enum class MixerInput : uint8_t {
    IN1   = 0,
    IN2   = 1,
    SYNTH = 2
};

enum class MixerOutput : uint8_t {
    OUT1 = 0,
    OUT2 = 1
};

struct MixerRoute {
    std::atomic<float>  volume{1.0f};
    std::atomic<bool>   enabled{false};
};

struct MixerChannel {
    std::atomic<float>   volume{1.0f};
    std::atomic<bool>    muted{false};
    std::atomic<bool>    soloed{false};
    std::atomic<int16_t> peakL{0};
    std::atomic<int16_t> peakR{0};
};

struct MixerOutputBus {
    std::atomic<float>   volume{1.0f};
    std::atomic<bool>    muted{false};
    std::atomic<int16_t> peakL{0};
    std::atomic<int16_t> peakR{0};
};

class AudioMixerEngine {
public:
    AudioMixerEngine() = default;
    ~AudioMixerEngine();

    /// Synchronously render `frames` of stereo audio into out0/out1 float buffers
    /// in [-1.0, 1.0]. Must be called from a single thread. Pulls IN1/IN2 from
    /// pc_platform_get_audio_input, renders the synth via ISynthEngine::process,
    /// applies channel gain/mute/solo, route matrix, and per-output volume.
    void render(float* out0, float* out1, uint32_t frames);

    /// Legacy thread-based driver — now no-ops kept for ABI/UI compatibility.
    /// Driving is done by IAudioModule pipeline calling render().
    void start() {}
    void stop()  {}
    bool isRunning() const { return true; }

    // Route matrix
    void setRouteEnabled(MixerInput in, MixerOutput out, bool enabled);
    bool isRouteEnabled(MixerInput in, MixerOutput out) const;
    void setRouteVolume(MixerInput in, MixerOutput out, float vol);
    float getRouteVolume(MixerInput in, MixerOutput out) const;

    // Channel control
    void setChannelVolume(MixerInput in, float vol);
    float getChannelVolume(MixerInput in) const;
    void setChannelMute(MixerInput in, bool muted);
    bool isChannelMuted(MixerInput in) const;
    void setChannelSolo(MixerInput in, bool soloed);
    bool isChannelSoloed(MixerInput in) const;

    // Output bus control
    void setOutputVolume(MixerOutput out, float vol);
    float getOutputVolume(MixerOutput out) const;
    void setOutputMute(MixerOutput out, bool muted);
    bool isOutputMuted(MixerOutput out) const;

    // Level metering
    void getChannelLevel(MixerInput in, int16_t& left, int16_t& right) const;
    void getOutputLevel(MixerOutput out, int16_t& left, int16_t& right) const;

    bool isAnySoloed() const;

    // State persistence
    void saveState(const std::string& path) const;
    void loadState(const std::string& path);

    // Set defaults (SYNTH->OUT1 enabled) without loading from file
    void setDefaults();

    // ── Tap buffer for audio capture (CI testing) ────────────────
    /// Set a ring buffer to receive a copy of OUT1 audio.
    /// Pass nullptr to disable. Caller owns the buffer.
    void setTapBuffer(crosspad::AudioRingBuffer<int16_t>* buf) { tapBuffer_.store(buf); }

    /// Which output to tap (default: OUT1)
    void setTapOutput(MixerOutput out) { tapOutput_.store(static_cast<uint8_t>(out)); }

private:
    MixerRoute     routes_[MIXER_NUM_INPUTS][MIXER_NUM_OUTPUTS];
    MixerChannel   channels_[MIXER_NUM_INPUTS];
    MixerOutputBus outputs_[MIXER_NUM_OUTPUTS];

    std::atomic<crosspad::AudioRingBuffer<int16_t>*> tapBuffer_{nullptr};
    std::atomic<uint8_t> tapOutput_{0};  // MixerOutput::OUT1

    // Render-time scratch (sized lazily on first render call).
    std::vector<float>   inFloatBuf_[MIXER_NUM_INPUTS];   // interleaved stereo float
    std::vector<int16_t> inInt16Buf_[MIXER_NUM_INPUTS - 1]; // IN1/IN2 raw read buffer
    std::vector<int16_t> tapBuf_;
    bool sampleRateLogged_ = false;
};

/// Global mixer engine accessor (initialized in crosspad_app.cpp)
AudioMixerEngine& getMixerEngine();
