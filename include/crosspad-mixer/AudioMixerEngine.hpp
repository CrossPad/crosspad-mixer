#pragma once

/**
 * @file AudioMixerEngine.hpp
 * @brief Cross-platform audio routing engine.
 *
 * Mixes any number of IAudioNode sources (sampler, synth, USB input, …) into
 * any number of output buses (multiple physical outputs / codecs / streams).
 * Driven synchronously from the audio thread via render() — single-writer.
 *
 * Slot layout: kMaxChannels × kMaxOutputs fixed array. Channels register via
 * addChannel() / removeChannel(); both are lock-against-each-other (mutex)
 * but lock-free against the render thread (atomic active flag with
 * release/acquire publish).
 *
 * Legacy enums MixerInput / MixerOutput are kept as numeric aliases for the
 * PC predefined IN1/IN2/SYNTH × OUT1/OUT2 layout — they are just typed
 * uint8_t indices and stay valid in the new dynamic API.
 */

#include <atomic>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <vector>
#include <crosspad/audio/AudioRingBuffer.hpp>

namespace crosspad { class IAudioNode; }

// ── Limits ─────────────────────────────────────────────────────────────────
static constexpr uint8_t kMixerMaxChannels = 16;
static constexpr uint8_t kMixerMaxOutputs  = 8;
static constexpr uint8_t kMixerInvalidChannelId = 0xFF;
static constexpr uint8_t kMixerNameLen = 24;

// ── Legacy enums (PC default IN1/IN2/SYNTH × OUT1/OUT2 layout) ─────────────
// These remain valid IDs in the new API; PC bootstrap registers nodes in
// IN1=0, IN2=1, SYNTH=2 order so existing UI/tests keep working.
enum class MixerInput : uint8_t {
    IN1   = 0,
    IN2   = 1,
    SYNTH = 2
};

enum class MixerOutput : uint8_t {
    OUT1 = 0,
    OUT2 = 1
};

class AudioMixerEngine {
public:
    using ChannelId = uint8_t;
    using OutputId  = uint8_t;

    AudioMixerEngine();
    ~AudioMixerEngine();

    // ── Lifecycle ──────────────────────────────────────────────────────────

    /// Configure render-time scratch sizing. Call once at audio init, before
    /// any render() call. `numOutputs` is clamped to [1, kMixerMaxOutputs].
    void setup(uint32_t frameCount, uint32_t sampleRate, uint8_t numOutputs);

    /// Reset routing/volume to defaults; channel slots stay registered.
    void setDefaults();

    // ── Channel registration (dynamic, called from app threads) ────────────

    /// Register a sound-producing node as a mixer channel. Node ownership
    /// stays with the caller; caller MUST removeChannel() before destroying.
    /// Default routing for a freshly added channel: route to output 0 only,
    /// gain 1.0, unmuted, unsoloed.
    ChannelId addChannel(crosspad::IAudioNode* node, const char* name);

    /// Drop a channel slot. Render thread stops touching it once this returns.
    /// Safe to call multiple times; idempotent.
    void removeChannel(ChannelId ch);

    /// Number of slots currently active (not the max).
    uint8_t numActiveChannels() const;

    /// Slot capacity (always kMixerMaxChannels; included for symmetry).
    uint8_t maxChannels() const { return kMixerMaxChannels; }

    /// Read-only channel name for UI. Returns "" for inactive slots.
    const char* getChannelName(ChannelId ch) const;
    bool isChannelActive(ChannelId ch) const;

    // ── Output configuration ───────────────────────────────────────────────

    /// Active output count. setup() establishes initial value.
    uint8_t numOutputs() const { return numOutputs_; }
    uint8_t maxOutputs() const { return kMixerMaxOutputs; }

    /// What an output is called on screen ("Speakers", "USB rec"). Set by the
    /// platform, which is the only one that knows where each bus goes; ""
    /// until it does.
    void        setOutputName(OutputId out, const char* name);
    const char* getOutputName(OutputId out) const;
    /// Whether an output is heard -- a speaker, a jack -- or only records (a
    /// USB recording bus): what an input's MONITOR switches between. True
    /// until the platform says otherwise.
    void setOutputHeard(OutputId out, bool heard);
    bool isOutputHeard(OutputId out) const;

    // ── Routing matrix ─────────────────────────────────────────────────────

    void  setRouteEnabled(ChannelId ch, OutputId out, bool enabled);
    bool  isRouteEnabled (ChannelId ch, OutputId out) const;
    void  setRouteVolume (ChannelId ch, OutputId out, float vol);
    float getRouteVolume (ChannelId ch, OutputId out) const;

    // ── Channel control ────────────────────────────────────────────────────

    void  setChannelVolume(ChannelId ch, float vol);
    float getChannelVolume(ChannelId ch) const;
    void  setChannelMute  (ChannelId ch, bool muted);
    bool  isChannelMuted  (ChannelId ch) const;
    void  setChannelSolo  (ChannelId ch, bool soloed);
    bool  isChannelSoloed (ChannelId ch) const;
    /// Balance, -1 (left) .. 0 (centre) .. +1 (right). Unity at the centre:
    /// turning it only ever takes level away from the other side, so a
    /// centred channel sounds exactly as it did before pan existed.
    void  setChannelPan   (ChannelId ch, float pan);
    float getChannelPan   (ChannelId ch) const;
    bool  isAnySoloed     () const;

    // ── Output bus control ─────────────────────────────────────────────────

    void  setOutputVolume(OutputId out, float vol);
    float getOutputVolume(OutputId out) const;
    void  setOutputMute  (OutputId out, bool muted);
    bool  isOutputMuted  (OutputId out) const;

    // ── Metering (float [0.0, 1.0]) ────────────────────────────────────────

    void getChannelLevel(ChannelId ch, float& left, float& right) const;
    void getOutputLevel (OutputId out, float& left, float& right) const;

    // Legacy int16 overloads for existing UI code (rescales internally)
    void getChannelLevel(ChannelId ch, int16_t& left, int16_t& right) const;
    void getOutputLevel (OutputId out, int16_t& left, int16_t& right) const;

    // ── Render ─────────────────────────────────────────────────────────────

    /// Render `frames` of stereo audio into each of `numOut` output buses.
    /// `outBuses[i]` must point to a buffer of `frames * 2` floats. Audio
    /// thread only — single writer.
    void render(float* const* outBuses, uint8_t numOut, uint32_t frames);

    // ── Tap (CI/test capture) ──────────────────────────────────────────────

    void setTapBuffer(crosspad::AudioRingBuffer<int16_t>* buf);
    void setTapOutput(OutputId out);

    // ── Persistence ────────────────────────────────────────────────────────

    void saveState(const std::string& path) const;
    void loadState(const std::string& path);

    /// The same state as a JSON string, for a platform that keeps it
    /// somewhere other than a file (NVS on the board).
    std::string stateJson() const;
    /// Apply a string from stateJson(). False when it does not parse, in
    /// which case nothing is changed.
    bool applyStateJson(const std::string& json);

    /// Called from non-render threads after the GUI mutates routing/volumes.
    /// Use it to flush state to disk / NVS without putting platform-specific
    /// persistence calls inside MixerApp.cpp.
    using StateChangedFn = std::function<void()>;
    void setStateChangedCallback(StateChangedFn fn) { stateChangedCb_ = std::move(fn); }
    void notifyStateChanged() const { if (stateChangedCb_) stateChangedCb_(); }

    // ── Legacy thread driver (no-op; render is now driven externally) ──────
    void start() {}
    void stop()  {}
    bool isRunning() const { return true; }

    // ── Convenience overloads for legacy enum-based call sites (PC) ───────

    void  setRouteEnabled(MixerInput  in, MixerOutput out, bool enabled)
        { setRouteEnabled(static_cast<ChannelId>(in), static_cast<OutputId>(out), enabled); }
    bool  isRouteEnabled (MixerInput  in, MixerOutput out) const
        { return isRouteEnabled(static_cast<ChannelId>(in), static_cast<OutputId>(out)); }
    void  setRouteVolume (MixerInput  in, MixerOutput out, float vol)
        { setRouteVolume(static_cast<ChannelId>(in), static_cast<OutputId>(out), vol); }
    float getRouteVolume (MixerInput  in, MixerOutput out) const
        { return getRouteVolume(static_cast<ChannelId>(in), static_cast<OutputId>(out)); }

    void  setChannelVolume(MixerInput in, float vol) { setChannelVolume(static_cast<ChannelId>(in), vol); }
    float getChannelVolume(MixerInput in) const     { return getChannelVolume(static_cast<ChannelId>(in)); }
    void  setChannelMute  (MixerInput in, bool m)   { setChannelMute(static_cast<ChannelId>(in), m); }
    bool  isChannelMuted  (MixerInput in) const     { return isChannelMuted(static_cast<ChannelId>(in)); }
    void  setChannelSolo  (MixerInput in, bool s)   { setChannelSolo(static_cast<ChannelId>(in), s); }
    bool  isChannelSoloed (MixerInput in) const     { return isChannelSoloed(static_cast<ChannelId>(in)); }

    void  setOutputVolume(MixerOutput out, float v) { setOutputVolume(static_cast<OutputId>(out), v); }
    float getOutputVolume(MixerOutput out) const    { return getOutputVolume(static_cast<OutputId>(out)); }
    void  setOutputMute  (MixerOutput out, bool m)  { setOutputMute(static_cast<OutputId>(out), m); }
    bool  isOutputMuted  (MixerOutput out) const    { return isOutputMuted(static_cast<OutputId>(out)); }

    void getChannelLevel(MixerInput in, int16_t& l, int16_t& r) const
        { getChannelLevel(static_cast<ChannelId>(in), l, r); }
    void getOutputLevel(MixerOutput out, int16_t& l, int16_t& r) const
        { getOutputLevel(static_cast<OutputId>(out), l, r); }

    void setTapOutput(MixerOutput out) { setTapOutput(static_cast<OutputId>(out)); }

private:
    struct ChannelSlot {
        crosspad::IAudioNode* node = nullptr;
        char name[kMixerNameLen] = {0};

        std::atomic<bool>  active{false};
        std::atomic<float> volume{1.0f};
        std::atomic<bool>  muted{false};
        std::atomic<bool>  soloed{false};
        std::atomic<float> pan{0.0f};
        std::atomic<float> peakL{0.0f};
        std::atomic<float> peakR{0.0f};

        std::atomic<float> routeVol[kMixerMaxOutputs];
        std::atomic<bool>  routeEnabled[kMixerMaxOutputs];

        std::vector<float> scratch;  // sized in setup()
    };

    struct OutputSlot {
        char name[kMixerNameLen] = {0};
        std::atomic<float> volume{1.0f};
        std::atomic<bool>  muted{false};
        std::atomic<bool>  heard{true};
        std::atomic<float> peakL{0.0f};
        std::atomic<float> peakR{0.0f};
    };

    void resetChannelSlot(ChannelSlot& s);

    ChannelSlot channels_[kMixerMaxChannels];
    OutputSlot  outputs_[kMixerMaxOutputs];

    std::atomic<crosspad::AudioRingBuffer<int16_t>*> tapBuffer_{nullptr};
    std::atomic<uint8_t> tapOutput_{0};

    uint32_t frameCount_   = 0;
    uint32_t sampleRate_   = 0;
    uint8_t  numOutputs_   = 2;

    StateChangedFn stateChangedCb_;

    // Mutex protects against concurrent registration / persistence; render
    // path never takes it (uses atomic active flag for visibility).
    mutable std::mutex registrationMutex_;

    std::vector<int16_t> tapBuf_;

    // Per-render scratch used by the esp-dsp render path: holds (channel
    // signal × gain) before being added into the output bus. Sized in
    // setup() so the audio thread never allocates. On non-IDF builds
    // (no esp-dsp) this buffer is unused but kept to keep the layout
    // stable across platforms.
    std::vector<float> mixScratch_;
};

/// Global mixer engine accessor (initialized by platform bootstrap).
AudioMixerEngine& getMixerEngine();
