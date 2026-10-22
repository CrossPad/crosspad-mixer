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
#include <crosspad/audio/OutputEq.hpp>
#include <crosspad/dsp/AlignedVector.hpp>

namespace crosspad { class IAudioNode; }

// ── Limits ─────────────────────────────────────────────────────────────────
static constexpr uint8_t kMixerMaxChannels = 16;
static constexpr uint8_t kMixerMaxOutputs  = 8;
/// Outputs that can carry an EQ: the first two, which on every board so far
/// are the speakers and the headphones.
static constexpr uint8_t kMixerEqOutputs   = 2;
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

    // ── Output EQ ──────────────────────────────────────────────────────────
    //
    // Four bands across an output, for evening out what it plays through: a
    // speaker this size has a response, and so does a headphone amp, and the
    // two are not each other's. It sits where a console's monitor EQ sits --
    // after the mix, before that output's own fader -- so a band that lifts
    // something leaves the fader to take it back, and nothing downstream of
    // the correction can undo it.
    //
    // Saved with the desk (stateJson), so a board keeps its correction.

    /// Not from the render thread: the coefficients are worked out here.
    void setOutputEq(OutputId out, const crosspad::EqSettings& eq);
    crosspad::EqSettings getOutputEq(OutputId out) const;
    /// Whether that output's EQ is doing anything at all.
    bool isOutputEqActive(OutputId out) const;

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
    /// A second gain on the channel, beside its fader, applied wherever the
    /// fader is (the buses and the tap) and never saved with the desk: what
    /// another part of the instrument puts on it for a while -- an Arrange
    /// seq track's fader on the engine its notes play through, while the
    /// song plays. 1 is none.
    void  setChannelTrim  (ChannelId ch, float gain);
    float getChannelTrim  (ChannelId ch) const;
    bool  isAnySoloed     () const;

    // ── Output bus control ─────────────────────────────────────────────────

    void  setOutputVolume(OutputId out, float vol);
    float getOutputVolume(OutputId out) const;
    void  setOutputMute  (OutputId out, bool muted);
    bool  isOutputMuted  (OutputId out) const;
    /// The highest peak the limiter lets out of @p out (0..kLimitCeiling):
    /// lower where what follows the output -- a codec's analog stage turned
    /// up past 0 dB -- cannot carry a full-scale peak.
    static constexpr float kLimitCeiling = 0.98f;
    void  setOutputCeiling(OutputId out, float ceiling);
    float getOutputCeiling(OutputId out) const;
    /// Hold @p out back one block, so its limiter sees a peak a block before
    /// it leaves and brings the gain down across that whole block instead of
    /// in the frames just before it -- which on a sustained sound is a step,
    /// heard as a click. Costs the output one block of latency, so it is for
    /// a bus that is only recorded (a USB recording bus), never one a player
    /// hears. Not from the render thread.
    void  setOutputLookahead(OutputId out, bool on);
    /// An output whose level the hardware sets after the DAC (IOutputLevels):
    /// played at unity and never muted here, whatever a desk, a preset or a
    /// command asks -- setOutputVolume(), setOutputMute() and a loaded state
    /// leave it be.
    void  setOutputUnity (OutputId out, bool unity);
    bool  isOutputUnity  (OutputId out) const;

    // ── Metering (float [0.0, 1.0]) ────────────────────────────────────────

    void getChannelLevel(ChannelId ch, float& left, float& right) const;
    void getOutputLevel (OutputId out, float& left, float& right) const;

    // Legacy int16 overloads for existing UI code (rescales internally)
    void getChannelLevel(ChannelId ch, int16_t& left, int16_t& right) const;
    void getOutputLevel (OutputId out, int16_t& left, int16_t& right) const;

    /// The limiter's deepest gain on @p out since the last read (1.0 = it
    /// never acted), and resets it. kTapLimit reads the pads tap's.
    static constexpr OutputId kTapLimit = 0xFF;
    float takeOutputLimit(OutputId out);

    // ── Render ─────────────────────────────────────────────────────────────

    /// Render `frames` of stereo audio into each of `numOut` output buses.
    /// `outBuses[i]` must point to a buffer of `frames * 2` floats. Audio
    /// thread only — single writer.
    void render(float* const* outBuses, uint8_t numOut, uint32_t frames);

    /* ── Where a render pass goes, per channel ────────────────────────────
     *
     * `TASKS` said audio_rt costs 42 % of core 1 at the launcher with no kit
     * loaded, and RT_PROF narrowed that to 1040 us a block inside this
     * render -- for 64 frames, three buses and nothing playing. Which part
     * of it was not answerable without timing the pass from inside, so:
     * every channel's node, and the four stages around them.
     *
     * The clock comes from the platform (one cycle-counter read on the
     * board, nothing on a host) and the whole thing is off until asked for,
     * so a render that is not being profiled pays one relaxed bool load. */
    using CycFn = uint32_t (*)();
    void setProfClock(CycFn fn) { cyc_ = fn; }
    void profEnable(bool on);
    /// Mean microseconds per pass, per channel and per stage. Reads, clears.
    /// @p cpuMhz the clock the counter runs at.
    void profRead(char* out, uint32_t cap, float cpuMhz);
    /// RT, right after render(): the channels in @p chs as they went into the
    /// mix -- panned, at their fader, silent when muted or left out by a solo
    /// -- summed into @p out, whatever they are sent to. A tap the routing
    /// must not decide, such as the pads alone for a bounce. Limited as an
    /// output is: a bounce of a dense part must not be a clipped file.
    void renderTap(const ChannelId* chs, uint8_t n, float* out, uint32_t frames);

    // ── Tap (CI/test capture) ──────────────────────────────────────────────

    void setTapBuffer(crosspad::AudioRingBuffer<int16_t>* buf);
    void setTapOutput(OutputId out);

    // ── Persistence ────────────────────────────────────────────────────────

    void saveState(const std::string& path) const;
    void loadState(const std::string& path);

    /// The same state as a JSON string, for a platform that keeps it
    /// somewhere other than a file (NVS on the board). Desk format
    /// kDeskVersion: every channel carries its name, and the name -- not the
    /// slot -- is what it is loaded back onto.
    std::string stateJson() const;
    /// Apply a string from stateJson(), of this version or an older one.
    /// Each saved channel goes to the live channel of the same name wherever
    /// it now sits, so a build that inserts a channel (the native sampler's
    /// "Sampler2", 2026-10) or drops one loads an older desk onto the right
    /// channels instead of shifting them. False when it does not parse, in
    /// which case nothing is changed.
    bool applyStateJson(const std::string& json);
    /// 4: channels matched by name (desks up to 3 were matched by slot, with
    /// the name only a guard, and lost every channel a new one had moved).
    /// 5: a desk older than this onto a build with one sample engine gets
    /// that engine rescued once if it would load silent -- muted, at zero,
    /// or sent only to outputs nobody hears -- because 4 had already put the
    /// unused streamer's settings on it and saved them (2026-10-09).
    static constexpr int kDeskVersion = 5;
    /// The last applyStateJson() loaded a desk older than kDeskVersion: the
    /// platform saves it back once, so a rescue is not repeated on every boot
    /// over what the player does next.
    bool deskMigrated() const { return deskMigrated_; }

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
        std::atomic<float> trim{1.0f};
        std::atomic<bool>  muted{false};
        std::atomic<bool>  soloed{false};
        std::atomic<float> pan{0.0f};
        std::atomic<float> peakL{0.0f};
        std::atomic<float> peakR{0.0f};

        std::atomic<float> routeVol[kMixerMaxOutputs];
        std::atomic<bool>  routeEnabled[kMixerMaxOutputs];

        /* Sized in setup(); on 16 bytes for the S3's four-float loads. */
        crosspad::dsp::AlignedVector<float> scratch;
    };

    struct OutputSlot {
        char name[kMixerNameLen] = {0};
        std::atomic<float> volume{1.0f};
        std::atomic<bool>  muted{false};
        std::atomic<bool>  heard{true};
        std::atomic<bool>  unity{false};
        std::atomic<float> peakL{0.0f};
        std::atomic<float> peakR{0.0f};
        std::atomic<float> ceiling{kLimitCeiling};
        float              limGain = 1.0f;   // render thread only
        std::atomic<float> limMin{1.0f};     // deepest limGain since the last read
        uint16_t           limHold = 0;      // blocks before it may let go; render thread only
        std::atomic<bool>  lookahead{false}; // published after `held` is sized
        std::vector<float> held;             // the block held back, stereo
        float              heldPeak = 0.0f;  // render thread only
    };

    /* The last stage before a bus leaves the mixer, in place of a hard clip
     * at full scale. Render thread only. @p pL / @p pR: each side's peak as
     * the block came in. False when the block left untouched -- then those
     * are its peaks going out too. */
    bool limit(float* buf, uint32_t frames, float ceiling, float& gain, uint16_t& hold,
               std::atomic<float>& deepest, float& pL, float& pR);
    /* limit() one block late, for an output with lookahead on. */
    void limitAhead(OutputSlot& o, float* buf, uint32_t frames, float ceiling);

    void resetChannelSlot(ChannelSlot& s);

    /// The live slot a saved channel's settings belong to, or -1. Under
    /// registrationMutex_. See applyStateJson(): by name, the saved id only
    /// a hint; @p taken holds the slots an earlier entry of the same desk
    /// already went to.
    int slotForSaved(int id, const char* name, const bool* taken) const;

    bool deskMigrated_ = false;   ///< see deskMigrated(); under registrationMutex_

    ChannelSlot channels_[kMixerMaxChannels];
    OutputSlot  outputs_[kMixerMaxOutputs];
    /* One per output that can be EQ'd -- the ones a board plays through, not
     * all eight slots. Internal RAM on this hardware is counted in hundreds
     * of bytes at boot (see EqChain), and an EQ across a recording bus is
     * not a thing anybody asked for. */
    crosspad::EqChain outputEq_[kMixerEqOutputs];

    std::atomic<crosspad::AudioRingBuffer<int16_t>*> tapBuffer_{nullptr};
    std::atomic<uint8_t> tapOutput_{0};

    uint32_t frameCount_   = 0;
    uint32_t sampleRate_   = 0;
    uint8_t  numOutputs_   = 2;

    float              limRelease_ = 0.0f;   // per-block step back to unity, from setup()
    uint16_t           limHoldBlocks_ = 0;   // kLimHoldS in blocks, from setup()
    float              tapLimGain_ = 1.0f;   // render thread only
    uint16_t           tapLimHold_ = 0;      // render thread only
    std::atomic<float> tapLimMin_{1.0f};

    CycFn                 cyc_ = nullptr;
    std::atomic<bool>     prof_{false};
    std::atomic<uint32_t> profPasses_{0};
    /* Sums in units of 1 << kProfShift cycles, 32-bit: render() adds to them
     * every pass, and a 64-bit atomic is not lock-free on the S3. A whole
     * 1.45 ms pass is ~21 800 units at 240 MHz, so a sum wraps only after
     * ~3 minutes unread; profRead() reads and clears. */
    static constexpr uint32_t kProfShift = 4;
    static uint32_t profUnits(uint32_t cycles) { return (cycles + (1u << (kProfShift - 1))) >> kProfShift; }
    std::atomic<uint32_t> profNode_[kMixerMaxChannels] = {};
    std::atomic<uint32_t> profNodeMax_[kMixerMaxChannels] = {};   // the longest pass of each, same units
    std::atomic<uint32_t> profPeak_{0}, profClear_{0}, profRoute_{0}, profOut_{0};

    StateChangedFn stateChangedCb_;

    // Mutex protects against concurrent registration / persistence; render
    // path never takes it (uses atomic active flag for visibility).
    mutable std::mutex registrationMutex_;

    std::vector<int16_t> tapBuf_;
};

/// Global mixer engine accessor (initialized by platform bootstrap).
AudioMixerEngine& getMixerEngine();
