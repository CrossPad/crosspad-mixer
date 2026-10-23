#include <catch2/catch_test_macros.hpp>

#include "crosspad-mixer/AudioMixerEngine.hpp"
#include <crosspad/audio/IAudioNode.hpp>
#include <crosspad/dsp/AlignedVector.hpp>
#include <crosspad/dsp/AudioKernels.hpp>

#include <cstring>
#include <vector>

/* render() through core's dsp kernels against render() through the loops they
 * replaced: the same buses, the same meters, the same limiter, block after
 * block -- on the host, where nothing fuses a multiply-add, bit for bit. Those
 * loops left core once the board had chosen; what they rendered is kept as
 * the digest of this script's output, recorded through them (16.10.2026). */

namespace {

constexpr uint32_t kFrames = 64;
constexpr uint32_t kStereo = kFrames * 2;
constexpr uint8_t  kBuses  = 3;

/* Noise at a level that changes block to block, so the limiter comes in,
 * holds and lets go, and some blocks are over full scale before it. */
class NoiseNode : public crosspad::IAudioNode {
public:
    explicit NoiseNode(uint32_t seed, float level) : seed_(seed), level_(level) {}
    void mix(float* out, uint32_t frames) override {
        ++block_;
        const float swell = (block_ % 9 < 3) ? 2.2f : 0.4f;
        for (uint32_t i = 0; i < frames * 2; ++i) {
            seed_ = seed_ * 1664525u + 1013904223u;
            out[i] += (static_cast<float>(seed_ >> 8) / 16777216.0f - 0.5f) * level_ * swell;
        }
    }
    bool isGenerator() const override { return true; }
    const char* name() const override { return "noise"; }

private:
    uint32_t seed_;
    float level_;
    uint32_t block_ = 0;
};

struct Run {
    std::vector<float> buses;     // every bus of every block
    std::vector<float> taps;      // renderTap() of every block
    std::vector<float> meters;    // channel and output peaks
    std::vector<float> limits;    // each output's and the tap's deepest limiter gain
};

Run play(bool alignedBuses)
{
    NoiseNode n0(1, 0.9f), n1(2, 0.6f), n2(3, 1.1f), n3(4, 0.3f);
    AudioMixerEngine eng;
    eng.setup(kFrames, 44100, kBuses);
    AudioMixerEngine::ChannelId ch[4];
    NoiseNode* nodes[4] = {&n0, &n1, &n2, &n3};
    const float pan[4] = {-0.4f, 0.0f, 0.7f, 1.0f};
    const float send[4][kBuses] = {{1.0f, 0.5f, 0.0f}, {0.8f, 1.0f, 1.0f}, {0.0f, 0.9f, 0.6f}, {1.0f, 0.0f, 1.0f}};
    for (int c = 0; c < 4; ++c) {
        ch[c] = eng.addChannel(nodes[c], "n");
        eng.setChannelVolume(ch[c], 0.9f - 0.1f * c);
        eng.setChannelPan(ch[c], pan[c]);
        for (uint8_t o = 0; o < kBuses; ++o) {
            if (send[c][o] == 0.0f) continue;
            eng.setRouteEnabled(ch[c], o, true);
            eng.setRouteVolume(ch[c], o, send[c][o]);
        }
    }
    eng.setOutputVolume(1, 0.7f);
    eng.setOutputCeiling(2, 0.5f);
    crosspad::EqSettings eq;
    eq.on = true;
    eq.band[0] = {crosspad::EqKind::HighPass, 40.0f, 0.0f, 0.7f};
    eq.band[1] = {crosspad::EqKind::Peak, 2500.0f, 5.0f, 1.2f};
    eq.band[2] = {crosspad::EqKind::LowShelf, 120.0f, -3.0f, 0.7f};
    eq.band[3] = {crosspad::EqKind::LowPass, 15000.0f, 0.0f, 0.7f};
    eng.setOutputEq(0, eq);
    eng.setOutputEq(1, eq);

    /* Bus storage both on 16 bytes and off it: the PIE kernels take the
     * first, the portable ones the second, and both must sound the same. */
    crosspad::dsp::AlignedVector<float> store(kBuses * kStereo + 4);
    float* base = store.data() + (alignedBuses ? 0 : 1);
    float* ptrs[kBuses];
    for (uint8_t o = 0; o < kBuses; ++o) ptrs[o] = base + o * kStereo;
    std::vector<float> tap(kStereo);
    const AudioMixerEngine::ChannelId tapped[2] = {ch[0], ch[2]};

    Run r;
    for (int b = 0; b < 40; ++b) {
        if (b == 20) eng.setOutputMute(1, true);
        if (b == 25) eng.setOutputMute(1, false);
        eng.render(ptrs, kBuses, kFrames);
        eng.renderTap(tapped, 2, tap.data(), kFrames);
        r.buses.insert(r.buses.end(), base, base + kBuses * kStereo);
        r.taps.insert(r.taps.end(), tap.begin(), tap.end());
        for (int c = 0; c < 4; ++c) {
            float l = 0, rr = 0;
            eng.getChannelLevel(ch[c], l, rr);
            r.meters.push_back(l);
            r.meters.push_back(rr);
        }
        for (uint8_t o = 0; o < kBuses; ++o) {
            float l = 0, rr = 0;
            eng.getOutputLevel(o, l, rr);
            r.meters.push_back(l);
            r.meters.push_back(rr);
            r.limits.push_back(eng.takeOutputLimit(o));
        }
        r.limits.push_back(eng.takeOutputLimit(AudioMixerEngine::kTapLimit));
    }
    return r;
}

/* FNV-1a over the bits of every sample: what a recorded run is compared by. */
uint64_t digest(const std::vector<float>& v)
{
    uint64_t h = 1469598103934665603ull;
    for (float f : v) {
        uint32_t b;
        std::memcpy(&b, &f, sizeof b);
        for (int i = 0; i < 4; ++i) {
            h ^= (b >> (i * 8)) & 0xFFu;
            h *= 1099511628211ull;
        }
    }
    return h;
}

} // namespace

TEST_CASE("Mixer render: the dsp kernels sound as the loops they replaced", "[mixer][render][kernels]")
{
    Run ref;
    for (bool aligned : {true, false}) {
        INFO("buses on 16 bytes: " << aligned);
        const Run fast = play(aligned);
        REQUIRE(fast.buses.size() == 15360);
        REQUIRE(fast.taps.size() == 5120);
        REQUIRE(fast.meters.size() == 560);
        REQUIRE(fast.limits.size() == 160);
        CHECK(digest(fast.buses) == 0x940e866f98c7286aull);
        CHECK(digest(fast.taps) == 0x2117eecc9018e6a5ull);
        CHECK(digest(fast.meters) == 0xccbfd4b672fcf20eull);
        CHECK(digest(fast.limits) == 0x874d77dbcd13a5f9ull);
        ref = fast;
    }

    /* The script did what it is there for: the limiter acted, and the
     * meters saw sound. */
    bool limited = false, sounded = false;
    for (float g : ref.limits) limited = limited || g < 1.0f;
    for (float m : ref.meters) sounded = sounded || m > 0.3f;
    CHECK(limited);
    CHECK(sounded);
}

TEST_CASE("renderOnly() leaves the tapped channels as render() does: renderTap() reads the same bits",
          "[mixer][offline]")
{
    /* Two engines with their own pan and fader, one channel left out: the
     * tap of an offline pass must be the live pass's tap, block for block. */
    auto run = [](bool offline) {
        NoiseNode n0(7, 0.8f), n1(8, 0.5f), n2(9, 1.2f);
        AudioMixerEngine eng;
        eng.setup(kFrames, 44100, kBuses);
        const AudioMixerEngine::ChannelId a = eng.addChannel(&n0, "a");
        const AudioMixerEngine::ChannelId b = eng.addChannel(&n1, "b");
        const AudioMixerEngine::ChannelId c = eng.addChannel(&n2, "c");
        eng.setChannelPan(a, -0.6f);
        eng.setChannelPan(c, 0.3f);
        eng.setChannelVolume(c, 0.7f);
        for (auto ch : {a, b, c}) eng.setRouteEnabled(ch, 0, true);
        crosspad::dsp::AlignedVector<float> store(kBuses * kStereo);
        float* ptrs[kBuses];
        for (uint8_t o = 0; o < kBuses; ++o) ptrs[o] = store.data() + o * kStereo;
        const AudioMixerEngine::ChannelId tapped[2] = {a, c};
        std::vector<float> tap(kStereo), all;
        for (int blk = 0; blk < 30; ++blk) {
            if (offline) eng.renderOnly(tapped, 2, kFrames);
            else         eng.render(ptrs, kBuses, kFrames);
            eng.renderTap(tapped, 2, tap.data(), kFrames);
            all.insert(all.end(), tap.begin(), tap.end());
        }
        return all;
    };
    const std::vector<float> live = run(false), off = run(true);
    REQUIRE(live.size() == off.size());
    CHECK(std::memcmp(live.data(), off.data(), live.size() * sizeof(float)) == 0);
}
