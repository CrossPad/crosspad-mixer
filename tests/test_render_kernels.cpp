#include <catch2/catch_test_macros.hpp>

#include "crosspad-mixer/AudioMixerEngine.hpp"
#include <crosspad/audio/IAudioNode.hpp>
#include <crosspad/dsp/AlignedVector.hpp>
#include <crosspad/dsp/AudioKernels.hpp>

#include <cstring>
#include <vector>

/* render() through core's dsp kernels against render() through the loops they
 * replaced (dsp KernelMode::Reference): the same buses, the same meters, the
 * same limiter, block after block -- on the host, where nothing fuses a
 * multiply-add, bit for bit. */

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

Run play(crosspad::dsp::KernelMode mode, bool alignedBuses)
{
    const crosspad::dsp::KernelMode was = crosspad::dsp::kernelMode();
    crosspad::dsp::setKernelMode(mode);

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
    crosspad::dsp::setKernelMode(was);
    return r;
}

bool sameBits(const std::vector<float>& a, const std::vector<float>& b)
{
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
}

} // namespace

TEST_CASE("Mixer render: the dsp kernels sound as the loops they replaced", "[mixer][render][kernels]")
{
    const Run ref = play(crosspad::dsp::KernelMode::Reference, true);
    for (bool aligned : {true, false}) {
        INFO("buses on 16 bytes: " << aligned);
        const Run fast = play(crosspad::dsp::KernelMode::Portable, aligned);
        CHECK(sameBits(ref.buses, fast.buses));
        CHECK(sameBits(ref.taps, fast.taps));
        CHECK(sameBits(ref.meters, fast.meters));
        CHECK(sameBits(ref.limits, fast.limits));
    }

    /* The script did what it is there for: the limiter acted, and the
     * meters saw sound. */
    bool limited = false, sounded = false;
    for (float g : ref.limits) limited = limited || g < 1.0f;
    for (float m : ref.meters) sounded = sounded || m > 0.3f;
    CHECK(limited);
    CHECK(sounded);
}
