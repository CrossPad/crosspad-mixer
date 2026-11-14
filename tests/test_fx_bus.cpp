#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "crosspad-mixer/AudioMixerEngine.hpp"
#include <crosspad/audio/IAudioNode.hpp>

#include <vector>

namespace {

constexpr uint32_t kFrames = 32;
constexpr uint8_t  kOuts   = 2;

/* @p level on both sides, every frame, while it sounds. */
class Level : public crosspad::IAudioNode {
public:
    explicit Level(float v) : v_(v) {}
    void mix(float* out, uint32_t frames) override {
        for (uint32_t i = 0; i < frames * 2; ++i) out[i] += v_;
    }
    bool isGenerator() const override { return true; }
    const char* name() const override { return "level"; }
private:
    float v_;
};

/* Doubles what it is given, and adds 0.01 so a block with no input still
 * shows it ran; counts its calls. */
class Doubler : public crosspad::IAudioNode {
public:
    unsigned calls = 0;
    uint32_t preparedFrames = 0;
    void process(const float* in, float* out, uint32_t frames) override {
        ++calls;
        for (uint32_t i = 0; i < frames * 2; ++i) out[i] = in[i] * 2.0f + 0.01f;
    }
    void onPrepare(uint32_t maxFrames, uint32_t) override { preparedFrames = maxFrames; }
    const char* name() const override { return "doubler"; }
};

struct Rig {
    AudioMixerEngine eng;
    std::vector<float> bus[kOuts];
    float* ptrs[kOuts];
    Rig() {
        eng.setup(kFrames, 44100, kOuts);
        for (uint8_t o = 0; o < kOuts; ++o) {
            bus[o].assign(kFrames * 2, 0.0f);
            ptrs[o] = bus[o].data();
        }
    }
    void render() { eng.render(ptrs, kOuts, kFrames); }
};

using Catch::Matchers::WithinAbs;

} // namespace

TEST_CASE("Mixer FX bus: sent, processed and returned in the same pass", "[mixer][fx]") {
    Rig r;
    Level src(0.1f);
    Doubler fx;
    const auto ch = r.eng.addChannel(&src, "in");
    r.eng.setChannelVolume(ch, 0.5f);
    r.eng.setFxProcessor(&fx);
    CHECK(fx.preparedFrames == kFrames);
    r.eng.setFxSend(ch, 1.0f);
    r.eng.setFxReturn(1, 1.0f);

    SECTION("post-fader: the send carries the fader, the return only its output") {
        r.render();
        CHECK_THAT(r.bus[1][0], WithinAbs(0.1f * 0.5f * 2.0f + 0.01f, 1e-6));
        CHECK_THAT(r.bus[0][0], WithinAbs(0.0f, 1e-6));   // no route, no return
    }
    SECTION("pre-fader: the fader is not in the send") {
        r.eng.setFxPreFader(true);
        r.render();
        CHECK_THAT(r.bus[1][0], WithinAbs(0.1f * 2.0f + 0.01f, 1e-6));
    }
    SECTION("a muted channel sends nothing, and the processor still runs") {
        r.eng.setChannelMute(ch, true);
        r.render();
        CHECK(fx.calls == 1);
        CHECK_THAT(r.bus[1][0], WithinAbs(0.01f, 1e-6));
    }
    SECTION("taken away, nothing comes back") {
        r.eng.setFxProcessor(nullptr);
        r.render();
        CHECK_THAT(r.bus[1][0], WithinAbs(0.0f, 1e-6));
    }
}
