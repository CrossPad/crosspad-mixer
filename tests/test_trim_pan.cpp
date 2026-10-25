#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "crosspad-mixer/AudioMixerEngine.hpp"
#include <crosspad/audio/IAudioNode.hpp>

#include <vector>

namespace {

constexpr uint32_t kFrames = 64;

/* 0.1 on both sides, every frame. */
class ConstNode : public crosspad::IAudioNode {
public:
    void mix(float* out, uint32_t frames) override {
        for (uint32_t i = 0; i < frames * 2; ++i) out[i] += 0.1f;
    }
    bool isGenerator() const override { return true; }
    const char* name() const override { return "const"; }
};

struct One {
    AudioMixerEngine eng;
    ConstNode node;
    std::vector<float> bus = std::vector<float>(kFrames * 2, 0.0f);
    float* ptr = bus.data();
    AudioMixerEngine::ChannelId ch;

    One() {
        eng.setup(kFrames, 44100, 1);
        ch = eng.addChannel(&node, "ch");
        eng.setRouteEnabled(ch, 0, true);
        eng.setRouteVolume(ch, 0, 1.0f);
    }
    void render() { eng.render(&ptr, 1, kFrames); }
};

using Catch::Matchers::WithinAbs;

} // namespace

TEST_CASE("Mixer trim pan: a balance on top of the channel's own pan", "[mixer][pan]") {
    One one;
    SECTION("hard right silences the left") {
        one.eng.setChannelTrimPan(one.ch, 1.0f);
        one.render();
        CHECK_THAT(one.bus[0], WithinAbs(0.0f, 1e-6));
        CHECK_THAT(one.bus[1], WithinAbs(0.1f, 1e-6));
    }
    SECTION("the two multiply: pan left half, trim right half") {
        one.eng.setChannelPan(one.ch, -0.5f);
        one.eng.setChannelTrimPan(one.ch, 0.5f);
        one.render();
        CHECK_THAT(one.bus[0], WithinAbs(0.05f, 1e-6));
        CHECK_THAT(one.bus[1], WithinAbs(0.05f, 1e-6));
    }
    SECTION("clamped to -1..1, centre by default") {
        CHECK(one.eng.getChannelTrimPan(one.ch) == 0.0f);
        one.eng.setChannelTrimPan(one.ch, -3.0f);
        CHECK(one.eng.getChannelTrimPan(one.ch) == -1.0f);
    }
}
