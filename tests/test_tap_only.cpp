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

using Catch::Matchers::WithinAbs;

} // namespace

TEST_CASE("Mixer tap only: on the tap, on no output", "[mixer][tap]") {
    AudioMixerEngine eng;
    ConstNode node;
    std::vector<float> bus(kFrames * 2, 0.0f), tap(kFrames * 2, 0.0f);
    float* ptr = bus.data();
    eng.setup(kFrames, 44100, 1);
    const auto ch = eng.addChannel(&node, "ch");
    eng.setRouteEnabled(ch, 0, true);
    eng.setRouteVolume(ch, 0, 1.0f);

    eng.setChannelTapOnly(ch, true);
    CHECK(eng.isChannelTapOnly(ch));
    eng.render(&ptr, 1, kFrames);
    eng.renderTap(&ch, 1, tap.data(), kFrames);
    CHECK(bus[0] == 0.0f);
    CHECK(bus[kFrames * 2 - 1] == 0.0f);
    CHECK_THAT(tap[0], WithinAbs(0.1f, 1e-6f));
    float l = 0.0f, r = 0.0f;
    eng.getChannelLevel(ch, l, r);
    CHECK_THAT(l, WithinAbs(0.1f, 1e-6f));                // metered as it plays

    eng.setChannelTapOnly(ch, false);
    eng.render(&ptr, 1, kFrames);
    CHECK_THAT(bus[0], WithinAbs(0.1f, 1e-6f));
}
