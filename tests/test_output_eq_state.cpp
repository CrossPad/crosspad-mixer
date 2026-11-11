#include <catch2/catch_test_macros.hpp>

#include "crosspad-mixer/AudioMixerEngine.hpp"

#include <string>

/* An output's EQ through the desk's JSON. stateJson() leaves a flat, off EQ
 * out (a desk that has never had one keeps its blob the size it was), so a
 * desk without the keys has to mean "no EQ" when it is applied over one that
 * has an EQ on: that is what a preset saved flat and the presets' UNDO
 * snapshot of a desk that had none carry. */

namespace {

constexpr uint8_t kOuts = 3;

crosspad::EqSettings phonesLowPlus6() {
    crosspad::EqSettings eq;
    eq.on = true;
    eq.band[0].kind   = crosspad::EqKind::LowShelf;
    eq.band[0].hz     = 120.0f;
    eq.band[0].gainDb = 6.0f;
    return eq;
}

} // namespace

TEST_CASE("a desk with no EQ, applied over one with an EQ on, takes it away", "[desk][eq]") {
    AudioMixerEngine m;
    m.setup(64, 44100, kOuts);

    /* MixerPresets::load(): the UNDO snapshot of a desk that never had an EQ. */
    const std::string undo = m.stateJson();
    CHECK(undo.find("\"eq\"") == std::string::npos);

    /* The preset that was loaded: Phones (output 1) Low +6. */
    m.setOutputEq(1, phonesLowPlus6());
    REQUIRE(m.isOutputEqActive(1));

    /* MixerPresets::undo(). */
    REQUIRE(m.applyStateJson(undo));
    CHECK_FALSE(m.isOutputEqActive(1));
    CHECK(m.getOutputEq(1) == crosspad::EqSettings{});
    CHECK(m.stateJson() == undo);
}

TEST_CASE("a desk with an EQ switched off but set keeps the bands, off", "[desk][eq]") {
    AudioMixerEngine m;
    m.setup(64, 44100, kOuts);

    crosspad::EqSettings off = phonesLowPlus6();
    off.on = false;
    m.setOutputEq(0, off);
    const std::string saved = m.stateJson();

    m.setOutputEq(0, phonesLowPlus6());
    REQUIRE(m.isOutputEqActive(0));
    REQUIRE(m.applyStateJson(saved));
    CHECK_FALSE(m.isOutputEqActive(0));
    CHECK(m.getOutputEq(0) == off);
}

TEST_CASE("a desk with an EQ on still puts it on", "[desk][eq]") {
    AudioMixerEngine m;
    m.setup(64, 44100, kOuts);
    m.setOutputEq(1, phonesLowPlus6());
    const std::string saved = m.stateJson();

    AudioMixerEngine fresh;
    fresh.setup(64, 44100, kOuts);
    REQUIRE(fresh.applyStateJson(saved));
    CHECK(fresh.isOutputEqActive(1));
    CHECK(fresh.getOutputEq(1) == phonesLowPlus6());
}
