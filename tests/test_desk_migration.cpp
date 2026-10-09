#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "crosspad-mixer/AudioMixerEngine.hpp"
#include <crosspad/audio/IAudioNode.hpp>

#include <string>
#include <vector>

/* A desk saved by one build loaded by another whose channels sit in other
 * slots. The case that made this matter: the native sampler engine's
 * "Sampler2" channel went in after "Pitched", so on a build with it USB,
 * Click and the inputs are each one slot later than on main / wifi-remote,
 * and a build without the prebuilt engine has one "Sampler" (the native
 * engine) and no "Sampler2". */

namespace {

using Catch::Matchers::WithinAbs;
/* A float through the desk's JSON comes back to within its printed digits. */
constexpr double kJsonEps = 1e-6;

class SilentNode : public crosspad::IAudioNode {
public:
    void mix(float*, uint32_t) override {}
    bool isGenerator() const override { return true; }
    const char* name() const override { return "silent"; }
};

constexpr uint8_t kOuts = 3;

/* main and feat/wifi-remote: CrosspadPlatform.cpp's order before the native
 * engine. */
const std::vector<const char*> kMainLayout = {"Sampler", "Pitched", "USB", "Click",
                                              "IN1", "IN2", "IN3"};
/* integrate/native-engine-wifi with the prebuilt engine built in. */
const std::vector<const char*> kTwoEngineLayout = {"Sampler", "Pitched", "Sampler2", "USB",
                                                   "Click", "IN1", "IN2", "IN3"};

struct Desk {
    AudioMixerEngine m;
    std::vector<SilentNode> nodes;

    explicit Desk(const std::vector<const char*>& layout) : nodes(layout.size()) {
        m.setup(64, 44100, kOuts);
        for (size_t i = 0; i < layout.size(); ++i) {
            REQUIRE(m.addChannel(&nodes[i], layout[i]) == static_cast<AudioMixerEngine::ChannelId>(i));
        }
    }

    int slot(const char* name) const {
        for (int ch = 0; ch < kMixerMaxChannels; ++ch) {
            if (std::string(m.getChannelName(ch)) == name) return ch;
        }
        return -1;
    }
};

/* Something distinct per channel, so a channel that received another's
 * settings is caught: volume, pan, mute and one send all differ by name. */
void dress(Desk& d, const char* name, float vol, float pan, bool muted, bool sendRec) {
    const int ch = d.slot(name);
    REQUIRE(ch >= 0);
    d.m.setChannelVolume(ch, vol);
    d.m.setChannelPan(ch, pan);
    d.m.setChannelMute(ch, muted);
    d.m.setRouteEnabled(ch, 2, sendRec);
    d.m.setRouteVolume(ch, 2, vol * 0.5f);
}

void dressAll(Desk& d) {
    dress(d, "Sampler", 0.90f, -0.10f, false, true);
    dress(d, "Pitched", 0.80f,  0.20f, true,  false);
    dress(d, "USB",     0.30f,  0.00f, false, false);
    dress(d, "Click",   0.25f,  0.50f, true,  false);
    dress(d, "IN1",     0.60f, -0.70f, false, true);
    dress(d, "IN2",     0.55f,  0.70f, true,  true);
    dress(d, "IN3",     0.45f,  0.30f, false, false);
}

void checkDressed(const Desk& d, const char* name, float vol, float pan, bool muted, bool sendRec) {
    INFO("channel " << name);
    const int ch = d.slot(name);
    REQUIRE(ch >= 0);
    CHECK_THAT(d.m.getChannelVolume(ch), WithinAbs(vol, kJsonEps));
    CHECK_THAT(d.m.getChannelPan(ch), WithinAbs(pan, kJsonEps));
    CHECK(d.m.isChannelMuted(ch) == muted);
    CHECK(d.m.isRouteEnabled(ch, 2) == sendRec);
    CHECK_THAT(d.m.getRouteVolume(ch, 2), WithinAbs(vol * 0.5f, kJsonEps));
}

void checkAllDressed(const Desk& d) {
    checkDressed(d, "Sampler", 0.90f, -0.10f, false, true);
    checkDressed(d, "Pitched", 0.80f,  0.20f, true,  false);
    checkDressed(d, "USB",     0.30f,  0.00f, false, false);
    checkDressed(d, "Click",   0.25f,  0.50f, true,  false);
    checkDressed(d, "IN1",     0.60f, -0.70f, false, true);
    checkDressed(d, "IN2",     0.55f,  0.70f, true,  true);
    checkDressed(d, "IN3",     0.45f,  0.30f, false, false);
}

/* What the board's firmware wrote before this change: version 3, each
 * channel with its slot in *that* build. */
std::string asVersion3(std::string json) {
    const std::string v4 = "\"version\":4";
    const auto at = json.find(v4);
    REQUIRE(at != std::string::npos);
    json.replace(at, v4.size(), "\"version\":3");
    return json;
}

}  // namespace

TEST_CASE("the desk is saved as version 4", "[desk]") {
    Desk d(kMainLayout);
    CHECK(d.m.stateJson().find("\"version\":4") != std::string::npos);
    CHECK(AudioMixerEngine::kDeskVersion == 4);
}

TEST_CASE("a desk saved by main (v3, no Sampler2) loads onto the right channels of the two-engine build", "[desk]") {
    Desk old(kMainLayout);
    dressAll(old);
    const std::string saved = asVersion3(old.m.stateJson());

    Desk now(kTwoEngineLayout);
    REQUIRE(now.m.applyStateJson(saved));
    checkAllDressed(now);
    /* The channel the old desk did not have keeps its defaults. */
    const int s2 = now.slot("Sampler2");
    CHECK(now.m.getChannelVolume(s2) == 1.0f);
    CHECK_FALSE(now.m.isChannelMuted(s2));
}

TEST_CASE("before the migration, by slot, the shifted channels lost their settings", "[desk]") {
    /* The guard that rejected a name mismatch kept the wrong settings off the
     * wrong channel -- and lost them. Pinned so the regression is explicit:
     * USB's saved slot (2) is Sampler2 in the new layout. */
    Desk old(kMainLayout);
    dressAll(old);
    Desk now(kTwoEngineLayout);
    REQUIRE(now.m.applyStateJson(asVersion3(old.m.stateJson())));
    CHECK(now.slot("USB") == 3);
    CHECK_THAT(now.m.getChannelVolume(now.slot("USB")), WithinAbs(0.30f, kJsonEps));  // 1.0 before
    CHECK_THAT(now.m.getChannelVolume(now.slot("IN3")), WithinAbs(0.45f, kJsonEps));  // 1.0 before
}

TEST_CASE("a desk saved by the two-engine build loads onto the one-engine build", "[desk]") {
    /* Steve's builds played the native engine on "Sampler2"; "Sampler" was
     * the prebuilt streamer's. The one-engine image's "Sampler" *is* the
     * native engine, so it takes Sampler2's settings, and the streamer's are
     * dropped -- not the other way round. */
    Desk old(kTwoEngineLayout);
    dressAll(old);
    dress(old, "Sampler",  0.20f, 0.40f, true,  false);   // the streamer, unused and muted
    dress(old, "Sampler2", 0.70f, -0.30f, false, true);   // what was heard
    const std::string saved = old.m.stateJson();

    Desk now(kMainLayout);   // the default image: one "Sampler", no "Sampler2"
    REQUIRE(now.m.applyStateJson(saved));
    checkDressed(now, "Sampler", 0.70f, -0.30f, false, true);
    checkDressed(now, "Pitched", 0.80f,  0.20f, true,  false);
    checkDressed(now, "USB",     0.30f,  0.00f, false, false);
    checkDressed(now, "Click",   0.25f,  0.50f, true,  false);
    checkDressed(now, "IN1",     0.60f, -0.70f, false, true);
    checkDressed(now, "IN2",     0.55f,  0.70f, true,  true);
    checkDressed(now, "IN3",     0.45f,  0.30f, false, false);
}

TEST_CASE("a two-engine desk whose heard engine was muted or unrouted still sounds on the one-engine build", "[desk]") {
    /* A desk where Sampler2 was muted (the streamer was the one in use) or
     * sent nowhere would leave the only sample engine of this build silent,
     * with nothing on the Mixer saying why. On the migration it is unmuted,
     * and given back the routes this build set it up with. */
    Desk old(kTwoEngineLayout);
    dressAll(old);
    const int s2 = old.slot("Sampler2");
    old.m.setChannelVolume(s2, 0.65f);
    old.m.setChannelMute(s2, true);
    for (uint8_t o = 0; o < kOuts; ++o) old.m.setRouteEnabled(s2, o, false);
    const std::string saved = old.m.stateJson();

    Desk now(kMainLayout);
    const int s = now.slot("Sampler");
    now.m.setRouteEnabled(s, 0, true);          // how the platform routes it at boot
    now.m.setRouteEnabled(s, 1, true);
    REQUIRE(now.m.applyStateJson(saved));
    CHECK_THAT(now.m.getChannelVolume(s), WithinAbs(0.65f, kJsonEps));   // its level is kept
    CHECK_FALSE(now.m.isChannelMuted(s));
    CHECK(now.m.isRouteEnabled(s, 0));
    CHECK(now.m.isRouteEnabled(s, 1));
    CHECK_FALSE(now.m.isRouteEnabled(s, 2));

    /* Not on a load that is not that migration: a muted Sampler from this
     * same layout is the player's choice and stays muted. */
    Desk same(kMainLayout);
    same.m.setChannelMute(same.slot("Sampler"), true);
    Desk again(kMainLayout);
    REQUIRE(again.m.applyStateJson(same.m.stateJson()));
    CHECK(again.m.isChannelMuted(again.slot("Sampler")));
}

TEST_CASE("Mic and Line from a desk of Steve's input design go back to IN1 and IN2", "[desk]") {
    const std::vector<const char*> steve = {"Sampler", "Pitched", "Sampler2", "USB",
                                            "Click", "Mic", "Line", "IN3"};
    Desk old(steve);
    dress(old, "Mic",  0.60f, -0.70f, false, true);
    dress(old, "Line", 0.55f,  0.70f, true,  true);
    Desk now(kMainLayout);
    REQUIRE(now.m.applyStateJson(old.m.stateJson()));
    checkDressed(now, "IN1", 0.60f, -0.70f, false, true);
    checkDressed(now, "IN2", 0.55f,  0.70f, true,  true);
}

TEST_CASE("a desk from the same build round-trips", "[desk]") {
    Desk a(kTwoEngineLayout);
    dressAll(a);
    dress(a, "Sampler2", 0.10f, 0.0f, true, false);
    Desk b(kTwoEngineLayout);
    REQUIRE(b.m.applyStateJson(a.m.stateJson()));
    checkAllDressed(b);
    checkDressed(b, "Sampler2", 0.10f, 0.0f, true, false);
}

TEST_CASE("a renamed channel keeps its settings, a shifted renamed one too", "[desk]") {
    Desk old(kMainLayout);
    dressAll(old);
    const std::vector<const char*> renamed = {"Sampler", "Pitched", "Sampler2", "USB",
                                              "Click", "Mic", "Line", "IN3"};
    Desk now(renamed);
    REQUIRE(now.m.applyStateJson(asVersion3(old.m.stateJson())));
    checkDressed(now, "Mic",  0.60f, -0.70f, false, true);    // was IN1, slot 4 -> 5
    checkDressed(now, "Line", 0.55f,  0.70f, true,  true);    // was IN2, slot 5 -> 6
    checkDressed(now, "IN3",  0.45f,  0.30f, false, false);
}

TEST_CASE("a channel the build does not have is left out, not given to another", "[desk]") {
    const std::string json =
        R"({"version":3,"numOutputs":3,"channels":[)"
        R"({"id":0,"name":"Looper","volume":0.2,"muted":true,"soloed":false,"pan":0,"routes":[]},)"
        R"({"id":1,"name":"Pitched","volume":0.7,"muted":false,"soloed":false,"pan":0,"routes":[]}]})";
    Desk now(kMainLayout);
    REQUIRE(now.m.applyStateJson(json));
    CHECK(now.m.getChannelVolume(now.slot("Sampler")) == 1.0f);
    CHECK_FALSE(now.m.isChannelMuted(now.slot("Sampler")));
    CHECK_THAT(now.m.getChannelVolume(now.slot("Pitched")), WithinAbs(0.7f, kJsonEps));
}

TEST_CASE("two saved entries never land on one channel", "[desk]") {
    const std::string json =
        R"({"version":3,"numOutputs":3,"channels":[)"
        R"({"id":2,"name":"USB","volume":0.3,"muted":false,"soloed":false,"pan":0,"routes":[]},)"
        R"({"id":5,"name":"USB","volume":0.9,"muted":true,"soloed":false,"pan":0,"routes":[]}]})";
    Desk now(kMainLayout);
    REQUIRE(now.m.applyStateJson(json));
    CHECK_THAT(now.m.getChannelVolume(now.slot("USB")), WithinAbs(0.3f, kJsonEps));
    CHECK_FALSE(now.m.isChannelMuted(now.slot("USB")));
    CHECK(now.m.getChannelVolume(5) == 1.0f);   // IN2 untouched
}
