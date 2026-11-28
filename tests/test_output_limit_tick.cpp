#include <catch2/catch_test_macros.hpp>

#include "crosspad-mixer/AudioMixerEngine.hpp"
#include "crosspad-mixer/OutputLimitTick.hpp"
#include <crosspad/audio/IAudioNode.hpp>

#include <vector>

/* The OUT page shows Out 1/2 as two rows, Speakers and Phones. Both must light
 * LIM from one tick's limiter reading: takeOutputLimit() resets what it reads,
 * so a take per row left the second row dark (crosspad-mixer, Mixer 0.22.1). */

namespace {

constexpr uint32_t kFrames = 64;

class LoudNode : public crosspad::IAudioNode {
public:
    float level = 1.6f;   // over full scale: the limiter has to act
    void mix(float* out, uint32_t frames) override {
        for (uint32_t i = 0; i < frames * 2; ++i) out[i] += level;
    }
    bool isGenerator() const override { return true; }
    const char* name() const override { return "loud"; }
};

struct Desk {
    LoudNode node;
    AudioMixerEngine eng;
    std::vector<float> bus0 = std::vector<float>(kFrames * 2), bus1 = std::vector<float>(kFrames * 2);
    Desk() {
        eng.setup(kFrames, 44100, 2);
        const auto ch = eng.addChannel(&node, "loud");
        eng.setRouteEnabled(ch, 0, true);
        eng.setRouteVolume(ch, 0, 1.0f);
    }
    void block() {
        float* ptrs[2] = {bus0.data(), bus1.data()};
        eng.render(ptrs, 2, kFrames);
    }
};

} // namespace

TEST_CASE("takeOutputLimit resets: a second read in one tick sees no limiting", "[limit]")
{
    Desk d;
    d.block();
    REQUIRE(d.eng.takeOutputLimit(0) < 0.99f);
    REQUIRE(d.eng.takeOutputLimit(0) == 1.0f);   // the trap a per-row take fell into
}

TEST_CASE("OutputLimitTick: every row of one bus sees the tick's reading", "[limit]")
{
    Desk d;
    d.block();
    OutputLimitTick limit(d.eng);
    const float speakers = limit(0);
    const float phones = limit(0);
    REQUIRE(speakers < 0.99f);
    REQUIRE(phones == speakers);
    REQUIRE(limit(1) == 1.0f);                   // nothing on the other bus
}

TEST_CASE("OutputLimitTick: the reading is taken once, the next tick starts fresh", "[limit]")
{
    Desk d;
    d.block();
    { OutputLimitTick limit(d.eng); REQUIRE(limit(0) < 0.99f); }
    { OutputLimitTick limit(d.eng); REQUIRE(limit(0) == 1.0f); }   // no block since: reset

    d.node.level = 0.1f;                         // quiet, but the gain recovers over the release
    d.block();
    OutputLimitTick limit(d.eng);
    const float a = limit(0);
    REQUIRE(limit(0) == a);
}

/* The reading keeps the deepest limiting since the last read, and nothing reads
 * it while no page shows the outputs: entering OUT on a silent desk lit LIM for
 * a second from music played minutes before (board, v1.1.5-rc1). */
TEST_CASE("discardOutputLimits: limiting from before the page was shown lights nothing", "[limit]")
{
    Desk d;
    d.block();                                   // loud, with the page closed
    d.node.level = 0.0f;
    for (int i = 0; i < 2000; ++i) d.block();    // silence since: the gain is back at 1
    { OutputLimitTick stale(d.eng); REQUIRE(stale(0) < 0.99f); }   // the bug: still reported

    d.node.level = 1.6f;
    d.block();
    d.node.level = 0.0f;
    for (int i = 0; i < 2000; ++i) d.block();
    discardOutputLimits(d.eng);                  // the page is entered
    OutputLimitTick first(d.eng);
    REQUIRE(first(0) == 1.0f);
    REQUIRE(first(1) == 1.0f);
}

TEST_CASE("discardOutputLimits: limiting after the page is entered still shows", "[limit]")
{
    Desk d;
    discardOutputLimits(d.eng);
    d.block();
    OutputLimitTick limit(d.eng);
    REQUIRE(limit(0) < 0.99f);
}
