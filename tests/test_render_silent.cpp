#include <catch2/catch_test_macros.hpp>

#include "crosspad-mixer/AudioMixerEngine.hpp"
#include <crosspad/audio/IAudioNode.hpp>

#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace {

constexpr uint32_t kFrames = 128;
constexpr uint32_t kStereo = kFrames * 2;
constexpr uint8_t  kBuses  = 3;

/* A generator that adds a fixed block when it sounds and nothing when it
 * does not -- a pad between hits, a metronome off, a stream not playing.
 * With `overwrite` it writes the block instead, as an in-place effect would,
 * so a -0.0f in it reaches the channel's scratch as -0.0f (added to the +0
 * the engine clears the scratch to, it would come out +0). */
class BlockNode : public crosspad::IAudioNode {
public:
    std::vector<float> block = std::vector<float>(kStereo, 0.0f);
    bool sounding = false;
    bool overwrite = false;
    unsigned calls = 0;

    /* What this node leaves in a scratch the engine cleared to +0. */
    std::vector<float> scratch() const {
        std::vector<float> s(kStereo, 0.0f);
        put(s.data(), kFrames);
        return s;
    }
    void mix(float* out, uint32_t frames) override {
        ++calls;
        put(out, frames);
    }
    bool isGenerator() const override { return true; }
    const char* name() const override { return "block"; }

private:
    void put(float* out, uint32_t frames) const {
        if (!sounding) return;
        for (uint32_t i = 0; i < frames * 2; ++i) out[i] = overwrite ? block[i] : out[i] + block[i];
    }
};

/* Which sides of a block carry the noise; the other side stays +0. */
enum class Side { Both, LeftOnly, RightOnly };

void fillNoise(std::vector<float>& b, uint32_t seed, Side side = Side::Both) {
    for (size_t i = 0; i < b.size(); ++i) {
        seed = seed * 1664525u + 1013904223u;
        /* ±0.125: eleven channels summed stay under the limiter's ceiling on
         * every bus and on the tap. What is compared is which channels were
         * sent, not how the limiter shapes a loud block. */
        const float v = (static_cast<float>(seed >> 8) / 16777216.0f - 0.5f) * 0.25f;
        const bool left = (i % 2) == 0;
        b[i] = (side == Side::Both || (side == Side::LeftOnly) == left) ? v : 0.0f;
    }
}

/* The meter as render() reads it: |v| above the running peak, so -0.0f and
 * NaN read as 0. side 0 = left, 1 = right. */
float meterPeak(const std::vector<float>& s, size_t side) {
    float p = 0.0f;
    for (size_t i = side; i < s.size(); i += 2) {
        const float a = std::fabs(s[i]);
        if (a > p) p = a;
    }
    return p;
}

struct Strip {
    BlockNode* node;
    float vol;
    float pan;
    float send[kBuses];   // 0 = not routed
};

struct Mix {
    std::vector<float> bus[kBuses];
    float peakL[kBuses];
    float peakR[kBuses];
};

/* The channel's block as the old render() sent it: pulled, then panned. */
std::vector<float> pannedBlock(const Strip& st) {
    std::vector<float> s = st.node->scratch();
    if (st.pan != 0.0f) {
        const float gl = st.pan > 0.0f ? 1.0f - st.pan : 1.0f;
        const float gr = st.pan < 0.0f ? 1.0f + st.pan : 1.0f;
        for (uint32_t i = 0; i < kFrames; ++i) { s[i * 2] *= gl; s[i * 2 + 1] *= gr; }
    }
    return s;
}

/* What render() computed before a silent channel was skipped: every routed
 * channel panned and sent, whatever it held. A product rounded to float,
 * then a sum rounded to float: the host's scalar path, and the firmware's
 * dsps_mulc_f32 + dsps_add_f32 (esp_dsp_stub/ in the second binary). */
Mix referenceMix(const std::vector<Strip>& strips, const float outVol[kBuses]) {
    Mix m;
    for (auto& b : m.bus) b.assign(kStereo, 0.0f);
    for (const auto& st : strips) {
        const std::vector<float> s = pannedBlock(st);
        for (uint8_t o = 0; o < kBuses; ++o) {
            if (st.send[o] == 0.0f) continue;
            const float gain = st.vol * st.send[o];
            for (uint32_t i = 0; i < kStereo; ++i) m.bus[o][i] += s[i] * gain;
        }
    }
    for (uint8_t o = 0; o < kBuses; ++o) {
        for (uint32_t i = 0; i < kStereo; ++i) {
            const float v = m.bus[o][i] * outVol[o];
            /* Under the limiter's ceiling the output is the sum as it is;
             * the limiter itself is test_render_kernels' to check. */
            REQUIRE(std::fabs(v) < AudioMixerEngine::kLimitCeiling);
            m.bus[o][i] = v;
        }
        m.peakL[o] = meterPeak(m.bus[o], 0);
        m.peakR[o] = meterPeak(m.bus[o], 1);
    }
    return m;
}

struct Rig {
    AudioMixerEngine eng;
    std::vector<float> bus[kBuses];
    float* ptrs[kBuses];
    float outVol[kBuses] = {1.0f, 0.8f, 1.0f};

    Rig() {
        eng.setup(kFrames, 44100, kBuses);
        for (uint8_t o = 0; o < kBuses; ++o) {
            bus[o].assign(kStereo, 1.0f);   // render must clear it
            ptrs[o] = bus[o].data();
            eng.setOutputVolume(o, outVol[o]);
        }
    }
    AudioMixerEngine::ChannelId add(const Strip& st) {
        const auto ch = eng.addChannel(st.node, "ch");
        eng.setChannelVolume(ch, st.vol);
        eng.setChannelPan(ch, st.pan);
        for (uint8_t o = 0; o < kBuses; ++o) {
            if (st.send[o] == 0.0f) continue;
            eng.setRouteEnabled(ch, o, true);
            eng.setRouteVolume(ch, o, st.send[o]);
        }
        return ch;
    }
    void render() { eng.render(ptrs, kBuses, kFrames); }
};

bool sameBits(const std::vector<float>& a, const std::vector<float>& b) {
    return a.size() == b.size() &&
           std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
}

} // namespace

TEST_CASE("Mixer render: silent channels leave the buses bit for bit as the full mix",
          "[mixer][render]") {
    BlockNode full[4], left, right, leftHome, leftAway, rightHome, negZero, tiny;
    for (uint32_t i = 0; i < 4; ++i) fillNoise(full[i].block, i + 1);
    fillNoise(left.block, 5, Side::LeftOnly);
    fillNoise(right.block, 6, Side::RightOnly);
    fillNoise(leftHome.block, 7, Side::LeftOnly);
    fillNoise(leftAway.block, 8, Side::LeftOnly);
    fillNoise(rightHome.block, 9, Side::RightOnly);
    negZero.block.assign(kStereo, -0.0f);
    negZero.overwrite = true;
    for (uint32_t i = 0; i < kStereo; ++i) tiny.block[i] = (i % 3 == 0) ? -1e-42f : 1e-42f;

    /* Panned, at odd levels, sent to overlapping buses: a skipped send that
     * mattered would show up somewhere. */
    const std::vector<Strip> strips = {
        {&full[0],   0.9f, -0.3f, {1.0f, 0.5f, 0.0f}},
        {&full[1],   0.7f,  0.5f, {0.6f, 1.0f, 0.3f}},
        {&full[2],   1.0f,  0.0f, {0.0f, 0.8f, 1.0f}},
        {&full[3],   0.4f,  1.0f, {0.2f, 0.0f, 0.9f}},
        /* One side sounds, the other is +0 the whole block: a mono source,
         * a hard-panned one. Its peak is 0 on one side only, so it is sent. */
        {&left,      0.8f,  0.0f, {1.0f, 0.0f, 0.7f}},
        {&right,     0.6f,  0.4f, {0.0f, 1.0f, 0.5f}},
        /* The pan zeroes the silent side, or the sounding one (sent all the
         * same: the peak is taken before the pan, and it adds only ±0). */
        {&leftHome,  0.9f, -1.0f, {1.0f, 0.5f, 0.0f}},
        {&leftAway,  1.0f,  1.0f, {1.0f, 1.0f, 0.0f}},
        {&rightHome, 0.9f,  1.0f, {0.5f, 0.0f, 1.0f}},
        /* -0.0f in the scratch: peak 0, not sent. Subnormals: peak above 0,
         * sent -- alone on bus 0 they are all that bus holds. */
        {&negZero,   1.0f, -0.5f, {1.0f, 1.0f, 1.0f}},
        {&tiny,      1.0f,  0.0f, {1.0f, 1.0f, 0.0f}},
    };
    Rig rig;
    std::vector<AudioMixerEngine::ChannelId> ids;
    for (const auto& st : strips) ids.push_back(rig.add(st));

    /* Who sounds, block by block ('1' sounds, '.' silent), in the order of
     * `strips`: one silent among loud ones, a silent one coming in, all
     * silent, all sounding, then each kind of channel alone, so a skip that
     * should not happen leaves a bus empty instead of hiding under others. */
    const char* const pattern[] = {
        "1.11.......",
        ".11........",
        "...........",
        "11111111111",
        ".1.........",
        "....1......",
        ".....1.....",
        "......1....",
        ".......1...",
        "........1..",
        ".........1.",
        "..........1",
        "1...11..111",
    };
    /* renderTap() reads these channels' scratch after render(). */
    const size_t tapped[] = {0, 4, 5, 8, 9, 10};

    unsigned blocks = 0;
    for (const char* who : pattern) {
        REQUIRE(std::strlen(who) == strips.size());
        for (size_t i = 0; i < strips.size(); ++i) strips[i].node->sounding = who[i] == '1';
        rig.render();
        ++blocks;

        const Mix want = referenceMix(strips, rig.outVol);
        for (uint8_t o = 0; o < kBuses; ++o) {
            INFO("block " << blocks << " '" << who << "' bus " << int(o));
            CHECK(sameBits(rig.bus[o], want.bus[o]));
            float l = -1.0f, r = -1.0f;
            rig.eng.getOutputLevel(o, l, r);
            CHECK(l == want.peakL[o]);
            CHECK(r == want.peakR[o]);
        }

        /* The channel meters still read every channel, silent ones at 0. */
        for (size_t i = 0; i < strips.size(); ++i) {
            INFO("block " << blocks << " '" << who << "' channel " << i);
            const std::vector<float> s = strips[i].node->scratch();
            float l = -1.0f, r = -1.0f;
            rig.eng.getChannelLevel(ids[i], l, r);
            CHECK(l == meterPeak(s, 0));
            CHECK(r == meterPeak(s, 1));
        }

        /* A channel render() did not send is left unpanned in its scratch:
         * ±0 all the same, so the tap sums the same bits. */
        std::vector<AudioMixerEngine::ChannelId> tapChs;
        for (size_t i : tapped) tapChs.push_back(ids[i]);
        std::vector<float> tap(kStereo, 1.0f);
        rig.eng.renderTap(tapChs.data(), static_cast<uint8_t>(tapChs.size()), tap.data(), kFrames);
        std::vector<float> wantTap(kStereo, 0.0f);
        for (size_t i : tapped) {
            const std::vector<float> s = pannedBlock(strips[i]);
            for (uint32_t k = 0; k < kStereo; ++k) wantTap[k] += s[k] * strips[i].vol;
        }
        for (float v : wantTap) REQUIRE(std::fabs(v) < AudioMixerEngine::kLimitCeiling);
        INFO("block " << blocks << " '" << who << "' tap");
        CHECK(sameBits(tap, wantTap));
    }

    /* A silent channel's node is still pulled every block: its playhead,
     * envelope or stream keeps moving. */
    for (const auto& st : strips) CHECK(st.node->calls == blocks);
}

TEST_CASE("Mixer render: a block of nothing but NaN does not reach the buses",
          "[mixer][render]") {
    BlockNode loud, bad;
    fillNoise(loud.block, 7);
    bad.block.assign(kStereo, std::numeric_limits<float>::quiet_NaN());
    loud.sounding = true;
    bad.sounding = true;

    const std::vector<Strip> strips = {{&loud, 1.0f, 0.0f, {1.0f, 0.5f, 0.0f}}};
    Rig rig;
    rig.add(strips[0]);
    rig.add({&bad, 1.0f, 0.0f, {1.0f, 1.0f, 1.0f}});
    rig.render();

    /* The meter cannot see NaN, so the channel reads silent and is not sent;
     * every bus is the loud channel alone. */
    const Mix want = referenceMix(strips, rig.outVol);
    for (uint8_t o = 0; o < kBuses; ++o) {
        INFO("bus " << int(o));
        CHECK(sameBits(rig.bus[o], want.bus[o]));
    }
}

namespace {
/* BlockNode that says so when its block added nothing (IAudioNode). */
class SayingNode : public BlockNode {
public:
    bool lastMixSilent() const override { return !sounding; }
};
} // namespace

TEST_CASE("Mixer render: a node that says its block was silent is metered and sent as a scan of it would be",
          "[mixer][render]") {
    BlockNode quiet[3];
    SayingNode saying[3];
    for (uint32_t i = 0; i < 3; ++i) {
        fillNoise(quiet[i].block, 20 + i);
        fillNoise(saying[i].block, 20 + i);
    }
    const float sends[3][kBuses] = {{1.0f, 0.5f, 0.0f}, {0.6f, 1.0f, 0.3f}, {0.0f, 0.8f, 1.0f}};
    Rig a, b;
    std::vector<AudioMixerEngine::ChannelId> ia, ib;
    for (uint32_t i = 0; i < 3; ++i) {
        ia.push_back(a.add({&quiet[i], 0.9f, 0.25f * static_cast<float>(i), {sends[i][0], sends[i][1], sends[i][2]}}));
        ib.push_back(b.add({&saying[i], 0.9f, 0.25f * static_cast<float>(i), {sends[i][0], sends[i][1], sends[i][2]}}));
    }
    const char* const pattern[] = {"...", "1..", ".1.", "111", "...", "..1", "1.1"};
    for (const char* who : pattern) {
        for (uint32_t i = 0; i < 3; ++i) quiet[i].sounding = saying[i].sounding = who[i] == '1';
        a.render();
        b.render();
        for (uint8_t o = 0; o < kBuses; ++o) REQUIRE(sameBits(a.bus[o], b.bus[o]));
        for (uint32_t i = 0; i < 3; ++i) {
            float al = -1, ar = -1, bl = -1, br = -1;
            a.eng.getChannelLevel(ia[i], al, ar);
            b.eng.getChannelLevel(ib[i], bl, br);
            CHECK(al == bl);
            CHECK(ar == br);
        }
    }
}
