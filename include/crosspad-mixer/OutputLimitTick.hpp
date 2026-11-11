#pragma once
/**
 * @file OutputLimitTick.hpp
 * @brief One UI tick's limiter readings, taken once per output bus.
 *
 * AudioMixerEngine::takeOutputLimit() returns the limiter's deepest gain on a
 * bus since the last read and resets it. The Mixer's OUT page shows a bus the
 * hardware gives levels of its own as one row per speaker or jack (Speakers
 * and Phones both on Out 1/2), so a per-row take let the first row consume
 * the reading and every later row of the bus read 1.0: Phones' LIM never lit.
 * Make one of these per tick and read every row through it.
 */

#include "crosspad-mixer/AudioMixerEngine.hpp"

#include <cstdint>

class OutputLimitTick {
public:
    explicit OutputLimitTick(AudioMixerEngine& m) : m_(m) {}

    /// The deepest limiter gain on @p out since the previous tick (1.0 = the
    /// limiter never acted); the same value for every read in this tick.
    float operator()(AudioMixerEngine::OutputId out)
    {
        if (out >= kMixerMaxOutputs) return m_.takeOutputLimit(out);   // the tap: one reader
        const uint32_t bit = 1u << out;
        if (!(taken_ & bit)) {
            lim_[out] = m_.takeOutputLimit(out);
            taken_ |= bit;
        }
        return lim_[out];
    }

private:
    AudioMixerEngine& m_;
    uint32_t taken_ = 0;
    float    lim_[kMixerMaxOutputs] = {};
};
