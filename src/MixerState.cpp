/**
 * @file MixerState.cpp
 * @brief The desk as JSON: save, load, and the migration of older desks.
 *
 * Split out of AudioMixerEngine.cpp so a platform can build the render for
 * speed and this for size: none of it runs on the audio thread (the LVGL
 * thread saves and loads, boot loads), and ArduinoJson inlined at -O2 is most
 * of what the engine's object weighed (the board's image is PSRAM too).
 */

#include "crosspad-mixer/AudioMixerEngine.hpp"

#include <ArduinoJson.h>

#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

// ── State persistence (filesystem JSON; PC; gated to non-embedded) ────────

/* Which live slot a saved channel goes to.
 *
 *  1. The saved slot, when the channel there has the saved name: a desk saved
 *     by this same build.
 *  2. Else the active channel with the saved name, wherever it is now. This
 *     is the migration: the native sampler's "Sampler2" went in after
 *     "Pitched" and moved USB, Click and the inputs one slot on, so by slot
 *     a desk from before it gave nothing to any of them (the name guard
 *     refused every one) -- and a build without the channel would shift them
 *     back the other way.
 *  3. A channel that was only renamed (kRenamed, either way round: main's
 *     IN1/IN2 and Steve's Mic/Line are the same jacks): the channel with the
 *     other name, its saved slot first.
 *  4. A saved entry with no name (nothing written so far does that) or a
 *     saved slot nobody has registered yet: the saved slot, as before.
 * A slot an earlier entry of the same desk took is not given out twice. */
int AudioMixerEngine::slotForSaved(int id, const char* name, const bool* taken) const {
    static const char* const kRenamed[][2] = {
        {"IN1", "Mic"},     // the built-in mics
        {"IN2", "Line"},    // the line-in jacks
    };
    const bool idOk = id >= 0 && id < kMixerMaxChannels && !taken[id];
    if (!name || !name[0]) return idOk ? id : -1;

    auto liveNamed = [&](const char* want) -> int {
        if (idOk && channels_[id].active.load(std::memory_order_acquire) &&
            std::strcmp(channels_[id].name, want) == 0) {
            return id;
        }
        for (int ch = 0; ch < kMixerMaxChannels; ++ch) {
            if (taken[ch] || !channels_[ch].active.load(std::memory_order_acquire)) continue;
            if (std::strcmp(channels_[ch].name, want) == 0) return ch;
        }
        return -1;
    };

    int ch = liveNamed(name);
    if (ch >= 0) return ch;
    for (const auto& r : kRenamed) {
        for (int from = 0; from < 2; ++from) {
            if (std::strcmp(name, r[from]) != 0) continue;
            ch = liveNamed(r[1 - from]);
            if (ch >= 0) return ch;
        }
    }
    /* Not on this desk under any name: only a slot nothing has registered
     * keeps the saved settings (a desk applied before its channels exist). */
    if (idOk && !channels_[id].name[0]) return id;
    return -1;
}

std::string AudioMixerEngine::stateJson() const {
    std::lock_guard<std::mutex> lock(registrationMutex_);

    JsonDocument doc;
    doc["version"] = kDeskVersion;
    doc["numOutputs"] = numOutputs_;

    JsonArray channelsArr = doc["channels"].to<JsonArray>();
    for (uint8_t ch = 0; ch < kMixerMaxChannels; ++ch) {
        const auto& slot = channels_[ch];
        if (!slot.active.load(std::memory_order_acquire)) continue;
        JsonObject c = channelsArr.add<JsonObject>();
        c["id"]     = ch;
        c["name"]   = slot.name;
        c["volume"] = slot.volume.load(std::memory_order_relaxed);
        c["muted"]  = slot.muted .load(std::memory_order_relaxed);
        c["soloed"] = slot.soloed.load(std::memory_order_relaxed);
        c["pan"]    = slot.pan   .load(std::memory_order_relaxed);
        JsonArray routes = c["routes"].to<JsonArray>();
        for (uint8_t o = 0; o < numOutputs_; ++o) {
            JsonObject r = routes.add<JsonObject>();
            r["out"]     = o;
            r["enabled"] = slot.routeEnabled[o].load(std::memory_order_relaxed);
            r["volume"]  = slot.routeVol[o].load(std::memory_order_relaxed);
        }
    }

    JsonArray outArr = doc["outputs"].to<JsonArray>();
    for (uint8_t o = 0; o < numOutputs_; ++o) {
        JsonObject ob = outArr.add<JsonObject>();
        ob["volume"] = outputs_[o].volume.load(std::memory_order_relaxed);
        ob["muted"]  = outputs_[o].muted .load(std::memory_order_relaxed);
        const crosspad::EqSettings eq = o < kMixerEqOutputs ? outputEq_[o].settings()
                                                             : crosspad::EqSettings{};
        /* Only when there is something to say: a desk that has never had an
         * EQ set keeps the blob the size it was. */
        if (eq.on || eq.any()) {
            ob["eqOn"] = eq.on;
            JsonArray bands = ob["eq"].to<JsonArray>();
            for (const auto& b : eq.band) {
                JsonObject bo = bands.add<JsonObject>();
                bo["kind"] = (int)b.kind;
                bo["hz"]   = b.hz;
                bo["db"]   = b.gainDb;
                bo["q"]    = b.q;
            }
        }
    }

    std::string out;
    serializeJson(doc, out);
    return out;
}

bool AudioMixerEngine::applyStateJson(const std::string& json) {
    std::lock_guard<std::mutex> lock(registrationMutex_);

    JsonDocument doc;
    if (deserializeJson(doc, json)) return false;

    const int version = doc["version"] | 0;
    deskMigrated_ = version < kDeskVersion;

    /* Every version so far (2..5) saved each channel with its slot and its
     * name, so one rule loads them all: the name decides (slotForSaved()). A
     * desk saved by firmware without "Sampler2" puts USB, Click and the
     * inputs back on USB, Click and the inputs, not one slot early; a desk
     * that names a channel this build does not have is left out for that
     * channel, which keeps its defaults. */
    bool taken[kMixerMaxChannels] = {};
    JsonArray channelsArr = doc["channels"];

    /* A desk of a two-engine build (the prebuilt streamer on "Sampler", the
     * native engine on "Sampler2") onto a build with only the native one,
     * which is called "Sampler". What was heard there was Sampler2 -- Steve's
     * builds played it -- so its settings are the ones this build's Sampler
     * gets, and the streamer's (often muted, being unused) are dropped. By
     * name alone it went the other way: the streamer's mute on the only
     * engine, and Sampler2's settings nowhere. */
    auto liveSlot = [&](const char* want) -> int {
        for (int ch = 0; ch < kMixerMaxChannels; ++ch) {
            if (channels_[ch].active.load(std::memory_order_acquire) &&
                std::strcmp(channels_[ch].name, want) == 0) {
                return ch;
            }
        }
        return -1;
    };
    bool deskHasSampler2 = false;
    for (JsonObject c : channelsArr) {
        if (std::strcmp(c["name"] | "", "Sampler2") == 0) deskHasSampler2 = true;
    }
    /* This build's only sample engine: "Sampler" with no "Sampler2" beside it. */
    const int soleEngine = liveSlot("Sampler2") < 0 ? liveSlot("Sampler") : -1;
    /* Sampler2's settings go onto it (above). */
    const bool foldEngines = deskHasSampler2 && soleEngine >= 0;
    /* And it is rescued if it would load silent -- on that fold, and on any
     * desk older than v5, which is how a desk the fold should have healed but
     * v4 had already loaded wrongly (and saved again) gets healed too. */
    const bool rescueEngine = soleEngine >= 0 && (foldEngines || deskMigrated_);
    /* What this build gave its engine at boot, for the rescue below. */
    bool  bootRoute[kMixerMaxOutputs] = {};
    float bootRouteVol[kMixerMaxOutputs] = {};
    if (rescueEngine) {
        for (uint8_t o = 0; o < kMixerMaxOutputs; ++o) {
            bootRoute[o] = channels_[soleEngine].routeEnabled[o].load(std::memory_order_relaxed);
            bootRouteVol[o] = channels_[soleEngine].routeVol[o].load(std::memory_order_relaxed);
        }
    }

    for (JsonObject c : channelsArr) {
        const char* savedName = c["name"] | "";
        if (foldEngines) {
            if (std::strcmp(savedName, "Sampler") == 0) continue;     // the streamer's
            if (std::strcmp(savedName, "Sampler2") == 0) savedName = "Sampler";
        }
        const int id = slotForSaved(c["id"] | -1, savedName, taken);
        if (id < 0) continue;
        taken[id] = true;
        auto& slot = channels_[id];
        slot.volume.store(c["volume"] | 1.0f, std::memory_order_relaxed);
        slot.muted .store(c["muted"]  | false, std::memory_order_relaxed);
        slot.soloed.store(c["soloed"] | false, std::memory_order_relaxed);
        slot.pan   .store(c["pan"]    | 0.0f, std::memory_order_relaxed);
        JsonArray routes = c["routes"];
        for (JsonObject r : routes) {
            int o = r["out"] | -1;
            if (o < 0 || o >= kMixerMaxOutputs) continue;
            slot.routeEnabled[o].store(r["enabled"] | false, std::memory_order_relaxed);
            slot.routeVol[o]    .store(r["volume"]  | 1.0f,  std::memory_order_relaxed);
        }
    }

    /* The one sample engine of this build must not come out of a migration
     * silent: a desk where Sampler2 was muted (the streamer was in use), or
     * one v4 had already given the streamer's settings and saved, would leave
     * the pads dead with nothing on the Mixer saying why. Silent is muted, at
     * zero, or with no route it is heard on -- none enabled, every one at
     * zero, or only to an output nobody hears (the USB rec bus). It is
     * unmuted, brought to unity if it was at zero, and given the routes it
     * booted with; its pan and any level above zero stay the desk's. Only on
     * a migration -- a silent Sampler on a v5 desk is the player's choice --
     * and once: the platform saves the desk back (deskMigrated()). */
    if (rescueEngine) {
        auto& eng = channels_[soleEngine];
        if (eng.muted.load(std::memory_order_relaxed)) {
            eng.muted.store(false, std::memory_order_relaxed);
            printf("[Mixer] desk v%d: the sample engine loaded muted, unmuted\n", version);
        }
        if (!(eng.volume.load(std::memory_order_relaxed) > 0.0f)) {
            eng.volume.store(1.0f, std::memory_order_relaxed);
            printf("[Mixer] desk v%d: the sample engine loaded at zero, set to unity\n", version);
        }
        bool heard = false;
        for (uint8_t o = 0; o < numOutputs_; ++o) {
            heard = heard || (outputs_[o].heard.load(std::memory_order_relaxed) &&
                              eng.routeEnabled[o].load(std::memory_order_relaxed) &&
                              eng.routeVol[o].load(std::memory_order_relaxed) > 0.0f);
        }
        if (!heard) {
            for (uint8_t o = 0; o < kMixerMaxOutputs; ++o) {
                if (!bootRoute[o]) continue;
                eng.routeEnabled[o].store(true, std::memory_order_relaxed);
                if (!(eng.routeVol[o].load(std::memory_order_relaxed) > 0.0f)) {
                    eng.routeVol[o].store(bootRouteVol[o] > 0.0f ? bootRouteVol[o] : 1.0f,
                                          std::memory_order_relaxed);
                }
            }
            printf("[Mixer] desk v%d: the sample engine had no heard route, boot routes restored\n",
                   version);
        }
    }

    /* Said, not changed: an input that loads sent to an output is a live
     * microphone or line on the speakers from boot, which is how a noise
     * floor nobody asked for reaches a recording (rt_glitch, 2026-10-09). */
    static const char* const kInputs[] = {"IN1", "IN2", "IN3", "Mic", "Line"};
    for (int ch = 0; ch < kMixerMaxChannels; ++ch) {
        const auto& in = channels_[ch];
        if (!in.active.load(std::memory_order_acquire)) continue;
        bool isInput = false;
        for (const char* n : kInputs) isInput = isInput || std::strcmp(in.name, n) == 0;
        if (!isInput || in.muted.load(std::memory_order_relaxed)) continue;
        for (uint8_t o = 0; o < numOutputs_; ++o) {
            if (in.routeEnabled[o].load(std::memory_order_relaxed) &&
                in.routeVol[o].load(std::memory_order_relaxed) > 0.0f) {
                printf("[Mixer] desk: input %s loads routed to output %u\n", in.name, (unsigned)o);
            }
        }
    }

    int idx = 0;
    JsonArray outArr = doc["outputs"];
    for (JsonObject ob : outArr) {
        if (idx >= kMixerMaxOutputs) break;
        outputs_[idx].volume.store(ob["volume"] | 1.0f, std::memory_order_relaxed);
        outputs_[idx].muted .store(ob["muted"]  | false, std::memory_order_relaxed);
        /* Only a desk that has an EQ in it asks for one: an EQ's state is
         * taken on first use, and a board that has never set one should not
         * pay for it at boot. */
        if (!ob["eq"].is<JsonArrayConst>() && !(ob["eqOn"] | false)) { ++idx; continue; }
        crosspad::EqSettings eq;
        eq.on = ob["eqOn"] | false;
        int bi = 0;
        /* Named, not iterated off the subscript: the range expression would
         * be a temporary and the view into it a dangling reference. */
        JsonArrayConst bands = ob["eq"];
        for (JsonObjectConst bo : bands) {
            if (bi >= crosspad::kEqBands) break;
            crosspad::EqBand& b = eq.band[bi++];
            const int kind = bo["kind"] | 0;
            b.kind   = kind >= 0 && kind <= (int)crosspad::EqKind::LowPass
                           ? (crosspad::EqKind)kind : crosspad::EqKind::Off;
            b.hz     = bo["hz"] | 1000.0f;
            b.gainDb = bo["db"] | 0.0f;
            b.q      = bo["q"]  | 0.7f;
        }
        if (idx < kMixerEqOutputs) outputEq_[idx].set(eq);
        ++idx;
    }
    return true;
}

void AudioMixerEngine::saveState(const std::string& path) const {
    const std::string json = stateJson();

    /* Written beside the real file and swapped in. rename() will not replace
     * a file on FatFs, so the old one steps aside as .bak -- not removed, so
     * there is never a moment with neither -- and loadState() falls back to
     * .tmp, then .bak: a kill at any point costs at most this save. */
    const std::string tmpPath = path + ".tmp";
    const std::string bakPath = path + ".bak";
    {
        FILE* f = std::fopen(tmpPath.c_str(), "w");
        if (!f) {
            printf("[Mixer] Failed to save state to %s\n", tmpPath.c_str());
            return;
        }
        const bool wrote = std::fwrite(json.data(), 1, json.size(), f) == json.size();
        const bool closed = std::fclose(f) == 0;    // a full disk shows up on the flush
        if (!wrote || !closed) {
            printf("[Mixer] Failed to write %s\n", tmpPath.c_str());
            std::remove(tmpPath.c_str());
            return;
        }
    }
    std::remove(bakPath.c_str());
    const bool hadOld = std::rename(path.c_str(), bakPath.c_str()) == 0;
    if (std::rename(tmpPath.c_str(), path.c_str()) != 0) {
        printf("[Mixer] Failed to commit %s → %s\n", tmpPath.c_str(), path.c_str());
        if (hadOld) std::rename(bakPath.c_str(), path.c_str());
        std::remove(tmpPath.c_str());
        return;
    }
    std::remove(bakPath.c_str());
    printf("[Mixer] State saved to %s\n", path.c_str());
}

namespace {

struct FileCloser {
    void operator()(FILE* f) const { std::fclose(f); }
};

/// The whole file, or false if it would not open. The handle is owned while
/// the string grows, so a bad_alloc cannot leak it.
bool readFile(const std::string& path, std::string& out) {
    std::unique_ptr<FILE, FileCloser> f(std::fopen(path.c_str(), "r"));
    if (!f) return false;
    out.clear();
    constexpr size_t kChunk = 512;
    for (;;) {
        const size_t had = out.size();
        out.resize(had + kChunk);
        const size_t n = std::fread(&out[had], 1, kChunk, f.get());
        out.resize(had + n);
        if (n < kChunk) break;
    }
    return true;
}

} // namespace

void AudioMixerEngine::loadState(const std::string& path) {
    std::string json;
    for (const char* suffix : {"", ".tmp", ".bak"}) {
        const std::string p = path + suffix;
        if (!readFile(p, json)) continue;
        if (!applyStateJson(json)) {
            printf("[Mixer] State parse error in %s\n", p.c_str());
            continue;
        }
        printf("[Mixer] State loaded from %s\n", p.c_str());
        return;
    }
    printf("[Mixer] No saved state at %s, leaving defaults\n", path.c_str());
}
