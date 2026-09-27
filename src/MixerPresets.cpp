// SPDX-License-Identifier: MIT

#include "crosspad-mixer/MixerPresets.hpp"
#include "crosspad-mixer/AudioMixerEngine.hpp"

#include "crosspad/audio/IInputGain.hpp"
#include "crosspad/platform/PlatformServices.hpp"

#include <ArduinoJson.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif

namespace crosspad {
namespace {

/// Every folder of @p dir that is not there yet.
bool make_dirs(const std::string& dir)
{
    for (size_t at = 1; at <= dir.size(); ++at) {
        if (at != dir.size() && dir[at] != '/') continue;
        const std::string part = dir.substr(0, at);
#ifdef _WIN32
        if (_mkdir(part.c_str()) != 0 && errno != EEXIST) return false;
#else
        if (mkdir(part.c_str(), 0775) != 0 && errno != EEXIST) return false;
#endif
    }
    return true;
}

bool read_file(const std::string& path, std::string& out)
{
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    out.clear();
    char buf[512];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) out.append(buf, n);
    std::fclose(f);
    return !out.empty();
}

/// Written beside the file and swapped in, the old one kept as .bak until
/// the new one is in place: rename() does not replace a file on FatFs.
bool write_file(const std::string& path, const std::string& text)
{
    const std::string tmp = path + ".tmp", bak = path + ".bak";
    std::FILE* f = std::fopen(tmp.c_str(), "wb");
    if (!f) return false;
    const bool ok = std::fwrite(text.data(), 1, text.size(), f) == text.size();
    if (std::fclose(f) != 0 || !ok) { std::remove(tmp.c_str()); return false; }
    std::remove(bak.c_str());
    const bool hadOld = std::rename(path.c_str(), bak.c_str()) == 0;
    if (std::rename(tmp.c_str(), path.c_str()) != 0) {
        if (hadOld) std::rename(bak.c_str(), path.c_str());
        return false;
    }
    std::remove(bak.c_str());
    return true;
}

const char* const kDefaultName[MixerPresets::kSlots] = {
    "STREAM", "BEATS", "SAMPLING", "PRESET 4", "PRESET 5", "PRESET 6",
};

/// A starting point: where each channel goes -- out 1/2 (speakers), out 3/4
/// (phones), the recording bus. A channel not listed is left as it is.
struct Send { const char* channel; bool o12, o34, rec; };
struct Factory { const Send* sends; size_t count; };

/* Streaming: the stream records the pads, the song and the voice; the click
 * and the computer's own sound only in the phones, so neither ends up in it
 * twice. */
const Send kStream[] = {
    {"Sampler", 1, 1, 1}, {"Pitched", 1, 1, 1}, {"Song", 1, 1, 1}, {"Preview", 1, 1, 0},
    {"Click",   0, 1, 0}, {"USB",     0, 1, 0}, {"IN1",  0, 1, 1}, {"IN2",     0, 1, 1},
};
/* Beats: everything the board plays on both outputs, the click too, the
 * inputs off; the recording bus gets the pads. */
const Send kBeats[] = {
    {"Sampler", 1, 1, 1}, {"Pitched", 1, 1, 1}, {"Song", 1, 1, 0}, {"Preview", 1, 1, 0},
    {"Click",   1, 1, 0}, {"USB",     1, 1, 0}, {"IN1",  0, 0, 0}, {"IN2",     0, 0, 0},
};
/* Sampling: the computer's sound in the phones (the Recorder takes it before
 * the mixer anyway), auditions on both, no click. */
const Send kSampling[] = {
    {"Sampler", 1, 1, 1}, {"Pitched", 1, 1, 1}, {"Song", 1, 1, 0}, {"Preview", 1, 1, 0},
    {"Click",   0, 0, 0}, {"USB",     0, 1, 0}, {"IN1",  0, 0, 0}, {"IN2",     0, 0, 0},
};
const Factory kFactory[MixerPresets::kSlots] = {
    {kStream, sizeof(kStream) / sizeof(Send)},
    {kBeats, sizeof(kBeats) / sizeof(Send)},
    {kSampling, sizeof(kSampling) / sizeof(Send)},
    {nullptr, 0}, {nullptr, 0}, {nullptr, 0},
};

} // namespace

std::string MixerPresets::path(int slot) const
{
    char name[32];
    std::snprintf(name, sizeof(name), "/preset_%d.json", slot + 1);
    return dir_ + name;
}

const MixerPresets::Info& MixerPresets::info(int slot) const
{
    static const Info kNone;
    return slot >= 0 && slot < kSlots ? info_[slot] : kNone;
}

void MixerPresets::refresh()
{
    for (int s = 0; s < kSlots; ++s) {
        Info& i = info_[s];
        i.name = kDefaultName[s];
        i.kind = kFactory[s].sends ? Kind::Factory : Kind::Empty;
        std::string text;
        if (!available() || !read_file(path(s), text)) continue;
        JsonDocument doc;
        if (deserializeJson(doc, text) || !doc["desk"].is<JsonObject>()) continue;
        const char* n = doc["name"] | "";
        if (n[0]) i.name = n;
        i.kind = Kind::Saved;
    }
}

std::string MixerPresets::snapshot(const std::string& name) const
{
    JsonDocument doc;
    doc["version"] = 1;
    doc["name"] = name;
    doc["desk"] = serialized(getMixerEngine().stateJson());
    JsonArray gains = doc["gains"].to<JsonArray>();
    if (auto* g = getPlatformServices().inputGain) {
        for (int i = 0; i < g->inputs(); ++i) gains.add(g->gainDb(i));
    }
    std::string out;
    serializeJson(doc, out);
    return out;
}

bool MixerPresets::apply(const std::string& json)
{
    JsonDocument doc;
    if (deserializeJson(doc, json) || !doc["desk"].is<JsonObject>()) return false;
    std::string desk;
    serializeJson(doc["desk"], desk);
    auto& m = getMixerEngine();
    /* The recording bus's level is Settings' (applied at every boot): a preset
     * moving it would be undone by the next one. */
    float kept[kMixerMaxOutputs];
    for (uint8_t o = 0; o < m.numOutputs() && o < kMixerMaxOutputs; ++o) kept[o] = m.getOutputVolume(o);
    if (!m.applyStateJson(desk)) return false;
    for (uint8_t o = 0; o < m.numOutputs() && o < kMixerMaxOutputs; ++o) {
        if (!m.isOutputHeard(o)) m.setOutputVolume(o, kept[o]);
    }
    if (auto* g = getPlatformServices().inputGain) {
        JsonArrayConst gains = doc["gains"];
        int i = 0;
        for (JsonVariantConst v : gains) {
            if (i >= g->inputs()) break;
            g->setGainDb(i++, v.as<float>());
        }
    }
    return true;
}

bool MixerPresets::applyFactory(int slot)
{
    const Factory& f = kFactory[slot];
    if (!f.sends) return false;
    auto& m = getMixerEngine();
    int heard[2] = {-1, -1}, rec = -1, nHeard = 0;
    for (uint8_t o = 0; o < m.numOutputs(); ++o) {
        if (m.isOutputHeard(o)) { if (nHeard < 2) heard[nHeard++] = o; }
        else if (rec < 0) rec = o;
    }
    for (size_t k = 0; k < f.count; ++k) {
        const Send& s = f.sends[k];
        for (uint8_t ch = 0; ch < m.maxChannels(); ++ch) {
            if (!m.isChannelActive(ch) || std::strcmp(m.getChannelName(ch), s.channel) != 0) continue;
            m.setChannelVolume(ch, 1.0f);
            m.setChannelMute(ch, false);
            m.setChannelSolo(ch, false);
            m.setChannelPan(ch, 0.0f);
            const int outs[3] = {heard[0], heard[1], rec};
            const bool on[3] = {s.o12, s.o34, s.rec};
            for (int i = 0; i < 3; ++i) {
                if (outs[i] < 0) continue;
                m.setRouteVolume(ch, (uint8_t)outs[i], 1.0f);
                m.setRouteEnabled(ch, (uint8_t)outs[i], on[i]);
            }
        }
    }
    return true;
}

bool MixerPresets::save(int slot, const std::string& name)
{
    if (!available() || slot < 0 || slot >= kSlots || name.empty() || name.size() > kMaxName) return false;
    if (!make_dirs(dir_) || !write_file(path(slot), snapshot(name))) return false;
    info_[slot].name = name;
    info_[slot].kind = Kind::Saved;
    return true;
}

bool MixerPresets::load(int slot)
{
    if (slot < 0 || slot >= kSlots) return false;
    const std::string before = snapshot("undo");
    std::string text;
    const bool ok = info_[slot].kind == Kind::Saved ? read_file(path(slot), text) && apply(text)
                                                    : applyFactory(slot);
    if (!ok) return false;
    undo_ = before;
    getMixerEngine().notifyStateChanged();
    return true;
}

bool MixerPresets::undo()
{
    if (undo_.empty() || !apply(undo_)) return false;
    undo_.clear();
    getMixerEngine().notifyStateChanged();
    return true;
}

MixerPresets& getMixerPresets()
{
    static MixerPresets presets;
    return presets;
}

} // namespace crosspad
