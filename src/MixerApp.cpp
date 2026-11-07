/**
 * @file MixerApp.cpp
 * @brief Mixer and signal router, sized for a finger.
 *
 * Five tabs along the top, the section under them, the soft keys along the
 * bottom.
 *
 *   IN       what comes into the board: the mics, the line-in sockets, USB
 *            audio from the host. A strip per input: its LEVEL (or PAN), its
 *            gain under the name -- the name opens it to set --, MON sends it
 *            to every output you hear, and an input with a choice of sockets
 *            has a strip per socket and a key that listens to it. Where the
 *            board has more inputs than ADCs, PAIR picks the two it hears; the
 *            input left out stays on the desk, dimmed.
 *   SOURCES  what the board plays: LEVEL or PAN, M and S.
 *   OUT      the masters -- phones, line out, USB -- and a speaker or jack
 *            the hardware gives a level of its own: LEVEL, M, and LIM lit
 *            while the limiter works.
 *   ROUTE    where every source and input goes: a row per channel, a column
 *            per output named over it in its colour, and in each cell the
 *            send's slider and level ("off" at the bottom); CLEAR and RESET
 *            (everything to what is heard).
 *   PRESETS  the whole desk saved under a name, loaded back; UNDO puts back
 *            what a load replaced.
 *
 * A strip, as on a desk: the name and the line under it (a tap opens the rest
 * of the channel: an input's gain, its sends, monitor, pan -- an output's
 * tone), the fader up the strip with the meter beside its track (across the
 * strip on the PAN page), the value, two keys. The encoder turns the focused
 * strip (on ROUTE the framed send) and clicks to the next one, past the last
 * to the next section.
 *
 * The engine is the only state: strips are painted from it every tick, so
 * anything else that moves the mix (HIL, a loaded state) shows up here. On
 * close the app asks the engine to persist (notifyStateChanged). The pads
 * keep playing the kit.
 */

#include "crosspad-mixer/MixerApp.hpp"
#include "crosspad-mixer/AudioMixerEngine.hpp"
#include "crosspad-mixer/MixerPresets.hpp"

#if __has_include("pc_stubs/PcApp.hpp")
  #include "pc_stubs/PcApp.hpp"
#else
  #include "crosspad-gui/app/App.hpp"
#endif

#include "crosspad/app/AppRegistrar.hpp"
#include "crosspad/audio/IInputGain.hpp"
#include "crosspad/audio/IInputPairs.hpp"
#include "crosspad/platform/PlatformServices.hpp"
#include "crosspad/audio/IOutputLevels.hpp"
#include "crosspad-gui/components/ui_toast.h"
#include "crosspad-gui/widgets/gui_helpers.h"

#include "lvgl.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#ifdef ESP_PLATFORM
extern "C" {
  extern lv_indev_t* indev_encoderTop;
}
#define MIXER_USE_ENCODER 1
#endif

namespace {

constexpr int      kMaxRows    = kMixerMaxChannels + kMixerMaxOutputs;
constexpr uint32_t kTickMs     = 50;

const lv_color_t kBg      = lv_color_hex(0x16181B);
const lv_color_t kRowBg   = lv_color_hex(0x20242A);
const lv_color_t kWell    = lv_color_hex(0x1B1E22);
const lv_color_t kPress   = lv_color_hex(0x2C3138);
const lv_color_t kTrack   = lv_color_hex(0x3A3F47);
const lv_color_t kKnob    = lv_color_hex(0xFFFFFF);
const lv_color_t kText    = lv_color_hex(0xE8EAED);
const lv_color_t kDim     = lv_color_hex(0x7B828B);
const lv_color_t kInk     = lv_color_hex(0x06120C);
const lv_color_t kAccent  = lv_color_hex(0xC6FF47);
const lv_color_t kTeal    = lv_color_hex(0x4FE0B0);
const lv_color_t kSky     = lv_color_hex(0x4FC3F7);
const lv_color_t kWave    = lv_color_hex(0x9CD639);
const lv_color_t kMute    = lv_color_hex(0xFF5C5C);
const lv_color_t kSolo    = lv_color_hex(0xFFC857);
const lv_color_t kFocus   = lv_color_hex(0xFFFFFF);
constexpr float    kMeterFloorDb = -60.0f;

enum Page : uint8_t { kIn, kSources, kOut, kRoute, kPresets, kPages };

/// How a channel is named, coloured and filed, by the name it registers under.
struct Look { const char* name; const char* label; uint32_t color; Page page; };
const Look kLooks[] = {
    /* IN, in the order a player reads it: what the computer sends, then the
     * two line sockets, then the mics. The channels are named Mic and Line
     * by the platform -- they used to be IN1 and IN2, and this table was not
     * renamed with them, which quietly filed both onto SOURCES. */
    {"USB",     "USB in",  0xFFC857, kIn},
    {"Line",    "Line in", 0xFF9552, kIn},
    {"Mic",     "Mic",     0xB48CFF, kIn},
    /* The names master's platform gives its inputs (input pairs, 0.8.0),
     * kept beside the ones above: whichever the platform registers is filed. */
    {"IN1",     "Mic",      0xB48CFF, kIn},
    {"IN2",     "Line 1/2", 0xFF9552, kIn},
    {"IN3",     "Line 3/4", 0xFFCF99, kIn},
    {"Sampler", "Sampler", 0xC6FF47, kSources},
    {"Pitched", "Pitched", 0x5AC8FF, kSources},
    {"Click",   "Click",   0xFF7A9A, kSources},
    {"Song",    "Song",    0xF26DF9, kSources},
    {"Preview", "Preview", 0x4FE0B0, kSources},
};

const Look* look_for(const char* name)
{
    for (const auto& l : kLooks) if (std::strcmp(l.name, name) == 0) return &l;
    return nullptr;
}

/// What an output is, under its name, for the outputs a board names as pairs.
const char* output_role(const char* name)
{
    static const struct { const char* name; const char* role; } kRoles[] = {
        {"Out 1/2", "phones"},
        {"Out 3/4", "line out"},
    };
    for (const auto& r : kRoles) if (std::strcmp(r.name, name) == 0) return r.role;
    return "master";
}

/// An output as ROUTE and the sends name it: what it is and its jack
/// ("Phones 1/2", "Line out 3/4"), or its own name ("USB").
void out_label(char* buf, size_t n, uint8_t o)
{
    const char* name = getMixerEngine().getOutputName(o);
    const char* role = output_role(name);
    if (std::strcmp(role, "master") == 0) { std::snprintf(buf, n, "%s", name); return; }
    const char* sp = std::strrchr(name, ' ');
    std::snprintf(buf, n, "%c%s %s", role[0] - 'a' + 'A', role + 1, sp ? sp + 1 : name);
}

/// Each output's colour, the same on ROUTE's columns and on its OUT strip.
lv_color_t out_color(uint8_t o)
{
    const char* role = output_role(getMixerEngine().getOutputName(o));
    if (std::strcmp(role, "phones") == 0) return lv_color_hex(0x4FE0B0);
    if (std::strcmp(role, "line out") == 0) return lv_color_hex(0x4FC3F7);
    return lv_color_hex(0xFFC857);
}

struct Row {
    bool      output = false;
    uint8_t   id = 0;            // channel id, or output id
    /* Which socket this row is, for a channel that has more than one: the
     * board wires both line-in jacks to one converter, so "Line in 1/2" and
     * "Line in 3/4" are two rows over one channel and only the one that is
     * selected is live. -1 for every other row. */
    int8_t    src = -1;
    /* A speaker or a jack the output bus plays through (IOutputLevels index):
     * its fader and mute are the hardware's, its meter the bus's. -1 else. */
    int8_t    level = -1;
    Page      page = kSources;
    /* The widgets while the row's section is on screen, null otherwise. */
    lv_obj_t* box = nullptr;
    lv_obj_t* name = nullptr;
    lv_obj_t* sub = nullptr;     // under the name: pan, the socket, the role
    lv_obj_t* zone = nullptr;    // the fader's touch zone
    lv_obj_t* well = nullptr;    // the fader's track: up the strip, or across it for pan
    lv_obj_t* fill = nullptr;
    lv_obj_t* center = nullptr;  // the pan page's centre line
    lv_obj_t* meter = nullptr;
    lv_obj_t* meterWell = nullptr;
    lv_obj_t* peakMark = nullptr;
    lv_obj_t* knob = nullptr;
    lv_obj_t* val = nullptr;     // under the zone: dB, gain, pan
    lv_obj_t* k[2] = {};
    float     shown = 0.0f;      // meter, decaying
    float     peakHold = 0.0f;
    uint8_t   limitLit = 0;      // ticks the limiter's light stays on
};

Row       s_rows[kMaxRows];
int       s_rowCount = 0;
uint8_t   s_sources[kMixerMaxChannels];   // ROUTE's rows: every channel
int       s_sourceCount = 0;

lv_obj_t* s_root = nullptr;
lv_obj_t* s_tabs[kPages] = {};
Page      s_page = kSources;
lv_timer_t* s_timer = nullptr;

/* The channel window. */
lv_obj_t* s_detail = nullptr;
int       s_detailRow = -1;
lv_obj_t* s_dGain = nullptr;
lv_obj_t* s_dGainLabel = nullptr;
lv_obj_t* s_dPan = nullptr;
lv_obj_t* s_dPanLabel = nullptr;
lv_obj_t* s_dSends[kMixerMaxOutputs] = {};
lv_obj_t* s_dSendLabels[kMixerMaxOutputs] = {};
constexpr int kMaxMonChoices = kMixerMaxOutputs + 2;   // off, each heard output, all
lv_obj_t* s_dMon[kMaxMonChoices] = {};
uint32_t  s_dMonMask[kMaxMonChoices] = {};
int       s_gainInput = -1;          // IInputGain index of the channel in the window
constexpr int kMaxSrcChoices = 4;    // sockets sharing one converter; CrossPad has 2
/* In PSRAM: this board reserves its 64 kB DMA pool after the static data is
 * placed and boots with under 150 bytes of internal RAM to spare, so a new
 * array of pointers here is enough to abort main_task. Touched only from the
 * LVGL thread, never with the cache off. */
#ifdef ESP_PLATFORM
  #include "esp_attr.h"
  EXT_RAM_BSS_ATTR lv_obj_t* s_dSrc[kMaxSrcChoices];
#else
  lv_obj_t* s_dSrc[kMaxSrcChoices] = {};
#endif


#ifdef MIXER_USE_ENCODER
lv_group_t* s_encGroup = nullptr;
lv_group_t* s_prevEncGroup = nullptr;
lv_group_t* s_detailGroup = nullptr;
#endif

crosspad::IInputGain* gains() { return crosspad::getPlatformServices().inputGain; }
crosspad::IInputPairs* input_pairs() { return crosspad::getPlatformServices().inputPairs; }
crosspad::IOutputLevels* output_levels() { return crosspad::getPlatformServices().outputLevels; }

/// Whether mixer channel @p ch hears anything in the input pair: an input
/// the pair leaves out is on the desk, silent.
bool connected(uint8_t ch)
{
    auto* p = input_pairs();
    return !p || p->isConnected(getMixerEngine().getChannelName(ch));
}

/// A row as bright as its channel is live.
void dim(lv_obj_t* o, bool off)
{
    const lv_opa_t opa = off ? LV_OPA_40 : LV_OPA_COVER;
    if (o && lv_obj_get_style_opa(o, LV_PART_MAIN) != opa) lv_obj_set_style_opa(o, opa, 0);
}

/// The input-gain index feeding mixer channel @p ch, or -1.
int gain_input_of(uint8_t ch)
{
    auto* g = gains();
    if (!g) return -1;
    const char* name = getMixerEngine().getChannelName(ch);
    for (int i = 0; i < g->inputs(); ++i) if (std::strcmp(g->channelName(i), name) == 0) return i;
    return -1;
}

// ── monitor: an input's sends to the outputs you hear ─────────────────────

/// The heard outputs, in order; @return how many.
int heard_outputs(uint8_t* out)
{
    auto& m = getMixerEngine();
    int n = 0;
    for (uint8_t o = 0; o < m.numOutputs() && o < kMixerMaxOutputs; ++o) {
        if (m.isOutputHeard(o)) out[n++] = o;
    }
    return n;
}

/// Which heard outputs @p ch is sent to, bit i = the i-th heard output.
uint32_t monitor_mask(uint8_t ch)
{
    uint8_t heard[kMixerMaxOutputs];
    const int n = heard_outputs(heard);
    uint32_t mask = 0;
    for (int i = 0; i < n; ++i) if (getMixerEngine().isRouteEnabled(ch, heard[i])) mask |= 1u << i;
    return mask;
}

/// Send @p ch to exactly the heard outputs in @p mask; a send switched on
/// that was dragged to nothing comes back at unity.
void set_monitor(uint8_t ch, uint32_t mask)
{
    auto& m = getMixerEngine();
    uint8_t heard[kMixerMaxOutputs];
    const int n = heard_outputs(heard);
    for (int i = 0; i < n; ++i) {
        const bool on = mask & (1u << i);
        if (on && m.getRouteVolume(ch, heard[i]) <= 0.0f) m.setRouteVolume(ch, heard[i], 1.0f);
        m.setRouteEnabled(ch, heard[i], on);
    }
}

/// An output as MONITOR names it: the last word of its name ("Out 1/2" is
/// "1/2"), short enough for the button.
void short_output_name(char* buf, size_t n, uint8_t o)
{
    const char* name = getMixerEngine().getOutputName(o);
    const char* sp = std::strrchr(name, ' ');
    std::snprintf(buf, n, "%.5s", sp ? sp + 1 : name);
}

/// What the MON button says for @p mask.
// ── scales and text ────────────────────────────────────────────────────────

void set_text(lv_obj_t* l, const char* t)
{
    if (l && std::strcmp(lv_label_get_text(l), t) != 0) lv_label_set_text(l, t);
}

void db_text(char* buf, size_t n, float gain)
{
    if (gain <= 0.0f) { std::snprintf(buf, n, "-inf"); return; }
    const float db = 20.0f * log10f(gain);
    std::snprintf(buf, n, "%s%.1f", db > 0.05f ? "+" : "", (double)(fabsf(db) < 0.05f ? 0.0f : db));
}

void pan_text(char* buf, size_t n, float pan)
{
    const int p = (int)lroundf(pan * 50.0f);
    if (p == 0) std::snprintf(buf, n, "pan C");
    else        std::snprintf(buf, n, "pan %c%d", p < 0 ? 'L' : 'R', p < 0 ? -p : p);
}

int32_t peak_to_meter(float peak)
{
    if (peak <= 0.0f) return 0;
    const float db = 20.0f * log10f(peak);
    if (db <= kMeterFloorDb) return 0;
    if (db >= 0.0f) return 100;
    return (int32_t)(100.0f * (db - kMeterFloorDb) / -kMeterFloorDb);
}

// ── widgets ────────────────────────────────────────────────────────────────

lv_obj_t* make_box(lv_obj_t* parent)
{
    lv_obj_t* o = lv_obj_create(parent);
    lv_obj_remove_style_all(o);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    return o;
}

lv_obj_t* make_button(lv_obj_t* parent, int32_t w, int32_t h, const char* text, lv_event_cb_t cb, void* ud,
                      const lv_font_t* font = &lv_font_montserrat_14)
{
    lv_obj_t* b = lv_button_create(parent);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kTrack, 0);
    lv_obj_set_style_outline_color(b, kFocus, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 2, LV_STATE_FOCUS_KEY);
    if (cb) lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, kText, 0);
    lv_label_set_text(l, text);
    lv_obj_center(l);
    return b;
}

void paint(lv_obj_t* b, bool on, lv_color_t onColor)
{
    if (!b) return;
    const lv_color_t bg = on ? onColor : kTrack;
    if (!lv_color_eq(lv_obj_get_style_bg_color(b, LV_PART_MAIN), bg)) lv_obj_set_style_bg_color(b, bg, 0);
    lv_obj_t* l = lv_obj_get_child(b, 0);
    const lv_color_t ink = on ? kInk : kText;
    if (l && !lv_color_eq(lv_obj_get_style_text_color(l, LV_PART_MAIN), ink)) lv_obj_set_style_text_color(l, ink, 0);
}

/// A horizontal slider with a knob a finger can take: the track thin, the
/// knob tall, and the touch area beyond both.
lv_obj_t* make_slider(lv_obj_t* parent, int32_t min, int32_t max, lv_color_t fill, lv_event_cb_t cb, void* ud)
{
    lv_obj_t* s = lv_slider_create(parent);
    lv_obj_set_height(s, 8);
    lv_slider_set_range(s, min, max);
    lv_obj_set_style_bg_color(s, kTrack, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(s, 4, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s, fill, LV_PART_INDICATOR);
    lv_obj_set_style_radius(s, 4, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s, kKnob, LV_PART_KNOB);
    lv_obj_set_style_radius(s, 5, LV_PART_KNOB);
    lv_obj_set_style_pad_hor(s, 5, LV_PART_KNOB);
    lv_obj_set_style_pad_ver(s, 8, LV_PART_KNOB);
    lv_obj_set_style_shadow_width(s, 0, LV_PART_KNOB);
    lv_obj_set_style_bg_color(s, kAccent, (lv_style_selector_t)LV_PART_KNOB | LV_STATE_FOCUSED);
    lv_obj_set_style_bg_color(s, kAccent, (lv_style_selector_t)LV_PART_KNOB | LV_STATE_EDITED);
    lv_obj_set_style_outline_width(s, 0, LV_STATE_FOCUS_KEY);
    lv_obj_set_ext_click_area(s, 12);
    if (cb) lv_obj_add_event_cb(s, cb, LV_EVENT_VALUE_CHANGED, ud);
    return s;
}


// ── the channel window: gain, pan, sends ───────────────────────────────────

/* An output's window is its tone: three bands across what that output plays
 * through, which is what evening out a speaker or a pair of headphones takes.
 * The shapes are fixed and the sliders are their gain -- a correction is a
 * few dB in the right place, not a design exercise -- and the fourth band,
 * and any shape at all, is there over CDC (MIX_EQ) for the fine work. */
constexpr int kToneBands = 3;
const struct ToneBand {
    const char*      label;
    crosspad::EqKind kind;
    float            hz;
    float            q;
} kTone[kToneBands] = {
    {"Low",  crosspad::EqKind::LowShelf,   160.0f, 0.70f},
    {"Mid",  crosspad::EqKind::Peak,      1200.0f, 1.00f},
    {"High", crosspad::EqKind::HighShelf, 5000.0f, 0.70f},
};

lv_obj_t* s_dEq[kToneBands] = {};
lv_obj_t* s_dEqLabel[kToneBands] = {};

void detail_labels()
{
    if (s_detailRow < 0 || !s_detail) return;
    const Row& r = s_rows[s_detailRow];
    auto& m = getMixerEngine();
    char buf[48];
    if (r.output) {
        /* "Low   +3 dB", and the shape under the name of the band. */
        const crosspad::EqSettings eq = m.getOutputEq(r.id);
        for (int b = 0; b < kToneBands; ++b) {
            if (!s_dEqLabel[b]) continue;
            const int db = (int)lroundf(eq.band[b].gainDb);
            const float hz = kTone[b].hz;
            if (hz >= 1000.0f) {
                std::snprintf(buf, sizeof(buf), "%-5s %+3d dB   %.1f kHz", kTone[b].label, db,
                              (double)(hz / 1000.0f));
            } else {
                std::snprintf(buf, sizeof(buf), "%-5s %+3d dB   %d Hz", kTone[b].label, db, (int)hz);
            }
            set_text(s_dEqLabel[b], buf);
        }
        return;
    }
    if (s_dGainLabel && gains() && s_gainInput >= 0) {
        const float db = gains()->gainDb(s_gainInput);
        std::snprintf(buf, sizeof(buf), "Gain  %+.0f dB", (double)db);
        set_text(s_dGainLabel, buf);
    }
    if (s_dPanLabel) {
        pan_text(buf, sizeof(buf), m.getChannelPan(r.id));
        buf[0] = 'P';
        set_text(s_dPanLabel, buf);
    }
    const uint32_t mon = monitor_mask(r.id);
    for (int i = 0; i < kMaxMonChoices; ++i) paint(s_dMon[i], s_dMon[i] && s_dMonMask[i] == mon, kAccent);
    for (int o = 0; o < m.numOutputs() && o < kMixerMaxOutputs; ++o) {
        if (!s_dSendLabels[o]) continue;
        char out[24];
        out_label(out, sizeof(out), (uint8_t)o);
        if (!m.isRouteEnabled(r.id, o)) std::snprintf(buf, sizeof(buf), "%s  off", out);
        else std::snprintf(buf, sizeof(buf), "%s  %d", out, (int)lroundf(m.getRouteVolume(r.id, o) * 100.0f));
        set_text(s_dSendLabels[o], buf);
    }
}

void gain_cb(lv_event_t*)
{
    if (!gains() || s_gainInput < 0 || !s_dGain) return;
    gains()->setGainDb(s_gainInput, (float)lv_slider_get_value(s_dGain) * gains()->stepDb());
    detail_labels();
}

void dpan_cb(lv_event_t*)
{
    if (s_detailRow < 0) return;
    getMixerEngine().setChannelPan(s_rows[s_detailRow].id, (float)lv_slider_get_value(s_dPan) / 50.0f);
    detail_labels();
}

/// A send's level; dragged to zero it is switched off rather than left
/// enabled at nothing, so "off" means no signal and no work.
void set_send(uint8_t ch, int o, int32_t v)
{
    auto& m = getMixerEngine();
    m.setRouteEnabled(ch, o, v > 0);
    if (v > 0) m.setRouteVolume(ch, o, (float)v / 100.0f);
}

void dsend_cb(lv_event_t* e)
{
    if (s_detailRow < 0) return;
    const int o = (int)(intptr_t)lv_event_get_user_data(e);
    set_send(s_rows[s_detailRow].id, o, lv_slider_get_value(s_dSends[o]));
    detail_labels();
}


void eq_band_cb(lv_event_t* e)
{
    const int b = (int)(intptr_t)lv_event_get_user_data(e);
    if (s_detailRow < 0 || b < 0 || b >= kToneBands || !s_dEq[b]) return;
    auto& m = getMixerEngine();
    const uint8_t out = s_rows[s_detailRow].id;
    crosspad::EqSettings eq = m.getOutputEq(out);
    crosspad::EqBand& band = eq.band[b];
    band.kind   = kTone[b].kind;
    band.hz     = kTone[b].hz;
    band.q      = kTone[b].q;
    band.gainDb = (float)lv_slider_get_value(s_dEq[b]);
    eq.on = true;
    m.setOutputEq(out, eq);
    detail_labels();
}

void eq_flat_cb(lv_event_t*)
{
    if (s_detailRow < 0) return;
    auto& m = getMixerEngine();
    const uint8_t out = s_rows[s_detailRow].id;
    crosspad::EqSettings eq = m.getOutputEq(out);
    for (int b = 0; b < kToneBands; ++b) {
        eq.band[b].gainDb = 0.0f;
        if (s_dEq[b]) lv_slider_set_value(s_dEq[b], 0, LV_ANIM_OFF);
    }
    m.setOutputEq(out, eq);
    detail_labels();
}

void close_detail()
{
    if (!s_detail) return;
#ifdef MIXER_USE_ENCODER
    if (indev_encoderTop && s_encGroup) lv_indev_set_group(indev_encoderTop, s_encGroup);
    lv_group_delete_deferred(s_detailGroup);      // Done's click is still being read
    s_detailGroup = nullptr;
#endif
    lv_obj_delete_async(s_detail);
    s_detail = s_dGain = s_dGainLabel = s_dPan = s_dPanLabel = nullptr;
    for (auto& o : s_dEq) o = nullptr;
    for (auto& o : s_dEqLabel) o = nullptr;
    for (auto& s : s_dSends) s = nullptr;
    for (auto& s : s_dSendLabels) s = nullptr;
    for (auto& s : s_dMon) s = nullptr;
    s_detailRow = -1;
    s_gainInput = -1;
    for (auto& b : s_dSrc) b = nullptr;
}

lv_obj_t* detail_line(const char* text, lv_obj_t** label)
{
    lv_obj_t* l = lv_label_create(s_detail);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(l, kText, 0);
    lv_label_set_text(l, text);
    if (label) *label = l;
    return l;
}

lv_obj_t* detail_slider(int32_t min, int32_t max, int32_t v, lv_color_t fill, lv_event_cb_t cb, void* ud)
{
    lv_obj_t* s = make_slider(s_detail, min, max, fill, cb, ud);
    lv_obj_set_width(s, LV_PCT(94));
    lv_slider_set_value(s, v, LV_ANIM_OFF);
#ifdef MIXER_USE_ENCODER
    if (s_detailGroup) lv_group_add_obj(s_detailGroup, s);
#endif
    return s;
}

void open_detail(int idx)
{
    if (s_detail || idx < 0 || idx >= s_rowCount) return;
    /* An output opens too, for its tone -- but only one that can carry an
     * EQ (the ones a board plays through). */
    if (s_rows[idx].output && s_rows[idx].id >= kMixerEqOutputs) return;
    const Row& r = s_rows[idx];
    auto& m = getMixerEngine();
    const Look* look = look_for(m.getChannelName(r.id));
    const lv_color_t color = look ? lv_color_hex(look->color) : kAccent;
    s_detailRow = idx;
    s_gainInput = gain_input_of(r.id);

    s_detail = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_detail);
    lv_obj_set_size(s_detail, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(s_detail, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_bg_color(s_detail, kBg, 0);
    lv_obj_set_style_bg_opa(s_detail, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_detail, 8, 0);
    lv_obj_set_style_pad_row(s_detail, 8, 0);
    lv_obj_set_flex_flow(s_detail, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_detail, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(s_detail, LV_OBJ_FLAG_SCROLLABLE);
#ifdef MIXER_USE_ENCODER
    s_detailGroup = lv_group_create();
    if (indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_detailGroup);
#endif

    lv_obj_t* title = lv_label_create(s_detail);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(title, r.output ? kAccent : color, 0);
    lv_label_set_text(title, r.output ? m.getOutputName(r.id)
                                      : (look ? look->label : m.getChannelName(r.id)));

    if (r.output) {
        lv_obj_t* hint = lv_label_create(s_detail);
        lv_obj_set_width(hint, LV_PCT(94));
        lv_label_set_long_mode(hint, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_font(hint, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(hint, kDim, 0);
        lv_label_set_text_fmt(hint, "tone of the %s   .   160 Hz shelf, 1.2 kHz, 5 kHz shelf",
                              output_role(m.getOutputName(r.id)));
        const crosspad::EqSettings eq = m.getOutputEq(r.id);
        for (int b = 0; b < kToneBands; ++b) {
            detail_line("", &s_dEqLabel[b]);
            s_dEq[b] = detail_slider(-12, 12, (int32_t)lroundf(eq.band[b].gainDb),
                                     kAccent, eq_band_cb, (void*)(intptr_t)b);
            lv_slider_set_mode(s_dEq[b], LV_SLIDER_MODE_SYMMETRICAL);
        }
        lv_obj_t* flat = make_button(s_detail, 96, 26, "FLAT", eq_flat_cb, nullptr,
                                     &lv_font_montserrat_10);
        lv_obj_t* done = make_button(s_detail, 120, 28, LV_SYMBOL_OK " Done",
                                     [](lv_event_t*) { close_detail(); }, nullptr);
        lv_obj_set_style_bg_color(done, kAccent, 0);
        lv_obj_set_style_text_color(lv_obj_get_child(done, 0), kInk, 0);
#ifdef MIXER_USE_ENCODER
        if (s_detailGroup) {
            lv_group_add_obj(s_detailGroup, flat);
            lv_group_add_obj(s_detailGroup, done);
            lv_group_focus_obj(s_dEq[0]);
        }
#endif
        detail_labels();
        return;
    }

    if (s_gainInput >= 0 && gains()->sources(s_gainInput) >= 2) {
        /* Two sockets, one converter: the board can only listen to one of
         * them, so this is a switch rather than a second channel. Changing it
         * is heard at once -- the routing re-applies the claim. */
        auto* g = gains();
        detail_line("Source", nullptr);
        lv_obj_t* seg = make_box(s_detail);
        lv_obj_set_size(seg, LV_PCT(94), 28);
        lv_obj_set_flex_flow(seg, LV_FLEX_FLOW_ROW);
        lv_obj_set_style_pad_column(seg, 4, 0);
        const int cur = g->source(s_gainInput);
        for (int i = 0; i < g->sources(s_gainInput) && i < kMaxSrcChoices; ++i) {
            lv_obj_t* b = make_button(seg, 10, 28, g->sourceName(s_gainInput, i), [](lv_event_t* e) {
                const int k = (int)(intptr_t)lv_event_get_user_data(e);
                if (s_gainInput < 0 || !gains()) return;
                gains()->setSource(s_gainInput, k);
                for (int j = 0; j < kMaxSrcChoices; ++j) {
                    if (!s_dSrc[j]) continue;
                    const bool on = j == gains()->source(s_gainInput);
                    lv_obj_set_style_bg_color(s_dSrc[j], on ? kAccent : kRowBg, 0);
                    lv_obj_set_style_text_color(lv_obj_get_child(s_dSrc[j], 0), on ? kInk : kText, 0);
                }
            }, (void*)(intptr_t)i);
            lv_obj_set_flex_grow(b, 1);
            if (i == cur) {
                lv_obj_set_style_bg_color(b, kAccent, 0);
                lv_obj_set_style_text_color(lv_obj_get_child(b, 0), kInk, 0);
            }
#ifdef MIXER_USE_ENCODER
            if (s_detailGroup) lv_group_add_obj(s_detailGroup, b);
#endif
            s_dSrc[i] = b;
        }
    }

    if (s_gainInput >= 0) {
        auto* g = gains();
        detail_line("", &s_dGainLabel);
        const int32_t lo = (int32_t)lroundf(g->minDbOf(s_gainInput) / g->stepDb());
        const int32_t hi = (int32_t)lroundf(g->maxDbOf(s_gainInput) / g->stepDb());
        s_dGain = detail_slider(lo, hi, (int32_t)lroundf(g->gainDb(s_gainInput) / g->stepDb()), color, gain_cb, nullptr);
    }
    if (look && look->page == kIn) {
        /* Where it is heard, chosen outright: off, each output, all of them. */
        uint8_t heard[kMixerMaxOutputs];
        const int n = heard_outputs(heard);
        detail_line("Monitor", nullptr);
        lv_obj_t* seg = make_box(s_detail);
        lv_obj_set_size(seg, LV_PCT(94), 28);
        lv_obj_set_flex_flow(seg, LV_FLEX_FLOW_ROW);
        lv_obj_set_style_pad_column(seg, 4, 0);
        int k = 0;
        auto choice = [&](const char* text, uint32_t mask) {
            lv_obj_t* b = make_button(seg, 10, 28, text, [](lv_event_t* e) {
                const int i = (int)(intptr_t)lv_event_get_user_data(e);
                if (s_detailRow < 0) return;
                set_monitor(s_rows[s_detailRow].id, s_dMonMask[i]);
                detail_labels();
            }, (void*)(intptr_t)k);
            lv_obj_set_flex_grow(b, 1);
#ifdef MIXER_USE_ENCODER
            if (s_detailGroup) lv_group_add_obj(s_detailGroup, b);
#endif
            s_dMon[k] = b;
            s_dMonMask[k++] = mask;
        };
        choice("off", 0);
        for (int i = 0; i < n && k < kMaxMonChoices - 1; ++i) {
            char name[8];
            short_output_name(name, sizeof(name), heard[i]);
            choice(name, 1u << i);
        }
        if (n > 1) choice(n == 2 ? "both" : "all", (1u << n) - 1);
    }
    detail_line("", &s_dPanLabel);
    s_dPan = detail_slider(-50, 50, (int32_t)lroundf(m.getChannelPan(r.id) * 50.0f), color, dpan_cb, nullptr);
    lv_slider_set_mode(s_dPan, LV_SLIDER_MODE_SYMMETRICAL);
    for (int o = 0; o < m.numOutputs() && o < kMixerMaxOutputs; ++o) {
        detail_line("", &s_dSendLabels[o]);
        const int32_t v = m.isRouteEnabled(r.id, o) ? (int32_t)lroundf(m.getRouteVolume(r.id, o) * 100.0f) : 0;
        s_dSends[o] = detail_slider(0, 100, v, color, dsend_cb, (void*)(intptr_t)o);
    }
    lv_obj_t* done = make_button(s_detail, 120, 28, LV_SYMBOL_OK " Done", [](lv_event_t*) { close_detail(); }, nullptr);
    lv_obj_set_style_bg_color(done, kAccent, 0);
    lv_obj_set_style_text_color(lv_obj_get_child(done, 0), kInk, 0);
#ifdef MIXER_USE_ENCODER
    if (s_detailGroup) {
        lv_group_add_obj(s_detailGroup, done);
        lv_group_focus_obj(s_dGain ? s_dGain : s_dPan);
    }
#endif
    detail_labels();
}

/// Which socket a row's channel is listening to, or -1 where there is no choice.
int live_src_of(uint8_t ch)
{
    const int gi = gain_input_of(ch);
    if (gi < 0 || !gains() || gains()->sources(gi) < 2) return -1;
    return gains()->source(gi);
}

/// A tap on a row's name: one converter, two sockets -- the one that is not
/// live is made live rather than opening a channel the row does not really
/// own yet; any other row opens the rest of the channel.
void name_cb_idx(int idx)
{
    if (idx < 0 || idx >= s_rowCount) return;
    const Row& r = s_rows[idx];
    if (r.src >= 0 && live_src_of(r.id) != r.src) {
        if (gains()) gains()->setSource(gain_input_of(r.id), r.src);
        return;
    }
    open_detail(idx);
}

// ── presets: the name window ───────────────────────────────────────────────

lv_obj_t* s_nameOv = nullptr;          // the name window a SAVE opens
lv_obj_t* s_nameText = nullptr;
int       s_nameSlot = -1;
#ifdef MIXER_USE_ENCODER
lv_group_t* s_nameGroup = nullptr;
#endif

void build_section();

void preset_toast(const char* fmt, const std::string& name)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), fmt, name.c_str());
    ui_toast_show(buf);
}

void close_name()
{
    if (!s_nameOv) return;
#ifdef MIXER_USE_ENCODER
    if (indev_encoderTop && s_encGroup) lv_indev_set_group(indev_encoderTop, s_encGroup);
    lv_group_delete_deferred(s_nameGroup);       // the keyboard's key is still being read
    s_nameGroup = nullptr;
#endif
    lv_obj_delete_async(s_nameOv);
    s_nameOv = s_nameText = nullptr;
    s_nameSlot = -1;
}

void name_done_cb(lv_event_t* e)
{
    const bool ok = lv_event_get_code(e) == LV_EVENT_READY;
    const std::string name = s_nameText ? lv_textarea_get_text(s_nameText) : "";
    const int s = s_nameSlot;
    close_name();
    if (!ok || s < 0) return;
    if (name.empty()) { ui_toast_show("A preset needs a name"); return; }
    if (crosspad::getMixerPresets().save(s, name)) preset_toast("The desk is saved as %s", name);
    else                                           preset_toast("%s not saved: is the card in?", name);
    build_section();
}

/// SAVE AS asks for the name, the slot's own filled in: OK saves, the name
/// window is the "are you sure" too.
void open_name(int slot)
{
    if (s_nameOv) return;
    s_nameSlot = slot;
    s_nameOv = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_nameOv);
    lv_obj_set_size(s_nameOv, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(s_nameOv, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_bg_color(s_nameOv, kBg, 0);
    lv_obj_set_style_bg_opa(s_nameOv, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_nameOv, 4, 0);
    lv_obj_set_style_pad_row(s_nameOv, 4, 0);
    lv_obj_set_flex_flow(s_nameOv, LV_FLEX_FLOW_COLUMN);
    lv_obj_t* t = lv_label_create(s_nameOv);
    lv_obj_set_style_text_color(t, kText, 0);
    lv_label_set_text(t, "Save the desk as");
    s_nameText = lv_textarea_create(s_nameOv);
    lv_textarea_set_one_line(s_nameText, true);
    lv_textarea_set_max_length(s_nameText, crosspad::MixerPresets::kMaxName);
    lv_textarea_set_text(s_nameText, crosspad::getMixerPresets().info(s_nameSlot).name.c_str());
    lv_obj_set_width(s_nameText, LV_PCT(100));
    lv_obj_t* kb = lv_keyboard_create(s_nameOv);
    lv_obj_set_width(kb, LV_PCT(100));
    lv_obj_set_flex_grow(kb, 1);
    lv_keyboard_set_textarea(kb, s_nameText);
    lv_obj_add_event_cb(kb, name_done_cb, LV_EVENT_READY, nullptr);
    lv_obj_add_event_cb(kb, name_done_cb, LV_EVENT_CANCEL, nullptr);
#ifdef MIXER_USE_ENCODER
    s_nameGroup = lv_group_create();
    lv_group_add_obj(s_nameGroup, kb);
    if (indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_nameGroup);
    lv_group_focus_obj(kb);
    lv_group_set_editing(s_nameGroup, true);
#endif
}

bool back_handler(lv_obj_t*)
{
    if (s_nameOv) { close_name(); return true; }
    if (s_detail) { close_detail(); return true; }
    return false;
}

// ── the desk: tabs, strips, soft keys ──────────────────────────────────────
//
// 320 x 200 under the transport bar: the five tabs on top (59 x 26), the
// section under them (312 x 134), three or four soft keys along the bottom
// (28 tall). IN, SOURCES and OUT are channel strips side by side (60 wide,
// five to a screen, scrolling sideways); ROUTE and PRESETS are rows.

constexpr int32_t kTabY  = 2,   kTabW = 59, kTabH = 26, kTabStep = 63;
constexpr int32_t kListY = 32,  kListH = 134;
constexpr int32_t kKeyY  = 170, kKeyH = 28;
constexpr int32_t kRowW  = 312, kRowH = 44, kRowStep = 48;   // PRESETS
/* A strip, top to bottom: the name and the line under it (a tap opens the
 * channel), the fader with its meter beside the track, the value, two keys. */
constexpr int32_t kStripW = 60, kStripStep = 63;
constexpr int32_t kZoneY  = 32, kZoneH = 64;            // the fader's touch zone
constexpr int32_t kTrackX = 22, kMeterX = 32;           // in the zone
constexpr int32_t kPanX   = 4,  kPanW = kStripW - 8;    // the pan page's track
constexpr int32_t kTogY   = kListH - 25, kTogH = 22;
/* ROUTE: the outputs named over their columns, a row per channel, a send
 * slider in every cell. */
constexpr int32_t kHeadH  = 14;
constexpr int32_t kRouteH = 38, kRouteStep = 42, kRouteNameW = 72;
constexpr int     kKeys  = 4;
constexpr int     kVisMax = kMaxRows > kMixerMaxChannels ? kMaxRows : kMixerMaxChannels;

const char* const kTabNames[kPages] = {"IN", "SOURCES", "OUT", "ROUTE", "PRESETS"};

/// What each row's fader zone moves: the level or the pan.
uint8_t s_fpage[kPages] = {};
int     s_vis[kVisMax];            // the rows on screen: s_rows indices, ROUTE's channel ids or preset slots
int     s_nvis = 0;
int     s_focus = 0;               // index into s_vis
int     s_rcol = 0;                // ROUTE: the output column the encoder moves
int     s_psel = 0;                // PRESETS: the slot the keys act on
int     s_loaded = -1;             // PRESETS: the slot last loaded here
lv_obj_t* s_list = nullptr;
lv_obj_t* s_head = nullptr;        // ROUTE: the outputs over their columns
lv_obj_t* s_fade = nullptr;        // a list's bottom edge, rows scrolling on under it
lv_obj_t* s_keys[kKeys] = {};
lv_obj_t* s_visRow[kVisMax] = {};  // each visible row's box
/* ROUTE: each cell, the send's value (child 0) over its slider (child 1). */
lv_obj_t* s_chips[kMixerMaxChannels][kMixerMaxOutputs] = {};
lv_obj_t* s_rowSub[kVisMax] = {};  // ROUTE / PRESETS rows: the line under the name
lv_obj_t* s_rowTag[kVisMax] = {};  // PRESETS: LOADED

bool fader_page(Page p) { return p == kIn || p == kSources || p == kOut; }

// ── values: the zone's 0..1 and what it means ──────────────────────────────

/// Level <-> gain: unity at 78 % of the travel, a cube, so most of the travel
/// is where the ear works and the top is +6.5 dB.
constexpr float kUnityPos = 0.78f;
float pos_to_gain(float k)
{
    if (k <= 0.001f) return 0.0f;
    const float x = k / kUnityPos;
    return x * x * x;
}
float gain_to_pos(float g)
{
    if (g <= 0.0f) return 0.0f;
    const float k = cbrtf(g) * kUnityPos;
    return k > 1.0f ? 1.0f : k;
}

/// Whether row @p r is the socket its converter is not listening to.
bool socket_idle(const Row& r) { return r.src >= 0 && live_src_of(r.id) != r.src; }

/// The row's level, 0..1 along the zone.
float row_level(const Row& r)
{
    auto& m = getMixerEngine();
    if (r.level >= 0) return (float)output_levels()->level(r.level) / 100.0f;
    if (r.output) return gain_to_pos(m.getOutputVolume(r.id));
    return gain_to_pos(m.getChannelVolume(r.id));
}

void set_row_level(const Row& r, float k)
{
    k = k < 0.0f ? 0.0f : (k > 1.0f ? 1.0f : k);
    auto& m = getMixerEngine();
    if (r.level >= 0) { output_levels()->setLevel(r.level, (int)lroundf(k * 100.0f)); return; }
    if (r.output) { m.setOutputVolume(r.id, pos_to_gain(k)); return; }
    m.setChannelVolume(r.id, pos_to_gain(k));
}

/// "+0.0 dB", "80" -- what the level reads under the zone.
void level_text(const Row& r, char* buf, size_t n)
{
    auto& m = getMixerEngine();
    char db[12];
    if (r.level >= 0) { std::snprintf(buf, n, "%d", output_levels()->level(r.level)); return; }
    if (r.output) { db_text(db, sizeof(db), m.getOutputVolume(r.id)); std::snprintf(buf, n, "%s dB", db); return; }
    db_text(db, sizeof(db), m.getChannelVolume(r.id));
    std::snprintf(buf, n, "%s dB", db);
}

/// Rows whose zone has a pan page: channels, not outputs.
bool has_pan(const Row& r) { return !r.output; }

// ── widgets of the desk ────────────────────────────────────────────────────

lv_obj_t* plain(lv_obj_t* p, int32_t x, int32_t y, int32_t w, int32_t h, int32_t radius, lv_color_t c,
                lv_opa_t opa = LV_OPA_COVER)
{
    lv_obj_t* o = make_box(p);
    lv_obj_set_pos(o, x, y);
    lv_obj_set_size(o, w, h);
    lv_obj_set_style_radius(o, radius, 0);
    lv_obj_set_style_bg_color(o, c, 0);
    lv_obj_set_style_bg_opa(o, opa, 0);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_CLICKABLE);
    return o;
}

lv_obj_t* text(lv_obj_t* p, const lv_font_t* f, lv_color_t c, int32_t x, int32_t y, const char* t)
{
    lv_obj_t* l = lv_label_create(p);
    lv_obj_set_style_text_font(l, f, 0);
    lv_obj_set_style_text_color(l, c, 0);
    lv_obj_set_pos(l, x, y);
    lv_label_set_text(l, t ? t : "");
    lv_obj_remove_flag(l, LV_OBJ_FLAG_CLICKABLE);
    return l;
}

/// A key: a panel, pressed a shade lighter; SHORT_CLICKED, so a drag that
/// scrolls the list never toggles anything.
lv_obj_t* key(lv_obj_t* p, int32_t x, int32_t y, int32_t w, int32_t h, const char* t, lv_event_cb_t cb,
              void* ud)
{
    lv_obj_t* b = plain(p, x, y, w, h, 6, kRowBg);
    lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(b, kPress, LV_STATE_PRESSED);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_font(l, (std::strlen(t) > 2 && w < 60) ? &lv_font_montserrat_10 : &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(l, kText, 0);
    lv_label_set_text(l, t);
    lv_obj_center(l);
    if (cb) lv_obj_add_event_cb(b, cb, LV_EVENT_SHORT_CLICKED, ud);
    return b;
}

/// On: the key filled in @p on, ink on it; off: the panel, @p off on it.
void key_on(lv_obj_t* b, bool on, lv_color_t onCol, lv_color_t off = kText)
{
    if (!b) return;
    const lv_color_t bg = on ? onCol : kRowBg;
    if (!lv_color_eq(lv_obj_get_style_bg_color(b, LV_PART_MAIN), bg)) lv_obj_set_style_bg_color(b, bg, 0);
    lv_obj_t* l = lv_obj_get_child(b, 0);
    const lv_color_t ink = on ? kInk : off;
    if (l && !lv_color_eq(lv_obj_get_style_text_color(l, LV_PART_MAIN), ink)) lv_obj_set_style_text_color(l, ink, 0);
}

lv_color_t row_color(const Row& r)
{
    if (r.page == kIn) return kSky;
    if (r.output) return out_color(r.id);
    const Look* look = look_for(getMixerEngine().getChannelName(r.id));
    return look ? lv_color_hex(look->color) : kText;
}

const char* row_name(const Row& r)
{
    auto& m = getMixerEngine();
    if (r.level >= 0) return output_levels()->name(r.level);
    if (r.output) return m.getOutputName(r.id);
    if (r.src >= 0) return gains()->sourceName(gain_input_of(r.id), r.src);
    const Look* look = look_for(m.getChannelName(r.id));
    return look ? look->label : m.getChannelName(r.id);
}

// ── painting ───────────────────────────────────────────────────────────────

void paint_row(int vi);
void paint_keys();
void build_section();

/// A strip: the fader's fill and knob up the track (or the pan across the
/// strip), the value; the meter beside the track on the level page of IN and
/// OUT; the two keys.
void paint_fader_row(int vi)
{
    Row& r = s_rows[s_vis[vi]];
    if (!r.box) return;
    auto& m = getMixerEngine();
    const bool pan = s_fpage[s_page] == 1 && has_pan(r);
    const bool meter = !pan;            // beside the track, on every level page
    const bool idle = socket_idle(r) || !connected(r.id);
    bool muted = false, dimmed = idle;
    if (r.level >= 0) muted = output_levels()->muted(r.level);
    else if (r.output) muted = m.isOutputMuted(r.id);
    else if (s_page == kSources) {
        muted = m.isChannelMuted(r.id);
        dimmed = muted || (m.isAnySoloed() && !m.isChannelSoloed(r.id));
    }
    lv_obj_set_style_border_width(r.box, vi == s_focus ? 2 : 0, 0);
    lv_obj_set_style_text_color(r.name, dimmed ? kDim : kText, 0);

    char buf[24];
    if (pan) {
        const float p = m.getChannelPan(r.id);
        const int32_t mid = kPanX + kPanW / 2, y = kZoneH / 2 - 2;
        const int32_t x = mid + (int32_t)lroundf(p * (float)kPanW * 0.5f);
        lv_obj_set_pos(r.well, kPanX, y);
        lv_obj_set_size(r.well, kPanW, 4);
        lv_obj_remove_flag(r.center, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(r.fill, x < mid ? x : mid, y);
        lv_obj_set_size(r.fill, x < mid ? mid - x : x - mid, 4);
        lv_obj_set_size(r.knob, 10, 26);
        lv_obj_set_pos(r.knob, x - 5, kZoneH / 2 - 13);
        pan_text(buf, sizeof(buf), p);
        set_text(r.val, buf + 4);                  // "pan L12" -> "L12"
    } else {
        const int32_t y = kZoneH - (int32_t)lroundf(row_level(r) * (float)kZoneH);
        lv_obj_set_pos(r.well, kTrackX, 0);
        lv_obj_set_size(r.well, 4, kZoneH);
        lv_obj_add_flag(r.center, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(r.fill, kTrackX, y);
        lv_obj_set_size(r.fill, 4, kZoneH - y);
        lv_obj_set_size(r.knob, 30, 10);
        lv_obj_set_pos(r.knob, kTrackX + 2 - 15, y - 5);
        level_text(r, buf, sizeof(buf));
        set_text(r.val, buf);
    }
    lv_obj_set_style_bg_color(r.fill, dimmed ? kDim : row_color(r), 0);
    /* The knob lights while a finger or the encoder has hold of it. */
    bool held = lv_obj_has_state(r.zone, LV_STATE_PRESSED);
#ifdef MIXER_USE_ENCODER
    held = held || (s_encGroup && lv_group_get_editing(s_encGroup) && lv_group_get_focused(s_encGroup) == r.zone);
#endif
    const lv_color_t knob = held ? kAccent : kKnob;
    if (!lv_color_eq(lv_obj_get_style_bg_color(r.knob, LV_PART_MAIN), knob)) lv_obj_set_style_bg_color(r.knob, knob, 0);
    for (lv_obj_t* o : {r.meter, r.meterWell, r.peakMark}) {
        if (meter) lv_obj_remove_flag(o, LV_OBJ_FLAG_HIDDEN);
        else       lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
    }

    /* The line under the name: what the row is. */
    if (r.level >= 0)                     set_text(r.sub, m.getOutputName(r.id));
    else if (r.output)                    set_text(r.sub, output_role(m.getOutputName(r.id)));
    else if (r.src >= 0)                  set_text(r.sub, idle ? "tap name: listen" : "listening");
    else if (!connected(r.id))            set_text(r.sub, "not in pair");
    else if (s_page == kIn && gain_input_of(r.id) >= 0) {
        /* The preamp, set in the window the name opens. */
        std::snprintf(buf, sizeof(buf), "gain %+.0f dB", (double)gains()->gainDb(gain_input_of(r.id)));
        set_text(r.sub, buf);
    }
    else if (s_page == kSources || s_page == kIn) {
        pan_text(buf, sizeof(buf), m.getChannelPan(r.id));
        set_text(r.sub, buf);
    }

    /* The keys: IN MON + its socket, SOURCES M S, OUT M LIM. */
    if (s_page == kIn) {
        key_on(r.k[0], monitor_mask(r.id) != 0, kSky);
        if (r.k[1]) key_on(r.k[1], !idle, kSky);
    } else if (s_page == kSources) {
        key_on(r.k[0], muted, kMute);
        key_on(r.k[1], m.isChannelSoloed(r.id), kSolo);
    } else {
        key_on(r.k[0], muted, kMute);
        key_on(r.k[1], r.limitLit > 0, kSolo);
    }
}

/// A ROUTE row: in every cell the send's slider, and over it "off" or its
/// level in the output's colour; the cell the encoder moves framed.
void paint_route_row(int vi)
{
    lv_obj_t* box = s_visRow[vi];
    if (!box) return;
    auto& m = getMixerEngine();
    const uint8_t ch = (uint8_t)s_vis[vi];
    int n = 0;
    for (uint8_t o = 0; o < m.numOutputs() && o < kMixerMaxOutputs; ++o) {
        lv_obj_t* cell = s_chips[vi][o];
        if (!cell) continue;
        const bool on = m.isRouteEnabled(ch, o);
        const float vol = m.getRouteVolume(ch, o);
        lv_obj_t* val = lv_obj_get_child(cell, 0);
        lv_obj_t* s = lv_obj_get_child(cell, 1);
        const int32_t v = on ? (int32_t)lroundf(vol * 100.0f) : 0;
        if (!lv_obj_has_state(s, LV_STATE_PRESSED) && lv_slider_get_value(s) != v) lv_slider_set_value(s, v, LV_ANIM_OFF);
        char buf[16], db[12];
        if (on) { db_text(db, sizeof(db), vol); std::snprintf(buf, sizeof(buf), "%s dB", db); }
        set_text(val, on ? buf : "off");
        const lv_color_t ink = on ? out_color(o) : kDim;
        if (!lv_color_eq(lv_obj_get_style_text_color(val, LV_PART_MAIN), ink)) lv_obj_set_style_text_color(val, ink, 0);
        lv_obj_set_style_border_width(cell, vi == s_focus && o == s_rcol ? 2 : 0, 0);
        n += on;
    }
    char buf[12];
    std::snprintf(buf, sizeof(buf), "%d out%s", n, n == 1 ? "" : "s");
    set_text(s_rowSub[vi], buf);
    dim(box, !connected(ch));
}

void paint_preset_row(int vi)
{
    lv_obj_t* box = s_visRow[vi];
    if (!box) return;
    const bool sel = vi == s_psel;
    lv_obj_set_style_bg_color(box, sel ? kRowBg : kWell, 0);
    lv_obj_set_style_border_width(box, sel ? 2 : 0, 0);
    if (vi == s_loaded) lv_obj_remove_flag(s_rowTag[vi], LV_OBJ_FLAG_HIDDEN);
    else                lv_obj_add_flag(s_rowTag[vi], LV_OBJ_FLAG_HIDDEN);
}

void paint_row(int vi)
{
    if (vi < 0 || vi >= s_nvis) return;
    if (fader_page(s_page)) paint_fader_row(vi);
    else if (s_page == kRoute) paint_route_row(vi);
    else paint_preset_row(vi);
}

void paint_all()
{
    for (int i = 0; i < s_nvis; ++i) paint_row(i);
    paint_keys();
}

/// The soft keys: what the section's rows do and the actions on the focused one.
const char* key_label(int k, char* buf, size_t n)
{
    auto* p = input_pairs();
    switch (s_page) {
        case kIn: {
            static const char* const L[] = {"LEVEL", "PAN", "RESET"};
            if (k < 3) return L[k];
            if (p && p->pairs() >= 2) { std::snprintf(buf, n, "PAIR %s", p->pairName(p->pair())); return buf; }
            return nullptr;
        }
        case kSources: { static const char* const L[] = {"LEVEL", "PAN", "RESET", nullptr}; return L[k]; }
        case kOut:     { static const char* const L[] = {"LEVEL", "RESET", nullptr, nullptr}; return L[k]; }
        case kRoute:   { static const char* const L[] = {"CLEAR", "RESET", nullptr, nullptr}; return L[k]; }
        default:       { static const char* const L[] = {"LOAD", "SAVE AS", "UNDO", nullptr}; return L[k]; }
    }
}

void paint_keys()
{
    char bufs[kKeys][24];
    const char* labels[kKeys];
    int n = 0;
    for (int k = 0; k < kKeys; ++k) {
        labels[k] = key_label(k, bufs[k], sizeof(bufs[k]));
        if (labels[k]) n = k + 1;
    }
    const int32_t w = (kRowW - (n - 1) * 4) / (n ? n : 1);
    for (int k = 0; k < kKeys; ++k) {
        lv_obj_t* b = s_keys[k];
        if (k >= n) { lv_obj_add_flag(b, LV_OBJ_FLAG_HIDDEN); continue; }
        lv_obj_remove_flag(b, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(b, 4 + k * (w + 4), kKeyY);
        lv_obj_set_width(b, w);
        set_text(lv_obj_get_child(b, 0), labels[k]);
        lv_obj_set_style_text_font(lv_obj_get_child(b, 0),
                                   std::strlen(labels[k]) > 7 ? &lv_font_montserrat_10 : &lv_font_montserrat_14, 0);
        bool on = false;
        lv_color_t col = kTeal;
        if (s_page == kIn || s_page == kSources) on = k < 2 && s_fpage[s_page] == k;
        if (s_page == kOut) on = k == 0;
        if (s_page == kPresets) {
            col = kAccent;
            on = k == 0;
            const bool can = k == 2 ? crosspad::getMixerPresets().canUndo()
                                    : crosspad::getMixerPresets().available();
            lv_obj_set_style_opa(b, can ? LV_OPA_COVER : LV_OPA_40, 0);
        } else {
            lv_obj_set_style_opa(b, LV_OPA_COVER, 0);
        }
        key_on(b, on, col);
    }
    for (int t = 0; t < kPages; ++t) key_on(s_tabs[t], t == s_page, kTeal);
}

// ── input ──────────────────────────────────────────────────────────────────

void focus_row(int vi, bool scroll)
{
    if (vi < 0 || vi >= s_nvis) return;
    s_focus = vi;
    if (s_page == kPresets) s_psel = vi;
    if (scroll && s_visRow[vi]) lv_obj_scroll_to_view(s_visRow[vi], LV_ANIM_ON);
    paint_all();
}

void zone_cb(lv_event_t* e)
{
    /* A finger; the encoder's press and turns are zone_enc_cb's. */
    lv_indev_t* in = lv_indev_active();
    if (!in || lv_indev_get_type(in) != LV_INDEV_TYPE_POINTER) return;
    const int vi = (int)(intptr_t)lv_event_get_user_data(e);
    Row& r = s_rows[s_vis[vi]];
    const lv_event_code_t code = lv_event_get_code(e);
    if (socket_idle(r) || !connected(r.id)) return;
    if (code == LV_EVENT_PRESSED) {
        s_focus = vi;
        paint_all();
    }
    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        paint_row(vi);
        return;
    }
    lv_point_t p;
    lv_indev_get_point(lv_indev_active(), &p);
    lv_area_t a;
    lv_obj_get_coords(r.zone, &a);
    const bool pan = s_fpage[s_page] == 1 && has_pan(r);
    /* The level runs up the strip, the pan across it. */
    float k = pan ? (float)(p.x - a.x1 - kPanX) / (float)kPanW : (float)(a.y1 + kZoneH - p.y) / (float)kZoneH;
    k = k < 0.0f ? 0.0f : (k > 1.0f ? 1.0f : k);
    if (pan) {
        float v = k * 2.0f - 1.0f;
        if (fabsf(v) < 0.05f) v = 0.0f;
        getMixerEngine().setChannelPan(r.id, v);
    } else {
        if (fabsf(k - kUnityPos) < 0.03f && r.level < 0) k = kUnityPos;
        set_row_level(r, k);
    }
    paint_row(vi);
}

void tog_cb(lv_event_t* e)
{
    const uint32_t d = (uint32_t)(uintptr_t)lv_event_get_user_data(e);
    const int vi = d & 0xFF, k = (int)(d >> 8);
    Row& r = s_rows[s_vis[vi]];
    auto& m = getMixerEngine();
    s_focus = vi;
    if (s_page == kIn) {
        if (k == 0) {
            uint8_t heard[kMixerMaxOutputs];
            const int n = heard_outputs(heard);
            set_monitor(r.id, monitor_mask(r.id) ? 0 : (n ? (1u << n) - 1 : 0));
        } else if (r.src >= 0 && gains()) {
            gains()->setSource(gain_input_of(r.id), r.src);
        }
    } else if (s_page == kSources) {
        if (k == 0) m.setChannelMute(r.id, !m.isChannelMuted(r.id));
        else        m.setChannelSolo(r.id, !m.isChannelSoloed(r.id));
    } else if (k == 0) {
        if (r.level >= 0) output_levels()->setMuted(r.level, !output_levels()->muted(r.level));
        else              m.setOutputMute(r.id, !m.isOutputMuted(r.id));
    }
    paint_all();
}

void row_name_cb(lv_event_t* e)
{
    const int vi = (int)(intptr_t)lv_event_get_user_data(e);
    s_focus = vi;
    name_cb_idx(s_vis[vi]);
    paint_all();
}

/// A send slider on ROUTE: dragged, the send's level (to zero is off);
/// pressed, the cell the encoder moves.
void route_cb(lv_event_t* e)
{
    const uint32_t d = (uint32_t)(uintptr_t)lv_event_get_user_data(e);
    const int vi = d & 0xFF;
    const int o = (int)(d >> 8);
    if (vi >= s_nvis || !s_chips[vi][o]) return;
    if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
        set_send((uint8_t)s_vis[vi], o, lv_slider_get_value(lv_obj_get_child(s_chips[vi][o], 1)));
    }
    const int was = s_focus;
    s_focus = vi;
    s_rcol = o;
    if (was != vi) paint_route_row(was);
    paint_route_row(vi);
}

/// ROUTE's encoder: a turn moves the framed send by 5.
void route_turn(int d)
{
    if (s_focus >= s_nvis) return;
    auto& m = getMixerEngine();
    const uint8_t ch = (uint8_t)s_vis[s_focus];
    int32_t v = m.isRouteEnabled(ch, s_rcol) ? (int32_t)lroundf(m.getRouteVolume(ch, s_rcol) * 100.0f) : 0;
    v += d * 5;
    set_send(ch, s_rcol, v < 0 ? 0 : (v > 100 ? 100 : v));
    paint_route_row(s_focus);
}

void preset_row_cb(lv_event_t* e) { focus_row((int)(intptr_t)lv_event_get_user_data(e), false); }

void tab_cb(lv_event_t* e)
{
    s_page = (Page)(intptr_t)lv_event_get_user_data(e);
    s_focus = 0;
    s_rcol = 0;
    build_section();
}

/// RESET on a fader section: the focused row's value on the page shown --
/// unity, or the centre.
void reset_row(const Row& r)
{
    auto& m = getMixerEngine();
    if (s_fpage[s_page] == 1 && has_pan(r)) { m.setChannelPan(r.id, 0.0f); return; }
    if (r.level >= 0) return;                    // the hardware's level has no unity
    if (r.output) { m.setOutputVolume(r.id, 1.0f); return; }
    m.setChannelVolume(r.id, 1.0f);
}

void do_softkey(int k)
{
    auto& m = getMixerEngine();
    switch (s_page) {
        case kIn:
        case kSources:
            if (k < 2) s_fpage[s_page] = (uint8_t)k;
            else if (k == 2 && s_focus < s_nvis) reset_row(s_rows[s_vis[s_focus]]);
            else if (k == 3 && input_pairs()) {
                auto* p = input_pairs();
                p->setPair((p->pair() + 1) % p->pairs());
            }
            break;
        case kOut:
            if (k == 1 && s_focus < s_nvis) reset_row(s_rows[s_vis[s_focus]]);
            break;
        case kRoute:
            if (k == 0 && s_focus < s_nvis) {
                for (uint8_t o = 0; o < m.numOutputs(); ++o) m.setRouteEnabled((uint8_t)s_vis[s_focus], o, false);
            } else if (k == 1) {
                /* Everything to what is heard, at unity; nothing to a
                 * recording bus. */
                for (int i = 0; i < s_nvis; ++i) {
                    const uint8_t ch = (uint8_t)s_vis[i];
                    for (uint8_t o = 0; o < m.numOutputs(); ++o) {
                        const bool on = m.isOutputHeard(o);
                        if (on) m.setRouteVolume(ch, o, 1.0f);
                        m.setRouteEnabled(ch, o, on);
                    }
                }
            }
            break;
        default: {
            auto& pr = crosspad::getMixerPresets();
            const std::string name = pr.info(s_psel).name;
            if (k == 0) {
                if (pr.info(s_psel).kind == crosspad::MixerPresets::Kind::Empty) {
                    ui_toast_show("Nothing saved there yet");
                } else if (pr.load(s_psel)) {
                    s_loaded = s_psel;
                    preset_toast("%s loaded -- UNDO puts the desk back", name);
                } else {
                    preset_toast("%s could not be read", name);
                }
            } else if (k == 1) {
                open_name(s_psel);
            } else if (pr.undo()) {
                s_loaded = -1;
                ui_toast_show("The desk is back as it was");
            }
            build_section();
            return;
        }
    }
    paint_all();
}

void softkey_cb(lv_event_t* e) { do_softkey((int)(intptr_t)lv_event_get_user_data(e)); }

/// The encoder on a fader: a click takes it into editing (the knob lights)
/// and out again; while editing, a turn moves the level -- or the pan on the
/// PAN page.
void zone_enc_cb(lv_event_t* e)
{
    const int vi = (int)(intptr_t)lv_event_get_user_data(e);
    if (vi >= s_nvis) return;
    Row& r = s_rows[s_vis[vi]];
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        lv_indev_t* in = lv_indev_active();
        lv_group_t* g = lv_obj_get_group(r.zone);
        if (!in || lv_indev_get_type(in) != LV_INDEV_TYPE_ENCODER || !g) return;
        lv_group_set_editing(g, !lv_group_get_editing(g));
        paint_row(vi);
        return;
    }
    const uint32_t key = lv_event_get_key(e);
    const int d = (key == LV_KEY_RIGHT || key == LV_KEY_UP) ? 1 : (key == LV_KEY_LEFT || key == LV_KEY_DOWN) ? -1 : 0;
    if (!d || socket_idle(r) || !connected(r.id)) return;
    if (s_fpage[s_page] == 1 && has_pan(r)) {
        float p = getMixerEngine().getChannelPan(r.id) + (float)d * 0.02f;
        getMixerEngine().setChannelPan(r.id, p < -1.0f ? -1.0f : (p > 1.0f ? 1.0f : p));
    } else {
        const int gi = r.page == kIn && !r.output ? gain_input_of(r.id) : -1;
        const float step = r.level >= 0 ? 0.01f : (gi >= 0 ? gains()->stepDb() / gains()->maxDb() : 0.01f);
        set_row_level(r, row_level(r) + (float)d * step);
    }
    paint_row(vi);
}

/// A send slider being edited: a turn moves it by 5, not LVGL's 1.
void route_key_cb(lv_event_t* e)
{
    const uint32_t key = lv_event_get_key(e);
    const int d = (key == LV_KEY_RIGHT || key == LV_KEY_UP) ? 1 : (key == LV_KEY_LEFT || key == LV_KEY_DOWN) ? -1 : 0;
    if (!d) return;
    const uint32_t ud = (uint32_t)(uintptr_t)lv_event_get_user_data(e);
    s_focus = ud & 0xFF;
    s_rcol = (int)(ud >> 8);
    route_turn(d);
    lv_event_stop_processing(e);
}

/// The encoder's focus on anything of a strip or a row frames that strip
/// (on ROUTE, that send).
void focus_cb(lv_event_t* e)
{
    const uint32_t ud = (uint32_t)(uintptr_t)lv_event_get_user_data(e);
    const int vi = ud & 0xFF;
    if (vi >= s_nvis) return;
    s_focus = vi;
    if (s_page == kRoute) s_rcol = (int)(ud >> 8);
    if (s_page == kPresets) s_psel = vi;
    paint_all();
}

/// @p o into the encoder's ring, in the order it is called: the tabs, the
/// section left to right and top to bottom, the soft keys.
void enc_add(lv_obj_t* o, void* focusUd = nullptr, bool framed = false)
{
#ifdef MIXER_USE_ENCODER
    if (!s_encGroup || !o) return;
    lv_group_add_obj(s_encGroup, o);
    lv_obj_set_style_outline_color(o, kFocus, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(o, 2, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_pad(o, 1, LV_STATE_FOCUS_KEY);
    if (framed) lv_obj_add_event_cb(o, focus_cb, LV_EVENT_FOCUSED, focusUd);
#else
    (void)o; (void)focusUd; (void)framed;
#endif
}

// ── building a section ─────────────────────────────────────────────────────

lv_obj_t* row_base(int vi, int32_t h, int32_t step, lv_color_t stripe, const char* name, lv_obj_t** nameOut,
                   lv_obj_t** subOut)
{
    lv_obj_t* b = plain(s_list, 0, vi * step, kRowW, h, 6, kRowBg);
    lv_obj_set_style_clip_corner(b, true, 0);
    lv_obj_set_style_border_color(b, kAccent, 0);
    plain(b, 0, 0, 4, h, 0, stripe);
    lv_obj_t* n = text(b, &lv_font_montserrat_14, kText, 10, 4, name);
    lv_obj_set_width(n, kRouteNameW - 12);
    lv_label_set_long_mode(n, LV_LABEL_LONG_CLIP);
    lv_obj_t* s = text(b, &lv_font_montserrat_10, kDim, 10, h - 16, "");
    lv_obj_set_width(s, kRouteNameW - 12);
    lv_label_set_long_mode(s, LV_LABEL_LONG_CLIP);
    if (nameOut) *nameOut = n;
    if (subOut) *subOut = s;
    s_visRow[vi] = b;
    return b;
}

/// A label across a strip, centred and clipped to it.
lv_obj_t* strip_text(lv_obj_t* b, const lv_font_t* f, lv_color_t c, int32_t y, const char* t)
{
    lv_obj_t* l = text(b, f, c, 2, y, t);
    lv_obj_set_width(l, kStripW - 4);
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(l, LV_LABEL_LONG_CLIP);
    return l;
}

void build_fader_row(int vi, Row& r)
{
    lv_obj_t* b = plain(s_list, vi * kStripStep, 0, kStripW, kListH, 6, kRowBg);
    lv_obj_set_style_clip_corner(b, true, 0);
    lv_obj_set_style_border_color(b, kAccent, 0);
    plain(b, 0, 0, kStripW, 3, 0, row_color(r));
    s_visRow[vi] = b;
    r.box = b;
    const char* name = row_name(r);
    /* 14 px where the name fits the strip, 10 where it does not. */
    /* lv_text_get_size: public with this signature on LVGL 9.3 (board) and
     * 9.4+ (simulator); lv_text_get_width went private in 9.4. */
    lv_point_t nameSize;
    lv_text_get_size(&nameSize, name, &lv_font_montserrat_14, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    const bool fits = nameSize.x <= kStripW - 4;
    r.name = strip_text(b, fits ? &lv_font_montserrat_14 : &lv_font_montserrat_10, kText, 6, name);
    r.sub = strip_text(b, &lv_font_montserrat_10, kDim, 21, "");
    /* The name opens the rest of the channel (its sends, its tone). */
    lv_obj_t* hit = plain(b, 0, 0, kStripW, kZoneY, 0, kRowBg, LV_OPA_TRANSP);
    lv_obj_add_flag(hit, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(hit, row_name_cb, LV_EVENT_SHORT_CLICKED, (void*)(intptr_t)vi);
    enc_add(hit, (void*)(intptr_t)vi, true);

    /* The fader: the level up the track, the meter beside it; a drag on it
     * never scrolls the strips sideways. */
    r.zone = plain(b, 0, kZoneY, kStripW, kZoneH, 0, kRowBg, LV_OPA_TRANSP);
    lv_obj_add_flag(r.zone, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(r.zone, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
    lv_obj_remove_flag(r.zone, LV_OBJ_FLAG_SCROLL_CHAIN_VER);
    for (lv_event_code_t c : {LV_EVENT_PRESSED, LV_EVENT_PRESSING, LV_EVENT_RELEASED, LV_EVENT_PRESS_LOST}) {
        lv_obj_add_event_cb(r.zone, zone_cb, c, (void*)(intptr_t)vi);
    }
    lv_obj_add_event_cb(r.zone, zone_enc_cb, LV_EVENT_CLICKED, (void*)(intptr_t)vi);
    lv_obj_add_event_cb(r.zone, zone_enc_cb, LV_EVENT_KEY, (void*)(intptr_t)vi);
    enc_add(r.zone, (void*)(intptr_t)vi, true);
    r.well = plain(r.zone, kTrackX, 0, 4, kZoneH, 2, kWell);
    r.meterWell = plain(r.zone, kMeterX, 0, 4, kZoneH, 2, kWell);
    r.meter = plain(r.zone, kMeterX, kZoneH, 4, 0, 2, kMute);
    lv_obj_set_style_bg_grad_color(r.meter, kWave, 0);
    lv_obj_set_style_bg_grad_dir(r.meter, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_main_stop(r.meter, 0, 0);
    lv_obj_set_style_bg_grad_stop(r.meter, 85, 0);
    r.peakMark = plain(r.zone, kMeterX - 2, kZoneH, 8, 2, 0, kText);
    r.center = plain(r.zone, kStripW / 2 - 1, kZoneH / 2 - 7, 2, 14, 1, kDim);
    r.fill = plain(r.zone, kTrackX, kZoneH, 4, 0, 2, row_color(r));
    r.knob = plain(r.zone, kTrackX + 2 - 15, kZoneH - 5, 30, 10, 3, kKnob);
    lv_obj_set_style_shadow_width(r.knob, 3, 0);
    lv_obj_set_style_shadow_opa(r.knob, LV_OPA_50, 0);
    lv_obj_set_style_shadow_offset_y(r.knob, 1, 0);
    r.val = strip_text(b, &lv_font_montserrat_10, kDim, kZoneY + kZoneH + 1, "");

    /* IN: MON, and the socket where the input has a choice of them;
     * SOURCES: M S; OUT: M and the limiter's light. */
    const char* k0 = s_page == kIn ? "MON" : "M";
    const char* k1 = s_page == kSources ? "S" : (s_page == kOut ? "LIM" : nullptr);
    if (s_page == kIn && r.src >= 0) k1 = gains()->sourceName(gain_input_of(r.id), r.src);
    const int32_t kw = k1 ? (kStripW - 6) / 2 : kStripW - 4;
    r.k[0] = key(b, 2, kTogY, kw, kTogH, k0, tog_cb, (void*)(uintptr_t)(vi | (0 << 8)));
    r.k[1] = k1 ? key(b, 4 + kw, kTogY, kw, kTogH, k1, s_page == kOut ? nullptr : tog_cb,
                      (void*)(uintptr_t)(vi | (1 << 8)))
                : nullptr;
    enc_add(r.k[0], (void*)(intptr_t)vi, true);
    if (r.k[1] && s_page != kOut) enc_add(r.k[1], (void*)(intptr_t)vi, true);
}

void build_route_row(int vi, uint8_t ch)
{
    auto& m = getMixerEngine();
    const Look* look = look_for(m.getChannelName(ch));
    const lv_color_t col = look && look->page == kIn ? kSky : (look ? lv_color_hex(look->color) : kText);
    lv_obj_t* b = row_base(vi, kRouteH, kRouteStep, col, look ? look->label : m.getChannelName(ch), nullptr,
                           &s_rowSub[vi]);
    const int n = m.numOutputs() < kMixerMaxOutputs ? m.numOutputs() : kMixerMaxOutputs;
    const int32_t w = (kRowW - kRouteNameW - 4 - (n - 1) * 4) / (n ? n : 1);
    for (int o = 0; o < n; ++o) {
        lv_obj_t* cell = plain(b, kRouteNameW + o * (w + 4), 3, w, kRouteH - 6, 4, kWell);
        lv_obj_set_style_border_color(cell, out_color((uint8_t)o), 0);
        text(cell, &lv_font_montserrat_10, kDim, 5, 1, "off");
        void* ud = (void*)(uintptr_t)(vi | (o << 8));
        lv_obj_t* s = make_slider(cell, 0, 100, out_color((uint8_t)o), route_cb, ud);
        lv_obj_set_pos(s, 6, 16);
        lv_obj_set_width(s, w - 12);
        lv_obj_add_event_cb(s, route_cb, LV_EVENT_PRESSED, ud);
        lv_obj_add_event_cb(s, route_key_cb, (lv_event_code_t)(LV_EVENT_KEY | LV_EVENT_PREPROCESS), ud);
        enc_add(s, ud, true);
        s_chips[vi][o] = cell;
    }
}

/// ROUTE's head: every output's name over its column, in its colour.
void build_route_head()
{
    lv_obj_clean(s_head);
    auto& m = getMixerEngine();
    const int n = m.numOutputs() < kMixerMaxOutputs ? m.numOutputs() : kMixerMaxOutputs;
    const int32_t w = (kRowW - kRouteNameW - 4 - (n - 1) * 4) / (n ? n : 1);
    text(s_head, &lv_font_montserrat_10, kDim, 10, 0, "SEND TO");
    for (int o = 0; o < n; ++o) {
        char name[24];
        out_label(name, sizeof(name), (uint8_t)o);
        lv_obj_t* l = text(s_head, &lv_font_montserrat_10, out_color((uint8_t)o), kRouteNameW + o * (w + 4), 0, name);
        lv_obj_set_width(l, w);
        lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_long_mode(l, LV_LABEL_LONG_CLIP);
    }
}

void build_preset_row(int vi)
{
    const auto& i = crosspad::getMixerPresets().info(vi);
    lv_obj_t* b = plain(s_list, 0, vi * kRowStep, kRowW, kRowH, 6, kWell);
    lv_obj_set_style_border_color(b, kTeal, 0);
    lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(b, preset_row_cb, LV_EVENT_SHORT_CLICKED, (void*)(intptr_t)vi);
    enc_add(b, (void*)(intptr_t)vi, true);
    plain(b, 4, 12, 3, 20, 2, kAccent, i.kind == crosspad::MixerPresets::Kind::Saved ? LV_OPA_COVER : LV_OPA_30);
    text(b, &lv_font_montserrat_16, kText, 14, 6, i.name.c_str());
    s_rowSub[vi] = text(b, &lv_font_montserrat_10, kDim, 14, 27,
                        i.kind == crosspad::MixerPresets::Kind::Saved   ? "saved"
                        : i.kind == crosspad::MixerPresets::Kind::Factory ? "starting point"
                                                                          : "empty -- SAVE AS puts the desk here");
    lv_obj_set_width(s_rowSub[vi], 230);
    lv_label_set_long_mode(s_rowSub[vi], LV_LABEL_LONG_DOT);
    s_rowTag[vi] = text(b, &lv_font_montserrat_10, kAccent, 0, 0, "LOADED");
    lv_obj_align(s_rowTag[vi], LV_ALIGN_RIGHT_MID, -12, 0);
    s_visRow[vi] = b;
}

void build_section()
{
#ifdef MIXER_USE_ENCODER
    /* The ring is rebuilt with the section: the tabs, the section, the keys. */
    if (s_encGroup) {
        lv_group_remove_all_objs(s_encGroup);
        lv_group_set_editing(s_encGroup, false);
    }
#endif
    for (lv_obj_t* t : s_tabs) enc_add(t);
    for (auto& r : s_rows) {
        r.box = r.name = r.sub = r.zone = r.well = r.fill = r.center = r.meter = r.meterWell = r.peakMark = r.knob =
            r.val = nullptr;
        r.k[0] = r.k[1] = nullptr;
    }
    for (auto& v : s_visRow) v = nullptr;
    for (auto& v : s_rowSub) v = nullptr;
    for (auto& v : s_rowTag) v = nullptr;
    for (auto& row : s_chips) for (auto& c : row) c = nullptr;
    lv_obj_clean(s_list);
    s_nvis = 0;
    /* Strips scroll sideways; ROUTE's and PRESETS' rows down, ROUTE's under
     * the head naming its columns. */
    const bool strips = fader_page(s_page);
    const int32_t head = s_page == kRoute ? kHeadH : 0;
    lv_obj_set_pos(s_list, 4, kListY + head);
    lv_obj_set_size(s_list, kRowW, kListH - head);
    lv_obj_set_scroll_dir(s_list, strips ? LV_DIR_HOR : LV_DIR_VER);
    lv_obj_set_scroll_snap_x(s_list, strips ? LV_SCROLL_SNAP_START : LV_SCROLL_SNAP_NONE);
    lv_obj_set_scroll_snap_y(s_list, strips ? LV_SCROLL_SNAP_NONE : LV_SCROLL_SNAP_START);
    lv_obj_scroll_to(s_list, 0, 0, LV_ANIM_OFF);
    if (strips) lv_obj_add_flag(s_fade, LV_OBJ_FLAG_HIDDEN);
    else        lv_obj_remove_flag(s_fade, LV_OBJ_FLAG_HIDDEN);
    if (s_page == kRoute) { lv_obj_remove_flag(s_head, LV_OBJ_FLAG_HIDDEN); build_route_head(); }
    else                  lv_obj_add_flag(s_head, LV_OBJ_FLAG_HIDDEN);

    if (fader_page(s_page)) {
        for (int i = 0; i < s_rowCount && s_nvis < kVisMax; ++i) {
            if (s_rows[i].page != s_page) continue;
            s_vis[s_nvis] = i;
            build_fader_row(s_nvis, s_rows[i]);
            ++s_nvis;
        }
    } else if (s_page == kRoute) {
        /* The sources first, then the inputs. */
        for (Page want : {kSources, kIn}) {
            for (int i = 0; i < s_sourceCount && s_nvis < kVisMax; ++i) {
                const Look* look = look_for(getMixerEngine().getChannelName(s_sources[i]));
                if ((look ? look->page : kSources) != want) continue;
                s_vis[s_nvis] = s_sources[i];
                build_route_row(s_nvis, s_sources[i]);
                ++s_nvis;
            }
        }
    } else {
        auto& pr = crosspad::getMixerPresets();
        pr.refresh();                            // the card is read when the list is shown
        for (int s = 0; s < crosspad::MixerPresets::kSlots && s_nvis < kVisMax; ++s) {
            s_vis[s_nvis] = s;
            build_preset_row(s_nvis);
            ++s_nvis;
        }
        s_focus = s_psel;
    }
    if (s_focus >= s_nvis) s_focus = 0;
    for (lv_obj_t* k : s_keys) enc_add(k);
#ifdef MIXER_USE_ENCODER
    if (s_encGroup) lv_group_focus_obj(s_tabs[s_page]);
#endif
    paint_all();
    if (s_nvis && s_visRow[s_focus]) lv_obj_scroll_to_view(s_visRow[s_focus], LV_ANIM_OFF);
}

// ── the tick ───────────────────────────────────────────────────────────────

/// Meters and the limiter's light, and every value from the engine: anything
/// else that moves the mix (HIL, a loaded desk) shows up here.
void tick_cb(lv_timer_t*)
{
    auto& m = getMixerEngine();
    if (fader_page(s_page)) {
        for (int vi = 0; vi < s_nvis; ++vi) {
            Row& r = s_rows[s_vis[vi]];
            float l = 0.0f, rr = 0.0f;
            if (r.output || r.level >= 0) {
                m.getOutputLevel(r.id, l, rr);
                if (m.takeOutputLimit(r.id) < 0.99f) r.limitLit = 20;   // a second at 50 ms
                else if (r.limitLit) --r.limitLit;
            } else if (!socket_idle(r)) {
                m.getChannelLevel(r.id, l, rr);
            }
            const float peak = l > rr ? l : rr;
            /* Fast up, slow down, the way a meter is read; the peak mark
             * holds and falls slower. */
            r.shown = peak > r.shown ? peak : r.shown * 0.82f;
            r.peakHold = peak > r.peakHold ? peak : r.peakHold * 0.95f;
            if (r.meter && !lv_obj_has_flag(r.meter, LV_OBJ_FLAG_HIDDEN)) {
                const int32_t h = peak_to_meter(r.shown) * kZoneH / 100;
                lv_obj_set_pos(r.meter, kMeterX, kZoneH - h);
                lv_obj_set_height(r.meter, h);
                lv_obj_set_y(r.peakMark, kZoneH - peak_to_meter(r.peakHold) * kZoneH / 100 - 1);
            }
            paint_fader_row(vi);
        }
    } else if (s_page == kRoute) {
        for (int vi = 0; vi < s_nvis; ++vi) paint_route_row(vi);
    }
    if (s_detail) detail_labels();
}

} // namespace

lv_obj_t* Mixer_create(lv_obj_t* parent, App* app)
{
    auto& m = getMixerEngine();

    s_root = lv_CreateEditableButton(parent);
    lv_obj_remove_style_all(s_root);
    lv_obj_set_size(s_root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_root, kBg, 0);
    lv_obj_set_style_bg_opa(s_root, LV_OPA_COVER, 0);
    lv_obj_remove_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);

    for (int t = 0; t < kPages; ++t) {
        s_tabs[t] = key(s_root, 4 + t * kTabStep, kTabY, kTabW, kTabH, kTabNames[t], tab_cb, (void*)(intptr_t)t);
        lv_obj_set_style_text_font(lv_obj_get_child(s_tabs[t], 0),
                                   std::strlen(kTabNames[t]) > 5 ? &lv_font_montserrat_10 : &lv_font_montserrat_14, 0);
    }
    s_list = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_list);
    lv_obj_set_scrollbar_mode(s_list, LV_SCROLLBAR_MODE_OFF);
    s_head = make_box(s_root);
    lv_obj_set_pos(s_head, 4, kListY);
    lv_obj_set_size(s_head, kRowW, kHeadH);
    /* A fade along a list's bottom: rows scroll on under it. */
    s_fade = plain(s_root, 0, kListY + kListH - 14, 320, 14, 0, kBg);
    lv_obj_set_style_bg_main_opa(s_fade, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_grad_color(s_fade, kBg, 0);
    lv_obj_set_style_bg_grad_opa(s_fade, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_grad_dir(s_fade, LV_GRAD_DIR_VER, 0);
    for (int k = 0; k < kKeys; ++k) s_keys[k] = key(s_root, 4, kKeyY, 100, kKeyH, "", softkey_cb, (void*)(intptr_t)k);

    /* The channels in the order kLooks gives them -- the inputs, then the
     * Sampler first among the sources -- not the order they were added in;
     * one the table does not know goes last. */
    s_rowCount = 0;
    s_sourceCount = 0;
    auto add_row = [&](bool output, uint8_t id, Page where, int8_t src, int8_t level) {
        Row& r = s_rows[s_rowCount++];
        r = Row{};
        r.output = output;
        r.id = id;
        r.page = where;
        r.src = src;
        r.level = level;
    };
    auto place = [&](uint8_t ch) {
        const Look* look = look_for(m.getChannelName(ch));
        const Page where = look ? look->page : kSources;
        /* A channel whose input offers a choice of socket gets one row per
         * socket: the board shows both line-in jacks even though only the
         * selected one is live, because "where do I plug in" is the question
         * a mixer should answer without opening anything. */
        const int gi = gain_input_of(ch);
        const int sockets = (gi >= 0 && gains()) ? gains()->sources(gi) : 0;
        if (sockets >= 2) {
            for (int k = 0; k < sockets && s_rowCount < kMaxRows; ++k) add_row(false, ch, where, (int8_t)k, -1);
        } else {
            add_row(false, ch, where, -1, -1);
        }
        s_sources[s_sourceCount++] = ch;
    };
    for (const Look& l : kLooks) {
        for (uint8_t ch = 0; ch < m.maxChannels() && s_rowCount < kMaxRows; ++ch) {
            if (m.isChannelActive(ch) && std::strcmp(m.getChannelName(ch), l.name) == 0) place(ch);
        }
    }
    for (uint8_t ch = 0; ch < m.maxChannels() && s_rowCount < kMaxRows; ++ch) {
        if (m.isChannelActive(ch) && !look_for(m.getChannelName(ch))) place(ch);
    }
    /* A bus the hardware gives levels of its own (the engine plays it at
     * unity) is shown as what it plays through: a row per speaker or jack. */
    crosspad::IOutputLevels* ol = output_levels();
    for (uint8_t o = 0; o < m.numOutputs() && s_rowCount < kMaxRows; ++o) {
        bool hw = false;
        if (ol && m.isOutputUnity(o)) {
            for (int i = 0; i < ol->outputs() && s_rowCount < kMaxRows; ++i) {
                if (std::strcmp(ol->mixerOutput(i), m.getOutputName(o)) != 0) continue;
                add_row(true, o, kOut, -1, (int8_t)i);
                hw = true;
            }
        }
        if (!hw) add_row(true, o, kOut, -1, -1);
    }

#ifdef MIXER_USE_ENCODER
    /* The encoder walks everything on the desk, as everywhere: a turn moves
     * the focus, a click presses a key or takes a fader or a send into
     * editing, where a turn moves it and a click lets go. */
    if (indev_encoderTop) s_prevEncGroup = lv_indev_get_group(indev_encoderTop);
    if (!s_encGroup) s_encGroup = lv_group_create();
#endif
    s_page = kSources;
    s_focus = 0;
    s_psel = 0;
    build_section();
    s_timer = lv_timer_create(tick_cb, kTickMs, nullptr);

#ifdef MIXER_USE_ENCODER
    if (indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_encGroup);
#endif

    if (app) {
        app->setOnBack(back_handler);
        app->setOnShow([](lv_obj_t*) {
            if (s_timer) lv_timer_resume(s_timer);
#ifdef MIXER_USE_ENCODER
            if (indev_encoderTop) {
                lv_indev_set_group(indev_encoderTop, s_nameGroup ? s_nameGroup : (s_detailGroup ? s_detailGroup : s_encGroup));
            }
#endif
        });
        app->setOnHide([](lv_obj_t*) {
            if (s_timer) lv_timer_pause(s_timer);
#ifdef MIXER_USE_ENCODER
            if (indev_encoderTop && s_prevEncGroup) lv_indev_restore_group(indev_encoderTop, s_prevEncGroup);
#endif
        });
    }
    return s_root;
}

void Mixer_destroy(lv_obj_t* app_obj)
{
    if (s_timer) {
        lv_timer_delete(s_timer);
        s_timer = nullptr;
    }
    /* The desk as it is left is the desk the next boot starts with. */
    getMixerEngine().notifyStateChanged();

#ifdef MIXER_USE_ENCODER
    if (s_detailGroup) { lv_group_delete(s_detailGroup); s_detailGroup = nullptr; }
    if (s_nameGroup) { lv_group_delete(s_nameGroup); s_nameGroup = nullptr; }
#endif
    s_nameOv = s_nameText = nullptr;
    s_nameSlot = -1;
    s_detail = s_dGain = s_dGainLabel = s_dPan = s_dPanLabel = nullptr;
    for (auto& o : s_dEq) o = nullptr;
    for (auto& o : s_dEqLabel) o = nullptr;
    for (auto& s : s_dSends) s = nullptr;
    for (auto& s : s_dSendLabels) s = nullptr;
    for (auto& s : s_dMon) s = nullptr;
    s_detailRow = s_gainInput = -1;
    for (auto& r : s_rows) r = Row{};
    for (auto& v : s_visRow) v = nullptr;
    for (auto& v : s_rowSub) v = nullptr;
    for (auto& v : s_rowTag) v = nullptr;
    for (auto& row : s_chips) for (auto& c : row) c = nullptr;
    for (auto& t : s_tabs) t = nullptr;
    for (auto& k : s_keys) k = nullptr;
    s_rowCount = s_sourceCount = s_nvis = 0;
    s_root = s_list = s_head = s_fade = nullptr;

#ifdef MIXER_USE_ENCODER
    if (indev_encoderTop && s_prevEncGroup) lv_indev_restore_group(indev_encoderTop, s_prevEncGroup);
    if (s_encGroup) {
        lv_group_delete(s_encGroup);
        s_encGroup = nullptr;
    }
    s_prevEncGroup = nullptr;
#endif

    if (app_obj) lv_obj_delete_async(app_obj);
}

// No pad logic: the pads keep playing the kit, so the mix is heard as it moves.
// mixer.png is a black glyph: it asks the launcher to tint it.
REGISTER_APP(Mixer, nullptr, "mixer.png",
             Mixer_create, Mixer_destroy,
             nullptr, nullptr, nullptr, 0, true)
