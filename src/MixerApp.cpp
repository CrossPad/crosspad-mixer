/**
 * @file MixerApp.cpp
 * @brief Mixer and signal router, sized for a finger.
 *
 * The categories are tabs along the bottom, one page each:
 *
 *   IN       what comes into the board: the mics, the line-in jack, USB audio
 *            from the host. The mics and the jack carry their input GAIN, and
 *            each input its MONITOR: which of the outputs you hear -- 1/2,
 *            3/4, both or neither (the outputs the engine marks heard; a
 *            recording bus keeps its own send).
 *   SOURCES  what the board plays: the sampler, the pitched instrument, the
 *            click, the song, the recorder's and slicer's preview.
 *   OUT      the jacks' own volume first -- Out 2 (phones and the speakers
 *            beside them) with the speakers' switch, Out 1 (the line out),
 *            what the status bar's drawer had -- then the mix's masters:
 *            speakers, phones, the USB recording bus.
 *   ROUTE    the send matrix: which source goes to which output, how loud.
 *   PRESETS  (the list icon) the whole desk saved under a name, loaded back:
 *            STREAM, BEATS, SAMPLING start as a starting point for their
 *            name until saved over; UNDO puts back what a load replaced.
 *
 * A channel is one row: its name, a long fader in dB with a post-fader meter
 * under it, the dB, mute and solo -- an input has MON where the others have
 * solo. Tap the name for the rest of the channel: input gain, monitor, pan,
 * and its sends.
 *
 * The engine is the only state: widgets are written from it every tick
 * unless a finger or the encoder has hold of them, so anything else that
 * moves the mix (HIL, a loaded state) shows up here. On close the app asks
 * the engine to persist (notifyStateChanged). The pads keep playing the kit.
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
#include "crosspad/platform/PlatformServices.hpp"
#include "crosspad/settings/CrosspadSettings.hpp"
#include "crosspad/settings/ISettingsUI.hpp"
#include "crosspad/stm32/reg_map.h"
#include "crosspad-gui/platform/IGuiPlatform.h"
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
constexpr int32_t  kRowH       = 30;
constexpr int32_t  kTabH       = 30;
/* Fader travel: 0 is -inf, then 0.6 dB a step up to +6 dB at the top, which
 * puts unity at 90 -- most of the travel where the ear works, a little
 * headroom above it, as on a desk. */
constexpr int32_t  kFaderMax   = 100;
constexpr int32_t  kFaderUnity = 90;
constexpr float    kDbPerStep  = 0.6f;
constexpr float    kMeterFloorDb = -60.0f;

const lv_color_t kBg      = lv_color_hex(0x16181B);
const lv_color_t kRowBg   = lv_color_hex(0x20242A);
const lv_color_t kTrack   = lv_color_hex(0x3A3F47);
const lv_color_t kKnob    = lv_color_hex(0xE8EAED);
const lv_color_t kText    = lv_color_hex(0xE8EAED);
const lv_color_t kDim     = lv_color_hex(0x7B828B);
const lv_color_t kInk     = lv_color_hex(0x16181B);
const lv_color_t kAccent  = lv_color_hex(0xC6FF47);
const lv_color_t kMute    = lv_color_hex(0xFF5C5C);
const lv_color_t kSolo    = lv_color_hex(0xFFC857);
const lv_color_t kFocus   = lv_color_hex(0xFFFFFF);
const lv_color_t kMeterOk = lv_color_hex(0x4FE0B0);
const lv_color_t kMeterHi = lv_color_hex(0xFFC857);
const lv_color_t kMeterClip = lv_color_hex(0xFF5C5C);

enum Page : uint8_t { kIn, kSources, kOut, kRoute, kPresets, kPages };
const char* const kPageNames[kPages] = {"IN", "SOURCES", "OUT", "ROUTE", LV_SYMBOL_LIST};

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
        {"Out 1/2", "speakers"},
        {"Out 3/4", "phones"},
    };
    for (const auto& r : kRoles) if (std::strcmp(r.name, name) == 0) return r.role;
    return "master";
}

struct Row {
    bool      output = false;
    uint8_t   id = 0;            // channel id, or output id
    /* Which socket this row is, for a channel that has more than one: the
     * board wires both line-in jacks to one converter, so "Line in 1/2" and
     * "Line in 3/4" are two rows over one channel and only the one that is
     * selected is live. -1 for every other row. */
    int8_t    src = -1;
    Page      page = kSources;
    lv_obj_t* box = nullptr;
    lv_obj_t* name = nullptr;    // the button that opens the channel
    lv_obj_t* sub = nullptr;     // under the name: gain, pan, "master"
    lv_obj_t* fader = nullptr;
    lv_obj_t* meter = nullptr;
    lv_obj_t* db = nullptr;
    lv_obj_t* mute = nullptr;
    lv_obj_t* solo = nullptr;
    lv_obj_t* mon = nullptr;     // inputs: which heard outputs get it
    float     shown = 0.0f;      // meter, decaying
};

Row       s_rows[kMaxRows];
int       s_rowCount = 0;
uint8_t   s_sources[kMixerMaxChannels];   // ROUTE's rows: every channel
int       s_sourceCount = 0;
lv_obj_t* s_sends[kMixerMaxChannels][kMixerMaxOutputs] = {};
lv_obj_t* s_sendLabels[kMixerMaxChannels][kMixerMaxOutputs] = {};

lv_obj_t* s_root = nullptr;
lv_obj_t* s_pages[kPages] = {};
lv_obj_t* s_tabs[kPages] = {};
lv_obj_t* s_back = nullptr;
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

/* OUT's first rows: the jacks' volume, each a codec's own output level, and
 * the speakers' switch. Out 2 is codec 0, Out 1 codec 1 -- the jacks are
 * numbered the other way round from the codecs. */
struct Jack { lv_obj_t* fader = nullptr; lv_obj_t* pct = nullptr; uint8_t codec = 0; };
Jack      s_jacks[2] = {};
lv_obj_t* s_spkBtn = nullptr;
bool      s_jacksDirty = false;      // a level moved since the settings were saved
lv_group_t* s_prevEncGroup = nullptr;
lv_group_t* s_detailGroup = nullptr;
#endif

crosspad::IInputGain* gains() { return crosspad::getPlatformServices().inputGain; }

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
void monitor_text(char* buf, size_t n, uint32_t mask)
{
    uint8_t heard[kMixerMaxOutputs];
    const int count = heard_outputs(heard);
    if (!mask) { std::snprintf(buf, n, "off"); return; }
    if (count > 1 && mask == (1u << count) - 1) { std::snprintf(buf, n, count == 2 ? "both" : "all"); return; }
    for (int i = 0; i < count; ++i) {
        if (mask == (1u << i)) { short_output_name(buf, n, heard[i]); return; }
    }
    std::snprintf(buf, n, "some");
}

// ── scales and text ────────────────────────────────────────────────────────

float fader_to_gain(int32_t v)
{
    if (v <= 0) return 0.0f;
    return powf(10.0f, ((float)(v - kFaderUnity) * kDbPerStep) / 20.0f);
}

int32_t gain_to_fader(float g)
{
    if (g <= 0.0f) return 0;
    const int32_t v = (int32_t)lroundf(20.0f * log10f(g) / kDbPerStep) + kFaderUnity;
    return v < 1 ? 1 : (v > kFaderMax ? kFaderMax : v);
}

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

bool held(lv_obj_t* o)
{
    return o && (lv_obj_has_state(o, LV_STATE_EDITED) || lv_obj_has_state(o, LV_STATE_PRESSED));
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

// ── channel callbacks ──────────────────────────────────────────────────────

Row& row_of(lv_event_t* e) { return s_rows[(int)(intptr_t)lv_event_get_user_data(e)]; }

void fader_cb(lv_event_t* e)
{
    Row& r = row_of(e);
    const float g = fader_to_gain(lv_slider_get_value(r.fader));
    auto& m = getMixerEngine();
    if (r.output) m.setOutputVolume(r.id, g);
    else          m.setChannelVolume(r.id, g);
    char buf[12];
    db_text(buf, sizeof(buf), g);
    set_text(r.db, buf);
}

void mute_cb(lv_event_t* e)
{
    Row& r = row_of(e);
    auto& m = getMixerEngine();
    if (r.output) m.setOutputMute(r.id, !m.isOutputMuted(r.id));
    else          m.setChannelMute(r.id, !m.isChannelMuted(r.id));
}

void solo_cb(lv_event_t* e)
{
    Row& r = row_of(e);
    auto& m = getMixerEngine();
    m.setChannelSolo(r.id, !m.isChannelSoloed(r.id));
}

/// A tap moves MONITOR on: off, each heard output alone, then all of them.
void mon_cb(lv_event_t* e)
{
    Row& r = row_of(e);
    uint8_t heard[kMixerMaxOutputs];
    const int n = heard_outputs(heard);
    if (!n) return;
    set_monitor(r.id, (monitor_mask(r.id) + 1) & ((1u << n) - 1));
}

// ── the channel window: gain, pan, sends ───────────────────────────────────

void rebuild_group();
void add_presets_to_group();
void refresh_presets();

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
        std::snprintf(buf, sizeof(buf), "Gain  +%.0f dB", (double)db);
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
        if (!m.isRouteEnabled(r.id, o)) std::snprintf(buf, sizeof(buf), "%s  off", m.getOutputName(o));
        else std::snprintf(buf, sizeof(buf), "%s  %d", m.getOutputName(o), (int)lroundf(m.getRouteVolume(r.id, o) * 100.0f));
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
        const int32_t steps = (int32_t)lroundf(g->maxDb() / g->stepDb());
        s_dGain = detail_slider(0, steps, (int32_t)lroundf(g->gainDb(s_gainInput) / g->stepDb()), color, gain_cb, nullptr);
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

void name_cb(lv_event_t* e)
{
    const int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= s_rowCount) return;
    const Row& r = s_rows[idx];
    /* One converter, two sockets: a tap on the one that is not live makes it
     * live rather than opening a channel the row does not really own yet. */
    if (r.src >= 0 && live_src_of(r.id) != r.src) {
        if (gains()) gains()->setSource(gain_input_of(r.id), r.src);
        return;
    }
    open_detail(idx);
}

// ── pages ──────────────────────────────────────────────────────────────────

void add_to_group(lv_obj_t* o)
{
#ifdef MIXER_USE_ENCODER
    if (s_encGroup && o) lv_group_add_obj(s_encGroup, o);
#else
    (void)o;
#endif
}

/// The encoder walks the page on screen, then the tabs.
void rebuild_group()
{
#ifdef MIXER_USE_ENCODER
    if (!s_encGroup) return;
    lv_obj_t* focused = lv_group_get_focused(s_encGroup);
    lv_group_remove_all_objs(s_encGroup);
    if (s_page == kRoute) {
        for (int row = 0; row < s_sourceCount; ++row)
            for (int o = 0; o < kMixerMaxOutputs; ++o) add_to_group(s_sends[row][o]);
    } else if (s_page == kPresets) {
        add_presets_to_group();
    } else {
        if (s_page == kOut) {
            add_to_group(s_jacks[0].fader);
            add_to_group(s_spkBtn);
            add_to_group(s_jacks[1].fader);
        }
        for (int i = 0; i < s_rowCount; ++i) {
            const Row& r = s_rows[i];
            if (r.page != s_page) continue;
            /* An output's name opens its tone, so it belongs in the walk --
             * but only an output that can carry an EQ. */
            if (!r.output || r.id < kMixerEqOutputs) add_to_group(r.name);
            if (!lv_obj_has_state(r.fader, LV_STATE_DISABLED)) add_to_group(r.fader);
            add_to_group(r.mute);
            add_to_group(r.solo);
            add_to_group(r.mon);
        }
    }
    add_to_group(s_back);
    for (auto* t : s_tabs) add_to_group(t);
    bool alive = false;
    for (uint32_t k = 0; k < lv_group_get_obj_count(s_encGroup); ++k) {
        if (lv_group_get_obj_by_index(s_encGroup, k) == focused) { alive = true; break; }
    }
    if (alive) lv_group_focus_obj(focused);
#endif
}

void show_page(Page p)
{
    s_page = p;
    for (int i = 0; i < kPages; ++i) {
        if (!s_pages[i]) continue;
        if (i == p) lv_obj_remove_flag(s_pages[i], LV_OBJ_FLAG_HIDDEN);
        else        lv_obj_add_flag(s_pages[i], LV_OBJ_FLAG_HIDDEN);
        paint(s_tabs[i], i == p, kAccent);
    }
    if (p == kPresets) refresh_presets();     // the card is read when the list is shown
    rebuild_group();
}

void tab_cb(lv_event_t* e) { show_page((Page)(intptr_t)lv_event_get_user_data(e)); }

void back_cb(lv_event_t* e)
{
    auto* app = static_cast<App*>(lv_event_get_user_data(e));
    if (app) app->destroyAppAsync();
}

// ── presets ────────────────────────────────────────────────────────────────

struct PresetRow {
    lv_obj_t* name = nullptr;
    lv_obj_t* sub = nullptr;
    lv_obj_t* load = nullptr;
    lv_obj_t* save = nullptr;
};
PresetRow s_pre[crosspad::MixerPresets::kSlots];
lv_obj_t* s_preStatus = nullptr;
lv_obj_t* s_preUndo = nullptr;
lv_obj_t* s_nameOv = nullptr;          // the name window a SAVE opens
lv_obj_t* s_nameText = nullptr;
int       s_nameSlot = -1;
#ifdef MIXER_USE_ENCODER
lv_group_t* s_nameGroup = nullptr;
#endif

void add_presets_to_group()
{
    if (s_preUndo && !lv_obj_has_flag(s_preUndo, LV_OBJ_FLAG_HIDDEN)) add_to_group(s_preUndo);
    for (auto& r : s_pre) {
        add_to_group(r.load);
        add_to_group(r.save);
    }
}

void refresh_presets()
{
    auto& pr = crosspad::getMixerPresets();
    pr.refresh();
    for (int s = 0; s < crosspad::MixerPresets::kSlots; ++s) {
        const auto& i = pr.info(s);
        set_text(s_pre[s].name, i.name.c_str());
        set_text(s_pre[s].sub, i.kind == crosspad::MixerPresets::Kind::Saved   ? "saved"
                             : i.kind == crosspad::MixerPresets::Kind::Factory ? "starting point"
                                                                                : "empty -- SAVE the desk here");
        if (i.kind == crosspad::MixerPresets::Kind::Empty) lv_obj_add_state(s_pre[s].load, LV_STATE_DISABLED);
        else lv_obj_remove_state(s_pre[s].load, LV_STATE_DISABLED);
    }
    if (!pr.available()) set_text(s_preStatus, "Presets need the card.");
    if (pr.canUndo()) lv_obj_remove_flag(s_preUndo, LV_OBJ_FLAG_HIDDEN);
    else              lv_obj_add_flag(s_preUndo, LV_OBJ_FLAG_HIDDEN);
}

void preset_status(const char* fmt, const std::string& name)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), fmt, name.c_str());
    set_text(s_preStatus, buf);
}

void load_cb(lv_event_t* e)
{
    const int s = (int)(intptr_t)lv_event_get_user_data(e);
    auto& pr = crosspad::getMixerPresets();
    const std::string name = pr.info(s).name;
    if (pr.load(s)) preset_status("%s loaded -- UNDO puts the desk back.", name);
    else            preset_status("%s could not be read.", name);
    refresh_presets();
    rebuild_group();
}

void undo_cb(lv_event_t*)
{
    if (crosspad::getMixerPresets().undo()) set_text(s_preStatus, "The desk is back as it was.");
    refresh_presets();
    rebuild_group();
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
    if (name.empty()) { set_text(s_preStatus, "A preset needs a name."); return; }
    if (crosspad::getMixerPresets().save(s, name)) preset_status("The desk is saved as %s.", name);
    else                                           preset_status("%s not saved: is the card in?", name);
    refresh_presets();
}

/// SAVE asks for the name, the slot's own filled in: OK saves, the name
/// window is the "are you sure" too.
void save_cb(lv_event_t* e)
{
    if (s_nameOv) return;
    s_nameSlot = (int)(intptr_t)lv_event_get_user_data(e);
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

void build_presets(lv_obj_t* page)
{
    lv_obj_t* top = make_box(page);
    lv_obj_set_size(top, LV_PCT(100), 26);
    lv_obj_set_flex_flow(top, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(top, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(top, 4, 0);
    s_preStatus = lv_label_create(top);
    lv_obj_set_flex_grow(s_preStatus, 1);
    lv_obj_set_style_text_font(s_preStatus, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(s_preStatus, kDim, 0);
    lv_label_set_long_mode(s_preStatus, LV_LABEL_LONG_DOT);
    lv_label_set_text(s_preStatus, "Levels, mutes, pan, sends, outputs, input gain.");
    s_preUndo = make_button(top, 64, 24, LV_SYMBOL_REFRESH " UNDO", undo_cb, nullptr, &lv_font_montserrat_10);
    lv_obj_add_flag(s_preUndo, LV_OBJ_FLAG_HIDDEN);

    for (int s = 0; s < crosspad::MixerPresets::kSlots; ++s) {
        lv_obj_t* r = make_box(page);
        lv_obj_set_size(r, LV_PCT(100), kRowH);
        lv_obj_set_style_bg_color(r, kRowBg, 0);
        lv_obj_set_style_bg_opa(r, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(r, 6, 0);
        lv_obj_set_style_pad_hor(r, 6, 0);
        lv_obj_set_style_pad_column(r, 4, 0);
        lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(r, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_t* col = make_box(r);
        lv_obj_set_height(col, kRowH - 2);
        lv_obj_set_flex_grow(col, 1);
        lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(col, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
        s_pre[s].name = lv_label_create(col);
        lv_obj_set_style_text_font(s_pre[s].name, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_pre[s].name, kText, 0);
        lv_label_set_text(s_pre[s].name, "");
        s_pre[s].sub = lv_label_create(col);
        lv_obj_set_style_text_font(s_pre[s].sub, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(s_pre[s].sub, kDim, 0);
        lv_label_set_text(s_pre[s].sub, "");
        s_pre[s].load = make_button(r, 58, 24, "LOAD", load_cb, (void*)(intptr_t)s);
        lv_obj_set_style_bg_color(s_pre[s].load, kAccent, 0);
        lv_obj_set_style_text_color(lv_obj_get_child(s_pre[s].load, 0), kInk, 0);
        lv_obj_set_style_bg_opa(s_pre[s].load, LV_OPA_30, LV_STATE_DISABLED);
        s_pre[s].save = make_button(r, 58, 24, LV_SYMBOL_SAVE " SAVE", save_cb, (void*)(intptr_t)s);
    }
}

bool back_handler(lv_obj_t*)
{
    if (s_nameOv) { close_name(); return true; }
    if (s_detail) { close_detail(); return true; }
    return false;
}

// ── the tick ───────────────────────────────────────────────────────────────

void tick_row(Row& r, bool anySolo)
{
    auto& m = getMixerEngine();
    float l = 0.0f, rr = 0.0f, gain = 0.0f;
    bool muted = false;
    char buf[24];
    if (r.output) {
        m.getOutputLevel(r.id, l, rr);        // already post-master
        gain = m.getOutputVolume(r.id);
        muted = m.isOutputMuted(r.id);
    } else if (r.src >= 0 && live_src_of(r.id) != r.src) {
        /* Not the socket the converter is on: no signal can reach this row,
         * so it shows none rather than mirroring its twin. */
        l = rr = 0.0f;
        gain = m.getChannelVolume(r.id);
        muted = true;
    } else {
        m.getChannelLevel(r.id, l, rr);       // pre-fader: fader it here
        gain = m.getChannelVolume(r.id);
        muted = m.isChannelMuted(r.id);
        const bool soloed = m.isChannelSoloed(r.id);
        const float scale = (muted || (anySolo && !soloed)) ? 0.0f : gain;
        l *= scale;
        rr *= scale;
        paint(r.solo, soloed, kSolo);
        if (r.mon) {
            /* Lit while the input is heard anywhere; the caption dim when not. */
            const uint32_t mask = monitor_mask(r.id);
            monitor_text(buf, sizeof(buf), mask);
            lv_obj_t* cap = lv_obj_get_child(r.mon, 0);
            lv_obj_t* val = lv_obj_get_child(r.mon, 1);
            set_text(val, buf);
            const lv_color_t bg = mask ? kAccent : kTrack;
            const lv_color_t capInk = mask ? kInk : kDim, valInk = mask ? kInk : kText;
            if (!lv_color_eq(lv_obj_get_style_bg_color(r.mon, LV_PART_MAIN), bg)) lv_obj_set_style_bg_color(r.mon, bg, 0);
            if (!lv_color_eq(lv_obj_get_style_text_color(cap, LV_PART_MAIN), capInk)) lv_obj_set_style_text_color(cap, capInk, 0);
            if (!lv_color_eq(lv_obj_get_style_text_color(val, LV_PART_MAIN), valInk)) lv_obj_set_style_text_color(val, valInk, 0);
        }
        const int gi = gain_input_of(r.id);
        if (gi >= 0) std::snprintf(buf, sizeof(buf), "gain +%.0f dB", (double)gains()->gainDb(gi));
        else         pan_text(buf, sizeof(buf), m.getChannelPan(r.id));
        set_text(r.sub, buf);
    }
    paint(r.mute, muted, kMute);
    if (!held(r.fader)) {
        const int32_t v = gain_to_fader(gain);
        if (lv_slider_get_value(r.fader) != v) lv_slider_set_value(r.fader, v, LV_ANIM_OFF);
        db_text(buf, sizeof(buf), gain);
        set_text(r.db, buf);
    }
    /* Fast up, slow down, the way a meter is read. */
    const float peak = l > rr ? l : rr;
    r.shown = peak > r.shown ? peak : r.shown * 0.82f;
    const int32_t pos = peak_to_meter(r.shown);
    if (lv_bar_get_value(r.meter) != pos) lv_bar_set_value(r.meter, pos, LV_ANIM_OFF);
    const lv_color_t c = pos > 95 ? kMeterClip : (pos > 80 ? kMeterHi : kMeterOk);
    if (!lv_color_eq(lv_obj_get_style_bg_color(r.meter, LV_PART_INDICATOR), c)) {
        lv_obj_set_style_bg_color(r.meter, c, LV_PART_INDICATOR);
    }
}

void send_label(int row, int o)
{
    auto& m = getMixerEngine();
    lv_obj_t* l = s_sendLabels[row][o];
    if (!l) return;
    char buf[8];
    const uint8_t ch = s_sources[row];
    if (!m.isRouteEnabled(ch, o)) std::snprintf(buf, sizeof(buf), "off");
    else std::snprintf(buf, sizeof(buf), "%d", (int)lroundf(m.getRouteVolume(ch, o) * 100.0f));
    set_text(l, buf);
}

// ── the jacks' volume ──────────────────────────────────────────────────────
//
// The codecs' own output level, 0..100 -- the knob behind each jack, before
// anything the mix does. It used to be the status bar's drawer (Out 1, Out 2,
// Speaker); with the transport bar in its place nothing but Settings reached
// it, and a board left at Out 2 = 35 and Out 1 = 0 stayed that quiet. Same
// fields as Settings -> Audio and the drawer (masterFX.outVolumeCodec, by
// codec), applied at once, saved when the turn is over and on close. The
// speakers have no level of their own -- they hang off Out 2's codec -- only
// the amp's switch, which Stm32Manager's gate reads from the settings.

uint8_t* jack_level(uint8_t codec)
{
    auto* s = crosspad::CrosspadSettings::getInstance();
    return s ? &s->masterFX.outVolumeCodec[codec][0] : nullptr;
}

void save_jacks()
{
    if (!s_jacksDirty) return;
    s_jacksDirty = false;
    if (auto* ui = crosspad::getSettingsUI()) ui->saveSettings();
}

void jack_cb(lv_event_t* e)
{
    const Jack& j = s_jacks[(int)(intptr_t)lv_event_get_user_data(e)];
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) { save_jacks(); return; }
    uint8_t* v = jack_level(j.codec);
    if (!v || !j.fader) return;
    *v = (uint8_t)lv_slider_get_value(j.fader);
    auto& plat = crosspad_gui::getGuiPlatform();
    if (j.codec == 0) plat.setSpeakerVolume(*v);
    else              plat.setHeadphoneVolume(*v);
    s_jacksDirty = true;
}

/// The speakers' amp: off at once, on at the gate's next look -- as Settings.
void speakers_cb(lv_event_t*)
{
    auto* s = crosspad::CrosspadSettings::getInstance();
    if (!s) return;
    s->masterFX.speakersEnabled = !s->masterFX.speakersEnabled;
    s->stm.audioEn = s->masterFX.speakersEnabled ? 1 : 0;
    auto& plat = crosspad_gui::getGuiPlatform();
    if (!s->masterFX.speakersEnabled) plat.writeStmConfig8(REG_CTL_AUDIO_EN, 0);
    else                              plat.setSpeakerVolume(s->masterFX.outVolumeCodec[0][0]);
    s_jacksDirty = true;
    save_jacks();
}

/// From the settings, unless a finger or the encoder has the fader: Settings
/// and HIL move the same fields.
void tick_jacks()
{
    for (const Jack& j : s_jacks) {
        const uint8_t* v = jack_level(j.codec);
        if (!j.fader || !v) continue;
        if (!held(j.fader) && lv_slider_get_value(j.fader) != *v) lv_slider_set_value(j.fader, *v, LV_ANIM_OFF);
        char buf[8];
        std::snprintf(buf, sizeof(buf), "%u%%", (unsigned)lv_slider_get_value(j.fader));
        set_text(j.pct, buf);
    }
    auto* s = crosspad::CrosspadSettings::getInstance();
    if (s_spkBtn && s) paint(s_spkBtn, s->masterFX.speakersEnabled, kAccent);
}

void tick_cb(lv_timer_t*)
{
    auto& m = getMixerEngine();
    const bool anySolo = m.isAnySoloed();
    if (s_page == kOut) tick_jacks();
    if (s_page != kRoute) {
        for (int i = 0; i < s_rowCount; ++i) if (s_rows[i].page == s_page) tick_row(s_rows[i], anySolo);
    } else {
        for (int row = 0; row < s_sourceCount; ++row) {
            for (int o = 0; o < m.numOutputs() && o < kMixerMaxOutputs; ++o) {
                lv_obj_t* sl = s_sends[row][o];
                if (!sl || held(sl)) continue;
                const uint8_t ch = s_sources[row];
                const int32_t v = m.isRouteEnabled(ch, o) ? (int32_t)lroundf(m.getRouteVolume(ch, o) * 100.0f) : 0;
                if (lv_slider_get_value(sl) != v) lv_slider_set_value(sl, v, LV_ANIM_OFF);
                send_label(row, o);
            }
        }
    }
    if (s_detail) detail_labels();
}

// ── building ───────────────────────────────────────────────────────────────

void build_row(lv_obj_t* page, int idx, bool output, uint8_t id, Page where, int8_t src = -1)
{
    auto& m = getMixerEngine();
    Row& r = s_rows[idx];
    r = Row{};
    r.output = output;
    r.id = id;
    r.page = where;
    r.src = src;

    const Look* look = output ? nullptr : look_for(m.getChannelName(id));
    const char* label = output ? m.getOutputName(id) : (look ? look->label : m.getChannelName(id));
    if (src >= 0) label = gains() ? gains()->sourceName(gain_input_of(id), src) : "Line";
    const lv_color_t color = output ? kText : (look ? lv_color_hex(look->color) : kText);

    r.box = make_box(page);
    lv_obj_set_size(r.box, LV_PCT(100), kRowH);
    lv_obj_set_style_bg_color(r.box, kRowBg, 0);
    lv_obj_set_style_bg_opa(r.box, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(r.box, 6, 0);
    lv_obj_set_style_border_width(r.box, 4, 0);
    lv_obj_set_style_border_side(r.box, LV_BORDER_SIDE_LEFT, 0);
    lv_obj_set_style_border_color(r.box, color, 0);
    lv_obj_set_style_pad_hor(r.box, 4, 0);
    lv_obj_set_style_pad_column(r.box, 4, 0);
    lv_obj_set_flex_flow(r.box, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(r.box, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    /* The name, and under it what the row does not show: gain or pan. A tap
     * opens the rest of the channel. */
    r.name = lv_button_create(r.box);
    lv_obj_remove_style_all(r.name);
    lv_obj_set_size(r.name, 62, kRowH - 2);
    lv_obj_set_style_radius(r.name, 4, 0);
    lv_obj_set_style_outline_color(r.name, kFocus, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(r.name, 2, LV_STATE_FOCUS_KEY);
    lv_obj_set_flex_flow(r.name, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(r.name, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    /* An output opens its tone; a channel opens the rest of itself. The
     * outputs that carry no EQ (a recording bus) open nothing. */
    if (!output || id < kMixerEqOutputs) {
        lv_obj_add_event_cb(r.name, name_cb, LV_EVENT_CLICKED, (void*)(intptr_t)idx);
    }
    lv_obj_t* nl = lv_label_create(r.name);
    lv_obj_set_style_text_font(nl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(nl, color, 0);
    lv_label_set_long_mode(nl, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(nl, 62);
    lv_label_set_text(nl, label);
    r.sub = lv_label_create(r.name);
    lv_obj_set_style_text_font(r.sub, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(r.sub, kDim, 0);
    lv_label_set_text(r.sub, output ? output_role(m.getOutputName(id)) : "");

    /* The fader, long, with the post-fader meter under it. */
    lv_obj_t* lane = make_box(r.box);
    lv_obj_set_height(lane, kRowH - 4);
    lv_obj_set_flex_grow(lane, 1);
    lv_obj_set_style_pad_hor(lane, 10, 0);        // half the knob, so it stays in the row at the ends
    lv_obj_set_flex_flow(lane, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(lane, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(lane, 5, 0);
    r.fader = make_slider(lane, 0, kFaderMax, output ? kDim : color, fader_cb, (void*)(intptr_t)idx);
    lv_obj_set_width(r.fader, LV_PCT(100));
    r.meter = lv_bar_create(lane);
    lv_obj_set_size(r.meter, LV_PCT(100), 3);
    lv_bar_set_range(r.meter, 0, 100);
    lv_obj_set_style_bg_color(r.meter, lv_color_hex(0x101214), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(r.meter, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(r.meter, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(r.meter, 1, LV_PART_INDICATOR);
    lv_obj_set_style_anim_duration(r.meter, 0, 0);
    lv_obj_remove_flag(r.meter, LV_OBJ_FLAG_CLICKABLE);

    r.db = lv_label_create(r.box);
    lv_obj_set_width(r.db, 36);
    lv_obj_set_style_text_font(r.db, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(r.db, kText, 0);
    lv_obj_set_style_text_align(r.db, LV_TEXT_ALIGN_RIGHT, 0);
    lv_label_set_text(r.db, "");

    r.mute = make_button(r.box, 26, 24, "M", mute_cb, (void*)(intptr_t)idx);
    if (!output && where == kIn) {
        /* MON: a caption and where the input is heard, one under the other. */
        r.mon = make_button(r.box, 44, 24, "MON", mon_cb, (void*)(intptr_t)idx, &lv_font_montserrat_10);
        lv_obj_t* cap = lv_obj_get_child(r.mon, 0);
        lv_obj_align(cap, LV_ALIGN_TOP_MID, 0, 0);
        lv_obj_t* val = lv_label_create(r.mon);
        lv_obj_set_style_text_font(val, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(val, kText, 0);
        lv_label_set_text(val, "");
        lv_obj_align(val, LV_ALIGN_BOTTOM_MID, 0, 0);
    } else if (!output) {
        r.solo = make_button(r.box, 26, 24, "S", solo_cb, (void*)(intptr_t)idx);
    } else {
        lv_obj_t* gap = make_box(r.box);
        lv_obj_set_size(gap, 26, 1);
    }

    /* The USB recording bus's gain belongs to Settings -> USB, which applies
     * it at every boot: a fader here would be undone by the next one. It is
     * shown, metered and mutable, not moved. */
    if (output && std::strcmp(m.getOutputName(id), "USB rec") == 0) {
        lv_obj_add_state(r.fader, LV_STATE_DISABLED);
        lv_obj_set_style_bg_color(r.fader, kDim, (lv_style_selector_t)LV_PART_KNOB | LV_STATE_DISABLED);
        set_text(r.sub, "Settings");
    }
}

/// One jack's row on OUT: its name and what it feeds, the codec's level
/// 0..100 and the %, and on Out 2 the speakers' switch.
void build_jack(lv_obj_t* page, int idx, uint8_t codec, const char* name, const char* role)
{
    Jack& j = s_jacks[idx];
    j = Jack{};
    j.codec = codec;

    lv_obj_t* box = make_box(page);
    lv_obj_set_size(box, LV_PCT(100), kRowH);
    lv_obj_set_style_bg_color(box, kRowBg, 0);
    lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(box, 6, 0);
    lv_obj_set_style_border_width(box, 4, 0);
    lv_obj_set_style_border_side(box, LV_BORDER_SIDE_LEFT, 0);
    lv_obj_set_style_border_color(box, kAccent, 0);
    lv_obj_set_style_pad_hor(box, 4, 0);
    lv_obj_set_style_pad_column(box, 4, 0);
    lv_obj_set_flex_flow(box, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(box, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t* names = make_box(box);
    lv_obj_set_size(names, 62, kRowH - 2);
    lv_obj_set_flex_flow(names, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(names, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_t* nl = lv_label_create(names);
    lv_obj_set_style_text_font(nl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(nl, kAccent, 0);
    lv_label_set_text(nl, name);
    lv_obj_t* sub = lv_label_create(names);
    lv_obj_set_style_text_font(sub, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(sub, kDim, 0);
    lv_label_set_long_mode(sub, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(sub, 62);
    lv_label_set_text(sub, role);

    lv_obj_t* lane = make_box(box);
    lv_obj_set_height(lane, kRowH - 4);
    lv_obj_set_flex_grow(lane, 1);
    lv_obj_set_style_pad_hor(lane, 10, 0);
    lv_obj_set_flex_flow(lane, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(lane, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    j.fader = make_slider(lane, 0, 100, kAccent, jack_cb, (void*)(intptr_t)idx);
    lv_obj_set_width(j.fader, LV_PCT(100));
    /* Written to the card once the turn is over, not per step. */
    lv_obj_add_event_cb(j.fader, jack_cb, LV_EVENT_RELEASED, (void*)(intptr_t)idx);
    lv_obj_add_event_cb(j.fader, jack_cb, LV_EVENT_DEFOCUSED, (void*)(intptr_t)idx);

    j.pct = lv_label_create(box);
    lv_obj_set_width(j.pct, 36);
    lv_obj_set_style_text_font(j.pct, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(j.pct, kText, 0);
    lv_obj_set_style_text_align(j.pct, LV_TEXT_ALIGN_RIGHT, 0);
    lv_label_set_text(j.pct, "");

    if (codec == 0) {
        s_spkBtn = make_button(box, 56, 24, LV_SYMBOL_VOLUME_MAX " SPK", speakers_cb, nullptr,
                               &lv_font_montserrat_10);
    } else {
        lv_obj_t* gap = make_box(box);
        lv_obj_set_size(gap, 56, 1);
    }
}

lv_obj_t* make_page(lv_obj_t* parent)
{
    lv_obj_t* p = lv_obj_create(parent);
    lv_obj_remove_style_all(p);
    lv_obj_set_width(p, LV_PCT(100));
    lv_obj_set_flex_grow(p, 1);
    lv_obj_set_flex_flow(p, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(p, 3, 0);
    lv_obj_set_scroll_dir(p, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(p, LV_SCROLLBAR_MODE_ACTIVE);
    return p;
}

void build_route(lv_obj_t* page)
{
    auto& m = getMixerEngine();
    const int outs = m.numOutputs() < kMixerMaxOutputs ? m.numOutputs() : kMixerMaxOutputs;
    constexpr int32_t kNameW = 62;

    lv_obj_t* head = make_box(page);
    lv_obj_set_size(head, LV_PCT(100), 16);
    lv_obj_set_flex_flow(head, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(head, 4, 0);
    lv_obj_set_style_pad_left(head, 8, 0);
    lv_obj_t* corner = lv_label_create(head);
    lv_obj_set_width(corner, kNameW);
    lv_obj_set_style_text_font(corner, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(corner, kDim, 0);
    lv_label_set_text(corner, "send to");
    for (int o = 0; o < outs; ++o) {
        lv_obj_t* h = lv_label_create(head);
        lv_obj_set_flex_grow(h, 1);
        lv_obj_set_style_text_font(h, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(h, kText, 0);
        lv_obj_set_style_text_align(h, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_text(h, m.getOutputName(o));
    }

    for (int row = 0; row < s_sourceCount; ++row) {
        const uint8_t ch = s_sources[row];
        const Look* look = look_for(m.getChannelName(ch));
        const lv_color_t color = look ? lv_color_hex(look->color) : kText;

        lv_obj_t* r = make_box(page);
        lv_obj_set_size(r, LV_PCT(100), kRowH);
        lv_obj_set_style_bg_color(r, kRowBg, 0);
        lv_obj_set_style_bg_opa(r, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(r, 6, 0);
        lv_obj_set_style_border_width(r, 4, 0);
        lv_obj_set_style_border_side(r, LV_BORDER_SIDE_LEFT, 0);
        lv_obj_set_style_border_color(r, color, 0);
        lv_obj_set_style_pad_hor(r, 4, 0);
        lv_obj_set_style_pad_column(r, 4, 0);
        lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(r, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

        lv_obj_t* nl = lv_label_create(r);
        lv_obj_set_width(nl, kNameW);
        lv_obj_set_style_text_font(nl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(nl, color, 0);
        lv_label_set_long_mode(nl, LV_LABEL_LONG_CLIP);
        lv_label_set_text(nl, look ? look->label : m.getChannelName(ch));

        for (int o = 0; o < outs; ++o) {
            lv_obj_t* cell = make_box(r);
            lv_obj_set_height(cell, kRowH - 4);
            lv_obj_set_flex_grow(cell, 1);
            lv_obj_set_style_pad_hor(cell, 7, 0);
            lv_obj_set_flex_flow(cell, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_flex_align(cell, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
            lv_obj_set_style_pad_row(cell, 1, 0);
            lv_obj_t* vl = lv_label_create(cell);
            lv_obj_set_style_text_font(vl, &lv_font_montserrat_10, 0);
            lv_obj_set_style_text_color(vl, kDim, 0);
            s_sendLabels[row][o] = vl;
            lv_obj_t* sl = make_slider(cell, 0, 100, color, [](lv_event_t* e) {
                const int packed = (int)(intptr_t)lv_event_get_user_data(e);
                const int row = packed >> 4, o = packed & 0x0F;
                set_send(s_sources[row], o, lv_slider_get_value(s_sends[row][o]));
                send_label(row, o);
            }, (void*)(intptr_t)((row << 4) | o));
            lv_obj_set_width(sl, LV_PCT(100));
            lv_obj_set_height(sl, 6);
            lv_obj_set_style_pad_ver(sl, 6, LV_PART_KNOB);
            s_sends[row][o] = sl;
        }
    }
}

} // namespace

lv_obj_t* Mixer_create(lv_obj_t* parent, App* app)
{
#ifdef MIXER_USE_ENCODER
    if (indev_encoderTop) s_prevEncGroup = lv_indev_get_group(indev_encoderTop);
    if (!s_encGroup) s_encGroup = lv_group_create();
    if (indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_encGroup);
#endif
    auto& m = getMixerEngine();

    s_root = lv_obj_create(parent);
    lv_obj_set_size(s_root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_root, kBg, 0);
    lv_obj_set_style_border_width(s_root, 0, 0);
    lv_obj_set_style_radius(s_root, 0, 0);
    lv_obj_set_style_pad_all(s_root, 4, 0);
    lv_obj_set_style_pad_row(s_root, 4, 0);
    lv_obj_set_flex_flow(s_root, LV_FLEX_FLOW_COLUMN);
    lv_obj_remove_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);

    for (int p = 0; p < kPages; ++p) s_pages[p] = make_page(s_root);

    /* Channels onto their pages in the order kLooks gives them -- Mic, Line,
     * USB in; Sampler first among the sources -- not the order they were
     * added in; one the table does not know goes last. Outputs onto OUT. */
    s_rowCount = 0;
    s_sourceCount = 0;
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
            for (int k = 0; k < sockets && s_rowCount < kMaxRows; ++k) {
                build_row(s_pages[where], s_rowCount++, false, ch, where, (int8_t)k);
            }
        } else {
            build_row(s_pages[where], s_rowCount++, false, ch, where);
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
    build_jack(s_pages[kOut], 0, 0, "Out 2", "phones + spk");
    build_jack(s_pages[kOut], 1, 1, "Out 1", "line out");
    for (uint8_t o = 0; o < m.numOutputs() && s_rowCount < kMaxRows; ++o) {
        build_row(s_pages[kOut], s_rowCount++, true, o, kOut);
    }
    build_route(s_pages[kRoute]);
    build_presets(s_pages[kPresets]);

    /* The categories, along the bottom where the thumb is. */
    lv_obj_t* tabs = make_box(s_root);
    lv_obj_set_size(tabs, LV_PCT(100), kTabH);
    lv_obj_set_flex_flow(tabs, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(tabs, 4, 0);
    s_back = make_button(tabs, 36, kTabH, LV_SYMBOL_LEFT, back_cb, app);
    /* Shared out by how long the names are, so SOURCES and ROUTE fit. */
    static const uint8_t kTabGrow[kPages] = {2, 3, 2, 3, 0};
    for (int p = 0; p < kPages; ++p) {
        s_tabs[p] = make_button(tabs, 36, kTabH, kPageNames[p], tab_cb, (void*)(intptr_t)p);
        lv_obj_set_flex_grow(s_tabs[p], kTabGrow[p]);
    }

    show_page(kSources);
    tick_cb(nullptr);
#ifdef MIXER_USE_ENCODER
    if (s_encGroup) lv_group_focus_obj(s_tabs[kSources]);
#endif
    s_timer = lv_timer_create(tick_cb, kTickMs, nullptr);

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
    /* The desk as it is left is the desk the next boot starts with -- and the
     * jacks' levels, if a turn was still unsaved. */
    getMixerEngine().notifyStateChanged();
    save_jacks();
    for (auto& j : s_jacks) j = Jack{};
    s_spkBtn = nullptr;

#ifdef MIXER_USE_ENCODER
    if (s_detailGroup) { lv_group_delete(s_detailGroup); s_detailGroup = nullptr; }
    if (s_nameGroup) { lv_group_delete(s_nameGroup); s_nameGroup = nullptr; }
#endif
    s_nameOv = s_nameText = nullptr;
    s_nameSlot = -1;
    for (auto& r : s_pre) r = PresetRow{};
    s_preStatus = s_preUndo = nullptr;
    s_detail = s_dGain = s_dGainLabel = s_dPan = s_dPanLabel = nullptr;
    for (auto& o : s_dEq) o = nullptr;
    for (auto& o : s_dEqLabel) o = nullptr;
    for (auto& s : s_dSends) s = nullptr;
    for (auto& s : s_dSendLabels) s = nullptr;
    for (auto& s : s_dMon) s = nullptr;
    s_detailRow = s_gainInput = -1;
    for (auto& r : s_rows) r = Row{};
    for (auto& row : s_sends) for (auto& c : row) c = nullptr;
    for (auto& row : s_sendLabels) for (auto& c : row) c = nullptr;
    for (auto& p : s_pages) p = nullptr;
    for (auto& t : s_tabs) t = nullptr;
    s_rowCount = s_sourceCount = 0;
    s_root = s_back = nullptr;

#ifdef MIXER_USE_ENCODER
    if (indev_encoderTop && s_prevEncGroup) lv_indev_restore_group(indev_encoderTop, s_prevEncGroup);
    if (s_encGroup) {
        lv_group_delete(s_encGroup);
        s_encGroup = nullptr;
    }
    s_prevEncGroup = nullptr;
#endif

    lv_obj_delete_async(app_obj);
}

// No pad logic: the pads keep playing the kit, so the mix is heard as it moves.
// mixer.png is a black glyph: it asks the launcher to tint it.
REGISTER_APP(Mixer, nullptr, "mixer.png",
             Mixer_create, Mixer_destroy,
             nullptr, nullptr, nullptr, 0, true)
