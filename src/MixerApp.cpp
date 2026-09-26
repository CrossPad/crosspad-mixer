/**
 * @file MixerApp.cpp
 * @brief Mixer and signal router, laid out like a DAW's.
 *
 * Two pages over the one AudioMixerEngine every platform runs:
 *
 *   MIX    a vertical strip per source -- Sampler, Pitched, USB in, Click,
 *          the line inputs -- with its fader in dB, a post-fader meter, pan,
 *          mute and solo; then, past a divider, a master strip per output
 *          (Speakers, Phones, USB rec). The Click strip is the metronome's
 *          level: it is a source like any other.
 *   ROUTE  the send matrix: sources down, outputs across, and in every cell
 *          the level that source sends to that output. Zero is no send.
 *
 * The engine is the only state. Widgets are written from it every tick unless
 * the player has hold of them, so anything else that moves the mix (HIL, a
 * loaded state) shows up here. On close the app asks the engine to persist
 * (notifyStateChanged); where that goes is the platform's business.
 *
 * The pads keep playing the kit: no pad logic, so the mix is heard as it
 * changes.
 */

#include "crosspad-mixer/MixerApp.hpp"
#include "crosspad-mixer/AudioMixerEngine.hpp"

// `class App` lives in different headers on the two platforms but both derive
// from crosspad_gui::AppBase<App>, so the lifecycle methods used here
// (setOnShow / setOnHide / destroyApp) are the same.
#if __has_include("pc_stubs/PcApp.hpp")
  #include "pc_stubs/PcApp.hpp"
#else
  #include "crosspad-gui/app/App.hpp"
#endif

#include "crosspad/app/AppRegistrar.hpp"
#include "crosspad-gui/widgets/gui_helpers.h"

#include "lvgl.h"

#include <cmath>
#include <cstring>

// Encoder navigation is an embedded-only concern (PC uses pointer/touch via SDL).
#ifdef ESP_PLATFORM
extern "C" {
  extern lv_indev_t* indev_encoderTop;
}
#define MIXER_USE_ENCODER 1
#endif

namespace {

constexpr int      kMaxStrips  = 10;    // what fits across 320 px
constexpr int      kMaxSources = 7;
constexpr uint32_t kTickMs     = 50;
/* Fader travel: 0 is -inf, then 0.6 dB a step up to +6 dB at the top, which
 * puts unity at 90 -- most of the travel where the ear works, a little
 * headroom above it, as on a desk. */
constexpr int32_t  kFaderMax   = 100;
constexpr int32_t  kFaderUnity = 90;
constexpr float    kDbPerStep  = 0.6f;
constexpr float    kMeterFloorDb = -60.0f;

const lv_color_t kBg      = lv_color_hex(0x16181B);
const lv_color_t kStripBg = lv_color_hex(0x20242A);
const lv_color_t kMasterBg= lv_color_hex(0x2A2F36);
const lv_color_t kTrack   = lv_color_hex(0x3A3F47);
const lv_color_t kCap     = lv_color_hex(0xD8D8D8);
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

/// How a source is labelled and coloured, by the name it registers under.
struct Look { const char* name; const char* shortName; uint32_t color; };
const Look kLooks[] = {
    {"Sampler", "Smpl",  0xC6FF47},
    {"Pitched", "Pitch", 0x5AC8FF},
    {"USB",     "USB",   0xFFC857},
    {"Click",   "Click", 0xFF7A9A},
    {"IN1",     "In 1",  0xB48CFF},
    {"IN2",     "In 2",  0xFF9552},
    {"Song",    "Song",  0xF26DF9},
};

const Look* look_for(const char* name)
{
    for (const auto& l : kLooks) if (std::strcmp(l.name, name) == 0) return &l;
    return nullptr;
}

/// An output's short name from the one the platform gave it.
const char* short_output(const char* name, int o)
{
    if (std::strcmp(name, "Speakers") == 0) return "Spk";
    if (std::strcmp(name, "Phones") == 0)   return "Phn";
    if (std::strcmp(name, "USB rec") == 0)  return "Rec";
    static char buf[4][6];
    lv_snprintf(buf[o & 3], sizeof(buf[0]), "O%d", o + 1);
    return name[0] ? name : buf[o & 3];
}

struct Strip {
    bool      output = false;
    uint8_t   id = 0;            // channel id, or output id
    lv_obj_t* fader = nullptr;
    lv_obj_t* meter = nullptr;
    lv_obj_t* db = nullptr;
    lv_obj_t* pan = nullptr;
    lv_obj_t* mute = nullptr;
    lv_obj_t* solo = nullptr;
    float     shown = 0.0f;      // meter, decaying
};

Strip     s_strips[kMaxStrips];
int       s_stripCount = 0;
uint8_t   s_sources[kMaxSources];
int       s_sourceCount = 0;
lv_obj_t* s_sends[kMaxSources][kMixerMaxOutputs] = {};
lv_obj_t* s_sendLabels[kMaxSources][kMixerMaxOutputs] = {};

lv_obj_t* s_root = nullptr;
lv_obj_t* s_mixPage = nullptr;
lv_obj_t* s_routePage = nullptr;
lv_obj_t* s_mixTab = nullptr;
lv_obj_t* s_routeTab = nullptr;
lv_timer_t* s_timer = nullptr;

#ifdef MIXER_USE_ENCODER
lv_group_t* s_encGroup = nullptr;
lv_group_t* s_prevEncGroup = nullptr;
#endif

// ── scales ─────────────────────────────────────────────────────────────────

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

void set_db_label(lv_obj_t* l, float gain)
{
    if (gain <= 0.0f) { lv_label_set_text(l, "-inf"); return; }
    const float db = 20.0f * log10f(gain);
    char buf[12];
    lv_snprintf(buf, sizeof(buf), "%s%d.%d", db < -0.05f ? "-" : (db > 0.05f ? "+" : ""),
                (int)fabsf(db), (int)(fabsf(db) * 10.0f) % 10);
    lv_label_set_text(l, buf);
}

/// Peak to meter position, 0..100 over kMeterFloorDb..0 dBFS.
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

void add_to_group(lv_obj_t* o)
{
#ifdef MIXER_USE_ENCODER
    if (s_encGroup && o) lv_group_add_obj(s_encGroup, o);
#else
    (void)o;
#endif
}

lv_obj_t* make_box(lv_obj_t* parent)
{
    lv_obj_t* o = lv_obj_create(parent);
    lv_obj_remove_style_all(o);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    return o;
}

lv_obj_t* make_toggle(lv_obj_t* parent, const char* text, lv_event_cb_t cb, void* ud)
{
    lv_obj_t* b = lv_button_create(parent);
    lv_obj_set_size(b, 13, 14);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_set_style_radius(b, 3, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kTrack, 0);
    lv_obj_set_style_outline_color(b, kFocus, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 1, LV_STATE_FOCUS_KEY);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(l, kText, 0);
    lv_label_set_text(l, text);
    lv_obj_center(l);
    add_to_group(b);
    return b;
}

void paint_toggle(lv_obj_t* b, bool on, lv_color_t onColor)
{
    if (!b) return;
    lv_obj_set_style_bg_color(b, on ? onColor : kTrack, 0);
    lv_obj_set_style_text_color(lv_obj_get_child(b, 0), on ? kInk : kText, 0);
}

void style_slider(lv_obj_t* s, lv_color_t fill)
{
    lv_obj_set_style_bg_color(s, kTrack, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(s, 1, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s, fill, LV_PART_INDICATOR);
    lv_obj_set_style_radius(s, 1, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s, kCap, LV_PART_KNOB);
    lv_obj_set_style_radius(s, 1, LV_PART_KNOB);
    lv_obj_set_style_shadow_width(s, 0, LV_PART_KNOB);
    lv_obj_set_style_bg_color(s, kAccent, (lv_style_selector_t)LV_PART_KNOB | LV_STATE_FOCUSED);
    lv_obj_set_style_bg_color(s, kAccent, (lv_style_selector_t)LV_PART_KNOB | LV_STATE_EDITED);
    lv_obj_set_style_outline_width(s, 0, LV_STATE_FOCUS_KEY);
}

// ── callbacks ──────────────────────────────────────────────────────────────

Strip& strip_of(lv_event_t* e) { return s_strips[(int)(intptr_t)lv_event_get_user_data(e)]; }

void fader_cb(lv_event_t* e)
{
    Strip& s = strip_of(e);
    const float g = fader_to_gain(lv_slider_get_value(s.fader));
    auto& m = getMixerEngine();
    if (s.output) m.setOutputVolume(s.id, g);
    else          m.setChannelVolume(s.id, g);
    set_db_label(s.db, g);
}

void pan_cb(lv_event_t* e)
{
    Strip& s = strip_of(e);
    getMixerEngine().setChannelPan(s.id, (float)lv_slider_get_value(s.pan) / 50.0f);
}

void mute_cb(lv_event_t* e)
{
    Strip& s = strip_of(e);
    auto& m = getMixerEngine();
    if (s.output) m.setOutputMute(s.id, !m.isOutputMuted(s.id));
    else          m.setChannelMute(s.id, !m.isChannelMuted(s.id));
}

void solo_cb(lv_event_t* e)
{
    Strip& s = strip_of(e);
    auto& m = getMixerEngine();
    m.setChannelSolo(s.id, !m.isChannelSoloed(s.id));
}

void send_label(int row, int o)
{
    auto& m = getMixerEngine();
    const uint8_t ch = s_sources[row];
    lv_obj_t* l = s_sendLabels[row][o];
    if (!l) return;
    if (!m.isRouteEnabled(ch, o)) { lv_label_set_text(l, "off"); return; }
    lv_label_set_text_fmt(l, "%d", (int)lroundf(m.getRouteVolume(ch, o) * 100.0f));
}

/// A send's level; dragged to zero it is switched off rather than left
/// enabled at nothing, so "off" means no signal and no work.
void send_cb(lv_event_t* e)
{
    const int packed = (int)(intptr_t)lv_event_get_user_data(e);
    const int row = packed >> 4, o = packed & 0x0F;
    const int32_t v = lv_slider_get_value(s_sends[row][o]);
    auto& m = getMixerEngine();
    const uint8_t ch = s_sources[row];
    m.setRouteEnabled(ch, o, v > 0);
    if (v > 0) m.setRouteVolume(ch, o, (float)v / 100.0f);
    send_label(row, o);
}

void show_page(bool route)
{
    if (route) {
        lv_obj_add_flag(s_mixPage, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(s_routePage, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_remove_flag(s_mixPage, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_routePage, LV_OBJ_FLAG_HIDDEN);
    }
    paint_toggle(s_mixTab, !route, kAccent);
    paint_toggle(s_routeTab, route, kAccent);
}

void mix_tab_cb(lv_event_t*)   { show_page(false); }
void route_tab_cb(lv_event_t*) { show_page(true); }

void back_cb(lv_event_t* e)
{
    auto* app = static_cast<App*>(lv_event_get_user_data(e));
    if (app) app->destroyAppAsync();
}

// ── the tick: meters, and the engine written back onto the widgets ─────────

void tick_cb(lv_timer_t*)
{
    auto& m = getMixerEngine();
    const bool anySolo = m.isAnySoloed();

    for (int i = 0; i < s_stripCount; ++i) {
        Strip& s = s_strips[i];
        float l = 0.0f, r = 0.0f, gain = 0.0f;
        bool muted = false;
        if (s.output) {
            m.getOutputLevel(s.id, l, r);        // already post-master
            gain = m.getOutputVolume(s.id);
            muted = m.isOutputMuted(s.id);
        } else {
            m.getChannelLevel(s.id, l, r);       // pre-fader: fader it here
            gain = m.getChannelVolume(s.id);
            muted = m.isChannelMuted(s.id);
            const bool soloed = m.isChannelSoloed(s.id);
            const float scale = (muted || (anySolo && !soloed)) ? 0.0f : gain;
            l *= scale;
            r *= scale;
            paint_toggle(s.solo, soloed, kSolo);
            if (s.pan && !held(s.pan)) {
                const int32_t p = (int32_t)lroundf(m.getChannelPan(s.id) * 50.0f);
                if (lv_slider_get_value(s.pan) != p) lv_slider_set_value(s.pan, p, LV_ANIM_OFF);
            }
        }
        paint_toggle(s.mute, muted, kMute);

        if (!held(s.fader)) {
            const int32_t v = gain_to_fader(gain);
            if (lv_slider_get_value(s.fader) != v) lv_slider_set_value(s.fader, v, LV_ANIM_OFF);
            set_db_label(s.db, gain);
        }

        /* Fast up, slow down, the way a meter is read. */
        const float peak = l > r ? l : r;
        s.shown = peak > s.shown ? peak : s.shown * 0.82f;
        const int32_t pos = peak_to_meter(s.shown);
        lv_bar_set_value(s.meter, pos, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(s.meter, pos > 95 ? kMeterClip : (pos > 80 ? kMeterHi : kMeterOk),
                                  LV_PART_INDICATOR);
    }

    if (lv_obj_has_flag(s_routePage, LV_OBJ_FLAG_HIDDEN)) return;
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

// ── building ───────────────────────────────────────────────────────────────

void build_strip(lv_obj_t* parent, int idx, bool output, uint8_t id, int32_t w)
{
    auto& m = getMixerEngine();
    Strip& s = s_strips[idx];
    s = Strip{};
    s.output = output;
    s.id = id;

    const Look* look = output ? nullptr : look_for(m.getChannelName(id));
    const char* name = output ? short_output(m.getOutputName(id), id)
                              : (look ? look->shortName : m.getChannelName(id));
    const lv_color_t color = output ? kText : (look ? lv_color_hex(look->color) : kText);

    lv_obj_t* col = make_box(parent);
    lv_obj_set_size(col, w, LV_PCT(100));
    lv_obj_set_style_bg_color(col, output ? kMasterBg : kStripBg, 0);
    lv_obj_set_style_bg_opa(col, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(col, 3, 0);
    lv_obj_set_style_border_width(col, 2, 0);
    lv_obj_set_style_border_side(col, LV_BORDER_SIDE_TOP, 0);
    lv_obj_set_style_border_color(col, color, 0);
    lv_obj_set_style_pad_all(col, 2, 0);
    lv_obj_set_style_pad_row(col, 3, 0);
    lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(col, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t* nl = lv_label_create(col);
    lv_obj_set_style_text_font(nl, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(nl, color, 0);
    lv_label_set_long_mode(nl, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(nl, LV_PCT(100));
    lv_obj_set_style_text_align(nl, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(nl, name);

    /* Pan: a short balance bar filled from the centre. Masters have none;
     * the space is kept so the faders line up across the desk. */
    if (!output) {
        s.pan = lv_slider_create(col);
        lv_obj_set_size(s.pan, w - 8, 6);
        lv_slider_set_range(s.pan, -50, 50);
        lv_slider_set_mode(s.pan, LV_SLIDER_MODE_SYMMETRICAL);
        style_slider(s.pan, color);
        lv_obj_set_style_pad_all(s.pan, 2, LV_PART_KNOB);
        lv_obj_set_ext_click_area(s.pan, 4);
        lv_obj_add_event_cb(s.pan, pan_cb, LV_EVENT_VALUE_CHANGED, (void*)(intptr_t)idx);
        add_to_group(s.pan);
    } else {
        lv_obj_t* gap = make_box(col);
        lv_obj_set_size(gap, 1, 6);
    }

    lv_obj_t* btns = make_box(col);
    lv_obj_set_size(btns, LV_PCT(100), 14);
    lv_obj_set_flex_flow(btns, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btns, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(btns, 1, 0);
    s.mute = make_toggle(btns, "M", mute_cb, (void*)(intptr_t)idx);
    if (!output) s.solo = make_toggle(btns, "S", solo_cb, (void*)(intptr_t)idx);

    /* Fader and meter side by side, the meter post-fader. */
    lv_obj_t* lane = make_box(col);
    lv_obj_set_width(lane, LV_PCT(100));
    lv_obj_set_flex_grow(lane, 1);
    lv_obj_set_style_pad_ver(lane, 5, 0);
    lv_obj_set_flex_flow(lane, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(lane, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(lane, 6, 0);

    s.fader = lv_slider_create(lane);
    lv_obj_set_width(s.fader, 3);
    lv_obj_set_height(s.fader, LV_PCT(100));
    lv_slider_set_range(s.fader, 0, kFaderMax);
    style_slider(s.fader, kTrack);
    lv_obj_set_style_pad_left(s.fader, 5, LV_PART_KNOB);
    lv_obj_set_style_pad_right(s.fader, 5, LV_PART_KNOB);
    lv_obj_set_style_pad_top(s.fader, 3, LV_PART_KNOB);
    lv_obj_set_style_pad_bottom(s.fader, 3, LV_PART_KNOB);
    lv_obj_set_ext_click_area(s.fader, 8);
    lv_obj_add_event_cb(s.fader, fader_cb, LV_EVENT_VALUE_CHANGED, (void*)(intptr_t)idx);

    s.meter = lv_bar_create(lane);
    lv_obj_set_width(s.meter, 3);
    lv_obj_set_height(s.meter, LV_PCT(100));
    lv_bar_set_range(s.meter, 0, 100);
    lv_obj_set_style_bg_color(s.meter, lv_color_hex(0x101214), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s.meter, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(s.meter, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(s.meter, 0, LV_PART_INDICATOR);
    lv_obj_remove_flag(s.meter, LV_OBJ_FLAG_CLICKABLE);

    s.db = lv_label_create(col);
    lv_obj_set_style_text_font(s.db, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(s.db, kDim, 0);
    lv_label_set_text(s.db, "");

    /* The USB recording bus's gain belongs to Settings -> USB, which applies
     * it at every boot: a fader here would be undone by the next one. It is
     * shown, metered and mutable, not moved. */
    const bool fixed = output && std::strcmp(m.getOutputName(id), "USB rec") == 0;
    if (fixed) {
        lv_obj_add_state(s.fader, LV_STATE_DISABLED);
        lv_obj_set_style_bg_color(s.fader, kDim, (lv_style_selector_t)LV_PART_KNOB | LV_STATE_DISABLED);
    } else {
        add_to_group(s.fader);
    }
}

void build_mix(lv_obj_t* page)
{
    auto& m = getMixerEngine();
    lv_obj_set_flex_flow(page, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(page, 2, 0);

    const int outs = m.numOutputs() < 3 ? m.numOutputs() : 3;
    const int total = s_sourceCount + outs;
    lv_obj_update_layout(page);
    const int32_t w = (lv_obj_get_content_width(page) - 2 * (total - 1) - 4) / (total ? total : 1);

    s_stripCount = 0;
    for (int i = 0; i < s_sourceCount; ++i) build_strip(page, s_stripCount++, false, s_sources[i], w);

    lv_obj_t* divider = make_box(page);
    lv_obj_set_size(divider, 2, LV_PCT(100));
    lv_obj_set_style_bg_color(divider, kDim, 0);
    lv_obj_set_style_bg_opa(divider, LV_OPA_50, 0);

    for (int o = 0; o < outs; ++o) build_strip(page, s_stripCount++, true, (uint8_t)o, w);
}

void build_route(lv_obj_t* page)
{
    auto& m = getMixerEngine();
    const int outs = m.numOutputs() < kMixerMaxOutputs ? m.numOutputs() : kMixerMaxOutputs;
    lv_obj_set_flex_flow(page, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(page, 3, 0);

    constexpr int32_t kNameW = 52;

    lv_obj_t* head = make_box(page);
    lv_obj_set_size(head, LV_PCT(100), 14);
    lv_obj_set_flex_flow(head, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(head, 4, 0);
    lv_obj_t* corner = lv_label_create(head);
    lv_obj_set_width(corner, kNameW);
    lv_obj_set_style_text_font(corner, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(corner, kDim, 0);
    lv_label_set_text(corner, "send to");
    for (int o = 0; o < outs; ++o) {
        lv_obj_t* h = lv_label_create(head);
        lv_obj_set_flex_grow(h, 1);
        lv_obj_set_style_text_font(h, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(h, kText, 0);
        lv_obj_set_style_text_align(h, LV_TEXT_ALIGN_CENTER, 0);
        const char* n = m.getOutputName(o);
        lv_label_set_text(h, n[0] ? n : short_output(n, o));
    }

    for (int row = 0; row < s_sourceCount; ++row) {
        const uint8_t ch = s_sources[row];
        const Look* look = look_for(m.getChannelName(ch));

        lv_obj_t* r = make_box(page);
        lv_obj_set_width(r, LV_PCT(100));
        lv_obj_set_flex_grow(r, 1);
        lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(r, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_column(r, 4, 0);

        lv_obj_t* nl = lv_label_create(r);
        lv_obj_set_width(nl, kNameW);
        lv_obj_set_style_text_font(nl, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(nl, look ? lv_color_hex(look->color) : kText, 0);
        lv_obj_set_style_border_width(nl, 2, 0);
        lv_obj_set_style_border_side(nl, LV_BORDER_SIDE_LEFT, 0);
        lv_obj_set_style_border_color(nl, look ? lv_color_hex(look->color) : kText, 0);
        lv_obj_set_style_pad_left(nl, 4, 0);
        lv_label_set_text(nl, look ? look->shortName : m.getChannelName(ch));

        for (int o = 0; o < outs; ++o) {
            lv_obj_t* cell = make_box(r);
            lv_obj_set_height(cell, LV_PCT(90));
            lv_obj_set_flex_grow(cell, 1);
            lv_obj_set_style_bg_color(cell, kStripBg, 0);
            lv_obj_set_style_bg_opa(cell, LV_OPA_COVER, 0);
            lv_obj_set_style_radius(cell, 4, 0);

            lv_obj_t* sl = lv_slider_create(cell);
            lv_obj_set_size(sl, LV_PCT(80), 4);
            lv_obj_align(sl, LV_ALIGN_BOTTOM_MID, 0, -4);
            lv_slider_set_range(sl, 0, 100);
            style_slider(sl, look ? lv_color_hex(look->color) : kAccent);
            lv_obj_set_style_pad_all(sl, 2, LV_PART_KNOB);
            lv_obj_set_ext_click_area(sl, 8);
            lv_obj_add_event_cb(sl, send_cb, LV_EVENT_VALUE_CHANGED, (void*)(intptr_t)((row << 4) | o));
            add_to_group(sl);
            s_sends[row][o] = sl;

            lv_obj_t* vl = lv_label_create(cell);
            lv_obj_set_style_text_font(vl, &lv_font_montserrat_10, 0);
            lv_obj_set_style_text_color(vl, kText, 0);
            lv_obj_align(vl, LV_ALIGN_TOP_MID, 0, 1);
            s_sendLabels[row][o] = vl;
        }
    }
}

lv_obj_t* make_tab(lv_obj_t* parent, const char* text, lv_event_cb_t cb, void* ud, int32_t w)
{
    lv_obj_t* b = lv_button_create(parent);
    lv_obj_set_size(b, w, 18);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_set_style_radius(b, 5, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kTrack, 0);
    lv_obj_set_style_outline_color(b, kFocus, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 2, LV_STATE_FOCUS_KEY);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(l, kText, 0);
    lv_label_set_text(l, text);
    lv_obj_center(l);
    add_to_group(b);
    return b;
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
    s_sourceCount = 0;
    for (uint8_t ch = 0; ch < m.maxChannels() && s_sourceCount < kMaxSources; ++ch) {
        if (m.isChannelActive(ch)) s_sources[s_sourceCount++] = ch;
    }

    s_root = lv_obj_create(parent);
    lv_obj_set_size(s_root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_root, kBg, 0);
    lv_obj_set_style_border_width(s_root, 0, 0);
    lv_obj_set_style_radius(s_root, 0, 0);
    lv_obj_set_style_pad_all(s_root, 3, 0);
    lv_obj_set_style_pad_row(s_root, 3, 0);
    lv_obj_set_flex_flow(s_root, LV_FLEX_FLOW_COLUMN);
    lv_obj_remove_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* bar = make_box(s_root);
    lv_obj_set_size(bar, LV_PCT(100), 18);
    lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(bar, 4, 0);
    make_tab(bar, LV_SYMBOL_LEFT, back_cb, app, 28);
    s_mixTab = make_tab(bar, "MIX", mix_tab_cb, nullptr, 50);
    s_routeTab = make_tab(bar, "ROUTE", route_tab_cb, nullptr, 50);
    lv_obj_t* title = lv_label_create(bar);
    lv_obj_set_flex_grow(title, 1);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(title, kDim, 0);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_RIGHT, 0);
    lv_label_set_text(title, "Mixer / Router");

    s_mixPage = make_box(s_root);
    lv_obj_set_width(s_mixPage, LV_PCT(100));
    lv_obj_set_flex_grow(s_mixPage, 1);
    s_routePage = make_box(s_root);
    lv_obj_set_width(s_routePage, LV_PCT(100));
    lv_obj_set_flex_grow(s_routePage, 1);

    lv_obj_update_layout(s_root);
    build_mix(s_mixPage);
    build_route(s_routePage);
    show_page(false);
    tick_cb(nullptr);

#ifdef MIXER_USE_ENCODER
    if (s_encGroup && s_stripCount) lv_group_focus_obj(s_strips[0].fader);
#endif

    s_timer = lv_timer_create(tick_cb, kTickMs, nullptr);

    if (app) {
        app->setOnShow([](lv_obj_t*) {
            if (s_timer) lv_timer_resume(s_timer);
#ifdef MIXER_USE_ENCODER
            if (indev_encoderTop && s_encGroup) lv_indev_set_group(indev_encoderTop, s_encGroup);
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

    for (auto& s : s_strips) s = Strip{};
    for (auto& row : s_sends) for (auto& c : row) c = nullptr;
    for (auto& row : s_sendLabels) for (auto& c : row) c = nullptr;
    s_stripCount = 0;
    s_root = s_mixPage = s_routePage = s_mixTab = s_routeTab = nullptr;

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
