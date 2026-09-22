/**
 * @file MixerApp.cpp
 * @brief Audio Mixer/Router LVGL app — cross-platform GUI + registration.
 *
 * Single MixerApp source compiled on both PC and ESP-IDF. The AudioMixerEngine
 * runs globally and is bootstrapped by each platform's init code (registers
 * channels). This app is a pure view/controller — connects to the global
 * engine for display + control, disconnects on close.
 *
 * GUI layout (320x240 — same on PC and IDF):
 *   - Title bar (22px)
 *   - VU meters section (70px): per-channel + per-output bars
 *   - Routing matrix (auto-sized): N channels × M outputs
 *   - Channel strips (72px): volume slider + [M][S] per channel
 *   - Output masters (24px): per-output volume + mute
 *
 * Persistence: MixerApp calls AudioMixerEngine::notifyStateChanged() after
 * any GUI mutation. Platform bootstrap wires that to its own save sink
 * (PC writes JSON, IDF writes NVS — neither concern of this file).
 */

#include "crosspad-mixer/MixerApp.hpp"
#include "crosspad-mixer/AudioMixerEngine.hpp"
#include "crosspad-mixer/MixerPadLogic.hpp"

// `class App` lives in different headers on the two platforms but both derive
// from crosspad_gui::AppBase<App>, so the lifecycle methods used here
// (setOnShow / setOnHide / destroyApp) are the same.
#if __has_include("pc_stubs/PcApp.hpp")
  #include "pc_stubs/PcApp.hpp"
#else
  #include "crosspad-gui/app/App.hpp"
#endif

#include "crosspad/app/AppRegistrar.hpp"
#include "crosspad/pad/PadManager.hpp"
#include <memory>
#include <crosspad/app/AppRegistry.hpp>
#include <crosspad/pad/PadManager.hpp>
#include <crosspad/platform/PlatformServices.hpp>
#include "crosspad-gui/components/vu_meter.h"

#include "lvgl.h"

#include <cstdio>

// Encoder navigation is an embedded-only concern (PC uses pointer/touch via SDL).
// On IDF the launcher exposes indev_encoderTop globally. We snapshot whatever
// group was active before the mixer opened (typically the launcher's fresh
// app_group) and restore it on hide/destroy, instead of trusting a stale
// encoderTop_group that never tracks LoadMainScreen rebuilds.
#ifdef ESP_PLATFORM
extern "C" {
  extern lv_indev_t* indev_encoderTop;
}
#define MIXER_USE_ENCODER 1
#endif

/* ── Static state ──────────────────────────────────────────────────────── */

namespace {

constexpr uint8_t kMaxVisibleChannels = 4;   // GUI rows (3 fits 320px nicely; 4 uses scroll)
constexpr uint8_t kMaxVisibleOutputs  = 2;   // VU + matrix columns

App*         s_thisApp = nullptr;
lv_timer_t*  s_vuTimer = nullptr;

#ifdef MIXER_USE_ENCODER
lv_group_t*  s_encGroup     = nullptr;
lv_group_t*  s_prevEncGroup = nullptr;  // launcher group at create time
#endif

// VU bar objects: [0..numCh-1] = channels, [numCh..numCh+numOut-1] = outputs.
// Sized for max layout; overflow is ignored.
constexpr uint8_t kMaxVuBars = kMaxVisibleChannels + kMaxVisibleOutputs;
lv_obj_t*    s_vuBarsL [kMaxVuBars] = {};
lv_obj_t*    s_vuBarsR [kMaxVuBars] = {};

// Routing matrix — [channel][output]
lv_obj_t*    s_routeButtons[kMaxVisibleChannels][kMaxVisibleOutputs] = {};

// Channel strip controls
lv_obj_t*    s_muteButtons   [kMaxVisibleChannels] = {};
lv_obj_t*    s_soloButtons   [kMaxVisibleChannels] = {};
lv_obj_t*    s_channelSliders[kMaxVisibleChannels] = {};

// Output controls
lv_obj_t*    s_outSliders   [kMaxVisibleOutputs] = {};
lv_obj_t*    s_outMuteButtons[kMaxVisibleOutputs] = {};

int16_t      s_vuDecay[kMaxVuBars][2] = {};

uint8_t      s_visCh  = 0;   // captured at create time
uint8_t      s_visOut = 0;

const lv_color_t COL_BG        = lv_color_hex(0x000000);
const lv_color_t COL_SLIDER_BG = lv_color_hex(0x222244);
const lv_color_t COL_SLIDER_IND= lv_color_hex(0x6644AA);
const lv_color_t COL_KNOB      = lv_color_hex(0x9966FF);
const lv_color_t COL_MUTE_ON   = lv_color_hex(0xCC2222);
const lv_color_t COL_MUTE_OFF  = lv_color_hex(0x333333);
const lv_color_t COL_SOLO_ON   = lv_color_hex(0xCCAA00);
const lv_color_t COL_SOLO_OFF  = lv_color_hex(0x333333);
const lv_color_t COL_ROUTE_ON  = lv_color_hex(0x0099AA);
const lv_color_t COL_ROUTE_OFF = lv_color_hex(0x222222);
const lv_color_t COL_LABEL     = lv_color_hex(0x999999);
const lv_color_t COL_VU_BG     = lv_color_hex(0x111111);

void syncGuiFromEngine();

// No-op on PC, registers widget with the encoder focus group on IDF.
inline void encReg(lv_obj_t* obj) {
#ifdef MIXER_USE_ENCODER
    if (s_encGroup && obj) lv_group_add_obj(s_encGroup, obj);
#else
    (void)obj;
#endif
}

inline lv_color_t vuColor(int16_t level, int16_t max) {
    if (max <= 0) return lv_color_hex(0x00AA00);
    float pct = (float)level / (float)max;
    if (pct > 0.85f) return lv_color_hex(0xFF2222);
    if (pct > 0.60f) return lv_color_hex(0xDDAA00);
    return lv_color_hex(0x00AA00);
}

// Pack a (kind, index) pair into a single intptr_t carried as user_data.
// kind=0 channel, kind=1 output, kind=2 channel|output route.
inline void* makeUd(uint8_t kind, uint8_t a, uint8_t b = 0) {
    return reinterpret_cast<void*>(
        static_cast<uintptr_t>(kind) << 16 |
        static_cast<uintptr_t>(a)    << 8  |
        static_cast<uintptr_t>(b));
}
inline uint8_t udKind(void* p) { return (reinterpret_cast<uintptr_t>(p) >> 16) & 0xFF; }
inline uint8_t udA   (void* p) { return (reinterpret_cast<uintptr_t>(p) >> 8)  & 0xFF; }
inline uint8_t udB   (void* p) { return  reinterpret_cast<uintptr_t>(p)        & 0xFF; }

/* ── Callbacks ─────────────────────────────────────────────────────────── */

void on_close(lv_event_t*) {
    if (s_thisApp) s_thisApp->destroyApp();
}

void on_route_toggle(lv_event_t* e) {
    auto& engine = getMixerEngine();
    void* ud = lv_event_get_user_data(e);
    uint8_t ch  = udA(ud);
    uint8_t out = udB(ud);
    engine.setRouteEnabled(ch, out, !engine.isRouteEnabled(ch, out));
    syncGuiFromEngine();
    engine.notifyStateChanged();
}

void on_channel_mute(lv_event_t* e) {
    auto& engine = getMixerEngine();
    uint8_t ch = udA(lv_event_get_user_data(e));
    engine.setChannelMute(ch, !engine.isChannelMuted(ch));
    syncGuiFromEngine();
    engine.notifyStateChanged();
}

void on_channel_solo(lv_event_t* e) {
    auto& engine = getMixerEngine();
    uint8_t ch = udA(lv_event_get_user_data(e));
    engine.setChannelSolo(ch, !engine.isChannelSoloed(ch));
    syncGuiFromEngine();
    engine.notifyStateChanged();
}

void on_channel_volume(lv_event_t* e) {
    auto& engine = getMixerEngine();
    auto* slider = static_cast<lv_obj_t*>(lv_event_get_target(e));
    uint8_t ch = udA(lv_event_get_user_data(e));
    engine.setChannelVolume(ch, lv_slider_get_value(slider) / 100.0f);
    engine.notifyStateChanged();
}

void on_output_volume(lv_event_t* e) {
    auto& engine = getMixerEngine();
    auto* slider = static_cast<lv_obj_t*>(lv_event_get_target(e));
    uint8_t out = udA(lv_event_get_user_data(e));
    engine.setOutputVolume(out, lv_slider_get_value(slider) / 100.0f);
    engine.notifyStateChanged();
}

void on_output_mute(lv_event_t* e) {
    auto& engine = getMixerEngine();
    uint8_t out = udA(lv_event_get_user_data(e));
    engine.setOutputMute(out, !engine.isOutputMuted(out));
    syncGuiFromEngine();
    engine.notifyStateChanged();
}

/* ── Sync GUI state from engine ────────────────────────────────────────── */

void syncGuiFromEngine() {
    auto& engine = getMixerEngine();
    for (uint8_t ch = 0; ch < s_visCh; ++ch) {
        for (uint8_t out = 0; out < s_visOut; ++out) {
            if (!s_routeButtons[ch][out]) continue;
            bool en = engine.isRouteEnabled(ch, out);
            lv_obj_set_style_bg_color(s_routeButtons[ch][out],
                                      en ? COL_ROUTE_ON : COL_ROUTE_OFF, 0);
        }
        if (s_muteButtons[ch]) {
            lv_obj_set_style_bg_color(s_muteButtons[ch],
                engine.isChannelMuted(ch) ? COL_MUTE_ON : COL_MUTE_OFF, 0);
        }
        if (s_soloButtons[ch]) {
            lv_obj_set_style_bg_color(s_soloButtons[ch],
                engine.isChannelSoloed(ch) ? COL_SOLO_ON : COL_SOLO_OFF, 0);
        }
    }
    for (uint8_t out = 0; out < s_visOut; ++out) {
        if (!s_outMuteButtons[out]) continue;
        lv_obj_set_style_bg_color(s_outMuteButtons[out],
            engine.isOutputMuted(out) ? COL_MUTE_ON : COL_MUTE_OFF, 0);
    }
}

/* ── VU meter timer (60 Hz) ────────────────────────────────────────────── */

void vu_timer_cb(lv_timer_t*) {
    auto& engine = getMixerEngine();
    syncGuiFromEngine();   // pad-driven changes show up here too

    constexpr int DECAY = 230;
    constexpr int16_t BAR_MAX = 100;

    for (uint8_t ch = 0; ch < s_visCh; ++ch) {
        if (!s_vuBarsL[ch]) continue;
        int16_t rawL, rawR;
        engine.getChannelLevel(ch, rawL, rawR);
        int16_t logL = (int16_t)int16toLogScale(rawL, BAR_MAX);
        int16_t logR = (int16_t)int16toLogScale(rawR, BAR_MAX);
        s_vuDecay[ch][0] = (s_vuDecay[ch][0] * DECAY) / 256;
        s_vuDecay[ch][1] = (s_vuDecay[ch][1] * DECAY) / 256;
        if (logL > s_vuDecay[ch][0]) s_vuDecay[ch][0] = logL;
        if (logR > s_vuDecay[ch][1]) s_vuDecay[ch][1] = logR;
        lv_bar_set_value(s_vuBarsL[ch], s_vuDecay[ch][0], LV_ANIM_OFF);
        lv_bar_set_value(s_vuBarsR[ch], s_vuDecay[ch][1], LV_ANIM_OFF);
        lv_obj_set_style_bg_color(s_vuBarsL[ch], vuColor(s_vuDecay[ch][0], BAR_MAX), LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(s_vuBarsR[ch], vuColor(s_vuDecay[ch][1], BAR_MAX), LV_PART_INDICATOR);
    }
    for (uint8_t out = 0; out < s_visOut; ++out) {
        uint8_t idx = s_visCh + out;
        if (idx >= kMaxVuBars || !s_vuBarsL[idx]) continue;
        int16_t rawL, rawR;
        engine.getOutputLevel(out, rawL, rawR);
        int16_t logL = (int16_t)int16toLogScale(rawL, BAR_MAX);
        int16_t logR = (int16_t)int16toLogScale(rawR, BAR_MAX);
        s_vuDecay[idx][0] = (s_vuDecay[idx][0] * DECAY) / 256;
        s_vuDecay[idx][1] = (s_vuDecay[idx][1] * DECAY) / 256;
        if (logL > s_vuDecay[idx][0]) s_vuDecay[idx][0] = logL;
        if (logR > s_vuDecay[idx][1]) s_vuDecay[idx][1] = logR;
        lv_bar_set_value(s_vuBarsL[idx], s_vuDecay[idx][0], LV_ANIM_OFF);
        lv_bar_set_value(s_vuBarsR[idx], s_vuDecay[idx][1], LV_ANIM_OFF);
        lv_obj_set_style_bg_color(s_vuBarsL[idx], vuColor(s_vuDecay[idx][0], BAR_MAX), LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(s_vuBarsR[idx], vuColor(s_vuDecay[idx][1], BAR_MAX), LV_PART_INDICATOR);
    }
}

/* ── GUI builder helpers ───────────────────────────────────────────────── */

lv_obj_t* make_vu_pair(lv_obj_t* parent, uint8_t idx, const char* label, int32_t x, int32_t barH) {
    lv_obj_t* cont = lv_obj_create(parent);
    lv_obj_set_size(cont, 38, barH + 14);
    lv_obj_set_pos(cont, x, 0);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, 0, 0);
    lv_obj_remove_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* barL = lv_bar_create(cont);
    lv_obj_set_size(barL, 8, barH);
    lv_obj_set_pos(barL, 8, 0);
    lv_bar_set_range(barL, 0, 100);
    lv_obj_set_style_bg_color(barL, COL_VU_BG, 0);
    lv_obj_set_style_bg_color(barL, lv_color_hex(0x00AA00), LV_PART_INDICATOR);
    lv_obj_set_style_radius(barL, 1, 0);
    lv_obj_set_style_radius(barL, 1, LV_PART_INDICATOR);
    s_vuBarsL[idx] = barL;

    lv_obj_t* barR = lv_bar_create(cont);
    lv_obj_set_size(barR, 8, barH);
    lv_obj_set_pos(barR, 20, 0);
    lv_bar_set_range(barR, 0, 100);
    lv_obj_set_style_bg_color(barR, COL_VU_BG, 0);
    lv_obj_set_style_bg_color(barR, lv_color_hex(0x00AA00), LV_PART_INDICATOR);
    lv_obj_set_style_radius(barR, 1, 0);
    lv_obj_set_style_radius(barR, 1, LV_PART_INDICATOR);
    s_vuBarsR[idx] = barR;

    lv_obj_t* lbl = lv_label_create(cont);
    lv_label_set_text(lbl, label);
    lv_obj_set_style_text_color(lbl, COL_LABEL, 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
    lv_obj_align(lbl, LV_ALIGN_BOTTOM_MID, 0, 0);
    return cont;
}

lv_obj_t* make_small_button(lv_obj_t* parent, const char* text, lv_color_t bg,
                            lv_event_cb_t cb, void* userData) {
    lv_obj_t* btn = lv_button_create(parent);
    lv_obj_set_size(btn, 22, 18);
    lv_obj_set_style_bg_color(btn, bg, 0);
    lv_obj_set_style_radius(btn, 3, 0);
    lv_obj_set_style_shadow_width(btn, 0, 0);
    lv_obj_set_style_pad_all(btn, 0, 0);
    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    lv_obj_center(lbl);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, userData);
    encReg(btn);
    return btn;
}

// Short channel name (max 4 chars) for compact UI labels.
const char* shortName(const char* full, char* buf, size_t cap) {
    if (!full || !*full) return "?";
    size_t n = 0;
    while (full[n] && n < cap - 1 && n < 4) { buf[n] = full[n]; ++n; }
    buf[n] = 0;
    return buf;
}

} // namespace

/* ── App create / destroy ──────────────────────────────────────────────── */

lv_obj_t* Mixer_create(lv_obj_t* parent, App* a) {
    s_thisApp = a;
    auto& engine = getMixerEngine();

    // Register MixerPadLogic with PadManager so pushPadLogic("Mixer") (framework-
    // owned, see REGISTER_APP_PL below) resolves to an actual handler.
    // Constructed once on first open; reused on subsequent opens. Empty
    // deleter — lifetime owned by the static.
    static MixerPadLogic s_mixerPadLogic(engine);
    static auto s_mixerPadLogicPtr =
        std::shared_ptr<MixerPadLogic>(&s_mixerPadLogic, [](MixerPadLogic*){});
    crosspad::getPadManager().registerPadLogic("Mixer", s_mixerPadLogicPtr);

#ifdef MIXER_USE_ENCODER
    if (indev_encoderTop) {
        s_prevEncGroup = lv_indev_get_group(indev_encoderTop);
    }
    if (!s_encGroup) s_encGroup = lv_group_create();
    if (indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_encGroup);
#endif

    // Snapshot active channels / outputs at create time.
    s_visCh  = engine.numActiveChannels();
    if (s_visCh > kMaxVisibleChannels) s_visCh = kMaxVisibleChannels;
    s_visOut = engine.numOutputs();
    if (s_visOut > kMaxVisibleOutputs) s_visOut = kMaxVisibleOutputs;

    // Remap iteration index → real ChannelId (skip inactive slots).
    uint8_t chMap[kMaxVisibleChannels] = {};
    {
        uint8_t i = 0;
        for (uint8_t id = 0; id < kMixerMaxChannels && i < s_visCh; ++id) {
            if (engine.isChannelActive(id)) chMap[i++] = id;
        }
    }

    for (auto& d : s_vuDecay) { d[0] = 0; d[1] = 0; }

    /* Root container */
    lv_obj_t* cont = lv_obj_create(parent);
    lv_obj_set_size(cont, lv_pct(100), lv_pct(100));
    lv_obj_set_style_bg_color(cont, COL_BG, 0);
    lv_obj_set_style_bg_opa(cont, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(cont, 2, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(cont, 1, 0);
    lv_obj_remove_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    /* Title bar */
    lv_obj_t* titleBar = lv_obj_create(cont);
    lv_obj_set_size(titleBar, lv_pct(100), 22);
    lv_obj_set_style_bg_opa(titleBar, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(titleBar, 0, 0);
    lv_obj_set_style_pad_all(titleBar, 0, 0);
    lv_obj_remove_flag(titleBar, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* titleLabel = lv_label_create(titleBar);
    lv_label_set_text(titleLabel, "Mixer");
    lv_obj_set_style_text_color(titleLabel, lv_color_white(), 0);
    lv_obj_set_style_text_font(titleLabel, &lv_font_montserrat_14, 0);
    lv_obj_align(titleLabel, LV_ALIGN_LEFT_MID, 4, 0);

    lv_obj_t* closeBtn = lv_button_create(titleBar);
    lv_obj_set_size(closeBtn, 28, 18);
    lv_obj_align(closeBtn, LV_ALIGN_RIGHT_MID, -2, 0);
    lv_obj_set_style_bg_color(closeBtn, lv_color_hex(0x662222), 0);
    lv_obj_set_style_radius(closeBtn, 4, 0);
    lv_obj_set_style_shadow_width(closeBtn, 0, 0);
    lv_obj_t* closeLbl = lv_label_create(closeBtn);
    lv_label_set_text(closeLbl, "X");
    lv_obj_set_style_text_font(closeLbl, &lv_font_montserrat_14, 0);
    lv_obj_center(closeLbl);
    lv_obj_add_event_cb(closeBtn, on_close, LV_EVENT_CLICKED, nullptr);
    encReg(closeBtn);

    /* VU meters section (channels + outputs) */
    lv_obj_t* vuSection = lv_obj_create(cont);
    lv_obj_set_size(vuSection, lv_pct(100), 70);
    lv_obj_set_style_bg_opa(vuSection, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(vuSection, 0, 0);
    lv_obj_set_style_pad_all(vuSection, 0, 0);
    lv_obj_remove_flag(vuSection, LV_OBJ_FLAG_SCROLLABLE);

    char nameBuf[8];
    for (uint8_t i = 0; i < s_visCh; ++i) {
        const char* nm = shortName(engine.getChannelName(chMap[i]), nameBuf, sizeof(nameBuf));
        make_vu_pair(vuSection, i, nm, 4 + i * 44, 52);
    }
    for (uint8_t i = 0; i < s_visOut; ++i) {
        char buf[8]; std::snprintf(buf, sizeof(buf), "OUT%u", i + 1);
        make_vu_pair(vuSection, s_visCh + i, buf, 172 + i * 52, 52);
    }
    if (s_visCh > 0) {
        lv_obj_t* sep = lv_obj_create(vuSection);
        lv_obj_set_size(sep, 1, 50);
        lv_obj_set_pos(sep, 165, 4);
        lv_obj_set_style_bg_color(sep, lv_color_hex(0x444444), 0);
        lv_obj_set_style_bg_opa(sep, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(sep, 0, 0);
        lv_obj_remove_flag(sep, LV_OBJ_FLAG_SCROLLABLE);
    }

    /* Routing matrix */
    const int routeRows = s_visCh;
    const int routeH = 12 + routeRows * 14;
    lv_obj_t* routeSection = lv_obj_create(cont);
    lv_obj_set_size(routeSection, lv_pct(100), routeH);
    lv_obj_set_style_bg_opa(routeSection, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(routeSection, 0, 0);
    lv_obj_set_style_pad_all(routeSection, 0, 0);
    lv_obj_remove_flag(routeSection, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* routeHdr = lv_label_create(routeSection);
    lv_label_set_text(routeHdr, "Routing");
    lv_obj_set_style_text_color(routeHdr, lv_color_hex(0x78C8FF), 0);
    lv_obj_set_style_text_font(routeHdr, &lv_font_montserrat_10, 0);
    lv_obj_set_pos(routeHdr, 4, 0);

    for (uint8_t out = 0; out < s_visOut; ++out) {
        lv_obj_t* lbl = lv_label_create(routeSection);
        char buf[8]; std::snprintf(buf, sizeof(buf), "OUT%u", out + 1);
        lv_label_set_text(lbl, buf);
        lv_obj_set_style_text_color(lbl, COL_LABEL, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
        lv_obj_set_pos(lbl, 70 + out * 120, 0);
    }
    for (uint8_t i = 0; i < s_visCh; ++i) {
        int y = 13 + i * 13;
        lv_obj_t* lbl = lv_label_create(routeSection);
        const char* nm = shortName(engine.getChannelName(chMap[i]), nameBuf, sizeof(nameBuf));
        lv_label_set_text(lbl, nm);
        lv_obj_set_style_text_color(lbl, COL_LABEL, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
        lv_obj_set_pos(lbl, 4, y);

        for (uint8_t out = 0; out < s_visOut; ++out) {
            lv_obj_t* btn = lv_button_create(routeSection);
            lv_obj_set_size(btn, 28, 11);
            lv_obj_set_pos(btn, 70 + out * 120, y - 1);
            lv_obj_set_style_bg_color(btn, COL_ROUTE_OFF, 0);
            lv_obj_set_style_radius(btn, 2, 0);
            lv_obj_set_style_shadow_width(btn, 0, 0);
            lv_obj_set_style_pad_all(btn, 0, 0);
            lv_obj_add_event_cb(btn, on_route_toggle, LV_EVENT_CLICKED,
                                makeUd(2, chMap[i], out));
            encReg(btn);
            s_routeButtons[i][out] = btn;
        }
    }

    /* Channel strips: volume slider + Mute + Solo */
    const int stripH = 22 + s_visCh * 22;
    lv_obj_t* stripSection = lv_obj_create(cont);
    lv_obj_set_size(stripSection, lv_pct(100), stripH);
    lv_obj_set_style_bg_opa(stripSection, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(stripSection, 0, 0);
    lv_obj_set_style_pad_all(stripSection, 0, 0);
    lv_obj_remove_flag(stripSection, LV_OBJ_FLAG_SCROLLABLE);

    for (uint8_t i = 0; i < s_visCh; ++i) {
        int y = i * 22;
        lv_obj_t* lbl = lv_label_create(stripSection);
        const char* nm = shortName(engine.getChannelName(chMap[i]), nameBuf, sizeof(nameBuf));
        lv_label_set_text(lbl, nm);
        lv_obj_set_style_text_color(lbl, COL_LABEL, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
        lv_obj_set_pos(lbl, 4, y + 4);

        lv_obj_t* sl = lv_slider_create(stripSection);
        lv_obj_set_size(sl, 180, 12);
        lv_obj_set_pos(sl, 36, y + 4);
        lv_slider_set_range(sl, 0, 100);
        lv_slider_set_value(sl, (int)(engine.getChannelVolume(chMap[i]) * 100.0f), LV_ANIM_OFF);
        lv_obj_set_style_bg_color(sl, COL_SLIDER_BG, 0);
        lv_obj_set_style_bg_color(sl, COL_SLIDER_IND, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(sl, COL_KNOB, LV_PART_KNOB);
        lv_obj_add_event_cb(sl, on_channel_volume, LV_EVENT_VALUE_CHANGED, makeUd(0, chMap[i]));
        encReg(sl);
        s_channelSliders[i] = sl;

        s_muteButtons[i] = make_small_button(stripSection, "M", COL_MUTE_OFF,
                                             on_channel_mute, makeUd(0, chMap[i]));
        lv_obj_set_pos(s_muteButtons[i], 224, y + 2);
        s_soloButtons[i] = make_small_button(stripSection, "S", COL_SOLO_OFF,
                                             on_channel_solo, makeUd(0, chMap[i]));
        lv_obj_set_pos(s_soloButtons[i], 250, y + 2);
    }

    /* Output masters */
    lv_obj_t* outRow = lv_obj_create(cont);
    lv_obj_set_size(outRow, lv_pct(100), 24);
    lv_obj_set_style_bg_opa(outRow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(outRow, 0, 0);
    lv_obj_set_style_pad_all(outRow, 0, 0);
    lv_obj_remove_flag(outRow, LV_OBJ_FLAG_SCROLLABLE);

    for (uint8_t out = 0; out < s_visOut; ++out) {
        int xBase = 8 + out * 152;
        char buf[8]; std::snprintf(buf, sizeof(buf), "O%u", out + 1);
        lv_obj_t* lbl = lv_label_create(outRow);
        lv_label_set_text(lbl, buf);
        lv_obj_set_style_text_color(lbl, COL_LABEL, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
        lv_obj_set_pos(lbl, xBase, 6);

        lv_obj_t* sl = lv_slider_create(outRow);
        lv_obj_set_size(sl, 96, 12);
        lv_obj_set_pos(sl, xBase + 26, 4);
        lv_slider_set_range(sl, 0, 100);
        lv_slider_set_value(sl, (int)(engine.getOutputVolume(out) * 100.0f), LV_ANIM_OFF);
        lv_obj_set_style_bg_color(sl, COL_SLIDER_BG, 0);
        lv_obj_set_style_bg_color(sl, COL_SLIDER_IND, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(sl, COL_KNOB, LV_PART_KNOB);
        lv_obj_add_event_cb(sl, on_output_volume, LV_EVENT_VALUE_CHANGED, makeUd(1, out));
        encReg(sl);
        s_outSliders[out] = sl;

        s_outMuteButtons[out] = make_small_button(outRow, "M", COL_MUTE_OFF,
                                                  on_output_mute, makeUd(1, out));
        lv_obj_set_pos(s_outMuteButtons[out], xBase + 128, 2);
    }

    /* Lifecycle: pad logic activation/restore is framework-owned (see
     * REGISTER_APP_PL below) — these callbacks only manage encoder focus. */
    if (a) {
        a->setOnShow([](lv_obj_t*) {
#ifdef MIXER_USE_ENCODER
            if (s_encGroup && indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_encGroup);
#endif
        });
        a->setOnHide([](lv_obj_t*) {
#ifdef MIXER_USE_ENCODER
            if (indev_encoderTop && s_prevEncGroup) {
                lv_indev_set_group(indev_encoderTop, s_prevEncGroup);
            }
#endif
        });
    }

    s_vuTimer = lv_timer_create(vu_timer_cb, 16, nullptr);
    syncGuiFromEngine();
    return cont;
}

void Mixer_destroy(lv_obj_t* app_obj) {
    crosspad::getPadManager().unregisterPadLogic("Mixer");
    if (s_vuTimer) {
        lv_timer_delete(s_vuTimer);
        s_vuTimer = nullptr;
    }
    s_thisApp = nullptr;
    for (auto& p : s_vuBarsL)        p = nullptr;
    for (auto& p : s_vuBarsR)        p = nullptr;
    for (auto& row : s_routeButtons) for (auto& p : row) p = nullptr;
    for (auto& p : s_muteButtons)    p = nullptr;
    for (auto& p : s_soloButtons)    p = nullptr;
    for (auto& p : s_channelSliders) p = nullptr;
    for (auto& p : s_outSliders)     p = nullptr;
    for (auto& p : s_outMuteButtons) p = nullptr;
    s_visCh = 0;
    s_visOut = 0;
#ifdef MIXER_USE_ENCODER
    if (indev_encoderTop && s_prevEncGroup) {
        lv_indev_set_group(indev_encoderTop, s_prevEncGroup);
    }
    if (s_encGroup) {
        lv_group_delete(s_encGroup);
        s_encGroup = nullptr;
    }
    s_prevEncGroup = nullptr;
#endif
    lv_obj_delete_async(app_obj);
}

REGISTER_APP_PL(Mixer, nullptr, "mixer.png",
                Mixer_create, Mixer_destroy,
                nullptr, nullptr, nullptr, 0, "Mixer", true)
