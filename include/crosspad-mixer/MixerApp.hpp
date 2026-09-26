#pragma once

/**
 * @file MixerApp.hpp
 * @brief Mixer and signal router: a channel strip per source, a master per
 * output, and a send matrix between them.
 */

class App;

// Pull the real LVGL forward decls instead of redeclaring lv_obj_t locally —
// the underlying struct tag differs between LVGL 9.2 (Arduino-PlatformIO) and
// LVGL 9.3 (ESP-IDF), and a hand-rolled typedef collides with whichever one
// the consumer happens to include.
#include "lvgl.h"

lv_obj_t* Mixer_create(lv_obj_t* parent, App* app);
void Mixer_destroy(lv_obj_t* app_obj);
