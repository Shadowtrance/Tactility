#pragma once

#include "app.h"
#include "service_manifest.h"
#include "view_port.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Gui Gui;

void gui_show_app(App app, ViewPortShowCallback on_show, ViewPortHideCallback on_hide);

void gui_hide_app();

void gui_keyboard_show(lv_obj_t* textarea);

void gui_keyboard_hide();

#ifdef __cplusplus
}
#endif
