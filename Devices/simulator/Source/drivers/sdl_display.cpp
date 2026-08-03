// SPDX-License-Identifier: Apache-2.0
#include "sdl_display.h"

#include <tactility/device.h>
#include <tactility/driver.h>
#include <tactility/drivers/display.h>
#include <tactility/log.h>
#include <tactility/module.h>

#include <SDL2/SDL.h>

#include <cstdlib>
#include <cstring>

constexpr auto* TAG = "SdlDisplay";
#define GET_CONFIG(device) (static_cast<const SdlDisplayConfig*>((device)->config))

struct SdlDisplayInternal {
    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_Texture* texture;
    /**
     * Full-frame composite buffer, RGB565.
     *
     * LVGL renders in partial tiles - only the regions it considers dirty - so the texture
     * cannot be driven tile by tile: everything not redrawn this frame would be whatever
     * the texture happened to hold. Tiles are composited here instead, and the whole
     * buffer is uploaded once per frame. This mirrors what LVGL's own SDL driver does with
     * its `fb_act` (see Libraries/lvgl/src/drivers/sdl/lv_sdl_window.c), which is what the
     * simulator used before the devicetree rewrite.
     */
    uint16_t* frame_buffer;
    /**
     * The same frame converted to ARGB8888, which is what actually gets uploaded.
     *
     * SDL creates an RGB565 texture happily, but the renderer does not necessarily support
     * sampling one: the opengl backend advertises only ARGB8888/ABGR8888/RGB888/BGR888 and
     * the YUV formats, and silently renders black for anything else. That is what made the
     * simulator show a black window while the frame buffer held a perfectly good UI.
     */
    uint32_t* upload_buffer;
};

/** RGB565 -> ARGB8888, opaque. */
static inline uint32_t rgb565_to_argb8888(uint16_t pixel) {
    // Replicate the high bits into the low ones so full-scale input maps to full-scale
    // output (0x1F -> 0xFF, not 0xF8), which keeps whites white rather than slightly grey.
    const uint32_t r = ((pixel >> 11) & 0x1F);
    const uint32_t g = ((pixel >> 5) & 0x3F);
    const uint32_t b = (pixel & 0x1F);
    const uint32_t r8 = (r << 3) | (r >> 2);
    const uint32_t g8 = (g << 2) | (g >> 4);
    const uint32_t b8 = (b << 3) | (b >> 2);
    return 0xFF000000u | (r8 << 16) | (g8 << 8) | b8;
}

/**
 * Creates the renderer.
 *
 * Software by default, and deliberately so. The accelerated (opengl) backend does not
 * display anything under WSLg: every call succeeds, SDL_RenderReadPixels even reads back
 * the expected pixels, and the window still shows solid black - so the failure cannot be
 * detected at runtime, only observed. The software renderer works there and everywhere
 * else, and at simulator resolutions it costs nothing measurable.
 *
 * Set TACTILITY_SDL_ACCELERATED=1 to opt back into the accelerated path (native Linux
 * with a real GPU, where it does work).
 */
static SDL_Renderer* create_renderer(SDL_Window* window) {
    const char* accelerated = getenv("TACTILITY_SDL_ACCELERATED");
    const Uint32 flags = (accelerated != nullptr && accelerated[0] == '1')
        ? SDL_RENDERER_ACCELERATED
        : SDL_RENDERER_SOFTWARE;

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, flags);
    if (renderer == nullptr && flags == SDL_RENDERER_ACCELERATED) {
        LOG_W(TAG, "Accelerated renderer unavailable, using software: %s", SDL_GetError());
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
    }

    if (renderer != nullptr) {
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);

        // Logged because a broken video stack shows up as a black window with no error
        // anywhere - knowing the driver and renderer in use is the first useful question.
        SDL_RendererInfo info;
        LOG_I(TAG, "SDL video driver='%s' renderer='%s'",
              SDL_GetCurrentVideoDriver() ? SDL_GetCurrentVideoDriver() : "?",
              SDL_GetRendererInfo(renderer, &info) == 0 ? info.name : "?");
    }
    return renderer;
}

// region Driver lifecycle

static error_t start(Device* device) {
    const auto* config = GET_CONFIG(device);

    auto* internal = static_cast<SdlDisplayInternal*>(malloc(sizeof(SdlDisplayInternal)));
    if (internal == nullptr) {
        return ERROR_OUT_OF_MEMORY;
    }

    if (SDL_InitSubSystem(SDL_INIT_VIDEO) != 0) {
        LOG_E(TAG, "SDL_InitSubSystem failed: %s", SDL_GetError());
        free(internal);
        return ERROR_RESOURCE;
    }

    internal->window = SDL_CreateWindow(
        "Tactility",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        config->horizontal_resolution, config->vertical_resolution,
        SDL_WINDOW_SHOWN
    );
    internal->renderer = internal->window != nullptr
        ? create_renderer(internal->window)
        : nullptr;
    // ARGB8888 rather than RGB565: every SDL renderer backend supports it, whereas RGB565
    // is not in the opengl backend's format list and renders black without reporting an
    // error. LVGL still hands us RGB565; the conversion happens on upload.
    internal->texture = internal->renderer != nullptr
        ? SDL_CreateTexture(internal->renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
            config->horizontal_resolution, config->vertical_resolution)
        : nullptr;

    if (internal->window == nullptr || internal->renderer == nullptr || internal->texture == nullptr) {
        LOG_E(TAG, "Failed to create SDL window: %s", SDL_GetError());
        if (internal->texture != nullptr) SDL_DestroyTexture(internal->texture);
        if (internal->renderer != nullptr) SDL_DestroyRenderer(internal->renderer);
        if (internal->window != nullptr) SDL_DestroyWindow(internal->window);
        SDL_QuitSubSystem(SDL_INIT_VIDEO);
        free(internal);
        return ERROR_RESOURCE;
    }

    // calloc, so regions LVGL never redraws start black rather than showing whatever was
    // in the heap.
    const size_t pixel_count = (size_t)config->horizontal_resolution * config->vertical_resolution;
    internal->frame_buffer = static_cast<uint16_t*>(calloc(pixel_count, sizeof(uint16_t)));
    internal->upload_buffer = static_cast<uint32_t*>(calloc(pixel_count, sizeof(uint32_t)));

    if (internal->frame_buffer == nullptr || internal->upload_buffer == nullptr) {
        free(internal->frame_buffer);
        free(internal->upload_buffer);
        SDL_DestroyTexture(internal->texture);
        SDL_DestroyRenderer(internal->renderer);
        SDL_DestroyWindow(internal->window);
        SDL_QuitSubSystem(SDL_INIT_VIDEO);
        free(internal);
        return ERROR_OUT_OF_MEMORY;
    }

    device_set_driver_data(device, internal);
    return ERROR_NONE;
}

static error_t stop(Device* device) {
    auto* internal = static_cast<SdlDisplayInternal*>(device_get_driver_data(device));

    free(internal->frame_buffer);
    free(internal->upload_buffer);
    SDL_DestroyTexture(internal->texture);
    SDL_DestroyRenderer(internal->renderer);
    SDL_DestroyWindow(internal->window);
    SDL_QuitSubSystem(SDL_INIT_VIDEO);

    free(internal);
    device_set_driver_data(device, nullptr);
    return ERROR_NONE;
}

// endregion

// region DisplayApi

static error_t sdl_display_reset(Device*) { return ERROR_NONE; }
static error_t sdl_display_init(Device*) { return ERROR_NONE; }

/**
 * LVGL renders in partial tiles, so this is called several times per frame - once per
 * dirty region, e.g. (0,0)-(320,24) then (0,24)-(320,48).
 *
 * Each tile is composited into a full-frame buffer, and the *whole* buffer is uploaded and
 * presented. Pushing tiles straight to the texture is what made the simulator flash
 * colours and then go black: SDL_RenderClear wipes the entire target, so every call showed
 * a single strip against an otherwise cleared window, and the frame settled on whichever
 * strip arrived last.
 *
 * Presenting on every tile rather than only on the last one is deliberate. DisplayApi has
 * no end-of-frame callback - LVGL's own SDL driver uses lv_display_flush_is_last(), which
 * is not exposed here - and guessing the last tile from its coordinates is unreliable,
 * since a partial redraw need not touch the bottom-right corner. Re-uploading the frame is
 * a 150 KB memcpy at 320x240 on a desktop, which costs nothing that matters here, and it
 * is always correct.
 */
static error_t sdl_display_draw_bitmap(Device* device, int32_t x_start, int32_t y_start, int32_t x_end, int32_t y_end, const void* color_data) {
    auto* internal = static_cast<SdlDisplayInternal*>(device_get_driver_data(device));
    const auto* config = GET_CONFIG(device);

    const int32_t display_width = config->horizontal_resolution;
    const int32_t display_height = config->vertical_resolution;

    // Clip, so an unexpected area cannot write outside the buffer
    const int32_t clipped_x_end = x_end < display_width ? x_end : display_width;
    const int32_t clipped_y_end = y_end < display_height ? y_end : display_height;
    if (x_start < 0 || y_start < 0 || x_start >= clipped_x_end || y_start >= clipped_y_end) {
        return ERROR_NONE;
    }

    // Composite the tile into the full frame, row by row: source rows are packed to the
    // tile's width, destination rows to the display's.
    const int32_t tile_width = x_end - x_start;
    const auto* source = static_cast<const uint16_t*>(color_data);
    for (int32_t y = y_start; y < clipped_y_end; y++) {
        uint16_t* destination = internal->frame_buffer + (size_t)y * display_width + x_start;
        std::memcpy(destination, source + (size_t)(y - y_start) * tile_width,
                    (size_t)(clipped_x_end - x_start) * sizeof(uint16_t));
    }

    const size_t pixel_count = (size_t)display_width * display_height;
    for (size_t i = 0; i < pixel_count; i++) {
        internal->upload_buffer[i] = rgb565_to_argb8888(internal->frame_buffer[i]);
    }

    if (SDL_UpdateTexture(internal->texture, nullptr, internal->upload_buffer,
                          display_width * (int)sizeof(uint32_t)) != 0) {
        LOG_E(TAG, "SDL_UpdateTexture failed: %s", SDL_GetError());
        return ERROR_RESOURCE;
    }

    SDL_RenderClear(internal->renderer);
    SDL_RenderCopy(internal->renderer, internal->texture, nullptr, nullptr);
    SDL_RenderPresent(internal->renderer);

    return ERROR_NONE;
}

static enum DisplayColorFormat sdl_display_get_color_format(Device*) {
    return DISPLAY_COLOR_FORMAT_RGB565;
}

static uint16_t sdl_display_get_resolution_x(Device* device) {
    return GET_CONFIG(device)->horizontal_resolution;
}

static uint16_t sdl_display_get_resolution_y(Device* device) {
    return GET_CONFIG(device)->vertical_resolution;
}

static void sdl_display_get_frame_buffer(Device*, uint8_t, void** out_buffer) {
    *out_buffer = nullptr;
}

static uint8_t sdl_display_get_frame_buffer_count(Device*) {
    return 0;
}

// endregion

static const DisplayApi sdl_display_api = {
    .capabilities = 0,
    .reset = sdl_display_reset,
    .init = sdl_display_init,
    .draw_bitmap = sdl_display_draw_bitmap,
    .mirror = nullptr,
    .swap_xy = nullptr,
    .get_swap_xy = nullptr,
    .get_mirror_x = nullptr,
    .get_mirror_y = nullptr,
    .set_gap = nullptr,
    .get_gap_x = nullptr,
    .get_gap_y = nullptr,
    .invert_color = nullptr,
    .disp_on_off = nullptr,
    .disp_sleep = nullptr,
    .get_color_format = sdl_display_get_color_format,
    .get_resolution_x = sdl_display_get_resolution_x,
    .get_resolution_y = sdl_display_get_resolution_y,
    .get_frame_buffer = sdl_display_get_frame_buffer,
    .get_frame_buffer_count = sdl_display_get_frame_buffer_count,
    .get_backlight = nullptr,
    .has_capability = nullptr,
};

extern Module simulator_module;

Driver sdl_display_driver = {
    .name = "sdl-display",
    .compatible = (const char*[]) { "tactility,sdl-display", nullptr },
    .start_device = start,
    .stop_device = stop,
    .api = &sdl_display_api,
    .device_type = &DISPLAY_TYPE,
    .owner = &simulator_module,
    .internal = nullptr
};
