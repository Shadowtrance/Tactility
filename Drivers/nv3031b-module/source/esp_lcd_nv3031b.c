/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileContributor: Adapted from esp_lcd_sh8601 for the NV3031B
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <sys/cdefs.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "esp_log.h"

#include "esp_lcd_nv3031b.h"
#include "nv3031b_init_cmds.h"

#define LCD_OPCODE_WRITE_CMD        (0x02ULL)
#define LCD_OPCODE_WRITE_COLOR      (0x32ULL)

#define NV3031B_RESET_DELAY_MS      120
#define NV3031B_SLEEP_OUT_DELAY_MS  120

static const char *TAG = "nv3031b";

static esp_err_t panel_nv3031b_del(esp_lcd_panel_t *panel);
static esp_err_t panel_nv3031b_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_nv3031b_init(esp_lcd_panel_t *panel);
static esp_err_t panel_nv3031b_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_nv3031b_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_nv3031b_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_nv3031b_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_nv3031b_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_nv3031b_disp_on_off(esp_lcd_panel_t *panel, bool on_off);

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    int x_gap;
    int y_gap;
    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val;
    uint8_t colmod_val;
    const nv3031b_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
    struct {
        unsigned int reset_level: 1;
    } flags;
} nv3031b_panel_t;

esp_err_t esp_lcd_new_panel_nv3031b(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    ESP_RETURN_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    esp_err_t ret = ESP_OK;
    nv3031b_panel_t *nv3031b = calloc(1, sizeof(nv3031b_panel_t));
    ESP_GOTO_ON_FALSE(nv3031b, ESP_ERR_NO_MEM, err, TAG, "no mem for nv3031b panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    switch (panel_dev_config->rgb_ele_order) {
    case LCD_RGB_ELEMENT_ORDER_RGB:
        nv3031b->madctl_val = 0;
        break;
    case LCD_RGB_ELEMENT_ORDER_BGR:
        nv3031b->madctl_val = LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported color element order");
        break;
    }

    switch (panel_dev_config->bits_per_pixel) {
    case 16: // RGB565
        nv3031b->colmod_val = 0x55;
        nv3031b->fb_bits_per_pixel = 16;
        break;
    case 18: // RGB666, each component in the 6 high bits of a byte
        nv3031b->colmod_val = 0x66;
        nv3031b->fb_bits_per_pixel = 24;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    nv3031b->io = io;
    nv3031b->reset_gpio_num = panel_dev_config->reset_gpio_num;
    nv3031b->flags.reset_level = panel_dev_config->flags.reset_active_high;
    const nv3031b_vendor_config_t *vendor_config = (const nv3031b_vendor_config_t *)panel_dev_config->vendor_config;
    if (vendor_config && vendor_config->init_cmds) {
        nv3031b->init_cmds = vendor_config->init_cmds;
        nv3031b->init_cmds_size = vendor_config->init_cmds_size;
    } else {
        nv3031b->init_cmds = nv3031b_init_cmds_default;
        nv3031b->init_cmds_size = sizeof(nv3031b_init_cmds_default) / sizeof(nv3031b_lcd_init_cmd_t);
    }
    nv3031b->base.del = panel_nv3031b_del;
    nv3031b->base.reset = panel_nv3031b_reset;
    nv3031b->base.init = panel_nv3031b_init;
    nv3031b->base.draw_bitmap = panel_nv3031b_draw_bitmap;
    nv3031b->base.invert_color = panel_nv3031b_invert_color;
    nv3031b->base.set_gap = panel_nv3031b_set_gap;
    nv3031b->base.mirror = panel_nv3031b_mirror;
    nv3031b->base.swap_xy = panel_nv3031b_swap_xy;
    nv3031b->base.disp_on_off = panel_nv3031b_disp_on_off;
    *ret_panel = &(nv3031b->base);
    ESP_LOGD(TAG, "new nv3031b panel @%p", nv3031b);

    return ESP_OK;

err:
    if (nv3031b) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(nv3031b);
    }
    return ret;
}

static esp_err_t tx_param(esp_lcd_panel_io_handle_t io, int lcd_cmd, const void *param, size_t param_size)
{
    lcd_cmd &= 0xff;
    lcd_cmd <<= 8;
    lcd_cmd |= LCD_OPCODE_WRITE_CMD << 24;
    return esp_lcd_panel_io_tx_param(io, lcd_cmd, param, param_size);
}

static esp_err_t tx_color(esp_lcd_panel_io_handle_t io, int lcd_cmd, const void *param, size_t param_size)
{
    lcd_cmd &= 0xff;
    lcd_cmd <<= 8;
    lcd_cmd |= LCD_OPCODE_WRITE_COLOR << 24;
    return esp_lcd_panel_io_tx_color(io, lcd_cmd, param, param_size);
}

static esp_err_t panel_nv3031b_del(esp_lcd_panel_t *panel)
{
    nv3031b_panel_t *nv3031b = __containerof(panel, nv3031b_panel_t, base);

    if (nv3031b->reset_gpio_num >= 0) {
        gpio_reset_pin(nv3031b->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del nv3031b panel @%p", nv3031b);
    free(nv3031b);
    return ESP_OK;
}

static esp_err_t panel_nv3031b_reset(esp_lcd_panel_t *panel)
{
    nv3031b_panel_t *nv3031b = __containerof(panel, nv3031b_panel_t, base);
    esp_lcd_panel_io_handle_t io = nv3031b->io;

    if (nv3031b->reset_gpio_num >= 0) {
        gpio_set_level(nv3031b->reset_gpio_num, nv3031b->flags.reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(nv3031b->reset_gpio_num, !nv3031b->flags.reset_level);
        vTaskDelay(pdMS_TO_TICKS(NV3031B_RESET_DELAY_MS));
    } else {
        ESP_RETURN_ON_ERROR(tx_param(io, LCD_CMD_SWRESET, NULL, 0), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(NV3031B_RESET_DELAY_MS));
    }

    return ESP_OK;
}

static esp_err_t panel_nv3031b_init(esp_lcd_panel_t *panel)
{
    nv3031b_panel_t *nv3031b = __containerof(panel, nv3031b_panel_t, base);
    esp_lcd_panel_io_handle_t io = nv3031b->io;

    // A NOP frame first resynchronizes the QSPI interface
    ESP_RETURN_ON_ERROR(tx_param(io, LCD_CMD_NOP, NULL, 0), TAG, "send command failed");

    for (int i = 0; i < nv3031b->init_cmds_size; i++) {
        const nv3031b_lcd_init_cmd_t *cmd = &nv3031b->init_cmds[i];
        ESP_RETURN_ON_ERROR(tx_param(io, cmd->cmd, cmd->data, cmd->data_bytes), TAG, "send command failed");
        if (cmd->delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(cmd->delay_ms));
        }
    }

    ESP_RETURN_ON_ERROR(tx_param(io, LCD_CMD_COLMOD, (uint8_t[]) {
        nv3031b->colmod_val,
    }, 1), TAG, "send command failed");
    ESP_RETURN_ON_ERROR(tx_param(io, LCD_CMD_SLPOUT, NULL, 0), TAG, "send command failed");
    vTaskDelay(pdMS_TO_TICKS(NV3031B_SLEEP_OUT_DELAY_MS));
    ESP_RETURN_ON_ERROR(tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        nv3031b->madctl_val,
    }, 1), TAG, "send command failed");

    return ESP_OK;
}

static esp_err_t panel_nv3031b_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    nv3031b_panel_t *nv3031b = __containerof(panel, nv3031b_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");
    esp_lcd_panel_io_handle_t io = nv3031b->io;

    x_start += nv3031b->x_gap;
    x_end += nv3031b->x_gap;
    y_start += nv3031b->y_gap;
    y_end += nv3031b->y_gap;

    ESP_RETURN_ON_ERROR(tx_param(io, LCD_CMD_CASET, (uint8_t[]) {
        (x_start >> 8) & 0xFF,
        x_start & 0xFF,
        ((x_end - 1) >> 8) & 0xFF,
        (x_end - 1) & 0xFF,
    }, 4), TAG, "send command failed");
    ESP_RETURN_ON_ERROR(tx_param(io, LCD_CMD_RASET, (uint8_t[]) {
        (y_start >> 8) & 0xFF,
        y_start & 0xFF,
        ((y_end - 1) >> 8) & 0xFF,
        (y_end - 1) & 0xFF,
    }, 4), TAG, "send command failed");

    size_t len = (x_end - x_start) * (y_end - y_start) * nv3031b->fb_bits_per_pixel / 8;
    ESP_RETURN_ON_ERROR(tx_color(io, LCD_CMD_RAMWR, color_data, len), TAG, "send color data failed");

    return ESP_OK;
}

static esp_err_t panel_nv3031b_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    nv3031b_panel_t *nv3031b = __containerof(panel, nv3031b_panel_t, base);
    int command = invert_color_data ? LCD_CMD_INVON : LCD_CMD_INVOFF;
    ESP_RETURN_ON_ERROR(tx_param(nv3031b->io, command, NULL, 0), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_nv3031b_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    nv3031b_panel_t *nv3031b = __containerof(panel, nv3031b_panel_t, base);

    if (mirror_x) {
        nv3031b->madctl_val |= LCD_CMD_MX_BIT;
    } else {
        nv3031b->madctl_val &= ~LCD_CMD_MX_BIT;
    }
    if (mirror_y) {
        nv3031b->madctl_val |= LCD_CMD_MY_BIT;
    } else {
        nv3031b->madctl_val &= ~LCD_CMD_MY_BIT;
    }
    ESP_RETURN_ON_ERROR(tx_param(nv3031b->io, LCD_CMD_MADCTL, (uint8_t[]) {
        nv3031b->madctl_val
    }, 1), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_nv3031b_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    nv3031b_panel_t *nv3031b = __containerof(panel, nv3031b_panel_t, base);

    if (swap_axes) {
        nv3031b->madctl_val |= LCD_CMD_MV_BIT;
    } else {
        nv3031b->madctl_val &= ~LCD_CMD_MV_BIT;
    }
    ESP_RETURN_ON_ERROR(tx_param(nv3031b->io, LCD_CMD_MADCTL, (uint8_t[]) {
        nv3031b->madctl_val
    }, 1), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_nv3031b_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    nv3031b_panel_t *nv3031b = __containerof(panel, nv3031b_panel_t, base);
    nv3031b->x_gap = x_gap;
    nv3031b->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_nv3031b_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    nv3031b_panel_t *nv3031b = __containerof(panel, nv3031b_panel_t, base);

    if (on_off) {
        // DISPON needs one dummy parameter byte on this controller
        ESP_RETURN_ON_ERROR(tx_param(nv3031b->io, LCD_CMD_DISPON, (uint8_t[]) {
            0x00
        }, 1), TAG, "send command failed");
    } else {
        ESP_RETURN_ON_ERROR(tx_param(nv3031b->io, LCD_CMD_DISPOFF, NULL, 0), TAG, "send command failed");
    }
    return ESP_OK;
}
