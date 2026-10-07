/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileContributor: Adapted from esp_lcd_sh8601 for the NV3031B
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>

#include "esp_lcd_panel_vendor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int cmd;
    const void *data;
    size_t data_bytes;
    unsigned int delay_ms;
} nv3031b_lcd_init_cmd_t;

/**
 * @brief Vendor specific configuration, passed as `vendor_config` in `esp_lcd_panel_dev_config_t`.
 *
 * The panel is driven over QSPI: commands are sent as `0x02 0x00 <cmd> 0x00` on one line,
 * pixel data as `0x32 0x00 0x2C 0x00` followed by the data on four lines.
 */
typedef struct {
    const nv3031b_lcd_init_cmd_t *init_cmds; /*!< Initialization sequence, NULL for the built-in one */
    uint16_t init_cmds_size;                 /*!< Number of commands in init_cmds */
} nv3031b_vendor_config_t;

/**
 * @brief Create an LCD panel for the NV3031B.
 *
 * @param[in] io LCD panel IO handle (SPI panel IO in quad mode with 32 bit commands)
 * @param[in] panel_dev_config General panel device configuration
 * @param[out] ret_panel Returned LCD panel handle
 * @return ESP_OK on success, otherwise an error code
 */
esp_err_t esp_lcd_new_panel_nv3031b(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel);

#ifdef __cplusplus
}
#endif
