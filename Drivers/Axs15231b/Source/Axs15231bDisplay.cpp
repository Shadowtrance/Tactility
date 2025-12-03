#include "Axs15231bDisplay.h"

#include <Tactility/Log.h>

#include <esp_lcd_axs15231b.h>
#include <esp_lcd_panel_commands.h>
#include <driver/gpio.h>

constexpr const char* TAG = "AXS15231B";

typedef struct
{
    SemaphoreHandle_t te_v_sync_sem;
    SemaphoreHandle_t te_catch_sem;
    uint32_t time_Tvdl;
    uint32_t time_Tvdh;
    uint32_t te_timestamp;
    portMUX_TYPE lock;
} lcd_tear_t;

static const axs15231b_lcd_init_cmd_t lcd_init_cmds[] = {
    //Seems this is all we need?
    //https://github.com/straga/micropython_lcd/blob/master/device/JC3248W535/new_SPI/_axs15231b_init.py
    {0x13, (uint8_t []){0x00}, 0, 10}, //Disable Partial Display Mode (return to Normal Display Mode)
    {0x11, (uint8_t[]){0x00}, 0, 150}, //Sleep out
    {0x29, (uint8_t[]){0x00}, 0, 150}, //Display on
    {0x22, (uint8_t[]){0x00}, 0, 200} //All Pixels off
};

static void displaySyncCallback(lv_event_t *e)
{
    lcd_tear_t *tear_handle = (lcd_tear_t *)lv_event_get_user_data(e);
    
    if (tear_handle == nullptr) {
        return;
    }

    if (tear_handle->te_catch_sem) {
        xSemaphoreGive(tear_handle->te_catch_sem);
    }

    xSemaphoreTake(tear_handle->te_v_sync_sem, portMAX_DELAY);
}

static void teSyncRegistrationTask(void* param)
{
    Axs15231bDisplay* display = (Axs15231bDisplay*)param;
    
    vTaskDelay(pdMS_TO_TICKS(200));
    
    if (display != nullptr && display->getLvglDisplay() != nullptr) {
        display->registerTeSyncCallback();
    } else {
        TT_LOG_E(TAG, "Failed to register TE sync callback - LVGL display not ready after 200ms");
    }
    
    vTaskDelete(nullptr);
}

static void displaySyncTask(void *arg)
{
    assert(arg);
    lcd_tear_t *tear_handle = (lcd_tear_t *)arg;

    while (true) {
        if (pdPASS != xSemaphoreTake(tear_handle->te_catch_sem, pdMS_TO_TICKS(tear_handle->time_Tvdl))) {
            xSemaphoreTake(tear_handle->te_v_sync_sem, 0);
        }
    }
    vTaskDelete(nullptr);
}

static void displayTearInterrupt(void *arg)
{
    assert(arg);
    lcd_tear_t *tear_handle = (lcd_tear_t *)arg;
    BaseType_t xHigherPriorityTaskAwoken = pdFALSE;
    
    if (tear_handle->te_v_sync_sem) {
        portENTER_CRITICAL_ISR(&tear_handle->lock);
        tear_handle->te_timestamp = esp_log_timestamp();
        portEXIT_CRITICAL_ISR(&tear_handle->lock);
        xSemaphoreGiveFromISR(tear_handle->te_v_sync_sem, &xHigherPriorityTaskAwoken);

        if (xHigherPriorityTaskAwoken) {
            portYIELD_FROM_ISR();
        }
    }
}

bool Axs15231bDisplay::createIoHandle(esp_lcd_panel_io_handle_t& outHandle) {
    const esp_lcd_panel_io_spi_config_t panel_io_config = {
        .cs_gpio_num = configuration->csPin,
        .dc_gpio_num = configuration->dcPin,
        .spi_mode = 3,
        .pclk_hz = configuration->pixelClockFrequency,
        .trans_queue_depth = configuration->transactionQueueDepth,
        .on_color_trans_done = nullptr,
        .user_ctx = nullptr,
        .lcd_cmd_bits = 32,
        .lcd_param_bits = 8,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .flags = {
            .dc_high_on_cmd = 0,
            .dc_low_on_data = 0,
            .dc_low_on_param = 0,
            .octal_mode = 0,
            .quad_mode = 1,
            .sio_mode = 0,
            .lsb_first = 0,
            .cs_high_active = 0
        }
    };

    return esp_lcd_new_panel_io_spi(configuration->spiHostDevice, &panel_io_config, &outHandle) == ESP_OK;
}

bool Axs15231bDisplay::createPanelHandle(esp_lcd_panel_io_handle_t ioHandle, esp_lcd_panel_handle_t& panelHandle) {
    SemaphoreHandle_t te_catch_sem = nullptr;
    SemaphoreHandle_t te_v_sync_sem = nullptr;

    const axs15231b_vendor_config_t vendor_config = {
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
        .flags = {
            .use_qspi_interface = 1,
        },
    };

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = configuration->resetPin,
        .rgb_ele_order = configuration->rgbElementOrder,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel = 16,
        .flags = {
            .reset_active_high = false
        },
        .vendor_config = (void *)&vendor_config
    };

    if (esp_lcd_new_panel_axs15231b(ioHandle, &panel_config, &panelHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to create axs15231b");
        return false;
    }

    if (esp_lcd_panel_reset(panelHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to reset panel");
        return false;
    }

    if (esp_lcd_panel_init(panelHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to init panel");
        return false;
    }

    //SWPXY Doesn't work with the JC3248W535
    if (esp_lcd_panel_swap_xy(panelHandle, configuration->swapXY) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to swap XY ");
        return false;
    }

    if (esp_lcd_panel_mirror(panelHandle, configuration->mirrorX, configuration->mirrorY) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to mirror panel");
        return false;
    }

    if (esp_lcd_panel_invert_color(panelHandle, configuration->invertColor) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to init panel");
        return false;
    }

    if (esp_lcd_panel_disp_on_off(panelHandle, false) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to turn on panel");
        return false;
    }

    // TE sync initialization
    uint32_t time_Tvdl = 13;
    uint32_t time_Tvdh = 3;

    if (configuration->tePin > 0)
    {
        lcd_tear_t* tear_handle = (lcd_tear_t*)malloc(sizeof(lcd_tear_t));
        if (tear_handle == nullptr) {
            TT_LOG_E(TAG, "Not enough memory for tear_ctx allocation!");
            goto err;
        }

        te_v_sync_sem = xSemaphoreCreateCounting(1, 0);
        if (!te_v_sync_sem) {
            TT_LOG_E(TAG, "Failed to create TE vsync semaphore");
            goto err;
        }
        tear_handle->te_v_sync_sem = te_v_sync_sem;

        te_catch_sem = xSemaphoreCreateCounting(1, 0);
        if (!te_catch_sem) {
            TT_LOG_E(TAG, "Failed to create TE catch semaphore");
            goto err;
        }
        tear_handle->te_catch_sem = te_catch_sem;

        tear_handle->time_Tvdl = time_Tvdl;
        tear_handle->time_Tvdh = time_Tvdh;
        tear_handle->lock.owner = portMUX_FREE_VAL;
        tear_handle->lock.count = 0;

        const gpio_config_t te_detect_cfg = {
            .pin_bit_mask = BIT64(configuration->tePin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE
        };

        ESP_ERROR_CHECK(gpio_config(&te_detect_cfg));
        gpio_install_isr_service(0);
        ESP_ERROR_CHECK(gpio_isr_handler_add(configuration->tePin, displayTearInterrupt, tear_handle));

        BaseType_t res = xTaskCreate(displaySyncTask, "TE Sync", 2048, tear_handle, 4, nullptr);
        if (res != pdPASS) {
            TT_LOG_E(TAG, "Failed to create TE sync task");
            goto err;
        }
        
        tear_ctx = tear_handle;
        
        // Schedule TE sync callback registration after LVGL display is created
        xTaskCreate(teSyncRegistrationTask, "TE Reg", 2048, this, 3, nullptr);
    }

    return true;

err:
    if (te_v_sync_sem) {
        vSemaphoreDelete(te_v_sync_sem);
    }
    if (te_catch_sem) {
        vSemaphoreDelete(te_catch_sem);
    }
    if (tear_ctx) {
        free(tear_ctx);
        tear_ctx = nullptr;
    }
    if (panelHandle) {
        esp_lcd_panel_del(panelHandle);
    }
    if (ioHandle) {
        esp_lcd_panel_io_del(ioHandle);
    }
    return false;
}

lvgl_port_display_cfg_t Axs15231bDisplay::getLvglPortDisplayConfig(esp_lcd_panel_io_handle_t ioHandle, esp_lcd_panel_handle_t panelHandle) {
    return {
        .io_handle = ioHandle,
        .panel_handle = panelHandle,
        .control_handle = nullptr,
        .buffer_size = configuration->bufferSize,
        .double_buffer = true,
        .trans_size = configuration->horizontalResolution * configuration->verticalResolution / 4,
        .hres = configuration->horizontalResolution,
        .vres = configuration->verticalResolution,
        .monochrome = false,
        .rotation = {
            .swap_xy = configuration->swapXY,
            .mirror_x = configuration->mirrorX,
            .mirror_y = configuration->mirrorY,
        },
        .color_format = LV_COLOR_FORMAT_RGB565,
        .flags = {
            .buff_dma = false,
            .buff_spiram = true,
            .sw_rotate = true,
            .swap_bytes = true,
            .full_refresh = true,
            .direct_mode = false
        }
    };
}

void Axs15231bDisplay::registerTeSyncCallback() {
    if (tear_ctx != nullptr && getLvglDisplay() != nullptr) {
        lv_display_add_event_cb(getLvglDisplay(), displaySyncCallback, LV_EVENT_FLUSH_START, tear_ctx);
        TT_LOG_I(TAG, "TE sync callback registered successfully");
    } else {
        TT_LOG_W(TAG, "Cannot register TE sync callback - tear_ctx=%p display=%p", tear_ctx, getLvglDisplay());
    }
}
