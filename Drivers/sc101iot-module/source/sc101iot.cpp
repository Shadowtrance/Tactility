// SPDX-License-Identifier: Apache-2.0
#include <drivers/sc101iot.h>

#include <tactility/device.h>
#include <tactility/driver.h>
#include <tactility/drivers/esp32_i2c_master.h>
#include <tactility/drivers/gpio_controller.h>
#include <tactility/drivers/i2c_controller.h>
#include <tactility/log.h>

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/ppa.h>
#include <esp_cache.h>
#include <esp_heap_caps.h>
#include <esp_jpeg_enc.h>
#include <esp_video_device.h>
#include <esp_video_init.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/poll.h>
#include <unistd.h>

#include <cstring>
#include <new>

#define TAG "SC101IOT"

namespace {

constexpr int VIDEO_BUFFER_COUNT = 2;
constexpr uint32_t SCCB_FREQUENCY_HZ = 100000;
constexpr uint8_t PCLK_DELAY_REGISTER_HIGH = 0x36;
constexpr uint8_t PCLK_DELAY_REGISTER_LOW = 0x40;
constexpr uint8_t PCLK_DELAY_MASK = 0x03;

bool s_video_initialized = false;
SemaphoreHandle_t s_video_init_mutex = nullptr;
uint32_t s_video_handle_count = 0;

struct Sc101iotState : CameraHandleData {
    int fd;
    uint32_t native_width;
    uint32_t native_height;
    uint32_t stride;
    uint32_t width;
    uint32_t height;
    uint8_t* buffers[VIDEO_BUFFER_COUNT];
    size_t buffer_lengths[VIDEO_BUFFER_COUNT];
    int last_dqbuf_index;
    uint8_t* frame_buf;
    size_t frame_buf_size;
    jpeg_enc_handle_t encoder;
    uint32_t encoder_width;
    uint32_t encoder_height;
    ppa_client_handle_t ppa;
    GpioDescriptor* pwdn;
    CameraRotation rotation;
    bool flip_x = false;
    SemaphoreHandle_t mutex;
};

#define GET_CONFIG(device) (static_cast<const Sc101iotConfig*>((device)->config))

gpio_num_t pin_or_nc(const GpioPinSpec& pin) {
    return pin.gpio_controller == nullptr ? GPIO_NUM_NC : static_cast<gpio_num_t>(pin.pin);
}

CameraRotation add_rotation(CameraRotation rotation, uint16_t offset) {
    return static_cast<CameraRotation>((static_cast<uint16_t>(rotation) + offset) % 360);
}

bool is_rotated(CameraRotation rotation) {
    return rotation == CAMERA_ROTATION_90 || rotation == CAMERA_ROTATION_270;
}

ppa_srm_rotation_angle_t to_ppa_rotation(CameraRotation rotation) {
    switch (rotation) {
        case CAMERA_ROTATION_90:
            return PPA_SRM_ROTATION_ANGLE_90;
        case CAMERA_ROTATION_180:
            return PPA_SRM_ROTATION_ANGLE_180;
        case CAMERA_ROTATION_270:
            return PPA_SRM_ROTATION_ANGLE_270;
        default:
            return PPA_SRM_ROTATION_ANGLE_0;
    }
}

SemaphoreHandle_t get_video_init_mutex() {
    static portMUX_TYPE creation_lock = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&creation_lock);
    if (s_video_init_mutex == nullptr) {
        s_video_init_mutex = xSemaphoreCreateMutex();
    }
    portEXIT_CRITICAL(&creation_lock);
    return s_video_init_mutex;
}

// The official BSP applies this one-line PCLK-delay fix for SC101IOT. esp_cam_sensor does not
// include it yet, so write the same register from here after the sensor format was applied.
bool clear_pclk_delay(i2c_master_bus_handle_t bus, uint8_t address) {
    i2c_device_config_t device_config = {};
    device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device_config.device_address = address;
    device_config.scl_speed_hz = SCCB_FREQUENCY_HZ;

    i2c_master_dev_handle_t device = nullptr;
    if (i2c_master_bus_add_device(bus, &device_config, &device) != ESP_OK) {
        LOG_W(TAG, "Failed to add SCCB device for PCLK-delay fix");
        return false;
    }

    uint8_t pointer[2] = { PCLK_DELAY_REGISTER_HIGH, PCLK_DELAY_REGISTER_LOW };
    uint8_t value = 0;
    bool ok = i2c_master_transmit(device, pointer, sizeof(pointer), 100) == ESP_OK &&
        i2c_master_receive(device, &value, 1, 100) == ESP_OK;

    if (ok) {
        uint8_t write[3] = { PCLK_DELAY_REGISTER_HIGH, PCLK_DELAY_REGISTER_LOW,
                             static_cast<uint8_t>(value & ~PCLK_DELAY_MASK) };
        ok = i2c_master_transmit(device, write, sizeof(write), 100) == ESP_OK;
    }

    i2c_master_bus_rm_device(device);
    if (!ok) {
        LOG_W(TAG, "Failed to clear the SC101IOT PCLK delay");
    }
    return ok;
}

void configure_flash(const Sc101iotConfig* config) {
    if (config->pin_flash.gpio_controller == nullptr) return;
    const auto pin = static_cast<gpio_num_t>(config->pin_flash.pin);
    gpio_config_t gpio = {};
    gpio.pin_bit_mask = 1ULL << pin;
    gpio.mode = GPIO_MODE_OUTPUT_OD;
    gpio.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio.intr_type = GPIO_INTR_DISABLE;
    gpio_set_level(pin, 1);
    gpio_config(&gpio);
}

bool has_flash(Device* device) {
    return GET_CONFIG(device)->pin_flash.gpio_controller != nullptr;
}

error_t set_flash(Device* device, bool enabled) {
    const auto* config = GET_CONFIG(device);
    if (config->pin_flash.gpio_controller == nullptr) {
        return ERROR_NOT_SUPPORTED;
    }
    gpio_set_level(static_cast<gpio_num_t>(config->pin_flash.pin), enabled ? 0 : 1);
    return ERROR_NONE;
}

bool map_buffers(Sc101iotState* state) {
    v4l2_requestbuffers request = {};
    request.count = VIDEO_BUFFER_COUNT;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (ioctl(state->fd, VIDIOC_REQBUFS, &request) != 0) {
        return false;
    }

    for (int i = 0; i < VIDEO_BUFFER_COUNT; i++) {
        v4l2_buffer buffer = {};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = static_cast<uint32_t>(i);
        if (ioctl(state->fd, VIDIOC_QUERYBUF, &buffer) != 0) {
            return false;
        }
        state->buffer_lengths[i] = buffer.length;
        state->buffers[i] = static_cast<uint8_t*>(
            mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, state->fd, buffer.m.offset)
        );
        if (state->buffers[i] == MAP_FAILED) {
            state->buffers[i] = nullptr;
            return false;
        }
        if (ioctl(state->fd, VIDIOC_QBUF, &buffer) != 0) {
            return false;
        }
    }
    return true;
}

void release_video_buffers(Sc101iotState* state) {
    if (state->fd >= 0) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(state->fd, VIDIOC_STREAMOFF, &type);
    }
    for (int i = 0; i < VIDEO_BUFFER_COUNT; i++) {
        if (state->buffers[i] != nullptr) {
            munmap(state->buffers[i], state->buffer_lengths[i]);
            state->buffers[i] = nullptr;
            state->buffer_lengths[i] = 0;
        }
    }
    if (state->fd >= 0) {
        close(state->fd);
        state->fd = -1;
    }
}

error_t release_state(Sc101iotState* state) {
    if (state->encoder) jpeg_enc_close(state->encoder);
    release_video_buffers(state);
    heap_caps_free(state->frame_buf);
    if (state->ppa) ppa_unregister_client(state->ppa);
    if (state->pwdn) {
        gpio_descriptor_set_level(state->pwdn, true);
        gpio_descriptor_release(state->pwdn);
    }
    if (state->mutex) vSemaphoreDelete(state->mutex);
    delete state;
    return ERROR_NONE;
}

error_t start(Device* device) {
    auto* parent = device_get_parent(device);
    if (parent == nullptr || device_get_type(parent) != &I2C_CONTROLLER_TYPE) {
        LOG_E(TAG, "Parent is not an I2C controller");
        return ERROR_RESOURCE;
    }
    configure_flash(GET_CONFIG(device));
    return ERROR_NONE;
}

error_t stop(Device* device) {
    set_flash(device, false);
    return ERROR_NONE;
}

error_t open_camera(Device* device, CameraHandle* out_handle) {
    if (device == nullptr || out_handle == nullptr) return ERROR_INVALID_ARGUMENT;

    auto* parent = device_get_parent(device);
    i2c_master_bus_handle_t sccb_bus = esp32_i2c_master_get_bus_handle(parent);
    if (sccb_bus == nullptr) {
        LOG_E(TAG, "No SCCB bus handle");
        return ERROR_RESOURCE;
    }

    const auto* config = GET_CONFIG(device);
    auto* state = new (std::nothrow) Sc101iotState {};
    if (state == nullptr) return ERROR_OUT_OF_MEMORY;
    state->device = device;
    state->fd = -1;
    state->last_dqbuf_index = -1;
    state->rotation = CAMERA_ROTATION_0;
    state->mutex = xSemaphoreCreateMutex();
    if (state->mutex == nullptr) {
        delete state;
        return ERROR_OUT_OF_MEMORY;
    }

    if (config->pin_pwdn.gpio_controller != nullptr) {
        state->pwdn = gpio_descriptor_acquire(
            config->pin_pwdn.gpio_controller,
            config->pin_pwdn.pin,
            config->pin_pwdn.flags | GPIO_FLAG_DIRECTION_OUTPUT,
            GPIO_OWNER_GPIO
        );
        if (state->pwdn == nullptr) {
            LOG_E(TAG, "Failed to acquire PWDN pin");
            release_state(state);
            return ERROR_RESOURCE;
        }
        gpio_descriptor_set_level(state->pwdn, false);
        // The module needs a short settle time after power-down is released.
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    SemaphoreHandle_t init_mutex = get_video_init_mutex();
    if (init_mutex == nullptr) {
        release_state(state);
        return ERROR_OUT_OF_MEMORY;
    }
    xSemaphoreTake(init_mutex, portMAX_DELAY);
    if (!s_video_initialized) {
        const esp_video_init_dvp_config_t dvp_config = {
            .sccb_config = {
                .init_sccb = false,
                .i2c_handle = sccb_bus,
                .freq = SCCB_FREQUENCY_HZ,
            },
            .reset_pin = pin_or_nc(config->pin_reset),
            .pwdn_pin = GPIO_NUM_NC,
            .dvp_pin = {
                .data_width = CAM_CTLR_DATA_WIDTH_8,
                .data_io = {
                    pin_or_nc(config->pin_d0),
                    pin_or_nc(config->pin_d1),
                    pin_or_nc(config->pin_d2),
                    pin_or_nc(config->pin_d3),
                    pin_or_nc(config->pin_d4),
                    pin_or_nc(config->pin_d5),
                    pin_or_nc(config->pin_d6),
                    pin_or_nc(config->pin_d7),
                },
                .vsync_io = pin_or_nc(config->pin_vsync),
                .de_io = pin_or_nc(config->pin_de),
                .pclk_io = pin_or_nc(config->pin_pclk),
                .xclk_io = pin_or_nc(config->pin_xclk),
            },
            .xclk_freq = config->xclk_frequency_hz,
        };
        const esp_video_init_config_t video_config = { .dvp = &dvp_config };
        esp_err_t error = esp_video_init_with_flags(&video_config, ESP_VIDEO_INIT_FLAGS_DVP);
        if (error != ESP_OK) {
            LOG_W(TAG, "Camera not present or init failed: %s", esp_err_to_name(error));
            xSemaphoreGive(init_mutex);
            release_state(state);
            return ERROR_NOT_FOUND;
        }
        s_video_initialized = true;
    }
    ++s_video_handle_count;
    xSemaphoreGive(init_mutex);

    state->fd = ::open(ESP_VIDEO_DVP_DEVICE_NAME, O_RDONLY);
    if (state->fd < 0) {
        LOG_E(TAG, "Failed to open %s", ESP_VIDEO_DVP_DEVICE_NAME);
        goto err_deinit;
    }

    {
        v4l2_format format = {};
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(state->fd, VIDIOC_G_FMT, &format) != 0) {
            LOG_E(TAG, "VIDIOC_G_FMT failed");
            goto err_fd;
        }
        if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_UYVY) {
            LOG_E(TAG, "Unsupported pixel format 0x%08x", format.fmt.pix.pixelformat);
            goto err_fd;
        }
        state->native_width = format.fmt.pix.width;
        state->native_height = format.fmt.pix.height;
        state->stride = format.fmt.pix.bytesperline != 0 ? format.fmt.pix.bytesperline : state->native_width * 2;
        state->width = config->output_width != 0 ? config->output_width : state->native_width;
        state->height = config->output_height != 0 ? config->output_height : state->native_height;
    }

    state->frame_buf_size = static_cast<size_t>(state->width) * state->height * 2;
    state->frame_buf = static_cast<uint8_t*>(heap_caps_aligned_alloc(
        128,
        state->frame_buf_size,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT
    ));
    if (state->frame_buf == nullptr) {
        LOG_E(TAG, "Failed to allocate the RGB565 frame buffer");
        goto err_fd;
    }

    {
        ppa_client_config_t ppa_config = {};
        ppa_config.oper_type = PPA_OPERATION_SRM;
        ppa_config.max_pending_trans_num = 1;
        if (ppa_register_client(&ppa_config, &state->ppa) != ESP_OK) {
            LOG_E(TAG, "Failed to register the PPA client");
            goto err_fd;
        }
    }

    if (!clear_pclk_delay(sccb_bus, config->address)) {
        // Not fatal: the sensor may already be at the correct delay.
    }

    if (!map_buffers(state)) {
        LOG_E(TAG, "Failed to map the capture buffers");
        goto err_fd;
    }

    {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(state->fd, VIDIOC_STREAMON, &type) != 0) {
            LOG_E(TAG, "VIDIOC_STREAMON failed");
            goto err_fd;
        }
    }

    LOG_I(TAG, "Camera started: sensor %ux%u, output %ux%u",
          (unsigned)state->native_width, (unsigned)state->native_height,
          (unsigned)state->width, (unsigned)state->height);

    *out_handle = state;
    return ERROR_NONE;

err_fd:
    release_video_buffers(state);
err_deinit:
    xSemaphoreTake(init_mutex, portMAX_DELAY);
    if (s_video_handle_count > 0) --s_video_handle_count;
    if (s_video_handle_count == 0 && s_video_initialized) {
        esp_video_deinit_with_flags(ESP_VIDEO_INIT_FLAGS_DVP);
        s_video_initialized = false;
    }
    xSemaphoreGive(init_mutex);
    release_state(state);
    return ERROR_RESOURCE;
}

error_t close_camera(CameraHandle handle) {
    if (handle == nullptr) return ERROR_INVALID_ARGUMENT;
    auto* state = static_cast<Sc101iotState*>(handle);
    set_flash(state->device, false);

    // Stop capture, unmap the V4L2 buffers and close the video device file before
    // esp_video_deinit(). The deinit path destroys the SCCB/sensor handles first and then tries to
    // stop the sensor stream through them; doing that while the stream is still running is a
    // use-after-free (observed as an illegal-instruction crash in sc101iot_set_stream()).
    release_video_buffers(state);

    SemaphoreHandle_t init_mutex = get_video_init_mutex();
    xSemaphoreTake(init_mutex, portMAX_DELAY);
    if (s_video_handle_count > 0) --s_video_handle_count;
    if (s_video_handle_count == 0 && s_video_initialized) {
        esp_video_deinit_with_flags(ESP_VIDEO_INIT_FLAGS_DVP);
        s_video_initialized = false;
    }
    xSemaphoreGive(init_mutex);
    return release_state(state);
}

error_t set_rotation(CameraHandle handle, CameraRotation rotation) {
    if (handle == nullptr) return ERROR_INVALID_ARGUMENT;
    auto* state = static_cast<Sc101iotState*>(handle);
    xSemaphoreTake(state->mutex, portMAX_DELAY);
    state->rotation = rotation;
    xSemaphoreGive(state->mutex);
    return ERROR_NONE;
}

error_t set_flip(CameraHandle handle, bool flipped) {
    if (handle == nullptr) return ERROR_INVALID_ARGUMENT;
    auto* state = static_cast<Sc101iotState*>(handle);
    xSemaphoreTake(state->mutex, portMAX_DELAY);
    state->flip_x = flipped;
    xSemaphoreGive(state->mutex);
    return ERROR_NONE;
}

error_t release_frame(CameraHandle handle);

error_t get_frame(CameraHandle handle, uint8_t** buf, size_t* len, uint32_t timeout_ms,
                  uint32_t* out_width, uint32_t* out_height) {
    if (handle == nullptr || buf == nullptr || len == nullptr) return ERROR_INVALID_ARGUMENT;
    auto* state = static_cast<Sc101iotState*>(handle);

    pollfd descriptor = { .fd = state->fd, .events = POLLIN, .revents = 0 };
    int poll_result = poll(&descriptor, 1, static_cast<int>(timeout_ms));
    if (poll_result == 0) return ERROR_TIMEOUT;
    if (poll_result < 0) return ERROR_RESOURCE;

    v4l2_buffer buffer = {};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    if (ioctl(state->fd, VIDIOC_DQBUF, &buffer) != 0) return ERROR_RESOURCE;
    state->last_dqbuf_index = static_cast<int>(buffer.index);

    const size_t expected = static_cast<size_t>(state->stride) * state->native_height;
    if (buffer.index >= VIDEO_BUFFER_COUNT || buffer.bytesused < expected ||
        (buffer.flags & V4L2_BUF_FLAG_ERROR) != 0) {
        release_frame(handle);
        // Treat a dropped/corrupt frame as a timeout so the app keeps streaming.
        return ERROR_TIMEOUT;
    }

    esp_cache_msync(state->buffers[buffer.index], state->buffer_lengths[buffer.index],
                    ESP_CACHE_MSYNC_FLAG_DIR_M2C | ESP_CACHE_MSYNC_FLAG_INVALIDATE);

    CameraRotation rotation;
    bool flip_x;
    xSemaphoreTake(state->mutex, portMAX_DELAY);
    rotation = add_rotation(state->rotation, GET_CONFIG(state->device)->rotation_offset);
    flip_x = state->flip_x;
    xSemaphoreGive(state->mutex);

    const uint32_t crop = (state->native_width < state->native_height ? state->native_width : state->native_height) & ~1U;
    if (crop < 2) {
        release_frame(handle);
        return ERROR_RESOURCE;
    }
    const uint32_t offset_x = ((state->native_width - crop) / 2) & ~1U;
    const uint32_t offset_y = ((state->native_height - crop) / 2) & ~1U;

    const ppa_srm_oper_config_t operation = {
        .in = {
            .buffer = state->buffers[buffer.index],
            .pic_w = state->stride / 2,
            .pic_h = state->native_height,
            .block_w = crop,
            .block_h = crop,
            .block_offset_x = offset_x,
            .block_offset_y = offset_y,
            .srm_cm = PPA_SRM_COLOR_MODE_YUV422_UYVY,
            .yuv_range = PPA_COLOR_RANGE_LIMIT,
            .yuv_std = PPA_COLOR_CONV_STD_RGB_YUV_BT601,
        },
        .out = {
            .buffer = state->frame_buf,
            .buffer_size = static_cast<uint32_t>(state->frame_buf_size),
            .pic_w = state->width,
            .pic_h = state->height,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
            .yuv_range = PPA_COLOR_RANGE_LIMIT,
            .yuv_std = PPA_COLOR_CONV_STD_RGB_YUV_BT601,
        },
        .rotation_angle = to_ppa_rotation(rotation),
        .scale_x = static_cast<float>(state->width) / static_cast<float>(crop),
        .scale_y = static_cast<float>(state->height) / static_cast<float>(crop),
        .mirror_x = flip_x,
        .mirror_y = GET_CONFIG(state->device)->mirror_y,
        .rgb_swap = false,
        .byte_swap = false,
        .alpha_update_mode = PPA_ALPHA_NO_CHANGE,
        .alpha_fix_val = 0,
        .mode = PPA_TRANS_MODE_BLOCKING,
        .user_data = nullptr,
    };

    esp_cache_msync(state->frame_buf, state->frame_buf_size,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M);

    if (ppa_do_scale_rotate_mirror(state->ppa, &operation) != ESP_OK) {
        LOG_E(TAG, "PPA failed");
        release_frame(handle);
        return ERROR_RESOURCE;
    }

    esp_cache_msync(state->frame_buf, state->frame_buf_size,
                    ESP_CACHE_MSYNC_FLAG_DIR_M2C | ESP_CACHE_MSYNC_FLAG_INVALIDATE);

    *buf = state->frame_buf;
    *len = state->frame_buf_size;
    if (out_width != nullptr) *out_width = state->width;
    if (out_height != nullptr) *out_height = state->height;
    return ERROR_NONE;
}

error_t release_frame(CameraHandle handle) {
    if (handle == nullptr) return ERROR_INVALID_ARGUMENT;
    auto* state = static_cast<Sc101iotState*>(handle);
    if (state->last_dqbuf_index < 0) return ERROR_INVALID_STATE;
    v4l2_buffer buffer = {};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = static_cast<uint32_t>(state->last_dqbuf_index);
    state->last_dqbuf_index = -1;
    return ioctl(state->fd, VIDIOC_QBUF, &buffer) == 0 ? ERROR_NONE : ERROR_RESOURCE;
}

uint32_t get_width(CameraHandle handle) {
    auto* state = static_cast<Sc101iotState*>(handle);
    xSemaphoreTake(state->mutex, portMAX_DELAY);
    const auto rotation = add_rotation(state->rotation, GET_CONFIG(state->device)->rotation_offset);
    const uint32_t width = is_rotated(rotation) ? state->height : state->width;
    xSemaphoreGive(state->mutex);
    return width;
}

uint32_t get_height(CameraHandle handle) {
    auto* state = static_cast<Sc101iotState*>(handle);
    xSemaphoreTake(state->mutex, portMAX_DELAY);
    const auto rotation = add_rotation(state->rotation, GET_CONFIG(state->device)->rotation_offset);
    const uint32_t height = is_rotated(rotation) ? state->width : state->height;
    xSemaphoreGive(state->mutex);
    return height;
}

error_t capture_jpeg(CameraHandle handle, uint8_t** out_buf, size_t* out_len, uint8_t quality) {
    if (handle == nullptr || out_buf == nullptr || out_len == nullptr) return ERROR_INVALID_ARGUMENT;
    auto* state = static_cast<Sc101iotState*>(handle);

    uint8_t* frame = nullptr;
    size_t frame_len = 0;
    uint32_t frame_width = 0;
    uint32_t frame_height = 0;
    error_t error = get_frame(handle, &frame, &frame_len, 500, &frame_width, &frame_height);
    if (error != ERROR_NONE) return error;
    if (frame == nullptr || frame_len == 0) {
        release_frame(handle);
        return ERROR_RESOURCE;
    }

    size_t image_size = static_cast<size_t>(frame_width) * frame_height * 2;
    uint8_t* aligned_input = static_cast<uint8_t*>(heap_caps_aligned_alloc(16, image_size, MALLOC_CAP_SPIRAM));
    uint8_t* jpeg_output = static_cast<uint8_t*>(heap_caps_aligned_alloc(16, image_size, MALLOC_CAP_SPIRAM));
    if (aligned_input == nullptr || jpeg_output == nullptr) {
        heap_caps_free(aligned_input);
        heap_caps_free(jpeg_output);
        release_frame(handle);
        return ERROR_OUT_OF_MEMORY;
    }
    memcpy(aligned_input, frame, frame_len);

    if (state->encoder_width != frame_width || state->encoder_height != frame_height) {
        if (state->encoder) jpeg_enc_close(state->encoder);
        state->encoder = nullptr;
        jpeg_enc_config_t config = DEFAULT_JPEG_ENC_CONFIG();
        config.width = static_cast<int>(frame_width);
        config.height = static_cast<int>(frame_height);
        config.src_type = JPEG_PIXEL_FORMAT_RGB565_LE;
        config.subsampling = JPEG_SUBSAMPLE_420;
        config.quality = quality;
        if (jpeg_enc_open(&config, &state->encoder) != JPEG_ERR_OK) {
            heap_caps_free(aligned_input);
            heap_caps_free(jpeg_output);
            release_frame(handle);
            return ERROR_RESOURCE;
        }
        state->encoder_width = frame_width;
        state->encoder_height = frame_height;
    }

    jpeg_enc_set_quality(state->encoder, quality);
    int encoded_size = 0;
    jpeg_error_t encode_error = jpeg_enc_process(
        state->encoder, aligned_input, static_cast<int>(frame_len), jpeg_output,
        static_cast<int>(image_size), &encoded_size
    );
    heap_caps_free(aligned_input);
    release_frame(handle);

    if (encode_error != JPEG_ERR_OK || encoded_size <= 0) {
        LOG_E(TAG, "JPEG encoding failed: %d", static_cast<int>(encode_error));
        heap_caps_free(jpeg_output);
        return ERROR_RESOURCE;
    }

    *out_buf = jpeg_output;
    *out_len = static_cast<size_t>(encoded_size);
    return ERROR_NONE;
}

const CameraApi CAMERA_API = {
    .open = open_camera,
    .close = close_camera,
    .get_frame = get_frame,
    .release_frame = release_frame,
    .get_width = get_width,
    .get_height = get_height,
    .set_rotation = set_rotation,
    .set_flip = set_flip,
    .capture_jpeg = capture_jpeg,
    .has_flash = has_flash,
    .set_flash = set_flash,
};

} // namespace

extern "C" {

Driver sc101iot_driver = {
    .name = "sc101iot",
    .compatible = (const char*[]) { "smartsens,sc101iot", nullptr },
    .start_device = start,
    .stop_device = stop,
    .api = &CAMERA_API,
    .device_type = &CAMERA_TYPE,
    .internal = nullptr
};

} // extern "C"
