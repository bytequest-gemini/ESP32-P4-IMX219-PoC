/*
 * Hardware JPEG Encoder Wrapper for ESP32-P4
 * Supports GRAY, YUV422, RGB565, RGB888 input formats
 */
#include "jpeg_enc.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/jpeg_encode.h"
#include <linux/videodev2.h>

static const char *TAG = "jpeg_enc";

static jpeg_encoder_handle_t s_jpeg_enc_handle = NULL;

esp_err_t jpeg_enc_init(int width, int height)
{
    if (s_jpeg_enc_handle) {
        return ESP_OK;
    }

    jpeg_encode_engine_cfg_t encode_eng_cfg = {
        .timeout_ms = 5000,
    };
    
    ESP_RETURN_ON_ERROR(jpeg_new_encoder_engine(&encode_eng_cfg, &s_jpeg_enc_handle), TAG, "Failed to create JPEG encoder engine");
    
    ESP_LOGI(TAG, "JPEG Hardware Encoder Initialized");
    return ESP_OK;
}

esp_err_t jpeg_enc_process(const void *src_data, int src_size, int width, int height,
                           uint32_t pixel_format, uint8_t **out_data, int *out_size)
{
    if (!s_jpeg_enc_handle) {
         ESP_LOGE(TAG, "Encoder not initialized");
         return ESP_ERR_INVALID_STATE;
    }

    // Map V4L2 pixel format to JPEG encoder format
    jpeg_enc_input_format_t enc_format;
    jpeg_down_sampling_type_t sub_sample;
    
    switch (pixel_format) {
        case V4L2_PIX_FMT_YUV422P:
        case V4L2_PIX_FMT_YUYV:
            enc_format = JPEG_ENCODE_IN_FORMAT_YUV422;
            sub_sample = JPEG_DOWN_SAMPLING_YUV422;
            ESP_LOGD(TAG, "Encoding YUV422 %dx%d", width, height);
            break;
        case V4L2_PIX_FMT_RGB565:
            enc_format = JPEG_ENCODE_IN_FORMAT_RGB565;
            sub_sample = JPEG_DOWN_SAMPLING_YUV422;
            break;
        case V4L2_PIX_FMT_RGB24:
            enc_format = JPEG_ENCODE_IN_FORMAT_RGB888;
            sub_sample = JPEG_DOWN_SAMPLING_YUV444;
            break;
        default:
            // Default to GRAY for unknown formats (like RAW)
            enc_format = JPEG_ENCODE_IN_FORMAT_GRAY;
            sub_sample = JPEG_DOWN_SAMPLING_GRAY;
            ESP_LOGD(TAG, "Encoding GRAY %dx%d (format: 0x%lx)", width, height, pixel_format);
            break;
    }

    jpeg_encode_cfg_t enc_config = {
        .width = width,
        .height = height,
        .src_type = enc_format,
        .sub_sample = sub_sample,
        .image_quality = 80,
    };

    uint32_t out_len = 0;
    uint8_t *out_buf = NULL;
    
    jpeg_encode_memory_alloc_cfg_t mem_cfg = {
        .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
    };
    
    int max_size = (int)(width * height * 0.7); 
    size_t actual_alloc_size;
    
    out_buf = (uint8_t*)jpeg_alloc_encoder_mem(max_size, &mem_cfg, &actual_alloc_size);
    if (!out_buf) {
        ESP_LOGE(TAG, "Failed to alloc JPEG buffer");
        return ESP_ERR_NO_MEM;
    }
    
    esp_err_t ret = jpeg_encoder_process(s_jpeg_enc_handle, &enc_config, 
                                         (void*)src_data, src_size, 
                                         out_buf, actual_alloc_size, &out_len);
                                         
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "JPEG Encode Failed: %s", esp_err_to_name(ret));
        free(out_buf);
        return ret;
    }
    
    *out_data = out_buf;
    *out_size = out_len;
    
    return ESP_OK;
}

void jpeg_enc_free(uint8_t *data)
{
    if (data) free(data);
}
