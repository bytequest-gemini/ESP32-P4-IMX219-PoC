#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "imx219.h"
#include "imx219_regs.h"
#include "esp_log.h"
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "esp_sccb_intf.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "imx219";

#define IMX219_PID 0x0219

static const esp_cam_sensor_format_t imx219_format_1080p;

void imx219_force_link(void) {
    // This function exists only to force the linker to include this object file
}

// Helper to write registers
static esp_err_t imx219_write_reg(esp_cam_sensor_device_t *dev, uint16_t reg, uint8_t val) {
    return esp_sccb_transmit_reg_a16v8(dev->sccb_handle, reg, val);
}

// Set format (Resolution/FPS)
static esp_err_t imx219_set_format(esp_cam_sensor_device_t *dev, const esp_cam_sensor_format_t *format) {
    ESP_LOGI(TAG, "imx219_set_format called. Dev: %p, Fmt: %p", dev, format);
    
    if (!dev) {
        ESP_LOGE(TAG, "Dev is NULL!");
        return ESP_ERR_INVALID_ARG;
    }
    if (!format) {
        ESP_LOGW(TAG, "Format is NULL! Attempting fallback...");
        if (dev->cur_format) {
             ESP_LOGW(TAG, "Using dev->cur_format as fallback");
             format = dev->cur_format;
        } else {
             ESP_LOGW(TAG, "Using static default as fallback");
             format = &imx219_format_1080p;
        }
    }

    ESP_LOGI(TAG, "Setting format: %s, MIPI Clk: %lu", format->name ? format->name : "NULL Name", format->mipi_info.mipi_clk);
    
    // Apply the big register table
    int num_regs = sizeof(imx219_1080p_30fps) / sizeof(imx219_reg_t);
    for (int i = 0; i < num_regs; i++) {
        if (imx219_write_reg(dev, imx219_1080p_30fps[i].reg, imx219_1080p_30fps[i].val) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write reg 0x%04X", imx219_1080p_30fps[i].reg);
            return ESP_FAIL;
        }
    }
    
    // Update current format in device struct
    dev->cur_format = format;
    return ESP_OK;
}
// Helper to read registers
static esp_err_t imx219_read_reg(esp_cam_sensor_device_t *dev, uint16_t reg, uint8_t *val) {
    return esp_sccb_transmit_receive_reg_a16v8(dev->sccb_handle, reg, val);
}

// IOCTL for stream control
static esp_err_t imx219_ioctl(esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg) {
    int ret = ESP_OK;
    switch (cmd) {
        case ESP_CAM_SENSOR_IOC_S_STREAM:
            {
                int stream_on = *(int *)arg;
                ESP_LOGI(TAG, "Stream Control: %d", stream_on);
                
                // Write stream register
                ret = imx219_write_reg(dev, 0x0100, stream_on ? 0x01 : 0x00);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to write stream register!");
                    return ret;
                }
                
                // Wait for sensor to start streaming
                vTaskDelay(pdMS_TO_TICKS(100));
                
                // Read back to verify
                uint8_t reg_val = 0;
                ret = imx219_read_reg(dev, 0x0100, &reg_val);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Stream register 0x0100 read back: 0x%02X (expected: 0x%02X)", 
                             reg_val, stream_on ? 0x01 : 0x00);
                    if (reg_val != (stream_on ? 0x01 : 0x00)) {
                        ESP_LOGE(TAG, "Stream register mismatch! Sensor may not be streaming.");
                    }
                } else {
                    ESP_LOGE(TAG, "Failed to read back stream register!");
                }
            }
            break;
        default:
            break;
    }
    return ret;
}

static const esp_cam_sensor_format_t imx219_format_1080p = {
    .name = "1080p_30fps",
    .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
    .port = ESP_CAM_SENSOR_MIPI_CSI,
    .width = 1920,
    .height = 1080,
    .mipi_info = {
        .mipi_clk = 456000000, 
        .lane_num = 2,
        .hs_settle = 0,
    }
};

static esp_err_t imx219_get_format(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_t *format) {
    ESP_LOGI(TAG, "imx219_get_format called");
    if (dev->cur_format) {
        memcpy(format, dev->cur_format, sizeof(esp_cam_sensor_format_t));
        return ESP_OK;
    }
    // Fallback to default if not set
    memcpy(format, &imx219_format_1080p, sizeof(esp_cam_sensor_format_t));
    return ESP_OK;
}

static esp_err_t imx219_query_support_formats(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_array_t *parry) {
    parry->count = 1;
    parry->format_array = &imx219_format_1080p;
    return ESP_OK;
}

// Stub for parameter value retrieval
// Parameter value retrieval
static int imx219_get_para_value(esp_cam_sensor_device_t *dev, uint32_t id, void *arg, size_t size) {
    if (!arg) return ESP_ERR_INVALID_ARG;

    switch (id) {
        case ESP_CAM_SENSOR_DATA_SEQ: // 0x20014
             // Verify arg size
             if (size != sizeof(int)) return ESP_ERR_INVALID_ARG;
             *(int *)arg = ESP_CAM_SENSOR_DATA_SEQ_NONE;
             return ESP_OK;
        
        default:
             ESP_LOGW(TAG, "get_para_value called with ID: %lu (0x%X)", id, id);
             return ESP_ERR_NOT_SUPPORTED;
    }
}

// Stop stream and clean up
static int imx219_del(esp_cam_sensor_device_t *dev) {
    ESP_LOGI(TAG, "imx219_del called");
    if (dev) {
        // Stop streaming
        imx219_write_reg(dev, 0x0100, 0x00);
        
        if (dev->cur_format) {
            // Free any dynamic allocations if we had them (none here)
        }
        // Use free(dev) if it was malloced by separate logic, but here it's part of detection?
        // Usually detect allocates it. 
        // For now just stop stream.
        free(dev);
    }
    return ESP_OK;
}

static const esp_cam_sensor_ops_t imx219_ops = {
    .query_support_formats = imx219_query_support_formats,
    .set_format = imx219_set_format,
    .get_format = imx219_get_format,
    .get_para_value = imx219_get_para_value,
    .priv_ioctl = imx219_ioctl,
    .del = imx219_del,
};

// Main detection function registered via macro
ESP_CAM_SENSOR_DETECT_FN(imx219, ESP_CAM_SENSOR_MIPI_CSI, IMX219_I2C_ADDR) {
    esp_cam_sensor_config_t *sens_config = (esp_cam_sensor_config_t *)config;
    esp_sccb_io_handle_t sccb_handle = sens_config->sccb_handle;

    // 1. Hardware Detection (Read ID)
    // IMX219 ID is at 0x0000 (Model ID High) and 0x0001 (Model ID Low)
    // We can read 2 bytes starting from 0x0000? 
    // esp_sccb_transmit_receive_reg_a16v16 reads 16 bits?
    uint16_t id = 0;
    
    // Try reading model_id_high (0x0000) -> should be 0x02
    uint8_t id_h = 0;
    if (esp_sccb_transmit_receive_reg_a16v8(sccb_handle, 0x0000, &id_h) != ESP_OK) {
        ESP_LOGE(TAG, "I2C Error reading ID High");
        return NULL;
    }
    
    // Try reading model_id_low (0x0001) -> should be 0x19
    uint8_t id_l = 0;
    if (esp_sccb_transmit_receive_reg_a16v8(sccb_handle, 0x0001, &id_l) != ESP_OK) {
        ESP_LOGE(TAG, "I2C Error reading ID Low");
        return NULL;
    }
    
    id = (id_h << 8) | id_l;

    if (id != IMX219_PID) {
        ESP_LOGE(TAG, "ID Mismatch: Found 0x%04X, Expected 0x%04X", id, IMX219_PID);
        return NULL;
    }

    ESP_LOGI(TAG, "IMX219 Detected (ID: 0x%04X) via Auto-Detect", id);

    // 2. Create Object
    esp_cam_sensor_device_t *dev = calloc(1, sizeof(esp_cam_sensor_device_t));
    if (!dev) return NULL;

    dev->name = "IMX219";
    dev->id.pid = id;
    dev->ops = &imx219_ops;
    dev->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    dev->sccb_handle = sccb_handle; // Important: Save the handle
    dev->cur_format = &imx219_format_1080p; // Set default format to avoid NULL issues

    ESP_LOGI(TAG, "Device allocated at: %p", dev);
    
    return dev;
}
