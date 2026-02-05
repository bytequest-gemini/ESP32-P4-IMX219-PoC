#include <stdio.h>
#include "esp_log.h"
#include "imx219.h"
#include "esp_video_init.h"
#include "esp_video_ioctl.h"
#include "esp_video_device.h"
#include "esp_cam_sensor_detect.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/mman.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "jpeg_enc.h"
#include <linux/videodev2.h>

static const char *TAG = "app_main";

#define I2C_MASTER_SCL_IO           8
#define I2C_MASTER_SDA_IO           7
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          100000
#define XCLK_PIN                    20
#define XCLK_FREQ_HZ                24000000

// Wi-Fi Config
#ifdef CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#else
#define EXAMPLE_ESP_WIFI_SSID      "myssid"
#endif

#ifdef CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#else
#define EXAMPLE_ESP_WIFI_PASS      "mypassword"
#endif

// Stream definition
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// Queue to pass JPEG buffers from Capture Task to HTTP Task
typedef struct {
    uint8_t *data;
    size_t len;
} jpeg_frame_t;

static QueueHandle_t s_jpeg_queue = NULL;

// XCLK Generation
static void enable_xclk(void)
{
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_1_BIT, 
        .freq_hz = XCLK_FREQ_HZ,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&ledc_timer) != ESP_OK) ESP_LOGE(TAG, "Failed to config LEDC timer");

    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 1, 
        .gpio_num   = XCLK_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0,
    };
    if (ledc_channel_config(&ledc_channel) != ESP_OK) ESP_LOGE(TAG, "Failed to config LEDC channel");
    ESP_LOGI(TAG, "Enabled XCLK on GPIO %d at %d Hz", XCLK_PIN, XCLK_FREQ_HZ);
}

// Wi-Fi Connection
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Retry to connect to the AP");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void wifi_init_sta(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

// HTTP Stream Handler
esp_err_t stream_handler(httpd_req_t *req) {
    esp_err_t res = ESP_OK;
    char * part_buf[64];

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK) return res;

    ESP_LOGI(TAG, "Stream client connected");

    jpeg_frame_t frame;
    
    while(true){
        // Wait for a frame (forever or timeout)
        if (xQueueReceive(s_jpeg_queue, &frame, pdMS_TO_TICKS(1000))) {
            
            // Send Boundary
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
            if(res != ESP_OK) break;

            // Send Header
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, frame.len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
            if(res != ESP_OK) break;

            // Send Data
            res = httpd_resp_send_chunk(req, (const char *)frame.data, frame.len);
            free(frame.data);
            
            if(res != ESP_OK) break;
        }
    }
    ESP_LOGI(TAG, "Stream client disconnected");
    return res;
}

static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    const char *html = "<html><body><h1>ESP32-P4 IMX219 Color Stream</h1><img src='/stream' style='width:100%;'></body></html>";
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t stream_uri = {
            .uri       = "/stream",
            .method    = HTTP_GET,
            .handler   = stream_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &stream_uri);

        httpd_uri_t index_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = index_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &index_uri);
    }
    return server;
}

// Software Bayer BGGR to RGB demosaicing (2x2 block averaging)
// Output is half resolution for speed
static void demosaic_bggr_to_rgb(const uint8_t *raw10, uint8_t *rgb, int width, int height)
{
    int out_w = width / 2;
    int out_h = height / 2;
    
    for (int y = 0; y < out_h; y++) {
        for (int x = 0; x < out_w; x++) {
            int row0 = y * 2;
            int row1 = y * 2 + 1;
            int col = x * 2;
            
            int group = col / 4;
            int pos_in_group = col % 4;
            
            // RAW10 packed: 5 bytes per 4 pixels
            int offset0 = row0 * (width * 5 / 4) + group * 5;
            int offset1 = row1 * (width * 5 / 4) + group * 5;
            
            // BGGR pattern
            uint8_t b = raw10[offset0 + pos_in_group];
            uint8_t g1 = raw10[offset0 + pos_in_group + 1];
            uint8_t g2 = raw10[offset1 + pos_in_group];
            uint8_t r = raw10[offset1 + pos_in_group + 1];
            
            uint8_t g = (g1 + g2) / 2;
            
            int out_idx = (y * out_w + x) * 3;
            rgb[out_idx + 0] = r;
            rgb[out_idx + 1] = g;
            rgb[out_idx + 2] = b;
        }
    }
}

void app_main(void)
{
    // Enable Logs
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("cam_hal", ESP_LOG_WARN);
    esp_log_level_set("esp_video", ESP_LOG_WARN);
    
    // Init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Init WiFi
    wifi_init_sta();

    // Init JPEG Encoder (Hardware) - half resolution for color output
    jpeg_enc_init(960, 540);

    // Create Queue
    s_jpeg_queue = xQueueCreate(2, sizeof(jpeg_frame_t));

    // Start Webserver
    start_webserver();

    // Camera Init
    enable_xclk();
    imx219_force_link();

    esp_video_init_csi_config_t csi_config = {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port = I2C_MASTER_NUM,
                .scl_pin = I2C_MASTER_SCL_IO,
                .sda_pin = I2C_MASTER_SDA_IO,
            },
            .freq = I2C_MASTER_FREQ_HZ,
        },
        .reset_pin = -1, 
        .pwdn_pin = -1,
    };
    esp_video_init_config_t cam_config = { .csi = &csi_config };
    
    if (esp_video_init(&cam_config) != ESP_OK) {
        ESP_LOGE(TAG, "Video Init Failed");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(500)); 

    int fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDWR);
    if (fd < 0) return;

    struct v4l2_requestbuffers req = {
        .count = 2,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    ioctl(fd, VIDIOC_REQBUFS, &req);

    struct v4l2_buffer buf_q;
    for (int i = 0; i < 2; i++) {
        buf_q.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf_q.memory = V4L2_MEMORY_MMAP;
        buf_q.index = i;
        ioctl(fd, VIDIOC_QBUF, &buf_q);
    }

    // Set Format
    struct v4l2_format fmt = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .fmt.pix.width = 1920,
        .fmt.pix.height = 1080,
        .fmt.pix.pixelformat = V4L2_PIX_FMT_SBGGR10, // IMX219 Raw10
    };
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        ESP_LOGE(TAG, "Failed to set format");
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMON, &type);

    ESP_LOGI(TAG, "Camera Started (Color Mode). Waiting for clients...");
    
    struct v4l2_buffer buf_dq = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    
    // Map buffers
    void *mapped_bufs[2];
    for(int i=0; i<2; i++){
         struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = i };
         ioctl(fd, VIDIOC_QUERYBUF, &b);
         mapped_bufs[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
         if (mapped_bufs[i] == MAP_FAILED) ESP_LOGE(TAG, "Map failed");
    }

    uint32_t frame_count = 0;
    uint64_t last_time = esp_timer_get_time();

    // RGB buffer for demosaiced output (half resolution: 960x540x3)
    uint8_t *rgb_buf = heap_caps_malloc(960 * 540 * 3, MALLOC_CAP_SPIRAM);
    if (!rgb_buf) ESP_LOGE(TAG, "Failed to alloc RGB buffer");

    while (1) {
        if (ioctl(fd, VIDIOC_DQBUF, &buf_dq) == 0) {
            
            uint8_t *jpg_buf = NULL;
            int jpg_len = 0;
            
            uint8_t *raw_data = (uint8_t*)mapped_bufs[buf_dq.index];

            if (rgb_buf) {
                // Software demosaic: RAW10 BGGR -> RGB888 (half resolution)
                demosaic_bggr_to_rgb(raw_data, rgb_buf, 1920, 1080);
                
                // Encode RGB -> JPEG
                if (jpeg_enc_process(rgb_buf, 960 * 540 * 3, 960, 540, 
                                     V4L2_PIX_FMT_RGB24, &jpg_buf, &jpg_len) == ESP_OK) {
                    jpeg_frame_t jf = { .data = jpg_buf, .len = jpg_len };
                    if (xQueueSend(s_jpeg_queue, &jf, 0) != pdTRUE) {
                        free(jpg_buf);
                    }
                }
            }
            
            frame_count++;
            ioctl(fd, VIDIOC_QBUF, &buf_dq);

        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        uint64_t now = esp_timer_get_time();
        if ((now - last_time) >= 1000000) {
            ESP_LOGI(TAG, "FPS: %lu", frame_count);
            frame_count = 0;
            last_time = now;
        }
    }
}
