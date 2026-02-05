idf_component_register(SRCS "main.c" "jpeg_enc.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_video esp_cam_sensor driver imx219 esp_timer esp_wifi esp_http_server esp_driver_jpeg nvs_flash esp_netif)
