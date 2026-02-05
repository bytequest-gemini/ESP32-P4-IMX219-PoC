#include "esp_cam_sensor.h"

#define IMX219_I2C_ADDR 0x10

esp_cam_sensor_device_t *imx219_detect_sensor(void *config);

void imx219_force_link(void);
