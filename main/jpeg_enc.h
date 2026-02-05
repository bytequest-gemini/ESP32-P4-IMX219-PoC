#pragma once

#include "esp_err.h"
#include <stdint.h>

esp_err_t jpeg_enc_init(int width, int height);
esp_err_t jpeg_enc_process(const void *src_data, int src_size, int width, int height, 
                           uint32_t pixel_format, uint8_t **out_data, int *out_size);
void jpeg_enc_free(uint8_t *data);
