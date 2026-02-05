#pragma once

#include <stdint.h>

typedef struct {
    uint16_t reg;
    uint8_t val;
} imx219_reg_t;

/*
 * IMX219 1080p 30fps Settings
 * Based on 24MHz External Clock
 * 2 Lanes
 * MIPI Clock: 456 MHz (912 Mbps/lane)
 * Pixel Clock: 182.4 MHz
 */
static const imx219_reg_t imx219_1080p_30fps[] = {
    {0x0100, 0x00}, // Access standby mode

    // --- Special Access init sequences ---
    {0x30EB, 0x05}, {0x30EB, 0x0C}, {0x300A, 0xFF}, {0x300B, 0xFF},
    {0x30EB, 0x05}, {0x30EB, 0x09},

    // --- PLL Settings ---
    // Target: 912 Mbps/lane (456 MHz DDR), 182.4 MHz Pixel Clock
    {0x0301, 0x05}, // VTPXCK_DIV
    {0x0303, 0x01}, // VTSYCK_DIV
    {0x0304, 0x03}, // PREPLLCK_VT_DIV
    {0x0305, 0x03}, // PREPLLCK_OP_DIV
    {0x0306, 0x00}, // PLL_VT_MPY (High)
    {0x0307, 0x39}, // PLL_VT_MPY (Low) = 57
    {0x0309, 0x0A}, // OPPXCK_DIV (10)
    {0x030B, 0x01}, // OPSYCK_DIV
    {0x030C, 0x00}, // PLL_OP_MPY (High) 
    {0x030D, 0x72}, // PLL_OP_MPY (Low) = 114

    // --- CSI-2 Setup ---
    {0x0114, 0x01}, // 2 Lanes
    {0x0128, 0x01}, // DPHY CNTRL: Continuous Clock Mode
    {0x012A, 0x18}, // EXCK_FREQ = 24MHz
    {0x012B, 0x00},

    // --- Frame Timing 1080p30 ---
    {0x0160, 0x06}, {0x0161, 0xE3}, // Frame Length = 1763
    {0x0162, 0x0D}, {0x0163, 0x78}, // Line Length = 3448

    // --- Window / Crop ---
    {0x0164, 0x00}, {0x0165, 0x00}, // X Start
    {0x0166, 0x0C}, {0x0167, 0xCF}, // X End (3279)
    {0x0168, 0x00}, {0x0169, 0x00}, // Y Start
    {0x016A, 0x09}, {0x016B, 0x9F}, // Y End (2463)

    // --- Output Size 1920x1080 ---
    {0x016C, 0x07}, {0x016D, 0x80}, // Width 1920
    {0x016E, 0x04}, {0x016F, 0x38}, // Height 1080

    // --- Binning ---
    {0x0174, 0x00}, {0x0175, 0x00}, // No Binning

    // --- Data Format ---
    {0x018C, 0x0A}, // RAW10
    {0x018D, 0x0A}, // RAW10

    // --- Default Exposure & Gain ---
    {0x0157, 0xE8}, // Analogue Gain (Max x10.5)
    {0x015A, 0x06}, {0x015B, 0xD6}, // Coarse Integration Time (1750 lines)
};
