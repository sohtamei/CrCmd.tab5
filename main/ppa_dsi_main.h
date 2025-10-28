/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SetL32(buf,data) {uint32_t d=(data);(buf)[0]=(d);(buf)[1]=(d)>>8;(buf)[2]=(d)>>16;(buf)[3]=(d)>>24;}
#define SetL16(buf,data) {uint32_t d=(data);(buf)[0]=(d);(buf)[1]=(d)>>8;}

#define GetL32(buf) (((buf)[0]<<0)|((buf)[1]<<8)|((buf)[2]<<16)|((buf)[3]<<24))
#define GetL16(buf) (((buf)[0]<<0)|((buf)[1]<<8))

#define numof(a) (sizeof(a)/sizeof((a)[0]))


#define RAW_BUF_W  1024
#define RAW_BUF_H  1024
#define PPA_BUF_W  1024
#define PPA_BUF_H   600

struct rect {
	uint32_t x;
	uint32_t y;
	uint32_t w;
	uint32_t h;
	uint16_t color;
};

esp_err_t _init_port(void);
esp_err_t app_touch_init(esp_lcd_touch_handle_t *tp);

esp_err_t dsi_init(void);
void dsi_close(void);

esp_err_t display_jpeg(const uint8_t* jpeg_buf, uint32_t jpeg_size, const char* linebuf, const struct rect* rects, int rectCount, int osd);


#ifdef __cplusplus
}
#endif
