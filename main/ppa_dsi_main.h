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
#define PPA_BUF_W  1280
#define PPA_BUF_H   720

esp_err_t _init_port(void);

void esp_dsi_resource_alloc(esp_lcd_dsi_bus_handle_t *mipi_dsi_bus, esp_lcd_panel_io_handle_t *mipi_dbi_io, esp_lcd_panel_handle_t *mipi_dpi_panel, void **frame_buffer);
void esp_dsi_resource_destroy(esp_lcd_dsi_bus_handle_t mipi_dsi_bus, esp_lcd_panel_io_handle_t mipi_dbi_io, esp_lcd_panel_handle_t mipi_dpi_panel);

esp_err_t display_jpeg(	jpeg_decoder_handle_t jpgd_handle,
						ppa_client_handle_t ppa_srm_handle,
						esp_lcd_panel_handle_t mipi_dpi_panel,
						uint8_t* jpeg_buf, uint32_t jpeg_size,
						uint8_t* raw_buf, size_t raw_size,
						uint8_t* ppa_buf, size_t ppa_size);


#ifdef __cplusplus
}
#endif
