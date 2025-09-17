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

#define size_mul 1.5
#define mul      1.5

#if 1
#define EXAMPLE_IMAGE_W  640
#define EXAMPLE_IMAGE_H  424
#else
#define EXAMPLE_IMAGE_W  320
#define EXAMPLE_IMAGE_H  240
#endif

esp_err_t _init_port(void);

void esp_dsi_resource_alloc(esp_lcd_dsi_bus_handle_t *mipi_dsi_bus, esp_lcd_panel_io_handle_t *mipi_dbi_io, esp_lcd_panel_handle_t *mipi_dpi_panel, void **frame_buffer);
void esp_dsi_resource_destroy(esp_lcd_dsi_bus_handle_t mipi_dsi_bus, esp_lcd_panel_io_handle_t mipi_dbi_io, esp_lcd_panel_handle_t mipi_dpi_panel);

esp_err_t display_jpeg(	jpeg_decoder_handle_t jpgd_handle,
						ppa_client_handle_t ppa_srm_handle,
						esp_lcd_panel_handle_t mipi_dpi_panel,
						uint8_t* jpeg_buf, uint32_t jpeg_size,
						uint8_t* raw_buf, size_t raw_size,
						uint8_t* scale_buf, size_t scale_size);


#ifdef __cplusplus
}
#endif
