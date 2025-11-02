/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "driver/ppa.h"
#include "driver/i2c.h"
#include "driver/jpeg_decode.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_jd9165.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_ldo_regulator.h"
#include "sdkconfig.h"

#include "ppa_dsi_main.h"

#include "lgfx.h"


static esp_lcd_dsi_bus_handle_t  mipi_dsi_bus = NULL;
static esp_lcd_panel_io_handle_t mipi_dbi_io = NULL;
static esp_lcd_panel_handle_t    mipi_dpi_panel = NULL;

static jpeg_decoder_handle_t jpgd_handle = NULL;
static ppa_client_handle_t ppa_blend_handle = NULL;

static size_t raw_size = 0;
static uint8_t* raw_buf = NULL;

#define GPIO_LCD_BL		(gpio_num_t)23	//22
#define TOUCH_RST			(22)
#define TOUCH_INT			(21)

esp_err_t _init_port(void)
{
	// esp_lcd_touch_new_i2c_gt911
	gpio_set_direction(TOUCH_INT, GPIO_MODE_OUTPUT);
	gpio_set_level(TOUCH_INT, 0);
	gpio_set_direction(TOUCH_RST, GPIO_MODE_OUTPUT);
	gpio_set_level(TOUCH_RST, 0);
	vTaskDelay(10/portTICK_PERIOD_MS);
	gpio_set_level(TOUCH_RST, 1);
	vTaskDelay(60/portTICK_PERIOD_MS);

	gpio_set_direction(GPIO_LCD_BL, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_LCD_BL, 1);			// ON

	return ESP_OK;
}

#define BSP_LCD_H_RES (800)
#define BSP_LCD_V_RES (480)

esp_err_t app_touch_init(esp_lcd_touch_handle_t *tp)
{
	esp_lcd_panel_io_handle_t tp_io_handle = NULL;
	esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
	ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_NUM_0, &tp_io_config, &tp_io_handle));

	const esp_lcd_touch_config_t tp_cfg = {
		.x_max = BSP_LCD_H_RES,
		.y_max = BSP_LCD_V_RES,
		.rst_gpio_num = TOUCH_RST,
		.int_gpio_num = TOUCH_INT,
		.levels = {
			.reset	 = 0,
			.interrupt = 0,
		},
		.flags = {
			.swap_xy  = 0,
			.mirror_x = 0,
			.mirror_y = 0,
		},
	};
	ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, tp));

	return ESP_OK;
}

esp_err_t dsi_init(void)
{
	_init_port();

	uint8_t* frame_buffer = NULL;
//	esp_dsi_resource_alloc(&mipi_dsi_bus, &mipi_dbi_io, &mipi_dpi_panel, (void**)&frame_buffer);

	#define USED_LDO_CHAN_ID    3			// LDO_VO3 is connected to VDD_MIPI_DPHY
	#define USED_LDO_VOLTAGE_MV 2500

    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = USED_LDO_CHAN_ID,
        .voltage_mv = USED_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));

    //---------------DSI resource allocation------------------//
    esp_lcd_dsi_bus_config_t bus_config = JD9165_PANEL_BUS_DSI_2CH_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus));

    esp_lcd_dbi_io_config_t dbi_config = JD9165_PANEL_IO_DBI_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &mipi_dbi_io));

    esp_lcd_dpi_panel_config_t dpi_config = JD9165_1024_600_PANEL_60HZ_DPI_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
    jd9165_vendor_config_t vendor_config = {
        .mipi_config = {
            .dsi_bus = mipi_dsi_bus,
            .dpi_config = &dpi_config,
        },
    };
    esp_lcd_panel_dev_config_t lcd_dev_config = {
        .reset_gpio_num = 27,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_config,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_jd9165(mipi_dbi_io, &lcd_dev_config, &mipi_dpi_panel));

    ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(mipi_dpi_panel, 1, (void**)&frame_buffer));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(mipi_dpi_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(mipi_dpi_panel));

	/* jpeg, ppa */

	jpeg_decode_memory_alloc_cfg_t raw_buf_cfg = { .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER };
	raw_buf = (uint8_t*)jpeg_alloc_decoder_mem(RAW_BUF_W * RAW_BUF_H * 2, &raw_buf_cfg, &raw_size);
    if (!raw_buf) { ESP_LOGE("xx", "no mem for raw_buf"); return -1; }

	jpeg_decode_engine_cfg_t decode_eng_cfg = {
		.intr_priority = 0,
		.timeout_ms = 40,
	};
	jpeg_new_decoder_engine(&decode_eng_cfg, &jpgd_handle);

    ppa_client_config_t ppa_blend_config = {
        .oper_type = PPA_OPERATION_BLEND,
        .max_pending_trans_num = 1,
    };
    ESP_ERROR_CHECK(ppa_register_client(&ppa_blend_config, &ppa_blend_handle));

	lgfx_init(RAW_BUF_W, RAW_BUF_H, raw_buf);

	return 0;
}

void dsi_close(void)
{
	free(raw_buf);

    ESP_ERROR_CHECK(ppa_unregister_client(ppa_blend_handle));
	jpeg_del_decoder_engine(jpgd_handle);
    ESP_ERROR_CHECK(esp_lcd_panel_del(mipi_dpi_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_io_del(mipi_dbi_io));
    ESP_ERROR_CHECK(esp_lcd_del_dsi_bus(mipi_dsi_bus));
}

//-------------------------------------------------------------------------------------

static int last_rectCount = 0;
esp_err_t display_jpeg(const uint8_t* jpeg_buf, uint32_t jpeg_size, const char* linebuf, const struct rect* rects, int rectCount, int osd)
{
	struct param {
		int  out_size;
		int  raw_width;
		int  raw_height;
		int  ppa_width;
		int  x_offset;
		float scale;
	} const paramTable[] = {
		{640*360*2, 640,360, 1280,   0, 720.0/360},		// 16:9
		{640*424*2, 640,424, 1086,  97, 720.0/424},		// 3:2
		{640*480*2, 640,480,  960, 160, 720.0/480},		// 4:3
		{640*640*2, 640,640,  720, 280, 720.0/640},		// 1:1

		{1024*576*2, 1024,576, 1280,   0, 720.0/576},		// 16:9
		{1024*680*2, 1024,680, 1084,  98, 720.0/680},		// 3:2
		{1024*768*2, 1024,768,  960, 160, 720.0/768},		// 4:3
		{1024*1024*2,1024,1024, 720, 280, 720.0/1024},		// 1:1
	};
	uint64_t base_time = 0; (void)base_time;
	uint32_t time1; (void)time1;
	uint32_t time2; (void)time2;
	uint32_t time3; (void)time3;
	uint32_t time4; (void)time4;
	uint32_t time5; (void)time5;
	base_time = esp_timer_get_time();

	//### jpeg dec ###
	jpeg_decode_cfg_t jpeg_decode_cfg = {
		.output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
		.rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_RGB,
		.conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
	};

	uint32_t out_size = 0;
	jpeg_decoder_process(jpgd_handle, &jpeg_decode_cfg,
        jpeg_buf, jpeg_size,
		raw_buf, raw_size, &out_size);

	int i;
	for(i = 0; i < numof(paramTable); i++) {
		if(paramTable[i].out_size == out_size) break;
	}
	if(i >= numof(paramTable)) return -1;
	const struct param* param = &paramTable[i];
	time1 = esp_timer_get_time() - base_time;
	time2 = time1;

	//### blend ###
	if(osd) {
		if(rectCount || last_rectCount) {
		//	lgfx_fillScreen(LGFX_TFT_BLACK);
		//	lgfx_fillRect(0,0, RAW_BUF_W,param->raw_height, LGFX_TFT_BLACK);
			for(int j = 0; j < rectCount; j++) {
				lgfx_drawRect(
					rects[j].x*param->raw_width /640/1024,	// 640->width
					rects[j].y*param->raw_height/480/1024,	// 480->height
					rects[j].w*param->raw_width /640/1024,
					rects[j].h*param->raw_height/480/1024,
					rects[j].color);
			}
		}
		last_rectCount = rectCount;

		lgfx_drawString(linebuf, 10, param->raw_height - 10);
		time2 = esp_timer_get_time() - base_time;

		//this operation will blend the bg_buf with the fg_buf
		ppa_blend_oper_config_t blend_config = {
			.in_bg.buffer = raw_buf,
			.in_bg.pic_w = param->raw_width,
			.in_bg.pic_h = param->raw_height,
			.in_bg.block_w = param->raw_width,
			.in_bg.block_h = param->raw_height,
			.in_bg.block_offset_x = 0,
			.in_bg.block_offset_y = 0,
			.in_bg.blend_cm = PPA_BLEND_COLOR_MODE_RGB565,
			.in_fg.buffer = lgfx_getBuffer(),
			.in_fg.pic_w = param->raw_width,
			.in_fg.pic_h = param->raw_height,
			.in_fg.block_w = param->raw_width,
			.in_fg.block_h = param->raw_height,
			.in_fg.block_offset_x = 0,
			.in_fg.block_offset_y = 0,
			.in_fg.blend_cm = PPA_BLEND_COLOR_MODE_RGB565,
			.out.buffer = raw_buf,
			.out.buffer_size = raw_size,
			.out.pic_w = param->raw_width,
			.out.pic_h = param->raw_height,
			.out.block_offset_x = 0,
			.out.block_offset_y = 0,
			.out.blend_cm = PPA_BLEND_COLOR_MODE_RGB565,
			.bg_rgb_swap = 0,
			.bg_byte_swap = 1,
			.bg_alpha_update_mode = PPA_ALPHA_NO_CHANGE,

		//	.bg_alpha_update_mode = PPA_ALPHA_SCALE,
		//	.bg_alpha_scale_ratio = 0.9,

		//	.bg_alpha_update_mode = PPA_ALPHA_FIX_VALUE,  // 215: invalid fg_alpha_fix_val
		//	.bg_alpha_fix_val = 0,		// union
			.bg_ck_en = false,

			.fg_rgb_swap = 0,
			.fg_byte_swap = 1,
			.fg_alpha_update_mode = PPA_ALPHA_NO_CHANGE,

		//	.fg_alpha_update_mode = PPA_ALPHA_SCALE,
		//	.fg_alpha_scale_ratio = 0.5,

		//	.fg_alpha_update_mode = PPA_ALPHA_FIX_VALUE,  // 215: invalid fg_alpha_fix_val
		//	.fg_alpha_fix_val = 128,	// 0~255, union

			.fg_fix_rgb_val = { .b = 0xd3, .g = 0x03, .r = 0xff, },
			.fg_ck_en = true,
			.fg_ck_rgb_low_thres = { .b = 0x00, .g = 0x00, .r = 0x00, },
			.fg_ck_rgb_high_thres = { .b = 0x01, .g = 0x01, .r = 0x01, },
			.mode = PPA_TRANS_MODE_BLOCKING,
		};
		ESP_ERROR_CHECK(ppa_do_blend(ppa_blend_handle, &blend_config));
	}
	time3 = esp_timer_get_time() - base_time;
	time4 = esp_timer_get_time() - base_time;

	//### draw ###
	ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(mipi_dpi_panel,
			0,0,
			param->raw_width,param->raw_height,
			raw_buf));

	time5 = esp_timer_get_time() - base_time;
//	printf("%6ld,%6ld,%6ld,%6ld,%6ld\n", time1, time2-time1, time3-time2, time4-time3, time5-time4);
	return 0;
}

#if 0
extern const uint8_t image_jpg_start[] asm("_binary_image_jpg_start");
extern const uint8_t image_jpg_end[] asm("_binary_image_jpg_end");

void app_main(void)
{
	ESP_ERROR_CHECK(dsi_init());

	lgfx_init(RAW_BUF_W, RAW_BUF_H);
	lgfx_drawString("Hello from LovyanGFX Sprite!", 10, 20);
	display_jpeg(image_jpg_start, image_jpg_end-image_jpg_start, NULL, NULL, 0, 0);

	vTaskDelay(10000 / portTICK_PERIOD_MS);

	dsi_close();
}
#endif
