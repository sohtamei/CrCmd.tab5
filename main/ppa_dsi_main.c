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
#include "esp_lcd_ili9881c.h"
#include "esp_ldo_regulator.h"
#include "sdkconfig.h"

#include "ppa_dsi_main.h"




#define E1_ADDR			0x43  // ADDR=GND
#define E2_ADDR			0x44  // ADDR=VDD
#define SYS_SDA			31
#define SYS_SCL			32

#define PIO_REG_DEV_ID		0x01	// Device ID and Control
#define PIO_REG_IO_DIR		0x03	// 0=input, 1=output
#define PIO_REG_OUT_STATE	0x05	// 0=Low, 1=High
#define PIO_REG_OUT_HIZ		0x07	// Output High-Impedance (0=enable, 1=Hi-Z)
#define PIO_REG_PUPD_EN		0x0B
#define PIO_REG_PUPD_SEL	0x0D
#define PIO_REG_INP_STAT	0x0F

#define GPIO_LCD_BL		(gpio_num_t)22
/*
E1.P0	RF_PTH (H:ext,L:int)
E1.P1	SKP_EN
E1.P2	EXT5V_EN
E1.P4	LCD_RST
E1.P5	TP_RST
E1.P6	CAM_RST
E1.P7	HP_DETECT(I)

E2.P0	WLAN_PWR_EN
E2.P3	USB5V_EN
E2.P4	PWROFF_PULSE
E2.P5	nCHG_CURRENT(H:0.5A,L:1A)
E2.P6	CHG_STAT(I)
E2.P7	CHG_EN
*/
esp_err_t i2c0_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(
    			I2C_NUM_0, addr,
    			buf, sizeof(buf),
    			1000/portTICK_PERIOD_MS);
}

esp_err_t i2c0_read_reg(uint8_t addr, uint8_t reg, uint8_t* val)
{
    return i2c_master_write_read_device(
    			I2C_NUM_0, addr,
    			&reg, 1,
    			val, 1,
    			1000/portTICK_PERIOD_MS);
}

esp_err_t _init_port(void)
{
//	esp_err_t ret;

	const i2c_config_t i2c_conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SYS_SDA,
		.scl_io_num = SYS_SCL,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master = { .clk_speed = 100000 },
		.clk_flags = 0,
	};
	i2c_param_config(I2C_NUM_0, &i2c_conf);
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0/*I2C_MASTER_RX_BUF_DISABLE*/, 0/*I2C_MASTER_TX_BUF_DISABLE*/, 0));

	uint8_t reg1, reg2, reg3;
	i2c0_read_reg(E1_ADDR, PIO_REG_IO_DIR,   &reg1);	// 0=input, 1=output
	i2c0_read_reg(E1_ADDR, PIO_REG_OUT_HIZ,  &reg2);	// Output Hi-z(0=enable, 1=Hi-Z)
	i2c0_read_reg(E1_ADDR, PIO_REG_OUT_STATE,&reg3);	// 0=Low, 1=High
	printf("%02x %02x %02x\n", reg1,reg2,reg3);

	i2c0_read_reg(E2_ADDR, PIO_REG_IO_DIR,   &reg1);	// 0=input, 1=output
	i2c0_read_reg(E2_ADDR, PIO_REG_OUT_HIZ,  &reg2);	// Output Hi-z(0=enable, 1=Hi-Z)
	i2c0_read_reg(E2_ADDR, PIO_REG_OUT_STATE,&reg3);	// 0=Low, 1=High
	printf("%02x %02x %02x\n", reg1,reg2,reg3);

	// PI4IOE5V6408
	/*
	E1.P7	i	HP_DETECT
	E1.P6	H	CAM_RST
	E1.P5	H	TP_RST
	E1.P4	H	LCD_RST
	E1.P3	x
	E1.P2	H	EXT5V_EN
	E1.P1	H	SKP_EN
	E1.P0	L	RF_PTH (H:ext,L:int)
	*/
														//      iHHHLHHL
	ESP_ERROR_CHECK(i2c0_write_reg(E1_ADDR, PIO_REG_IO_DIR,   0b01111111));	// 0=input, 1=output
	ESP_ERROR_CHECK(i2c0_write_reg(E1_ADDR, PIO_REG_OUT_HIZ,  0b00000000));	// Output Hi-z(0=enable, 1=Hi-Z)
	ESP_ERROR_CHECK(i2c0_write_reg(E1_ADDR, PIO_REG_OUT_STATE,0b01110110));	// 0=Low, 1=High
	/*
	E2.P7	H	CHG_EN
	E2.P6	i	CHG_STAT
	E2.P5	H	nCHG_CURRENT(H:0.5A,L:1A)
	E2.P4	L	PWROFF_PULSE
	E2.P3	H	USB5V_EN
	E2.P2	x
	E2.P1	x
	E2.P0	L	WLAN_PWR_EN
	*/
														//      HiHLHiiL
	ESP_ERROR_CHECK(i2c0_write_reg(E2_ADDR, PIO_REG_IO_DIR,   0b10111001));	// 0=input, 1=output
	ESP_ERROR_CHECK(i2c0_write_reg(E2_ADDR, PIO_REG_OUT_HIZ,  0b00000110));	// Output Hi-z(0=enable, 1=Hi-Z)
	ESP_ERROR_CHECK(i2c0_write_reg(E2_ADDR, PIO_REG_OUT_STATE,0b10101000));	// 0=Low, 1=High

	gpio_set_direction(GPIO_LCD_BL, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_LCD_BL, 1);			// ON

	return ESP_OK;
}


void esp_dsi_resource_alloc(esp_lcd_dsi_bus_handle_t *mipi_dsi_bus, esp_lcd_panel_io_handle_t *mipi_dbi_io, esp_lcd_panel_handle_t *mipi_dpi_panel, void **frame_buffer)
{
	#define USED_LDO_CHAN_ID 3
	#define USED_LDO_VOLTAGE_MV 2500

    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = USED_LDO_CHAN_ID,
        .voltage_mv = USED_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));


	#define BSP_MIPI_DSI_PHY_PWR_LDO_CHAN       (3)    // LDO_VO3 is connected to VDD_MIPI_DPHY
	#define BSP_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV (2500)

	esp_ldo_channel_handle_t phy_pwr_chan = NULL;
	esp_ldo_channel_config_t ldo_cfg;
	memset(&ldo_cfg, 0, sizeof(ldo_cfg));
	ldo_cfg.chan_id = BSP_MIPI_DSI_PHY_PWR_LDO_CHAN;
	ldo_cfg.voltage_mv = BSP_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV;

	esp_ldo_acquire_channel(&ldo_cfg, &phy_pwr_chan);

    //---------------DSI resource allocation------------------//
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id = 0,
        .num_data_lanes = 2,
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = 730, // 1000 Mbps
    };
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_config, mipi_dsi_bus));

    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(*mipi_dsi_bus, &dbi_config, mipi_dbi_io));

#if 1  // M5GFX
    esp_lcd_dpi_panel_config_t dpi_config = {
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = 60,
        .virtual_channel = 0,
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565,
        .num_fbs = 1,
#else  // ppa_dsi sample
    // Refresh Rate = 80000000/(40+140+40+800)/(4+16+16+1280) = 60Hz
    esp_lcd_dpi_panel_config_t dpi_config = {
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = 80,
        .virtual_channel = 0,
        .in_color_format = LCD_COLOR_FMT_RGB565,
#endif
        .video_timing = {
            .h_size = 720,
            .v_size = 1280,
            .hsync_back_porch = 140,
            .hsync_pulse_width = 40,
            .hsync_front_porch = 40,
            .vsync_back_porch = 20,
            .vsync_pulse_width = 4,
            .vsync_front_porch = 20,
        },
        .flags.use_dma2d = true,
    };

    ili9881c_vendor_config_t vendor_config = {
        .mipi_config = {
            .dsi_bus = *mipi_dsi_bus,
            .dpi_config = &dpi_config,
            .lane_num = 2,
        },
    };
    esp_lcd_panel_dev_config_t lcd_dev_config = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_config,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9881c(*mipi_dbi_io, &lcd_dev_config, mipi_dpi_panel));

    ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(*mipi_dpi_panel, 1, frame_buffer));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(*mipi_dpi_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(*mipi_dpi_panel));
}

void esp_dsi_resource_destroy(esp_lcd_dsi_bus_handle_t mipi_dsi_bus, esp_lcd_panel_io_handle_t mipi_dbi_io, esp_lcd_panel_handle_t mipi_dpi_panel)
{
    ESP_ERROR_CHECK(esp_lcd_panel_del(mipi_dpi_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_io_del(mipi_dbi_io));
    ESP_ERROR_CHECK(esp_lcd_del_dsi_bus(mipi_dsi_bus));
}

//-------------------------------------------------------------------------------------

esp_err_t display_jpeg(	jpeg_decoder_handle_t jpgd_handle,
						ppa_client_handle_t ppa_srm_handle,
						esp_lcd_panel_handle_t mipi_dpi_panel,
						uint8_t* jpeg_buf, uint32_t jpeg_size,
						uint8_t* raw_buf, size_t raw_size,
						uint8_t* ppa_buf, size_t ppa_size)
{
	struct {
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
//	uint64_t base_time = 0;
//	base_time = esp_timer_get_time();

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

    ppa_srm_oper_config_t srm_config = {
        .in.buffer = (uint16_t*)raw_buf,
        .in.pic_w = paramTable[i].raw_width,
        .in.pic_h = paramTable[i].raw_height,
        .in.block_w = paramTable[i].raw_width,
        .in.block_h = paramTable[i].raw_height,
        .in.block_offset_x = 0,
        .in.block_offset_y = 0,
        .in.srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        .out.buffer = ppa_buf,
        .out.buffer_size = ppa_size,
        .out.pic_w = PPA_BUF_H,
        .out.pic_h = paramTable[i].ppa_width,
        .out.block_offset_x = 0,
        .out.block_offset_y = 0,
        .out.srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_90,
        .scale_x = paramTable[i].scale,
        .scale_y = paramTable[i].scale,
        .rgb_swap = 0,
        .byte_swap = 1,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    ESP_ERROR_CHECK(ppa_do_scale_rotate_mirror(ppa_srm_handle, &srm_config));

    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(mipi_dpi_panel,
    										0, paramTable[i].x_offset,
    										srm_config.out.pic_w, paramTable[i].x_offset + srm_config.out.pic_h,
    										ppa_buf));

//	printf("%6ld\n", (uint32_t)(esp_timer_get_time() - base_time));
	return 0;
}

#if 0
extern const uint8_t image_jpg_start[] asm("_binary_image_jpg_start");
extern const uint8_t image_jpg_end[] asm("_binary_image_jpg_end");

#define EXAMPLE_IMAGE_W  640
#define EXAMPLE_IMAGE_H  424


void app_main(void)
{
	_init_port();

    esp_lcd_dsi_bus_handle_t  mipi_dsi_bus = NULL;
    esp_lcd_panel_io_handle_t mipi_dbi_io = NULL;
    esp_lcd_panel_handle_t    mipi_dpi_panel = NULL;
    esp_dsi_resource_alloc(&mipi_dsi_bus, &mipi_dbi_io, &mipi_dpi_panel, NULL);

	size_t raw_size;
	jpeg_decode_memory_alloc_cfg_t rx_mem_cfg = { .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER };
	uint8_t* raw_buf = (uint8_t*)jpeg_alloc_decoder_mem(RAW_BUF_W*RAW_BUF_H*2, &raw_buf_cfg, &raw_size);

    size_t ppa_size = PPA_BUF_H * PPA_BUF_W * 4 * 2;
    uint8_t* ppa_buf = heap_caps_calloc(ppa_size, 1, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    if (!ppa_buf) {
        ESP_LOGE("xx", "no mem for ppa_buf");
        return ;
    }

	jpeg_decoder_handle_t jpgd_handle;
	jpeg_decode_engine_cfg_t decode_eng_cfg = {
		.intr_priority = 0,
		.timeout_ms = 40,
	};
	jpeg_new_decoder_engine(&decode_eng_cfg, &jpgd_handle);

    ppa_client_handle_t ppa_srm_handle = NULL;
    ppa_client_config_t ppa_srm_config = {
        .oper_type = PPA_OPERATION_SRM,
        .max_pending_trans_num = 1,
    };
    ESP_ERROR_CHECK(ppa_register_client(&ppa_srm_config, &ppa_srm_handle));


	display_jpeg(jpgd_handle, ppa_srm_handle, mipi_dpi_panel,
					//	jpeg_buf, jpeg_size,
						(uint8_t*)image_jpg_start, image_jpg_end-image_jpg_start,
						raw_buf, raw_size,
						ppa_buf, ppa_size);


	vTaskDelay(10000 / portTICK_PERIOD_MS);

    free(ppa_buf);
	free(raw_buf);

    ESP_ERROR_CHECK(ppa_unregister_client(ppa_srm_handle));
	jpeg_del_decoder_engine(jpgd_handle);
    esp_dsi_resource_destroy(mipi_dsi_bus, mipi_dbi_io, mipi_dpi_panel);
}
#endif
