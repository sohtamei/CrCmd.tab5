// カメラUSB挿抜

/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ppa.h"
#include "driver/jpeg_decode.h"
#include "usb/usb_host.h"
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#include "PTPDef.h"
#include "_8encoder.h"
#include "ppa_dsi_main.h"
#include "lgfx.h"

static const char *TAG = "DAEMON";

#define DAEMON_TASK_PRIORITY    2
#define CLASS_TASK_PRIORITY     3

#define CLIENT_NUM_EVENT_MSG    5

#define PARAM_NUM  5
/*
#include "esp_memory_utils.h"
void checkArea(int line, const void* dp)
{
	if(esp_ptr_internal(dp)) printf("(%d)I-RAM\n", line);
	else if(esp_ptr_external_ram(dp)) printf("(%d)PSRAM\n", line);
	else printf("(%d)unknown\n", line);
	return;
}
*/
struct _Param {
	uint16_t pcode;
	uint16_t datatype;
	uint8_t  getset;		// 0-R, 1-R/W
	uint8_t  isenabled;		// 0-invalid, 1-R/W, 2-R
	int32_t  current;
	uint8_t  formflag;
	int32_t* enums;
	int      enumNum;

	int32_t  currentIndex;
	uint32_t led;
} paramTable[PARAM_NUM] = {
	{DPC_SHUTTER_SPEED,			0,0,0,0,0,NULL,0,0,0},
	{DPC_FNUMBER,				0,0,0,0,0,NULL,0,0,0},
	{DPC_EXPOSURE_COMPENSATION,	0,0,0,0,0,NULL,0,0,0},
	{DPC_ISO,					0,0,0,0,0,NULL,0,0,0},
	{DPC_EXPOSURE_MODE,			0,0,0,0,0,NULL,0,0,0},
};

#define TIMEOUT_MS 100

enum {
	EVENT_NewDev  = 0x01,
	EVENT_CloseDev = 0x20,
	EVENT_DP_Changed = 0x02,
};

typedef struct {
	uint8_t                  dev_addr;
	usb_host_client_handle_t client_hdl;
	usb_device_handle_t      dev_hdl;
	uint32_t                 clientEvent;
	uint32_t                 ptpEvent;
} class_driver_t;
class_driver_t g_driver_obj = {0,NULL,NULL,0,0};

SemaphoreHandle_t sem_class;

usb_transfer_t *xfer_out = NULL;
usb_transfer_t *xfer_in = NULL;
usb_transfer_t *event_in = NULL;


static int updateDeviceProp(int onlyDiff);

static char g_linebuf[64] = {0};
struct rect g_rects[64];

#define JpegBuf_SIZE	0x40000
static uint8_t* jpegBuf = NULL;

//------------ usb host ------------

#define RECV_BUFFER  0x10000//0x20000-NG
#define BULK_IN_EP_ADDR		0x81
#define BULK_OUT_EP_ADDR	0x02
#define INT_IN_EP_ADDR		0x83

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
	class_driver_t *driver_obj = (class_driver_t*)arg;
	switch(event_msg->event) {
	case USB_HOST_CLIENT_EVENT_NEW_DEV:
		if(driver_obj->dev_addr == 0) {
			driver_obj->dev_addr = event_msg->new_dev.address;
			driver_obj->clientEvent |= EVENT_NewDev;
		}
		break;
	case USB_HOST_CLIENT_EVENT_DEV_GONE:
		if(driver_obj->dev_hdl != NULL) {
			driver_obj->clientEvent |= EVENT_CloseDev;
		}
		break;
	default:
		//Should never occur
		abort();
	}
}

static void xfer_cb(usb_transfer_t *transfer)
{
	assert(transfer);
	//hid_device_t *hid_device = (hid_device_t *)transfer->context;
	xSemaphoreGive(sem_class);
}

static void event_cb(usb_transfer_t *transfer)
{
	assert(transfer);
	//hid_device_t *hid_device = (hid_device_t *)transfer->context;
	//ESP_LOG_BUFFER_HEXDUMP(TAG, event_in->data_buffer, event_in->actual_num_bytes, ESP_LOG_INFO);

	uint16_t code = GetL16(event_in->data_buffer+6);
	switch(code) {
	case PTP_SDIE_DevicePropChanged:
		g_driver_obj.ptpEvent |= EVENT_DP_Changed;
		break;
	case PTP_SDIE_ObjectAdded:
	case PTP_SDIE_ObjectRemoved:
	case PTP_SDIE_CapturedEvent:
	case PTP_SDIE_CWBCapturedResult:
	case PTP_SDIE_CameraSettingReadResult:
	case PTP_SDIE_FTPSettingReadResult:
	case PTP_SDIE_MediaFormatResult:
	case PTP_SDIE_FTPDisplayNameListChanged:
	case PTP_SDIE_ContentsTransferEvent:
	case PTP_SDIE_DisplayListChangedEvent:
	case PTP_SDIE_FocusPositionResult:
	case PTP_SDIE_LensInformationChanged:
	case PTP_SDIE_OperationResults:
	case PTP_SDIE_AFStatus:
	case PTP_SDIE_MovieRecOperationResults:
	default:
		break;
	}

	int ret = usb_host_transfer_submit(event_in);
	if(ret) ESP_LOGE(TAG, "Error(%x)", ret);
}

int usb_host_init(void)
{
	ESP_LOGI(TAG, "Installing USB Host Library");
	usb_host_config_t host_config = {
		.skip_phy_setup = false,
		.root_port_unpowered = false,
		.intr_flags = ESP_INTR_FLAG_LEVEL1,
		.enum_filter_cb = NULL,
		.fifo_settings_custom = {0,0,0},
	};
	ESP_ERROR_CHECK(usb_host_install(&host_config));

	// 
	usb_host_client_config_t client_config = {
		.is_synchronous = false,
		.max_num_event_msg = CLIENT_NUM_EVENT_MSG,
		.async = {
			.client_event_callback = client_event_cb,
			.callback_arg = (void*)&g_driver_obj,
		},
	};
	ESP_ERROR_CHECK(usb_host_client_register(&client_config, &g_driver_obj.client_hdl));

	ESP_ERROR_CHECK(usb_host_transfer_alloc(256, 0, &xfer_out));
	xfer_out->callback = xfer_cb;
	xfer_out->context = NULL;
	xfer_out->bEndpointAddress = BULK_OUT_EP_ADDR;
	xfer_out->timeout_ms = TIMEOUT_MS;

	ESP_ERROR_CHECK(usb_host_transfer_alloc(RECV_BUFFER, 0, &xfer_in));
	xfer_in->callback = xfer_cb;
	xfer_in->context = NULL;
	xfer_in->bEndpointAddress = BULK_IN_EP_ADDR;
	xfer_in->timeout_ms = TIMEOUT_MS;

	ESP_ERROR_CHECK(usb_host_transfer_alloc(256, 0, &event_in));
	event_in->callback = event_cb;
	event_in->context = NULL;
	event_in->bEndpointAddress = INT_IN_EP_ADDR;
//	event_in->timeout_ms = TIMEOUT_MS;
	return 0;
}

int usb_host_connect(void)
{
	assert(g_driver_obj.dev_addr != 0);
	ESP_LOGI(TAG, "Opening device at address %d", g_driver_obj.dev_addr);
	ESP_ERROR_CHECK(usb_host_device_open(g_driver_obj.client_hdl, g_driver_obj.dev_addr, &g_driver_obj.dev_hdl));
	ESP_ERROR_CHECK(usb_host_interface_claim(g_driver_obj.client_hdl, g_driver_obj.dev_hdl, 0/*bInterfaceNumber*/, 0/*bAlternateSetting*/));

	xfer_out->device_handle = g_driver_obj.dev_hdl;
	xfer_in->device_handle = g_driver_obj.dev_hdl;
	event_in->device_handle = g_driver_obj.dev_hdl;

	assert(g_driver_obj.dev_hdl != NULL);
	usb_device_info_t dev_info;
	ESP_ERROR_CHECK(usb_host_device_info(g_driver_obj.dev_hdl, &dev_info));
	ESP_LOGI(TAG, "\t%s speed", (dev_info.speed == USB_SPEED_LOW) ? "Low" : "Full");
	ESP_LOGI(TAG, "\tbConfigurationValue %d", dev_info.bConfigurationValue);

	const usb_device_desc_t *dev_desc;
	ESP_ERROR_CHECK(usb_host_get_device_descriptor(g_driver_obj.dev_hdl, &dev_desc));
	usb_print_device_descriptor(dev_desc);

	const usb_config_desc_t *config_desc;
	ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(g_driver_obj.dev_hdl, &config_desc));
	usb_print_config_descriptor(config_desc, NULL);

	ESP_ERROR_CHECK(usb_host_device_info(g_driver_obj.dev_hdl, &dev_info));
	if(dev_info.str_desc_manufacturer) {
		ESP_LOGI(TAG, "Getting Manufacturer string descriptor");
		usb_print_string_descriptor(dev_info.str_desc_manufacturer);
	}
	if(dev_info.str_desc_product) {
		ESP_LOGI(TAG, "Getting Product string descriptor");
		usb_print_string_descriptor(dev_info.str_desc_product);
	}
	if(dev_info.str_desc_serial_num) {
		ESP_LOGI(TAG, "Getting Serial Number string descriptor");
		usb_print_string_descriptor(dev_info.str_desc_serial_num);
	}

	event_in->num_bytes = 256;
	int ret = usb_host_transfer_submit(event_in);
	if(ret) ESP_LOGE(TAG, "Error(%x)", ret);
	return 0;
}

static void class_driver_task(void *arg)
{
	while (1) {
		if(g_driver_obj.clientEvent == 0) {
			usb_host_client_handle_events(g_driver_obj.client_hdl, portMAX_DELAY);    // ★

		} else if(g_driver_obj.clientEvent & EVENT_NewDev) {
			g_driver_obj.clientEvent &= ~EVENT_NewDev;
			usb_host_connect();

		} else if(g_driver_obj.clientEvent & EVENT_CloseDev) {
			g_driver_obj.clientEvent &= ~EVENT_CloseDev;
			ESP_ERROR_CHECK(usb_host_device_close(g_driver_obj.client_hdl, g_driver_obj.dev_hdl));
			g_driver_obj.dev_hdl = NULL;
			g_driver_obj.dev_addr = 0;
			//We need to exit the event handler loop
			break;
		}
	}
	ESP_LOGI(TAG, "Deregistering Client");
	ESP_ERROR_CHECK(usb_host_client_deregister(g_driver_obj.client_hdl));

	xSemaphoreGive(sem_class);
	vTaskSuspend(NULL);
	return;
}

static void host_lib_daemon_task(void *arg)
{
	bool has_clients = true;
	bool has_devices = true;
	while (has_clients || has_devices) {
		uint32_t event_flags;
		ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));   // ★
		if(event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
			has_clients = false;
		}
		if(event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
			has_devices = false;
		}
	}
	ESP_LOGI(TAG, "No more clients and devices");

	ESP_ERROR_CHECK(usb_host_uninstall());
	xSemaphoreGive(sem_class);
	vTaskSuspend(NULL);
}

//-------------------- PTP ------------------

static uint32_t ptp_index = 0;

static int usb_ptp_transfer(uint32_t pcode,
							uint32_t num, uint32_t param0, uint32_t param1, uint32_t param2, uint32_t param3, uint32_t param4,
							const uint8_t* outbuf, uint32_t outsize,
							uint8_t* inbuf, uint32_t inbufSize, uint32_t* insize)
{
	int ret;
	BaseType_t received;

	assert(g_driver_obj.dev_hdl != NULL);

	// cmd

	xfer_out->num_bytes = 12+num*4;
	SetL32(xfer_out->data_buffer+0, 12+num*4);
	SetL16(xfer_out->data_buffer+4, 1);
	SetL16(xfer_out->data_buffer+6, pcode);
	SetL32(xfer_out->data_buffer+8, ptp_index++);
	for(int i = 0; i < num; i++) {
		switch(i) {
		case 0: SetL32(xfer_out->data_buffer+12+0, param0); break;
		case 1: SetL32(xfer_out->data_buffer+12+4, param1); break;
		case 2: SetL32(xfer_out->data_buffer+12+8, param2); break;
		case 3: SetL32(xfer_out->data_buffer+12+12, param3); break;
		case 4: SetL32(xfer_out->data_buffer+12+16, param4); break;
		}
	}

	ret = usb_host_transfer_submit(xfer_out);
	if(ret) ESP_LOGE(TAG, "Error(%x)", ret);
	received = xSemaphoreTake(sem_class, pdMS_TO_TICKS(100));
	if(received != pdTRUE) {
		ESP_LOGE(TAG, "Control Transfer Timeout");
		return -1;
	}

	// data(send)

	if(outbuf) {
		vTaskDelay(2);
		xfer_out->num_bytes = 12+outsize;
		SetL32(xfer_out->data_buffer+0, 12+outsize);
		SetL16(xfer_out->data_buffer+4, 2);
		memcpy(xfer_out->data_buffer+12, outbuf, outsize);

		ret = usb_host_transfer_submit(xfer_out);
		if(ret) ESP_LOGE(TAG, "Error(%x)", ret);
		received = xSemaphoreTake(sem_class, pdMS_TO_TICKS(100));
		if(received != pdTRUE) {
			ESP_LOGE(TAG, "Control Transfer Timeout");
			return -1;
		}
	}

	xfer_in->num_bytes = RECV_BUFFER;
	ret = usb_host_transfer_submit(xfer_in);
	if(ret) ESP_LOGE(TAG, "Error(%x)", ret);
	received = xSemaphoreTake(sem_class, pdMS_TO_TICKS(100));
	if(received != pdTRUE) {
		ESP_LOGE(TAG, "Control Transfer Timeout");
		return -1;
	}

	if(GetL16(xfer_in->data_buffer+4) == 2) {
	//	int size = xfer_in->actual_num_bytes; if(size >= 32) size = 32;
	//	ESP_LOG_BUFFER_HEXDUMP(TAG, xfer_in->data_buffer, size, ESP_LOG_INFO);
		if(inbuf && insize) {
			if(xfer_in->actual_num_bytes <= 12) {
				ESP_LOGE(TAG, "recv error");
				return -1;
			}
			int packetSize = GetL32(xfer_in->data_buffer+0);
			for(int count = 0; count < packetSize; count += RECV_BUFFER) {
				if(count == 0) {
					memcpy(inbuf+0, xfer_in->data_buffer+12, xfer_in->actual_num_bytes-12);

				} else if(count+xfer_in->actual_num_bytes <= inbufSize) {
					memcpy(inbuf+(count-12), xfer_in->data_buffer, xfer_in->actual_num_bytes);
				}
				if(xfer_in->actual_num_bytes == RECV_BUFFER) {
					ret = usb_host_transfer_submit(xfer_in);
					if(ret) ESP_LOGE(TAG, "Error(%x)", ret);
					received = xSemaphoreTake(sem_class, pdMS_TO_TICKS(100));
					if(received != pdTRUE) {
						ESP_LOGE(TAG, "Control Transfer Timeout");
						return -1;
					}
				}
			}
			*insize = packetSize-12;
		}

		ret = usb_host_transfer_submit(xfer_in);
		if(ret) ESP_LOGE(TAG, "Error(%x)", ret);
		received = xSemaphoreTake(sem_class, pdMS_TO_TICKS(100));
		if(received != pdTRUE) {
			ESP_LOGE(TAG, "Control Transfer Timeout");
			return -1;
		}
	}
//	ESP_LOG_BUFFER_HEXDUMP(TAG, xfer_in->data_buffer, xfer_in->actual_num_bytes, ESP_LOG_INFO);
	return 0;
}

//-------------------- client ------------------

int incParam(int index, int diff)
{
	if(paramTable[index].isenabled != 1) return -1;

	int currentIndex = paramTable[index].currentIndex + diff;
	if(currentIndex < 0)
		currentIndex = 0;

	if(currentIndex > paramTable[index].enumNum-1)
		currentIndex = paramTable[index].enumNum-1;

	if(paramTable[index].currentIndex == currentIndex) return 0;

	if(index != 4)
		paramTable[index].currentIndex = currentIndex;

	int32_t data = paramTable[index].enums[currentIndex];
	int size = 0;
	uint8_t buf[8]; 

	switch(paramTable[index].datatype) {
	case PTP_DT_INT8:
	case PTP_DT_UINT8:
		buf[0] = data;
		size = 1;
		break;

	case PTP_DT_INT16:
	case PTP_DT_UINT16:
		SetL32(buf, data);
		size = 2;
		break;

	case PTP_DT_INT32:
	case PTP_DT_UINT32:
		SetL32(buf, data);
		size = 4;
		break;

	default:
		return -1;
	}
	return usb_ptp_transfer(PTP_OC_SDIOSetExtDevicePropValue, 1, paramTable[index].pcode,0,0,0,0, buf,size, NULL,0,NULL);
}

int32_t getVariableVal(int datatype, uint8_t** dpp)
{
	int32_t val = 0;
	switch(datatype) {
	case PTP_DT_INT8:	val=*( int8_t*)(*dpp); (*dpp)++; break;
	case PTP_DT_UINT8:	val=*(uint8_t*)(*dpp); (*dpp)++; break;

	case PTP_DT_INT16:	val=( int16_t)GetL16(*dpp); (*dpp)+=2; break;
	case PTP_DT_UINT16:	val=(uint16_t)GetL16(*dpp); (*dpp)+=2; break;

	case PTP_DT_INT32:	val=( int32_t)GetL32(*dpp); (*dpp)+=4; break;
	case PTP_DT_UINT32:	val=(uint32_t)GetL32(*dpp); (*dpp)+=4; break;

	case PTP_DT_INT64:	(*dpp)+=8; break;
	case PTP_DT_UINT64:	(*dpp)+=8; break;

	case PTP_DT_STR: {
		int strLen = *(uint8_t*)((*dpp)++);
		(*dpp) += strLen*2;
		break;
		}
	default:
		ESP_LOGE(TAG, "unknown datatype");
		break;
	}
	return val;
}

static int updateDeviceProp(int onlyDiff)
{
	uint32_t recv_size = 0;
	int ret = usb_ptp_transfer(PTP_OC_SDIOGetAllExtDevicePropInfo, 
								1, onlyDiff,0/*Device Property Option*/,0,0,0,
								NULL,0,
								jpegBuf,JpegBuf_SIZE,&recv_size);
	if(ret) return ret;

	if(!jpegBuf || !recv_size)
		return -1;

	int propNum = GetL32(jpegBuf+0);
	(void)propNum;
	uint8_t* dp = jpegBuf+8;
	int ledUpdated = 0;
	for(; dp < jpegBuf+recv_size;) {
		uint16_t pcode    = GetL16(dp); dp+=2;
		uint16_t datatype = GetL16(dp); dp+=2;
		uint8_t getset    = *dp++;
		uint8_t isenabled = *dp++;
		int32_t factory = getVariableVal(datatype, &dp);
		(void)factory;
		int32_t current = getVariableVal(datatype, &dp);
		uint8_t formflag = *dp++;

		int i;
		int index = -1;
		for(i = 0; i < PARAM_NUM; i++) {
			if(paramTable[i].pcode == pcode) {
				paramTable[i].datatype = datatype;
				paramTable[i].getset = getset;
				paramTable[i].isenabled = isenabled;
				paramTable[i].current = current;
				paramTable[i].formflag = formflag;
				paramTable[i].currentIndex = 0;
				paramTable[i].enumNum = 0;
				if(paramTable[i].enums) {
					free(paramTable[i].enums);
					paramTable[i].enums = NULL;
				}
				index = i;
				break;
			}
		}

		switch(formflag) {
		case 0:
			break;
		case 1:		// range
			getVariableVal(datatype, &dp);
			getVariableVal(datatype, &dp);
			getVariableVal(datatype, &dp);
			break;
		case 2: {	// enum
			uint16_t num;
			int data;

			// set
			num = GetL16(dp); dp+=2;
			for(i = 0; i < num; i++)
				data = getVariableVal(datatype, &dp);

			// set/get
			num = GetL16(dp); dp+=2;
			if(index >= 0) {
				paramTable[index].enumNum = num;
				paramTable[index].enums = (int32_t*)malloc(num*sizeof(int32_t));
			}
			for(i = 0; i < num; i++) {
				data = getVariableVal(datatype, &dp);
				if(index >= 0) {
					paramTable[index].enums[i] = data;
					if(data == current)
						paramTable[index].currentIndex = i;
				}
			}
			break;
		  }
		default:
			ESP_LOGE(TAG, "Control Transfer Timeout");
			break;
		}
		if(index >= 0) {
			ESP_LOGI(TAG, "DP:%04x, %d,%d,%d,%d, %ld,%ld", pcode, datatype,getset,isenabled,formflag, paramTable[index].currentIndex,current);
			int led = (paramTable[index].isenabled==1 ? 0x101010: 0x000000);
			if(paramTable[index].led != led) {
				paramTable[index].led = led;
				ledUpdated = 1;
			}
		}
	}
	if(ledUpdated) {
		uint32_t buf[PARAM_NUM];
		for(int i = 0; i < PARAM_NUM; i++) buf[i] = paramTable[i].led;
		_8encoder_write(_8ENCODER_REG_RGB, buf, PARAM_NUM);
	}

	memset(g_linebuf, ' ', sizeof(g_linebuf)-1);
	g_linebuf[sizeof(g_linebuf)-1] = 0;

	format_shutter_speed(g_linebuf+0, paramTable[0].current);
	format_f_number(g_linebuf+7, paramTable[1].current);
	if(paramTable[2].current >= 0)
		sprintf(g_linebuf+12, "+%.1f", paramTable[2].current/1000.0);
	else
		sprintf(g_linebuf+12, "%.1f", paramTable[2].current/1000.0);
	format_iso_sensitivity(g_linebuf+17, paramTable[3].current);
	format_exposure_program_mode(g_linebuf+27, paramTable[4].current);

	for(int i = 0; i < sizeof(g_linebuf)-1; i++)
		if(g_linebuf[i] == 0) g_linebuf[i] = ' ';
//	printf("%s\n", g_linebuf);

	return 0;
}

#if 1
void parseLvProp(const uint8_t* buf, int bufSize, struct rect* rect, int rectSize, int* rectCount)
{
	int ver = GetL16(buf);
	int framesNum = 0;
	switch(ver) {
	case 101:
	case 102:
		framesNum = 3;
		break;
	case 103:
	case 104:
		framesNum = 4;
		break;
	default:
		ESP_LOGE(TAG, "(%d)illegal prop", __LINE__);
		return;
	}

	/*
	type FocalFrameInfo {
		+0		Version(2)		// Data version (100x value)
		+2		reserved(6)
		+8		reserved(8+24)
		+40		reserved Frame(16+24*0)

		+56		FocusFrame:
				FaceFrames:		// Version 1.01 or later
				TrackingFrames:	// Version 1.01 or later
				FramingFrames:	// Version 1.03 or later
	};
	*/
	const uint8_t* dp = buf + 56;

	*rectCount = 0;

	for(int i = 0; i < framesNum; i++) {
		struct focalFrames* frames = (struct focalFrames*)dp;

		if(frames->x_denominator == 0 || frames->y_denominator == 0) {
			return;
		}
		if(frames->x_denominator != (640*1024) || frames->y_denominator != (480*1024)) {
			ESP_LOGE(TAG, "(%d)illegal prop", __LINE__);
			return;
		}

		for(int j = 0; j < frames->frameNum; j++) {
			struct rect* rect = &g_rects[*rectCount];
			rect->x = frames->frames[j].x_numerator;
			rect->y = frames->frames[j].y_numerator;
			rect->w = frames->frames[j].width;
			rect->h = frames->frames[j].height;

			switch(i) {
			case 0:  rect->color = LGFX_TFT_GREEN; break;		// FocusFrame
			case 1:  rect->color = LGFX_TFT_GREEN; break;		// FaceFrames
			case 2:  rect->color = LGFX_TFT_WHITE; break;		// TrackingFrames
			case 3:  rect->color = LGFX_TFT_WHITE; break;		// FramingFrames
			}
			(*rectCount)++;
			if(*rectCount >= rectSize) return;
		}
		dp += 16/*focalFrames*/ + sizeof(struct focalFrame) * frames->frameNum;
		if(dp + 16 >= buf + bufSize) return;
	}
}
#endif

extern "C" void app_main(void)
{
	ESP_ERROR_CHECK(dsi_init());

	lgfx_init(RAW_BUF_W, RAW_BUF_H);

	jpegBuf = (uint8_t*)heap_caps_malloc(JpegBuf_SIZE, (MALLOC_CAP_DMA | MALLOC_CAP_CACHE_ALIGNED | MALLOC_CAP_SPIRAM));
	if(!jpegBuf) { ESP_LOGE(TAG, "(%d)mallocError", __LINE__); return; }

	usb_host_init();

	sem_class = xSemaphoreCreateBinary();

	TaskHandle_t daemon_task_hdl;
	TaskHandle_t class_driver_task_hdl;

	//Create daemon task
	xTaskCreatePinnedToCore(host_lib_daemon_task,
							"daemon",
							4096,
							(void*)sem_class,
							DAEMON_TASK_PRIORITY,
							&daemon_task_hdl,
							0);

	//Create the class driver task
	xTaskCreatePinnedToCore(class_driver_task,
							"class",
							4096,
							(void*)sem_class,
							CLASS_TASK_PRIORITY,
							&class_driver_task_hdl,
							0);
	int i;

	_8encoder_init();

	while(g_driver_obj.dev_hdl == NULL)
		vTaskDelay(100);
	vTaskDelay(50);

	// open session, connect(v3)
	int ret = 0;
	ret = usb_ptp_transfer(PTP_OC_OpenSession,          1, 1,  0,0,0,0, NULL,0, NULL,0,NULL);
	ret = usb_ptp_transfer(PTP_OC_SDIOConnect,          3, 1,  0,0,0,0, NULL,0, NULL,0,NULL);
	ret = usb_ptp_transfer(PTP_OC_SDIOConnect,          3, 2,  0,0,0,0, NULL,0, NULL,0,NULL);
	ret = usb_ptp_transfer(PTP_OC_SDIOGetExtDeviceInfo, 1, 300,0,0,0,0, NULL,0, NULL,0,NULL);
	ret = usb_ptp_transfer(PTP_OC_SDIOConnect,          3, 3,  0,0,0,0, NULL,0, NULL,0,NULL);

	vTaskDelay(200);

	// set up parametors
	uint8_t buf[32];
//	buf[0] = 0x01;			// PC Remote
//	usb_ptp_transfer(PTP_OC_SDIOSetExtDevicePropValue, 2, DPC_POSITION_KEY,1,0,0,0, buf,1, NULL,0,NULL);

	SetL16(buf, 0x0010);	// Camera
	usb_ptp_transfer(PTP_OC_SDIOSetExtDevicePropValue, 2, DPC_SAVE_MEDIA,1,0,0,0, buf,2, NULL,0,NULL);

	buf[0] = 0x01;			// off
	usb_ptp_transfer(PTP_OC_SDIOSetExtDevicePropValue, 2, DPC_USB_POWER_SUPPLY,1,0,0,0, buf,1, NULL,0,NULL);

//	buf[0] = 0x01;			// low
	buf[0] = 0x02;			// high
	usb_ptp_transfer(PTP_OC_SDIOSetExtDevicePropValue, 2, DPC_LIVEVIEW_MODE,1,0,0,0, buf,1, NULL,0,NULL);

	vTaskDelay(100);

	updateDeviceProp(false);

	uint32_t cnt_last[PARAM_NUM] = {0};
	uint32_t cnt_cur[PARAM_NUM] = {0};

	(void)ret;
	ret = _8encoder_read(_8ENCODER_REG_COUNTER, cnt_last, PARAM_NUM);
//	if(!ret) printf("%3ld,%3ld,%3ld,%3ld,\n", cnt_last[0],cnt_last[1],cnt_last[2],cnt_last[3],cnt_last[4]);//

	uint8_t positionKey = 0;
	uint32_t button_last = 1;

	uint64_t base_time = 0;
	uint32_t time1; (void)time1;
	uint32_t time2; (void)time2;
	uint32_t time3; (void)time3;
	uint32_t time4; (void)time4;

	uint64_t last_time = esp_timer_get_time(); (void)last_time;
	int last_count  = 0; (void)last_count;
	int frame_count = 0; (void)frame_count;

	while(1) {
		base_time = esp_timer_get_time();

		// 8encoder
		ret = _8encoder_read(_8ENCODER_REG_COUNTER, cnt_cur, PARAM_NUM);
	//	if(!ret) printf("%3ld,%3ld,%3ld,%3ld,\n", cnt_cur[0],cnt_cur[1],cnt_cur[2],cnt_cur[3],cnt_cur[4]);//

		uint32_t button_cur = 1;
		ret = _8encoder_read(_8ENCODER_REG_BUTTON+4, &button_cur, 1);
		uint32_t osd = 1;
		ret = _8encoder_read(_8ENCODER_REG_SWITCH, &osd, 1);

		time1 = esp_timer_get_time() - base_time;

		//### update DP ###
		if(g_driver_obj.ptpEvent & EVENT_DP_Changed) {
			g_driver_obj.ptpEvent &= ~EVENT_DP_Changed;
			updateDeviceProp(true);
		}

		for(i = 0; i < PARAM_NUM; i++) {
			int diff = cnt_cur[i] - cnt_last[i];
		//	if(diff < 0) diff += 1;
			diff = diff>>1;
			if(diff) {
				incParam(i, diff);
				cnt_last[i] = cnt_cur[i] & ~1;
			}
		}
		if(button_last == 1 && button_cur == 0) {
			positionKey = (positionKey?0:1);	// Camera/PC Remote
			usb_ptp_transfer(PTP_OC_SDIOSetExtDevicePropValue, 2, DPC_POSITION_KEY,1,0,0,0, &positionKey,1, NULL,0,NULL);
		}
		button_last = button_cur;
		time2 = esp_timer_get_time() - base_time;

		//### get LV ###
		uint32_t lv_buf_size = 0;
		ret = usb_ptp_transfer(PTP_OC_GetObject,
								1, 0xFFFFC002,0,0,0,0,
								NULL,0,
								jpegBuf,JpegBuf_SIZE,&lv_buf_size);		// liveview
		if(ret) continue;

		/*
		# header  0
		F4 03 00 00 		LiveView Image Offset
		00 BE 00 00 		LiveView Image Size
		F4 01 00 00 		Focal Frame Info Offset
		00 02 00 00 		Focal Frame Info Size

		# Focal Frame Info  Focal Frame Info Offset
		68 00 02 00 00 00 00 00 01 00 00 00 00 00 00 00 02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 FF 7F 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0A 00 00 80 07 00 00 00 00 00 00 00 00 00 00 00 0A 00 00 80 07 00 00 00 00 00 00 00 00 00 00 00 0A 00 00 80 07 00 00 00 00 00 00 00 00 00 00 00 0A 00 00 80 07 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 30 71 16 01 00 00 00 00 08 00 00 00 00 00 00 00 80 71 16 01 00 00 00 00 80 71 16 01 00 00 00 00 80 73 16 01 00 00 00 00 48 71 16 01 00 00 00 00 80 71 16 01 00 00 00 00 80 71 16 01 00 00 00 00 80 73 16 01 00 00 00 00 48 71 16 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 04 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 46 00 00 D8 05 00 80 00 70 18 01 00 00 00 00 94 19 5A B6 7F 00 00 00 00 00 00 00 00 00 00 0F 11 02 00 00 00 00 00 00 70 71 16 01 00 00 00 00 F0 19 5A B6 7F 00 00 00 04 00 00 00 05 00 00 00 06 00 00 00 07 00 00 00 

		# LiveView Image    LiveView Image Offset
		FF D8 FF DB 00 84 00 03 03 03 03 03 03 05 05 05 05 05 05 05 05 05 0B 08 05 05 08 0B 0E 0B 08 08 08 0B 0E 13 0E 0B 08 08 0B 0E 13 13 10 0E 0B 0E 10 13 16 10 0E 0E 10 16 16 13 13 13 16 18 16 16 18 1B 1B 1B 20 20 26 01 05 05 05 05 05 05 08 05 05 08 10 08 05 08 10 20 10 0B 0B 10 20 29 20 13 0E 13 20 29 29 29 23 18 18 23 29 29 29 29 29 26 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 FF C4 01 A2 00 00 01 05 01 01 01 01 01 01 00 00 00 00 00 00 00 00 01 02 03 04 05 06 07 08 09 0A 0B 10 00 02 01 03 03 02 04 03 05 05 04 04 00 00 01 7D 01 02 03 00 04 11 05 12 21 31 41 06 13 51 61 07 22 71 14 32 81 91 A1 08 23 42 B1 C1 15 52 D1 F0 24 33 62 72 82 09 0A 16 17 18 19 1A 25 26 27 28 29 2A 34 35 36 37 38 39 3A 43 44 45 46 47 48 49 4A 53 54 55 56 57 58 59 5A 63 64 65 66 67 68 69 6A 73 74 75 76 77 78 79 7A 83 84 85 86 87 88 89 8A 92 93 94 95 96 97 98 99 9A A2 A3 A4 A5 A6 A7 A8 A9 AA B2 B3 B4 B5 B6 B7 B8 B9 BA C2 C3 C4 C5 C6 C7 C8 C9 CA D2 D3 D4 D5 D6 D7 D8 D9 DA E1 E2 E3 E4 E5 E6 E7 E8 E9 EA F1 F2 F3 F4 F5 F6 F7 F8 F9 FA 01 00 03 01 01 01 01 01 01 01 01 01 00 00 00 00 00 00 01 02 03 04 05 06 07 08 09 0A 0B 11 00 02 01 02 04 04 03 04 07 05 04 04 00 01 02 77 00 01 02 03 11 04 05 21 31 06 12 41 51 07 61 71 13 22 32 81 08 14 42 91 A1 B1 C1 09 23 33 52 F0 15 62 72 D1 0A 16 24 34 E1 25 F1 17 18 19 1A 26 27 28 29 2A 35 36 37 38 39 3A 43 44 45 46 47 48 49 4A 53 54 55 56 57 58 59 5A 63 64 65 66 67 68 69 6A 73 74 75 76 77 78 79 7A 82 83 84 85 86 87 88 89 8A 92 93 94 95 96 97 98 99 9A A2 A3 A4 A5 A6 A7 A8 A9 AA B2 B3 B4 B5 B6 B7 B8 B9 BA C2 C3 C4 C5 C6 C7 C8 C9 CA D2 D3 D4 D5 D6 D7 D8 D9 DA E2 E3 E4 E5 E6 E7 E8 E9 EA F2 F3 F4 F5 F6 F7 F8 F9 FA FF C0 00 11 08 01 E0 02 80 03 01 21 00 02 11 01 03 11 01 FF DA 00 0C 03 01 00 02 11 03 11 00 3F 00 F8 45 4E 09 E3 AD 3C 0C D7 39 D7 73 2B 51 F9 1C 73 9A A2 3E 61 9A CB A9 EE 60 DF EE D1 32 8C 63 9A 99 70 4D 67 23 D4 83 24 40 14 E0 D5 90 06 46 2A 63 B9 D1 19 16 13 19 3E B5 7A 28 CE 29 CB 51 CB 58 B6 47 22 E1 F9 EF 5E 9F E0 1D 36 D1 D5 AE 64 8C 3C AA F8 42 7B 7B 8A 49 D9 33 C0 C5 6B 27 1E E7 D0 1A 6D F3 2F 21 41 C6 01 C8 A9 7C 47 A5 41 E2 9D 12 5B 19 9B CA 59 C0 2B 20 FE 16 1D 09 AE 34 9E A6 57 E5 B5 8F 0B D1 27 92 DE 49 AC 25 60 D3 D9 C8 63 62 3A 30 1D 18 7B 57 7B A6 BA F3 9E A7 A8 AE A8 6A B4 39 EA 68 DA 38 DF 88 D6 26 E3 4C 8E E9 47 EF 2D 64 D9 28 FF 00 A6 4D D1 BF 03 5E 0D 2D D9 8D F0 B9 C8 3C 9E D5 12 A4 E5 51 F9 A3 16 95 C4 7B E6 00 65 BA F4 04 FF 00 2A 72 DF CE 06 0E 7F 0A 6B 0E EE 2B 68 4A B7 12 3E 72 0E 45 03 ED 32 F0 33 9F 4F E9 5A 2C 35 B5 64 58 6C BB A2 6C 39 DA 7B 8F 4A 9A 3D B1 BC 7E 63 10 92 AE E5 3E A0 55 7B 2D 34 34 57 5A 32 E6 A1 75 F6 D9 8A 85 44 31 C4 02 04 E9 81 EB EF 54 7C 89 8A 8C 1E 08 EB 59 CA 0F 4B 9D 74 A4 94 6C 39 A3 9E 38 D5 70 0B 7F 0E 3B D4 26 29 5B 82 46 E1 FA 55 5B 43 9A A5 4B 48 BD 03 34 0F 0B 9F 9C AB 8E 47 61 EF 5F 47 08 0A E9 AF 36 01 12 C6 0A 1F 41 8F E2 FE 95 51 7A 3F 23 1B F3 33 85 D1 B4 F5 D4 75 A4 05 D6 3C 4B D4 F7 3E 95 A5 E3 5D 2E 5B 36 5D F9 C2 BE 07 E3 D2 A1 35 CF E6 69 CA ED 72 5B 5C 4D A7 83 FD D5 03 1F 4E F4 FB 4B 72 D1 4C AA 09 65 FD E2 AF FB 3D C0 FE 75 5B 31 74 17 90 33 9F 61 4B 7B 04 F2 B2 ED 5C A2 A0 00 FB FA 53 8A 57 0B D9 81 8A 59 36 29 52 02 8E 3F AE 6A 9A 44 B7 17 6C 07 01 46 09 F4 C7 6A D6 D6 0D D9 21 88 2B 48 87 8D C3 E5 3E F4 FB 27 38 74 EE 83 81 53 70 68 E3 2F FE 50 F8 CE 4B 1E 7F C6 B0 08 F9 1B 27 DC 7F 88 A7 14 66 95 AE 73 96 E0 96 E7 AE 4D 6F C2 A3 6E 33 8F 6A C2 A4 7D E3 58 76 34 A3 53 8E 9D AB B3 F8 65 6B 18 F1 D9 96 6E 23 4B 40 E8 3B 13 9C 37 E9 57 05 CA AE 53 4E D6 EE 3A FB 44 90 6A 17 6F BD 4A CB 3B BA 01 D4 AE 78 19 F6 AF 29 F1 76 9A DA 61 31 B3 06 67 0A F9 1E 8D D0 1F 71 5B C7 55 CD D4 C2 6E CB 94 EA 7C 19 18 1E 20 D3 C7 7D E3 1F 5C 57 D6 BA BC 60 7C 1E F0 B6 39 2D AD B1 FF 00 C7 8D 67 04 BD A1 9C 97 EE 99 C4 78 A9 77 4E 9E BB CE 45 7A 3F 81 22 3F F0 95 DA E3 92 43 60 7A 61 6A 2A F4 5D 2E 69 4F AF 92 3D CD E1 DB 2E 58 02 4F 5E D5 A3 66 92 38 EB 8C 56 BB B4 64 C2 E6 25 13 A8 EE 56 AD 47 80 83 00 F1 5A F9 18 A1 99 70 E3 91 82 79 A6 5F 85 2F 19 1F DF C7 FF 00 5E A5 2D 46 63 CB 6F BA 43 96 CE 0F E1 54 E5 8E 31 94 3D 4F 52 28 6F 54 35 D8 E0 75 C5 08 92 28 1C 11 C9 F5 AF 18 75 2D 75 B0 02 4E EE 3E 94 E7 AA B1 71 D4 F9 6B C5 77 22 F3 C4 FA 84 A4 E5 5A E7 04 7A EC E9 59 D2 4A 24 95 5B B3 9D D3 11 D8 FA 57 3C D5 A4 74 C3 E1 1D 32 08 58 4B 9F 91 88 08 BF 5A 6F 94 B0 CA 22 3F EA C1 CB 7D 7D 45 24 D5 CD 13 24 84 84 77 0F CA 9E 10 FB 7A 53 B7 4D A6 29 68 CE 37 9C 03 F5 ED 49 DA E5 25 AD C8 9E 16 31 61 41 32 1E BE BF 4A 0C 71 CB 08 87 38 94 72 CA 6A 62 BD EB 86 C3 15 77 CC B0 4A 4E 06 79 F4 AD 28 10 CB 7C 62 2C 14 84 F9 64 3D 38 EC 69 C9 6B A0 9B 44 B1 49 FB B0 80 64 F9 B9 07 F9 8A FA 0B F6 7B D4 3C BF 8B DA 4E 70 BF 6A 8E 5B 77 6E C7 E4 CE 3F 1A E6 C6 B7 0C 34 E4 BA 15 45 5E 6F D1 9F A2 BA F6 13 4D 97 D4 2F 5A C9 B6 81 E3 B4 5C F0 1D 78 F5 AF A1 8D 9D 28 9E 05 BD E9 1F 3B 7E D3 90 98 7C 3F A4 73 9C DD 38 1F 82 D7 CB 7E 31 1B 7E 1C DA 60 8D 8F 22 63 FC 28 C6 45 73 E0 FC AE 6D 87 D3 0F 57 FC 47 90 C4 80 A0 18 ED D0 D6 DD B9 42 AE 7B F7 FF 00 1A F1 2B AB 4D 9E C4 15 E2 72 5B 57 AF 53 48 0E 7E B5 DA B6 38 96 E6 36 A3 FE B1 78 3D 2A A8 07 23 F5 AC 64 F5 3D EC 27 F0 D1 28 E7 A7 4A 95 07 CD 50 F5 67 A5 02 C0 5C 1F A5 59 00 11 4B 66 6E 89 E3 5A D1 8B 3C 7D 29 B6 91 6D FB A3 67 1C 8A F5 4F 00 38 36 8E A4 8D E6 4C 81 ED 59 C9 DA 2D 9E 16 21 37 52 C7 B4 69 AA 48 C0 20 E7 F8 73 5B EA 1D 00 DC 3E 51 D3 D0 57 34 6A 45 E8 9E A6 12 8B 4A F6 3E 54 D6 6E 64 D3 FC 61 7D 2A FD D1 78 77 0F 51 DE BD 3F 4F BB 59 55 64 46 DC 0F CC 08 F4 AE 8A 69 A8 93 59 27 6B 6E 8D 9D 56 CA 2B FB 17 42 77 24 F1 95 23 D3 3E F5 F3 0D FC 36 FA 7D CC B6 F3 8F DE C4 E5 4F 1C E4 74 3F 43 5D 14 EC DD DE E7 2C EE 8C A9 2E ED 01 FF 00 57 B8 8E FD EA DA DC 4A D6 E2 55 B6 8C 23 0C AB 01 92 47 A9 AE CE 54 B5 39 FD E6 65 35 E4 A2 72 DD 09 ED DB 1E 95 B1 A5 99 90 5C 5C 48 42 2B C4 44 44 F4 DD FE C8 A5 52 CA 17 2A 92 E6 A9 62 2F B7 79 E8 22 9A 35 91 18 6D 63 8F 98 FA 73 ED 45 D6 95 25 B1 58 DE E3 21 57 F7 68 46 70 3E BD 87 B5 73 A9 68 77 55 A1 69 2B 15 9A 2B 9B 19 D1 DB 04 32 FC AC 0E 55 87 B1 AD C8 DB 79 2C 08 00 A8 DA 7B 2F AD 45 55 78 F3 23 9E D2 8B E5 25 79 46 C0 70 01 53 90 7D BD 6B 34 CA 5F 71 C8 27 AF 1D FE 95 93 D8 CE A4 7A B0 8A 44 8F 24 1C 83 DB B5 7B 7F FC 25 36 ED E1 BB 18 8C 17 0C D1 C4 16 69 FF 00 83 70 E8 33 EB 8A CA A4 DC 23 EA 44 13 BB 97 44 1E 1B B7 92 EE EB ED 3B 7C AB 55 93 E6 99 B8 F9 87 61 ED EA 6B A5 F1 DD E5 BD F5 B2 DD FC B3 5B C8 70 BB 0F 04 AF 07 69 F5 06 B9 28 55 F6 95 E4 97 D9 3B 23 16 B0 EA 72 D3 9B 63 0B 45 8F 75 B2 64 72 E8 0A 66 AF 5B 9F EC FB F4 7E DB B0 7D 30 7A E6 BD 19 77 39 FA 58 92 FE D8 59 DC 38 51 88 C9 DF 19 FF 00 64 F4 C7 D2 98 EF 1C 32 05 67 54 3B 73 8C F6 F5 F6 AB 4D DC 4B 51 55 CB 2E 54 86 CF 71 D0 D2 61 4B 64 05 04 FD E6 03 91 F5 AA B9 A5 AC 45 30 1B C6 47 3D 8F B5 40 12 05 97 78 24 3B 70 57 B3 1A CF 5B E8 49 C4 DE 8C EE EB 9C 9A C3 68 F0 AD 9E 01 AD B6 33 67 35 0A E1 C9 3D 49 AD EB 75 F7 E0 F5 FF 00 EB D6 15 37 34 81 D4 47 1C 50 58 17 61 F7 80 C1 F4 15 66 D3 50 6B 46 59 10 B2 B8 CE C6 51 CF 34 AF A1 BC 62 DB 2C 9D 62 FD E5 50 30 AA 7A B0 1D 0F A7 E3 5C 8F C4 C8 CC 3A B3 29 EB E5 C5 9F C4 55 D1 6E CD 18 E2 A2 A0 93 46 C7 82 A3 DD AF 59 13 D9 89 CF D1 4D 7D 65 AE C6 53 E1 17 83 01 18 DF AB B1 3F 99 E2 AA 93 BD 6B 79 1C B3 7F BA F5 38 7F 14 A1 FB 4A 7A 89 0E 3F C2 BD 33 E1 FA 93 E2 FB 7E 99 48 9C FF 00 E3 B5 13 BC 9E BB 1A 53 6A D2 5E 47 B9 CE 9F E9 1E C1 6A D5 91 63 11 03 00 FA D7 47 63 1E 83 25 8D BE D1 CF 38 5E 0F BD 5B 46 5B 78 81 66 0B D8 55 90 C9 3C 95 91 57 1C E7 BD 41 79 02 A8 00 F7 35 49 75 15 CC D3 6E 79 E7 E5 CD 63 CC 76 DC 01 D3 FC 2A 24 B4 29 6A 70 BE 23 8F 6C 4F 83 C9 07 03 FF 00 AD 5E 25 77 2B 59 A3 3E 32 63 42 78 EF C7 F3 A5 25 61 C5 9F 20 DD 97 6B D9 4C B9 DE F2 B3 3F AE 49 CD 4B 0A 19 20 3D 43 EE E1 7F D9 F5 AC 6A 7C 47 5C 17 BA 59 0C B2 C0 14 8D DE 50 FD DF FB D5 69 2D CD D5 88 00 81 72 CD 80 A7 A8 1E BF 4A CA F6 2F 5B 16 63 B5 86 59 63 B7 6C F9 B0 8C B8 3C 1C 8E F5 00 2A 75 37 49 54 FD 9A 26 E5 B1 9C 1F 5F A6 69 73 5C A4 2C A2 48 6F 25 BA 04 18 95 3E 53 EF DB F0 AA D0 C1 1D CB C9 77 8E 70 49 1E E6 92 76 07 A0 FD 39 2D E5 82 57 B8 25 26 1F EA CF AF B5 32 D4 6C 82 57 9C 60 1C EC 6F 43 D8 FF 00 8D 5E AC 45 B8 50 84 80 1E 1A 46 FB DD 8F BF D2 BB DF 02 EA 92 68 1E 2A D0 EF D0 92 6D B5 84 25 47 42 09 DA 40 3F 43 58 E3 17 36 1A A4 56 F6 2E 8B B5 58 F9 B3 F5 7B C4 83 7E 99 29 03 92 46 3F 13 9A AE 08 41 12 E3 81 8C D7 B1 87 97 36 1A 8C BA B8 9E 14 D7 2D 49 C7 B3 3E 6B FD A8 F7 3E 8D A2 8C 63 FD 2A 5C 7F DF 35 F2 7F 8C A4 12 FC 39 D3 F6 91 81 30 01 3D C7 5A DF 17 75 2C 1A E9 66 69 87 FF 00 77 AB FE 23 CB 62 4F 95 79 CF F3 AD 1B 65 21 9F 23 A9 15 E2 56 F8 DA 3D 9A 5F 09 8B 26 9B 71 1C 6B 27 CA 43 0C E3 B8 FA 8A AA 23 71 9C 83 81 5D 51 6A D6 39 65 07 16 8C 4B F5 66 7E 95 52 38 8F 7A CE D7 67 B5 85 FE 19 66 38 F8 AB 51 DB 92 38 A9 B6 A7 7C 1A 44 C2 03 9A 9F C9 60 30 45 2B 3B 9B 29 2B 92 88 88 22 AE C6 8C 05 0D 5C D6 E9 A1 97 04 01 9F 6A E6 2F 75 1B A5 7D B1 4D 2C 68 3F B8 71 9F AD 28 53 E7 BC 5E A8 F0 31 D3 E4 93 B6 8C B1 67 A8 EA 10 92 DF 6A BA C9 1C 30 73 90 6B 72 C3 C7 BE 30 D2 DF F7 1A 9D CB AF FC F3 90 EF 43 F5 06 AE 58 2A 76 F7 63 6F 43 CC 8E 2A AE DC D7 45 8B BD 46 7D 44 35 E3 B0 32 5C 49 FB EC 7F 7C FF 00 4A E8 BC 17 E2 07 86 F4 D9 4E 7E 47 6C C3 21 E8 A7 FB 84 F6 A6 A2 92 71 EC 6F CF 29 6A 7B E6 9A CB 75 6C F0 37 DE DB 98 8F F4 AF 18 F8 85 A1 2C 52 7D B9 61 47 90 0D B2 96 1C 71 D1 BD E9 43 DD 61 25 76 8F 1F D9 67 3F CC E4 C4 DF DD 41 D7 DF DA AC 4F 0C B6 F6 B0 ED 62 0C 24 EC 3E AA 7A 57 47 3D D0 D5 16 A5 A9 9F 7A 9F BD 59 07 DC 95 41 C0 F5 EE 05 74 6B A5 4F 14 08 B3 91 B5 46 51 D0 EE 28 A7 9D AC 29 D4 92 E4 22 9D 37 1A F7 EC 2C 71 C5 A7 DC 41 30 CC B0 BA F0 FD 38 E8 71 EE 2B 48 EA 5A 55 CA B4 53 DB 48 F1 9F BB 28 61 E6 29 F5 1E D5 C6 E5 D8 F6 15 29 54 4A 4B 46 8A DA B5 92 DB E9 BA 7C 31 37 9D E6 CC F2 24 83 B0 3C 6C FF 00 11 54 AD F2 22 DB C8 74 24 1E 7D 2B 58 BE 6A 47 97 5D 72 D5 D4 90 C8 66 F3 03 00 09 4C 2E 3D 3D EB 33 6B C5 DB 1D B3 FD 6B 2D D1 8D 4B 58 9B E6 2A 06 7A F5 AF 4D F0 7C F7 F7 36 B3 D9 83 6A F6 C6 30 66 4B 96 DB 14 78 3F 7C 67 BF B0 AC AA C5 38 EA 6D 81 8C 67 37 17 B1 DD 6B 9A 9D 97 87 51 ED A0 55 DB 24 41 A0 55 FB B8 61 CB 0F 6C F4 AC FD 56 47 6F 0C E9 D6 B9 4D B6 D6 C7 00 11 93 B8 E4 B3 7B 93 5E 7E 16 2A 38 89 F2 BD CD F1 55 57 BB 41 2F 85 17 74 4E 6C 6C C9 FF 00 9E 64 1F C0 D6 B6 A9 03 79 6B 28 C8 C7 07 1F CE BD 67 B2 48 E1 9A B1 66 F9 4D DE 83 05 C7 F1 DB B6 C9 31 FF 00 3C DB EE 9F A8 35 C5 6B 71 B3 DD C3 9C E3 C9 04 9E E4 55 A8 EC CC EE E3 EA 69 78 7A 34 48 6E 58 72 A4 F7 F6 1D 2B 94 8F 51 31 CE 23 F3 4F 9A 49 DB 1E 79 23 35 4D 37 A0 EF 6B 79 9D 4E A7 3C 96 D6 B6 EC A4 65 F8 39 19 C9 C5 51 D3 AF 1E EA 76 0C AA 36 46 58 38 FE 95 9A 5D 4A BE B6 30 6F 62 62 1C 81 CE 38 AC 39 5D E4 83 05 76 E0 60 FA 9F 7A B3 3B EA 72 B1 65 DB 3D 89 AE 82 DC 1E 31 D6 B0 9E AC DA 27 55 71 02 C9 67 0A 92 71 B4 7E 75 52 20 8A 31 C8 0B DC D4 A3 A6 9B D0 BB 02 89 19 71 D3 70 C5 62 7C 52 01 B5 F9 B8 FE 18 80 1E F8 15 AD 3D 19 8E 37 E0 89 D0 78 0E 20 DA FD A0 EB 88 E4 23 F0 43 9A FA DB C4 B0 6D F8 45 E0 30 DD 5F 52 27 3F 9F 5A DA 8A FD EB 67 0D 5F E1 76 3C EB C4 E0 AD DC 7C 64 AB 9F D0 D7 A8 7C 3D B7 77 F1 60 3D 00 B6 76 CF E1 59 CD 25 35 77 D4 B8 F5 F4 3D B2 68 88 9F B9 F9 6A FD 94 64 29 ED E9 5B 59 DD 76 32 7D 42 E0 00 EC 4F 03 6F 03 D4 FA 54 F0 20 68 C6 E0 0F 1D EB 53 32 E4 70 30 18 C1 F6 AA B7 51 BC 8E 99 E8 0D 5A 13 66 73 C4 FB DC 74 03 9F A5 62 5C 46 03 A1 C6 58 75 FA 54 D8 69 9E 7D E2 C0 BB 1C F4 21 78 15 F3 7F 8C AE 5E D3 40 BC 91 64 DA 55 06 1F BE 33 D0 7D 69 49 5D A4 F6 2E 1A DC F9 7E F6 5F 3A F1 E4 19 F9 DB 23 3D 71 E9 56 A1 97 7E A1 B9 40 41 39 08 8B D9 73 C1 3F E3 5C 95 1D 9B 3A E0 9E 9A 9B 2D 67 FD 99 AB 43 6A E8 BE 5A 36 66 61 DF F1 F4 AB 77 5B 97 54 97 50 81 57 CA B5 5C 6D ED 8E F5 CF 27 7B 2E 86 DA A6 48 E2 1B 9D 26 4D 42 21 BE FE 69 7E 55 FA F4 1F E1 49 72 B1 DD 69 71 59 22 ED BF 66 F9 B3 C3 00 7A 93 42 4E DA 6E 53 8F 53 2E 79 45 B6 DB 09 70 33 80 CE 7A 8C 77 3E D5 4A EE 29 2D A5 8A 1B 77 0F 1B 63 91 FC 8F F8 D2 B5 9E BB 32 7B 16 2F A1 49 DE 18 A2 66 59 01 F9 D3 3C 1F 6A 96 F1 A2 F9 6D 5F 8D F8 E7 D3 EB ED 5A 44 4C B3 A8 DA 9B 5F 28 2B 09 50 0F 94 AF 38 15 2C 2D 79 F6 61 B0 ED E4 C9 07 A8 61 D0 83 EC 45 2A 96 94 5A EE 82 0A D3 8B 7A 59 9F AB 7E 12 F1 32 F8 CB E1 5E 8D AA A9 CB DC C1 1A CD EB E6 A7 CA F9 FC 45 76 71 95 1E 58 20 64 1E B5 D9 95 D4 75 72 EA 12 97 C4 95 9F C8 F2 F1 71 E5 C5 55 8A DA E7 CC DF B5 29 27 4A D0 D4 0F F9 79 93 A7 7E 2B E3 CF 14 96 1F 0F AD 0E 72 16 EC A0 15 E8 E3 17 FB 9B ED 71 E0 D7 EE EA AE 9C C7 9B C4 5B 6A 91 C6 7A 56 D4 20 B2 9C 75 AF 12 B7 C6 CF 5A 1A 45 0B 64 93 4D A6 24 B2 2F 0F CA 7F BB EF 55 8C 6A C7 A7 15 D3 38 BA 6F 94 E6 F6 9E D1 26 8C 0D 4E CD E4 94 15 07 1D E9 F6 FA 4A B0 CB 0E 6A 11 E8 D2 A9 CB 04 96 E5 F8 F4 A8 80 C6 3F 1A B7 1E 95 1E 3A 63 E9 DE 9E 86 EA B3 2E 45 A2 19 1B E5 EB E8 69 64 D1 9D 5B 0C 31 8E A7 14 24 8B 8D 77 72 3F EC A3 C6 39 A7 BD A2 A2 7C E3 1E F4 58 EB 85 7B AB 75 39 0F 10 31 B7 08 80 F0 F9 E7 E9 5C A0 5E 49 AD E8 41 5A E7 85 99 4D FB 5B 1D FF 00 81 74 ED 27 54 BA BD 82 EC 4A D2 AD 8B BD 84 51 8D CD 2D C2 F3 E5 91 FD DC 72 4F 6A E2 0B 16 90 F0 3A 9E 9D 31 5D 12 5E F5 BA 1E 5A F8 79 BA DC 98 3B DB 30 3D 8F 5A BA F3 3C 8A BE 59 C6 08 39 1D 77 7A 9F 6A E6 9C 2C EE 76 51 A8 9A B3 3D E3 C1 3E 25 9B 52 81 52 62 AB 75 00 DA D8 3F 7C 0E E3 DE BB CD 4E CA CB 5B B3 78 65 3F EB 17 0F FE 22 B9 A5 7B DD 1B CB 45 A6 C7 CA DA B5 82 E8 DA CC 90 02 24 FB 3B E0 83 EB E8 7D 6B 4E 09 A5 BF B7 99 24 45 45 96 23 F6 6F 76 1D 40 A5 19 5E 37 D9 9D DC BC D0 8C DE E6 18 8C 5D 69 AE 98 FD E5 B9 DE 07 FB 27 86 AD 60 F2 47 75 A7 B6 E3 99 20 53 20 EC 57 A6 08 AB 9C BD CB 02 A3 6A CD F7 26 D4 E5 5B C6 90 42 A2 28 2D 8E C4 8F D3 3F C4 3E B5 A0 D1 5B CD A5 19 36 22 22 DB 02 8E 3E F1 94 1C 30 AC 12 D5 DC EC F8 21 1E E6 74 32 30 D1 AE 5B 23 36 D7 11 C9 09 3F C2 5B 86 03 EB 59 56 97 0E 2E 09 27 EF E7 3F 8F 6A EA A5 1F DD 37 D4 F2 31 D6 55 AC 6C C5 6B B9 B2 A7 8C FD DF 7E F9 A8 AF 94 34 AC A3 0A 15 B9 C7 7A E5 6E CE CC E6 95 9A BA 32 D9 71 20 19 C0 CF E5 5D DF 84 26 D0 A2 9A 6F ED 45 2E 84 0F 27 FB A0 F7 2E 2A 6A B7 C9 74 74 60 6F ED 52 4E C7 41 E2 F9 61 88 2B B0 49 8C 91 89 2C EE 97 84 16 CA 38 8D 17 F9 E6 AD E8 DA 2D 93 58 45 34 01 C7 DA A2 0E 37 1C 80 58 73 81 DA B9 A8 47 96 72 99 32 4A 78 D9 2E C5 CB 79 35 0D 36 EA 28 10 07 46 94 28 88 F4 CB 75 61 EF E9 5E 95 36 9C 67 B1 93 3C 79 68 48 E3 39 F6 1E F5 DD 04 E5 77 D1 17 8A 82 87 2B 5B B2 8E 86 DF 6B B1 BE B1 70 3F 7F 08 F2 FD 72 BC E2 B2 24 94 2C 20 48 AA 71 85 07 1C 8F 41 9A D9 2B A4 71 CA CB 44 3D 3C 94 46 01 42 83 F7 80 AC 3B 8F 0F E9 72 CE B2 94 C3 29 E1 86 77 7E 75 7B 6A 43 94 64 AC FA 16 35 1B 68 EF 23 44 19 CC 6D B9 06 3B FA FF 00 F5 AB 26 CA 01 63 7A C6 59 57 EE 60 C7 8C 73 D8 D6 13 9D 95 8D 2E AF 72 CC 96 96 77 84 88 C0 91 CF 50 87 0D F9 1E D5 CC 6A B6 49 6B 11 60 5B 39 C1 0C 07 1F 4A 98 4B 98 39 6E EE 8E 02 0F BF C8 C7 35 BD 00 0B 8E 73 E9 E9 49 C4 A8 6E 75 4C BB AD 22 6E 08 65 C9 F6 AA 71 44 0A 91 B8 16 27 9F 63 E9 52 91 D3 19 59 1B 16 B6 FB 70 00 21 47 5C D7 2B F1 22 68 6E 35 C6 99 4F C8 F1 C5 8C 7B 0C 7E 95 B5 35 7D 0C 31 0F 9E 29 1D 77 C3 C8 C1 F1 0D BE 7A 88 65 23 FE F8 35 F5 DF 8C 63 D9 F0 A7 E1 E8 C7 DE BC 26 B5 A4 AD 57 4E C7 2D 6D 29 7C CF 31 F1 34 7B EF 61 FF 00 66 43 9F CF BD 7A 9F C3 B5 2F E2 47 1D 36 DA 37 4F C2 B0 9E B2 5D D3 2A 36 B3 F4 3D BA 50 7C F4 04 7F CB 3E B5 6E DD 58 36 7A 82 2B A4 C4 6D C4 65 C9 C9 19 F4 ED 56 ED A0 DE 80 F5 C0 AD 52 33 7B 1A B6 EA C7 3E D5 4E E9 4A 48 A3 D7 9F A5 52 27 A9 45 93 AB 75 02 B0 27 88 33 93 8C E3 A0 A6 D0 D1 E5 3E 32 46 82 19 A4 93 A2 46 58 8F A5 7C 77 E3 BD 57 ED 5E 1D 90 67 0B 73 20 03 DB BE 2B 29 FB B6 36 A6 EC 99 E2 59 F3 4A FA 85 C0 3F CA B4 6C 2D 96 7F 30 B1 C3 5B 20 6F 66 3E 95 C9 3D 59 DB 18 F5 36 6D EF C4 BA 74 C6 71 FE 91 2B E1 37 7D EE 78 00 55 38 F7 DA F9 76 2E 72 24 6C 96 FE 79 AC 5C 75 3A 14 4D 59 10 DA 6A F1 BC 61 8D B4 04 34 D1 8E 80 8F F3 C5 45 72 DE 75 DC 9A CC 32 85 27 21 22 1D 48 F5 3E DE B5 69 24 0D 18 E9 3F DA 95 EE E4 C1 90 F7 F5 A4 B2 B8 FB 1B B4 93 2E 57 B0 FE 95 32 8A F9 99 F2 8F B1 D2 EE 6E 84 F7 8A 42 A2 FC D2 31 38 C7 A0 1E F4 9A 7E 65 26 5B A1 B9 41 E4 9E 30 3B 51 6D 3D 01 A6 8B 76 37 1F 62 D4 C5 D9 06 5B 4C 95 DB DB 07 AE 3D EA C4 B3 A4 D7 CF 24 21 96 09 24 26 15 F4 07 F8 45 24 A4 E5 7F B3 6D 85 76 9A 3E FF 00 FD 9C 35 01 37 C2 59 6C FE 6D F6 3E 20 64 E7 A6 1C 6E 01 6B E8 DB 78 A6 67 52 C7 24 1F C3 F0 AE BC AD 72 61 7D 9A DA 32 7F 99 E7 E3 D5 B1 52 7D CF 9B 3F 6A 5E 74 CD 08 73 FE B6 53 F8 E2 BE 32 F1 2B 13 E0 7B 68 C9 CF FC 4C 0E EF E7 5E 8E 3D 69 84 7D 93 33 C1 D9 51 A8 FA F3 1C 4C 63 31 8F D2 B4 E0 8D 84 4C 73 D0 FD DA F0 EA BF 7D 9E BD 3F 85 1D 8F 88 52 D3 4D 10 59 EE 08 44 40 ED E9 80 3A 0A E4 C0 8F D4 57 75 59 73 D4 97 A9 E6 E1 93 54 A2 DF 52 21 1E E6 C8 E8 2A C0 8C FA 63 D2 B2 47 7A 6C 91 22 E6 AE C4 83 A1 18 A4 CD 54 89 D3 28 C3 07 F0 AE 8A DA E6 19 D4 2B 8E 83 8C F6 34 9D EC 6B 09 0D 92 D3 CB 6C 1E 73 CD 57 9F 4D 86 EE 32 84 0C 11 DF B5 38 B1 B9 B8 BB A3 CB 3C 55 A3 5C D8 C9 19 39 78 73 C3 FA 1F 43 5C 8B DB BE DC 81 D2 BB 68 EC 79 78 E9 B9 49 48 BB A2 DF EA 3A 2E A3 0D E5 AB 98 AE AD A4 DF 04 9E 8D 8C 7E 44 70 6A BA A4 8F 74 CC CB 83 23 96 6C 0C 00 58 E4 E3 DA BA 7D 9A 93 BF 53 83 DA 34 B9 7A 16 B5 9B 26 8A DE 07 1C 06 C8 3F 5A C9 8B 7E 02 A6 4B 1E 00 1D FD AB 3A 91 B3 D4 D6 8C 99 A7 A7 EA 37 1A 6B 0C 6E 0C AF B8 32 9C 30 3E 87 D4 57 A8 68 DF 11 2E A7 8D A3 9E D1 D9 80 C2 CC BC 0A E0 AB 1B 5D A3 D3 A6 DC 95 99 C2 6B 57 AB 79 7D 30 11 E6 42 D9 77 C7 35 9D 6B A9 CB 6D B1 58 09 12 33 F2 03 D5 7D 94 F6 AC 16 C7 A7 08 FB 89 36 5B B8 49 A0 98 5D 44 A5 63 98 6E 5F 41 EA AD EB 44 32 CF 7F 7F 1B 38 51 B0 8C ED 18 55 55 E7 F0 A3 7D 0D 13 8A 8F 37 54 5B 97 C9 D5 A7 79 22 2B 14 C5 8E F8 D8 E1 58 76 60 7F A5 49 2D ED B5 9D 9C 56 72 01 30 04 B4 C5 4F DD 27 A0 07 BD 69 EC 9B 6A C6 13 C4 25 0B 3D D1 9B 7F A9 47 2D A8 82 18 8C 50 86 DC C0 9F 99 9B D5 BF A0 AC A5 99 A2 F9 87 38 39 FC AB B2 9D 3E 58 5B A9 E3 E2 2B 7B 49 DC EE 20 B5 2E AB 2C 79 C3 80 5B FD D3 D5 B3 55 2E 6C 9E 4B 9D C9 8F 9D 7E 43 FD E2 3A 91 5E 5D 4F 76 6D 1A D3 8B 70 BF 42 8F F6 7C BB 19 F7 61 D5 B0 A8 47 5F A5 6E F8 5A CA D6 E2 E2 5F B6 47 71 32 28 05 61 88 65 99 87 45 6F F6 4F 7A CE AD 55 08 39 4B 64 14 66 A9 D4 BB 34 7C 41 75 71 7B 2C C6 64 F2 F6 C7 B1 61 C6 04 49 FD C5 5E D5 CF E8 5E 24 D4 7C 3D A8 C5 14 92 CB 25 8B 4C 23 30 B1 E1 01 E0 34 7F 4F 4A CF 0E FD A4 5B E8 63 ED 9C 71 32 A8 F4 EE 7B AA 43 E6 EB 56 69 8E 7C C6 72 7D 02 8E B5 EB 4E A1 6D A6 E0 61 62 CE 31 DF 6F 5A EF A3 75 1D 4E 9C 5C B9 9C 2D D8 F3 BF 0F 46 67 D5 A1 1D 09 3C 63 BE EE C7 DB DE B3 EF 30 A1 40 E8 6E 80 FA 8C D5 C7 64 71 4E 4F 54 4A C8 AA 5C B2 F1 CF E3 55 8A 80 40 55 3C F3 81 5A BB 1C 8E 6D 32 FD A4 46 59 82 83 B5 88 3B 4F A1 F5 AE 77 52 B6 FB 34 87 CD 8F 0C 4F DF EE DE E3 DA B8 AB A6 E3 A7 43 AE 1C D2 8D D6 E6 14 D2 B1 C0 0E 57 69 EA 3A 83 E8 4F A5 47 AC BC B2 D8 44 D2 8C 3E E2 09 FE F0 C7 07 E9 59 50 4F 9B 53 78 B6 93 B9 E6 90 7C C7 F1 AD D8 7A 8C 64 56 F3 76 65 44 E9 ED EF 62 8E D1 43 29 7D A3 95 1D 09 F5 AF 44 8F C3 57 F6 7E 1F 8F 53 68 61 7B 7B BB 33 35 B4 8B CE DF 69 33 DE 9D 38 F3 CB 71 55 93 82 F5 3C EF FB 56 EA 64 CE ED A0 F6 00 7E B5 E7 BA C9 79 54 16 62 4B 4E 39 3D 7A D5 C3 7B 03 B7 29 EC 3F 0E 63 07 C4 31 F5 F9 6C E6 2A 3D C2 77 AF AF BC 74 81 3E 1A 7C 37 1D 8C 85 8D 6F 43 F8 DF 26 73 57 77 A6 BD 4F 2B F1 2A FF 00 A7 C5 9E 32 E7 8F C6 BD 73 E1 8D BB 36 BF 74 47 F0 D9 72 7D 41 35 83 57 92 7E 65 45 BD 7D 0F 61 92 39 3C FE 39 E2 B5 2C E3 C8 20 8E DF 95 74 A5 A9 83 7A 05 C4 04 64 E3 1C 71 57 ED 21 C5 BA E0 64 8F F3 C5 69 63 32 F4 40 90 40 18 C9 AA 73 C4 04 C7 3E 9C 1A A4 26 67 4B C3 6D 18 E5 73 58 F2 43 E5 B2 81 F3 16 C9 3F 5A 2E 34 79 0F C4 60 5A DA F9 7F BB 66 C7 1E E4 57 C2 9E 31 C4 7E 1F B4 5E F2 32 B2 FF 00 5A CE A2 D8 D6 9B E8 79 B6 DD A1 7F D8 6C E7 DB D0 D6 99 89 E0 16 CC BC FD A0 6E 90 7A A8 3C 7E 06 B8 A5 BD CF 4E 16 B1 AF AA C1 1E A9 7E B2 59 29 22 08 83 38 03 1C 8E E3 E9 4B A6 DB AE AF 05 ED C3 C8 12 4B 75 1B 7E 80 72 45 43 B9 AA 68 A7 0C B7 FA 5D 9C C6 58 DB CB BB 4C 19 5B B8 ED 8A CD 9A 65 B5 DB 0A A8 08 C9 F3 1F 6F 41 54 81 BB 89 34 6B 73 32 47 6C AC 54 E0 95 1D 07 AF E3 5A E2 3B 6D 5F 52 4B 36 C4 4B 19 1B DC F1 9F AF A1 A9 96 84 F5 B7 52 CE BF A6 CB A7 4B 1D B5 A4 8D 2C 23 05 F6 F4 03 D0 E3 A9 F7 A8 2E EF AD 6E 6D 4D BF 97 B2 62 30 06 31 C7 BD 4D D3 5A 0A 5A 36 BA 98 EB 71 2D 94 22 D4 9D D1 3F 50 7B 1F 51 E8 6B 56 D5 0A 49 0C 08 BB 9D 1C 48 A0 75 C7 7F C2 AA 3B B3 26 F5 47 DB 1F B3 2E A1 F6 8D 2B 5A B7 3C 96 D5 23 B8 C7 41 CA E3 8A FB 0A DC 16 20 1C A9 E8 2B 6C BD E9 56 3F CB 26 71 E6 0A D5 93 EE 91 F3 47 ED 4C 86 3D 1F 42 3E 93 CB FC AB E1 EF 12 4A 47 86 6D E3 1C 9F B7 1E 3F 0F EB 5E AE 3F 4A 78 5B F9 98 61 2E A8 54 FF 00 11 CB 44 49 DB E8 7A D6 C5 BE 0C 24 FF 00 10 6F D2 BE 7E AF C4 CF 62 9F C2 71 7E 24 F1 0C DA F6 BD 77 78 09 54 92 52 21 4F EE C6 38 51 FD 6A 95 A5 CD CB CE 8A 18 90 4E 31 5D 58 74 DC 54 A5 BB D4 E1 F8 2D 1E C7 A3 5B C4 55 46 7B 8A B4 14 8E BD 7D 69 F5 3A 56 C4 D1 A0 C7 AE 7A 9A B9 1C 58 51 8A 06 9B 1C D1 67 D3 AF 5A 72 12 AD 9E 73 FD EA 2D 72 B9 9A 66 B4 77 2E F1 EC 3C 8E C3 BD 4A AE 01 E8 69 A8 84 A7 72 69 F4 FB 7D 4E D9 E2 90 02 1C 63 FF 00 AF 5E 35 AE 68 77 1A 2D C3 23 02 50 F2 8D EA 3D 3E B5 D1 41 DA 56 EE 71 D7 5C D1 BF 63 0D 09 32 8E DF 29 E6 AE 47 90 FF 00 E7 A5 7A F4 62 9A 3C BA 8E C6 AE B6 88 74 B8 77 0F E3 E3 F2 AE 36 DD CD A4 EA C4 65 73 C1 FF 00 0A E6 C4 2B 5D 9D 18 67 76 58 92 E5 E0 90 E1 63 95 09 CA 16 19 E0 D0 75 4B A7 75 3B 80 D9 CA AA F0 07 E1 5E 54 F5 3E 82 84 63 6B B3 4A EE 31 71 74 67 B5 96 30 EE BF BC 8C 9C 10 48 E7 19 AC F3 2C 36 11 B2 3E D9 5E 43 F3 8E C0 7B 1F 5A 88 C4 A9 D5 49 5B A8 E7 D6 27 53 FB B5 FD C8 40 BE 53 
		*/
		int lv_offset = GetL32(jpegBuf+0);
		int lv_size   = GetL32(jpegBuf+4);
		int prop_offset = GetL32(jpegBuf+8);
		int prop_size   = GetL32(jpegBuf+12);

	//	for(int i = 0; i < prop_size; i++) {printf("%02x", jpegBuf[prop_offset+i]);} printf("\n");
		int rectCount = 0;
		if(prop_size)
			parseLvProp(jpegBuf+prop_offset, prop_size, g_rects, numof(g_rects), &rectCount);
		time3 = esp_timer_get_time() - base_time;

		//### render LV ###
		if(lv_size) {
			frame_count++;
			display_jpeg(jpegBuf+lv_offset, lv_size, g_linebuf, g_rects, rectCount, osd);
		}
#if 0
		time4 = esp_timer_get_time() - base_time;
	//	printf("%6ld,%6ld,%6ld,%6ld\n", time1, time2-time1, time3-time2, time4-time3);
#else
		uint64_t cur_time = esp_timer_get_time();
		if(cur_time > last_time + 3*1000*1000) {
			printf("%.1f\n", (frame_count - last_count)/((cur_time - last_time)/(1000.0*1000.0)));
			last_time = cur_time;
			last_count = frame_count;
		}
#endif
	//	vTaskDelay(1);
	}

	vTaskDelay(200);

	usb_ptp_transfer(PTP_OC_CloseSession, 1, 1,0,0,0,0, NULL,0, NULL,0,NULL);

	//Wait for the tasks to complete
	for (i = 0; i < 2; i++) {
		xSemaphoreTake(sem_class, portMAX_DELAY);
	}
//Error:
	_8encoder_delete();

	//Delete the tasks
	vTaskDelete(class_driver_task_hdl);
	vTaskDelete(daemon_task_hdl);

	dsi_close();
}

/*
esp_err_t usb_control_transfer(void)
{
	ESP_LOGI(TAG, "setconfig1");
	usb_transfer_t *transfer;
	
	ESP_ERROR_CHECK(usb_host_transfer_alloc(USB_SETUP_PACKET_SIZE+2, 0, &transfer));
	//ESP_ERROR_CHECK(usb_host_transfer_alloc(BULK_PACKET_SIZE, 0, &transfer));

	transfer->device_handle = g_driver_obj.dev_hdl;
	transfer->callback = xfer_cb;
	transfer->context = NULL;
	transfer->bEndpointAddress = 0;
	transfer->timeout_ms = TIMEOUT_MS;


	transfer->num_bytes = USB_SETUP_PACKET_SIZE+2;
  //uint8_t tmp[8] = {0x00,0x09,0x01,0x00,0x00,0x00,0x00,0x00};
	uint8_t tmp[8] = {0x80,0x00,0x00,0x00,0x00,0x00,0x02,0x00};
	memcpy(transfer->data_buffer, tmp, 8);

	ESP_ERROR_CHECK(usb_host_transfer_submit_control(g_driver_obj.client_hdl, transfer));

	BaseType_t received = xSemaphoreTake(sem_class, pdMS_TO_TICKS(100));
	if(received != pdTRUE) {
		ESP_LOGE(TAG, "Control Transfer Timeout");
		return ESP_ERR_TIMEOUT;
	}

	transfer->num_bytes = 8;
	uint8_t tmp2[8] = {0x00,0x09,0x01,0x00,0x00,0x00,0x00,0x00};
	memcpy(transfer->data_buffer, tmp2, 8);

	ESP_ERROR_CHECK(usb_host_transfer_submit_control(g_driver_obj.client_hdl, transfer));

	received = xSemaphoreTake(sem_class, pdMS_TO_TICKS(100));
	if(received != pdTRUE) {
		ESP_LOGE(TAG, "Control Transfer Timeout");
		return ESP_ERR_TIMEOUT;
	}
	ESP_LOGI(TAG, "setconfig3");
	usb_host_transfer_free(transfer);

	return ESP_OK;
}
*/
