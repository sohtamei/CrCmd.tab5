#include "esp_log.h"
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#include "lgfx.h"

static LGFX_Sprite sprite;
static LGFX_Sprite sprite2;


void lgfx_init(int32_t w, int32_t h, uint8_t* raw_buf)
{
	sprite.setPsram(true);
	sprite.setColorDepth(lgfx::color_depth_t::rgb565_2Byte);
	if (!sprite.createSprite(w, h)) { ESP_LOGE("xx", "createSprite failed"); return; }
	sprite.fillScreen(TFT_BLACK);
//	sprite.setTextDatum(textdatum_t::top_left);
	sprite.setTextDatum(textdatum_t::bottom_left);
	sprite.setTextColor(TFT_WHITE, TFT_BLACK);
//	sprite.setTextSize(2);
	sprite.setFont(&fonts::FreeMonoBold12pt7b);

	const int BPP = 16;                   // RGB565
	sprite2.setColorDepth(lgfx::color_depth_t::rgb565_2Byte);
	sprite2.setBuffer((void*)raw_buf, w, h, BPP);
}

void lgfx_drawString(const char *string, int32_t x, int32_t y)
{
	sprite.drawString(string, x, y);
}
/*
void lgfx_fillScreen(int color)
{
	sprite.fillScreen(color);
}

void lgfx_fillRect(int32_t x, int32_t y, int32_t w, int32_t h, int color)
{
	sprite.fillRect(x, y, w, h, color);
}
*/
void lgfx_drawRect(int32_t x, int32_t y, int32_t w, int32_t h, int color)
{
	sprite2.drawRect(x - w/2, y - h/2, w, h, color);
}

uint8_t* lgfx_getBuffer(void)
{
	return (uint8_t*)sprite.getBuffer();
}
