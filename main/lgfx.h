#ifdef __cplusplus
extern "C" {
#endif


#define LGFX_TFT_BLACK       0x0000      //   0,   0,   0
#define LGFX_TFT_NAVY        0x000F      //   0,   0, 128
#define LGFX_TFT_DARKGREEN   0x03E0      //   0, 128,   0
#define LGFX_TFT_DARKCYAN    0x03EF      //   0, 128, 128
#define LGFX_TFT_MAROON      0x7800      // 128,   0,   0
#define LGFX_TFT_PURPLE      0x780F      // 128,   0, 128
#define LGFX_TFT_OLIVE       0x7BE0      // 128, 128,   0
#define LGFX_TFT_LIGHTGREY   0xD69A      // 211, 211, 211
#define LGFX_TFT_LIGHTGRAY   0xD69A      // 211, 211, 211
#define LGFX_TFT_DARKGREY    0x7BEF      // 128, 128, 128
#define LGFX_TFT_DARKGRAY    0x7BEF      // 128, 128, 128
#define LGFX_TFT_BLUE        0x001F      //   0,   0, 255
#define LGFX_TFT_GREEN       0x07E0      //   0, 255,   0
#define LGFX_TFT_CYAN        0x07FF      //   0, 255, 255
#define LGFX_TFT_RED         0xF800      // 255,   0,   0
#define LGFX_TFT_MAGENTA     0xF81F      // 255,   0, 255
#define LGFX_TFT_YELLOW      0xFFE0      // 255, 255,   0
#define LGFX_TFT_WHITE       0xFFFF      // 255, 255, 255
#define LGFX_TFT_ORANGE      0xFDA0      // 255, 180,   0
#define LGFX_TFT_GREENYELLOW 0xB7E0      // 180, 255,   0
#define LGFX_TFT_PINK        0xFE19      // 255, 192, 203 //Lighter pink, was 0xFC9F
#define LGFX_TFT_BROWN       0x9A60      // 150,  75,   0
#define LGFX_TFT_GOLD        0xFEA0      // 255, 215,   0
#define LGFX_TFT_SILVER      0xC618      // 192, 192, 192
#define LGFX_TFT_SKYBLUE     0x867D      // 135, 206, 235
#define LGFX_TFT_VIOLET      0x915C      // 180,  46, 226
#define LGFX_TFT_TRANSPARENT 0x0120

void lgfx_init(int32_t w, int32_t h, uint8_t* raw_buf);
void lgfx_drawString(const char *string, int32_t x, int32_t y);

void lgfx_fillScreen(int color);
void lgfx_fillRect(int32_t x, int32_t y, int32_t w, int32_t h, int color);
void lgfx_drawRect(int32_t x, int32_t y, int32_t w, int32_t h, int color);


uint8_t* lgfx_getBuffer(void);

#ifdef __cplusplus
}
#endif
