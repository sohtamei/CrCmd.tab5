idf_component_register(
    SRCS "CrCmd.tab5.cpp" "_8encoder.c" "ppa_dsi_main.c" "esp_lcd_ili9881c.c" "lgfx.cpp"
    INCLUDE_DIRS "."
    REQUIRES usb esp_driver_ppa esp_driver_jpeg esp_lcd esp_timer LovyanGFX
)
