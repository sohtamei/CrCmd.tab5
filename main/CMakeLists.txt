idf_component_register(
    SRCS "CrCmd.tab5.cpp" "_8encoder.c"
   #SRCS "drawImageData.cpp"
    INCLUDE_DIRS "."
    REQUIRES M5GFX M5Unified usb esp_driver_jpeg esp_psram
)
