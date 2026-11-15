idf_component_register(
    SRCS 
        "esp_hid_gap.c" 
        "WiFi.c" 
        "main.c" 
        "eye_state.c" 
        "decoder.c" 
        "ili9488.c"
    INCLUDE_DIRS "."
    REQUIRES 
        bt 
        esp_hid 
        driver 
        freertos 
        esp_wifi 
        esp_event 
        nvs_flash 
        lwip 
        esp_netif 
        mqtt 
        lvgl 
        espressif__esp_lcd_st7796 
        esp_lcd 
        esp_timer
)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)