
idf_component_register(
    SRC_DIRS        "." "ble"
    INCLUDE_DIRS    "." "ble"
    REQUIRES        "NeoPixel-WS2812" "esp_wifi" "bt" "nvs_flash" "esp_hw_support"
)

idf_build_set_property(COMPILE_OPTIONS "-Wno-format-nonliteral;-Wno-format-security;-Wformat=0" APPEND)
