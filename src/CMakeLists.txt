idf_component_register(
    SRCS "main.cpp" "dto.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES nvs_flash
               driver
               ieee802154
               esp_driver_gpio
               espressif__esp-zigbee-lib
               espressif__esp-zboss-lib
)
