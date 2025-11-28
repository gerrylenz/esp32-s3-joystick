idf_component_register(
    SRCS "main.cpp" "led_control.cpp"
    PRIV_REQUIRES usb
    usb_host_cdc_acm
    esp_driver_uart 
    vfs
    esp_driver_gpio
    esp_adc
    driver
    esp_timer
    freertos 
    esp_driver_usb_serial_jtag
    INCLUDE_DIRS "."
    )
