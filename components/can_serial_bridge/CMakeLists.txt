idf_component_register(SRCS "usb_can_bridge.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES can_queue esp_driver_uart vfs)
