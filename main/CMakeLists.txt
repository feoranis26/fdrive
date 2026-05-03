idf_component_register(SRCS "led_ind.cpp" "h_bridge_driver.cpp" "drive_config_store.cpp" "drive_control.cpp" "drive_management.cpp" "can_driver.cpp" "sense_service.cpp" "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES can_queue led_strip littlefs esp_driver_ledc esp_driver_twai esp_driver_uart esp_timer ads131m06 can_serial_bridge config_store command_router)

set_source_files_properties(led_ind.cpp h_bridge_driver.cpp PROPERTIES COMPILE_FLAGS "-x c")
