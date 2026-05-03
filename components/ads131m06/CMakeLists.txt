idf_component_register(SRCS "adc_sense.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_gpio esp_driver_spi esp_timer)

set_source_files_properties(adc_sense.cpp PROPERTIES COMPILE_FLAGS "-x c")
