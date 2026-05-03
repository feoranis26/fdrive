idf_component_register(
    SRCS "can_queue.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_twai esp_netif freertos
)
