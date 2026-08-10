# Auto-discover all .c files under main/, tasks/, drivers/, libs/
file(GLOB_RECURSE app_sources
    ${CMAKE_SOURCE_DIR}/main/*.c
    ${CMAKE_SOURCE_DIR}/tasks/*.c
    ${CMAKE_SOURCE_DIR}/drivers/*.c
    ${CMAKE_SOURCE_DIR}/libs/*.c)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS
        ${CMAKE_SOURCE_DIR}/main
        ${CMAKE_SOURCE_DIR}/tasks
        ${CMAKE_SOURCE_DIR}/drivers
        ${CMAKE_SOURCE_DIR}/drivers/ld14p
        ${CMAKE_SOURCE_DIR}/drivers/drv8833
        ${CMAKE_SOURCE_DIR}/libs
        ${CMAKE_SOURCE_DIR}
    REQUIRES
        driver
        esp_driver_gpio
        esp_driver_ledc
        esp_driver_rmt
        esp_driver_tsens
        esp_driver_uart
        esp_event
        esp_http_server
        esp_netif
        esp_timer
        esp_wifi
        nvs_flash
        spiffs
    )

# Build SPIFFS image from data/ and flash it together with the firmware
spiffs_create_partition_image(spiffs ${CMAKE_SOURCE_DIR}/data FLASH_IN_PROJECT)
