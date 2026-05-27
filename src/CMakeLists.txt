# Auto-discover all .c files in src/ and subdirectories
FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.c)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS 
        ${CMAKE_SOURCE_DIR}/include
    REQUIRES
        driver
        esp_event
        esp_http_server
        esp_netif
        esp_timer
        esp_wifi
        json
        nvs_flash
        spiffs
)
