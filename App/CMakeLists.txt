add_library(App STATIC)

target_sources(App PRIVATE
    app.c
    storage/param_store.c
    usb/usb_app.c
)

target_include_directories(App PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(App PUBLIC stm32cubemx)
