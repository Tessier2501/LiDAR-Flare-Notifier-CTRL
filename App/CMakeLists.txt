add_library(App STATIC)

target_sources(App PRIVATE
    app.c
    board/board_critical.c
    board/board_sensor.c
    board/board_time.c
    flight/flight_core.c
    flight/flight_math.c
    flight/flight_params.c
    sensor/ptgl_cmd.c
    sensor/ptgl_parser.c
    sensor/ptgl_sensor.c
)

target_include_directories(App PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(App PUBLIC stm32cubemx)