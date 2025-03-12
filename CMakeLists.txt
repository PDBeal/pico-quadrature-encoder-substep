
cmake_minimum_required(VERSION 3.12)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

project(pico-quadrature-encoder-substep
    VERSION 1.0.0
    DESCRIPTION "RP2040 library for reading quadrature encoder interface with sub-step accuracy using pico-sdk"
)

add_library(${PROJECT_NAME} INTERFACE
    quadrature_encoder_substep.cpp
)

target_sources(${PROJECT_NAME} INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder_substep.cpp
)

target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR})

if(PICO_SDK)
    pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder_substep.pio)
endif(PICO_SDK)


# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(${PROJECT_NAME} INTERFACE
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_pwm
    hardware_watchdog
)
