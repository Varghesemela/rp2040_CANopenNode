project(utils)
file(GLOB FILES *.c *.h)
add_library(${PROJECT_NAME} functions.cpp functions.h can_functions.cpp can_functions.h can_frames.h)

# pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/pio/can2040.pio)

target_sources(${PROJECT_NAME}
    PRIVATE
        functions.cpp
        functions.h
        can_functions.cpp
        can_functions.h  
        can_frames.h         
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    ../CAN
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC 
        cmsis_core
        pico_stdlib
        pico_multicore
        hardware_pio
        hardware_dma
        hardware_irq
        hardware_uart
        hardware_adc
        hardware_spi
        hardware_i2c
        hardware_flash
        hardware_sync
        hardware_watchdog
        hardware_pwm
        CAN
        
)