project(CAN)
add_library(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
        can2040.c
        can2040.h        
)

target_link_libraries(${PROJECT_NAME} 
    PRIVATE 
        cmsis_core
        pico_stdlib
        pico_multicore
        hardware_sync
        hardware_pio
        hardware_dma
        hardware_irq
)