add_executable(${NAME}
        main.cpp
        ina219.cpp
        )

target_link_libraries(${NAME} 
        hardware_i2c
        hardware_timer
        hardware_adc
        hardware_dma
        hardware_pio
        pico_time
	pico_stdlib
        )
target_include_directories(${NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        )
  
pico_add_extra_outputs(${NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)
