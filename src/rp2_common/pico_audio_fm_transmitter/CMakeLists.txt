if (NOT TARGET pico_audio_fm_transmitter)
    add_library(pico_audio_fm_transmitter INTERFACE)

    pico_generate_pio_header(pico_audio_fm_transmitter ${CMAKE_CURRENT_LIST_DIR}/audio_fm_transmitter.pio)

    target_sources(pico_audio_fm_transmitter INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/audio_fm_transmitter.c
            ${CMAKE_CURRENT_LIST_DIR}/sample_encoding.cpp
            )

    target_include_directories(pico_audio_fm_transmitter INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/pico-fractional-pll)
    target_link_libraries(pico_audio_fm_transmitter INTERFACE
            hardware_dma
            hardware_exception
            hardware_pio
            hardware_pwm
            hardware_irq
            hardware_interp
            pico_audio
            pico_multicore
            pico_fractional_pll)
endif()