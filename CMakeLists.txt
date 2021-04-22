cmake_minimum_required(VERSION 3.15.3)
include(${CMAKE_TOOLCHAIN_FILE})
project(stwin-sensiml-simplestream C CXX)

enable_language(CXX ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)


set(APP_FILES
        STWin-SimpleStream/src/audio_process.c
        STWin-SimpleStream/src/datalog_application.c
        STWin-SimpleStream/src/main.c
        STWin-SimpleStream/src/sd_diskio.c
        STWin-SimpleStream/src/stm32l4xx_hal_msp.c
        STWin-SimpleStream/src/stm32l4xx_it.c
        STWin-SimpleStream/src/syscalls.c
        STWin-SimpleStream/src/system_stm32l4xx.c
        STWin-SimpleStream/src/usbd_cdc_interface.c
        STWin-SimpleStream/src/usbd_conf.c
        STWin-SimpleStream/src/usbd_desc.c
        STWin-SimpleStream/startup_stm32l4r9xx.s
        STWin-SimpleStream/knowledgepack/src/sml_output.c
        STWin-SimpleStream/knowledgepack/src/sml_recognition_run.c
        )

set(BSP_DRIVER_COMPONENT_FILES
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/hts221/hts221_reg.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/hts221/hts221.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/hts221/hts221_reg.h
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/hts221/hts221.h
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/iis2dh/iis2dh_reg.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/iis2dh/iis2dh.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/iis2dh/iis2dh_reg.h
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/iis2dh/iis2dh.h
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/iis2mdc/iis2mdc_reg.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/iis2mdc/iis2mdc.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/iis2mdc/iis2mdc_reg.h
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/iis2mdc/iis2mdc.h
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/iis3dwb/iis3dwb_reg.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/iis3dwb/iis3dwb.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/iis3dwb/iis3dwb_reg.h
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/iis3dwb/iis3dwb.h
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/ism330dhcx/ism330dhcx_reg.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/ism330dhcx/ism330dhcx.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/ism330dhcx/ism330dhcx_reg.h
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/ism330dhcx/ism330dhcx.h
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/lps22hh/lps22hh_reg.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/lps22hh/lps22hh.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/lps22hh/lps22hh_reg.h
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/lps22hh/lps22hh.h
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/stts751/stts751_reg.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/stts751/stts751.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/stts751/stts751_reg.h
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/stts751/stts751.h
        )

set(BSP_DRIVER_STWIN_FILES
        STSW-STWINKT01_V2.1.0/Drivers/BSP/STWIN/STWIN_audio.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/STWIN/STWIN_audio.h
        STSW-STWINKT01_V2.1.0/Drivers/BSP/STWIN/STWIN.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/STWIN/STWIN.h
        STSW-STWINKT01_V2.1.0/Drivers/BSP/STWIN/STWIN_bus.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/STWIN/STWIN_bus.h
        STSW-STWINKT01_V2.1.0/Drivers/BSP/STWIN/STWIN_bc.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/STWIN/STWIN_bc.h
        STSW-STWINKT01_V2.1.0/Drivers/BSP/STWIN/STWIN_env_sensors.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/STWIN/STWIN_env_sensors.h
        STSW-STWINKT01_V2.1.0/Drivers/BSP/STWIN/STWIN_motion_sensors.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/STWIN/STWIN_motion_sensors.h
        STSW-STWINKT01_V2.1.0/Drivers/BSP/STWIN/STWIN_motion_sensors_ex.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/STWIN/STWIN_motion_sensors_ex.h
        STSW-STWINKT01_V2.1.0/Drivers/BSP/STWIN/STWIN_sd.c
        STSW-STWINKT01_V2.1.0/Drivers/BSP/STWIN/STWIN_sd.h
        )

set(ST_HAL_FILES
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_dfsdm.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_cortex.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_dma.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_i2c_ex.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_i2c.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pcd_ex.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pcd.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_sd_ex.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_sd.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_spi_ex.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_spi.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim_ex.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart_ex.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_usart_ex.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_usart.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_sdmmc.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_usb.h
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc_ex.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd_ex.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sd_ex.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sd.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_usart_ex.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_usart.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_sdmmc.c
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.c
        )

set(APP_ST_MIDDLEWARES
        STSW-STWINKT01_V2.1.0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc/usbd_core.h
        STSW-STWINKT01_V2.1.0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc/usbd_ioreq.h
        STSW-STWINKT01_V2.1.0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc/usbd_ctlreq.h
        STSW-STWINKT01_V2.1.0/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
        STSW-STWINKT01_V2.1.0/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
        STSW-STWINKT01_V2.1.0/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
        STSW-STWINKT01_V2.1.0/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
        STSW-STWINKT01_V2.1.0/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc/usbd_cdc.h
        )

set(FATFS_MIDDLEWARES
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FatFs/src/diskio.c
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FatFs/src/diskio.h
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FatFs/src/ff_gen_drv.c
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FatFs/src/ff_gen_drv.h
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FatFs/src/ff.c
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FatFs/src/ff.h
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FatFs/src/integer.h
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FatFs/src/option/syscall.c
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FatFs/src/option/unicode.c
        )

set(FREERTOS_MIDDLEWARES
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/timers.c
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/tasks.c
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/queue.c
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/list.c
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/include/timers.h
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/include/task.h
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/include/queue.h
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/include/list.h
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/include/FreeRTOS.h
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.h
        )

if (${SENSIML_RECOGNITION})
    set(EXECUTABLE ${PROJECT_NAME}-recognition.elf)
else()
    set(EXECUTABLE ${PROJECT_NAME}-data-collection.elf)
endif ()



add_executable(${EXECUTABLE}
        ${APP_FILES}
        ${BSP_DRIVER_COMPONENT_FILES}
        ${BSP_DRIVER_STWIN_FILES}
        ${ST_HAL_FILES}
        ${APP_ST_MIDDLEWARES}
        ${FATFS_MIDDLEWARES}
        ${FREERTOS_MIDDLEWARES}
        )
set_target_properties(${EXECUTABLE} PROPERTIES LINKER_LANGUAGE CXX)

target_compile_definitions(${EXECUTABLE} PRIVATE
        -DSTM32L4R9xx
        -DUSE_HAL_DRIVER
        -DARM_MATH_CM4
        )

target_compile_options(${EXECUTABLE} PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -fdata-sections
        -ffunction-sections
        -Wall
        $<$<CONFIG:Debug>:-Og>
        )

target_include_directories(${EXECUTABLE} PRIVATE
        STWin-SimpleStream/inc
        STWin-SimpleStream/knowledgepack/inc
        STWin-SimpleStream/knowledgepack/lib/inc
        STSW-STWINKT01_V2.1.0/Drivers/CMSIS/Device/ST/STM32L4xx/Include
        STSW-STWINKT01_V2.1.0/Drivers/STM32L4xx_HAL_Driver/Inc
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/hts221
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/lps22hh
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/iis2mdc
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/ism330dhcx
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/Common
        STSW-STWINKT01_V2.1.0/Drivers/BSP/STWIN
        STSW-STWINKT01_V2.1.0/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
        STSW-STWINKT01_V2.1.0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FreeRTOS/Source/include
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FatFs/src
        STSW-STWINKT01_V2.1.0/Middlewares/Third_Party/FatFs/src/drivers
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/iis2dh
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/iis3dwb
        STSW-STWINKT01_V2.1.0/Drivers/BSP/Components/stts751
        STSW-STWINKT01_V2.1.0/Drivers/CMSIS/Include
        STSW-STWINKT01_V2.1.0/Drivers/CMSIS/DSP/Include
        )

target_link_directories(${EXECUTABLE} PRIVATE
        STWin-SimpleStream/knowledgepack/lib
        )

if (${SENSIML_RECOGNITION})
    target_compile_definitions(${EXECUTABLE} PRIVATE
            -DSENSIML_RECOGNITION=1
            )
    target_link_libraries(${EXECUTABLE} PRIVATE
            sensiml
            tensorflow-microlite
            m
            )
else()
    target_compile_definitions(${EXECUTABLE} PRIVATE
            -DSENSIML_RECOGNITION=0
            )
    target_link_libraries(${EXECUTABLE} PRIVATE
            m
            )
endif ()


target_link_options(${EXECUTABLE} PRIVATE
        -T${CMAKE_SOURCE_DIR}/STWin-SimpleStream/STM32L4R9ZITx_FLASH.ld
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -specs=nano.specs
        -lc
        -lm
        -Wl,-Map=${PROJECT_NAME}.map,--cref
        -Wl,--gc-sections
        -static
        )


# Print executable size
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND arm-none-eabi-size ${EXECUTABLE})

# Create hex file
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND arm-none-eabi-objcopy -O ihex ${EXECUTABLE} ${PROJECT_NAME}.hex
        COMMAND arm-none-eabi-objcopy -O binary ${EXECUTABLE} ${PROJECT_NAME}.bin)
