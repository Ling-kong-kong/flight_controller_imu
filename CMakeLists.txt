cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/cmake/arm-none-eabi.cmake"
      CACHE FILEPATH "ARM bare-metal GCC toolchain")
endif()
project(bmi088_raw_imu VERSION 3.0.0 LANGUAGES C ASM)

option(ENABLE_EXPERIMENTAL_MAG_FUSION "Enable experimental BMM150 relative yaw aiding" ON)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(FLASH_OFFSET_KB "0" CACHE STRING "Application offset in KB: 0 (standalone) or 128 (existing bootloader)")
if(NOT FLASH_OFFSET_KB MATCHES "^(0|128)$")
  message(FATAL_ERROR "FLASH_OFFSET_KB must be 0 or 128")
endif()
math(EXPR FLASH_ORIGIN "0x08000000 + ${FLASH_OFFSET_KB} * 1024" OUTPUT_FORMAT HEXADECIMAL)
math(EXPR FLASH_LENGTH_KB "2048 - ${FLASH_OFFSET_KB}")
configure_file(linker/stm32h743.ld.in "${CMAKE_CURRENT_BINARY_DIR}/stm32h743.ld" @ONLY)

find_package(Python3 COMPONENTS Interpreter REQUIRED)
foreach(dep stm32h7xx_hal_driver cmsis_device_h7 cmsis_5 tinyusb)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/${dep}/.source-sha256")
    message(FATAL_ERROR "Missing ${dep}. Run: python3 scripts/bootstrap_deps.py")
  endif()
endforeach()

add_library(platform_config INTERFACE)
target_compile_options(platform_config INTERFACE
  -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
  -ffunction-sections -fdata-sections -fno-common -g3
  "$<$<COMPILE_LANGUAGE:C>:-fno-strict-aliasing>"
  "$<$<CONFIG:Release>:-O2>"
  "$<$<CONFIG:Debug>:-Og>")
target_compile_definitions(platform_config INTERFACE
  STM32H743xx USE_HAL_DRIVER USE_PWR_LDO_SUPPLY
  HSE_VALUE=16000000U FLASH_OFFSET_KB=${FLASH_OFFSET_KB})
target_include_directories(platform_config INTERFACE firmware/include)
target_include_directories(platform_config SYSTEM INTERFACE
  third_party/stm32h7xx_hal_driver/Inc
  third_party/cmsis_device_h7/Include
  third_party/cmsis_5/CMSIS/Core/Include
  third_party/tinyusb/src)

set(HAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/stm32h7xx_hal_driver/Src")
add_library(stm32_hal STATIC
  "${HAL_DIR}/stm32h7xx_hal.c"
  "${HAL_DIR}/stm32h7xx_hal_cortex.c"
  "${HAL_DIR}/stm32h7xx_hal_dma.c"
  "${HAL_DIR}/stm32h7xx_hal_dma_ex.c"
  "${HAL_DIR}/stm32h7xx_hal_gpio.c"
  "${HAL_DIR}/stm32h7xx_hal_rcc.c"
  "${HAL_DIR}/stm32h7xx_hal_rcc_ex.c"
  "${HAL_DIR}/stm32h7xx_hal_pwr.c"
  "${HAL_DIR}/stm32h7xx_hal_pwr_ex.c"
  "${HAL_DIR}/stm32h7xx_hal_flash.c"
  "${HAL_DIR}/stm32h7xx_hal_flash_ex.c"
  "${HAL_DIR}/stm32h7xx_hal_spi.c"
  "${HAL_DIR}/stm32h7xx_hal_spi_ex.c"
  "${HAL_DIR}/stm32h7xx_hal_exti.c"
  third_party/cmsis_device_h7/Source/Templates/system_stm32h7xx.c)
target_link_libraries(stm32_hal PUBLIC platform_config)

set(TUSB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinyusb/src")
add_library(tinyusb STATIC
  "${TUSB_DIR}/tusb.c"
  "${TUSB_DIR}/common/tusb_fifo.c"
  "${TUSB_DIR}/device/usbd.c"
  "${TUSB_DIR}/device/usbd_control.c"
  "${TUSB_DIR}/class/cdc/cdc_device.c"
  "${TUSB_DIR}/portable/synopsys/dwc2/dcd_dwc2.c"
  "${TUSB_DIR}/portable/synopsys/dwc2/dwc2_common.c")
target_link_libraries(tinyusb PUBLIC platform_config)

add_library(bmm150 STATIC third_party/bmm150/bmm150.c)
target_link_libraries(bmm150 PUBLIC platform_config)
target_include_directories(bmm150 PUBLIC third_party/bmm150)
target_compile_definitions(bmm150 PUBLIC BMM150_USE_FLOATING_POINT)

file(GLOB APP_SOURCES CONFIGURE_DEPENDS firmware/src/*.c)
add_executable(imu_firmware ${APP_SOURCES} cmake/runtime.c
  third_party/cmsis_device_h7/Source/Templates/gcc/startup_stm32h743xx.s)
set_target_properties(imu_firmware PROPERTIES SUFFIX ".elf")
target_compile_definitions(imu_firmware PRIVATE
  FUSION_MAG_ENABLE=$<BOOL:${ENABLE_EXPERIMENTAL_MAG_FUSION}>)

target_compile_options(imu_firmware PRIVATE
  "$<$<COMPILE_LANGUAGE:C>:-Wall;-Wextra;-Werror;-Wshadow;-Wformat=2;-Wundef>")
target_link_libraries(imu_firmware PRIVATE stm32_hal tinyusb bmm150 m)
target_link_options(imu_firmware PRIVATE
  -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
  -nostartfiles --specs=nano.specs --specs=nosys.specs
  "-T${CMAKE_CURRENT_BINARY_DIR}/stm32h743.ld"
  "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/imu_firmware.map,--cref"
  -Wl,--gc-sections -Wl,--print-memory-usage)
set_property(TARGET imu_firmware APPEND PROPERTY LINK_DEPENDS
  "${CMAKE_CURRENT_BINARY_DIR}/stm32h743.ld")

add_custom_command(TARGET imu_firmware POST_BUILD
  COMMAND "${CMAKE_OBJCOPY}" -O binary "$<TARGET_FILE:imu_firmware>"
          "${CMAKE_CURRENT_BINARY_DIR}/imu_firmware.bin"
  COMMAND "${CMAKE_OBJCOPY}" -O ihex "$<TARGET_FILE:imu_firmware>"
          "${CMAKE_CURRENT_BINARY_DIR}/imu_firmware.hex"
  COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:imu_firmware>"
  VERBATIM)
if(FLASH_OFFSET_KB STREQUAL "128")
  add_custom_command(TARGET imu_firmware POST_BUILD
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/make_apj.py"
      "${CMAKE_CURRENT_BINARY_DIR}/imu_firmware.bin"
      "${CMAKE_CURRENT_BINARY_DIR}/imu_firmware.apj"
    VERBATIM)
endif()
message(STATUS "Firmware starts at ${FLASH_ORIGIN}; standalone MCU is FLASH_OFFSET_KB=0")
