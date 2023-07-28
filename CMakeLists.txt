cmake_minimum_required(VERSION 3.22.0)

if(NOT ARM_NONE_EABI_PATH)
    message(FATAL_ERROR "ERROR - ARM_NONE_EABI_PATH not set: I need the path to the toolchain!")
endif()

if(NOT SDK_ROOT_DIR)
    message(FATAL_ERROR "ERROR - SDK_ROOT_DIR not set: I need the path to the nRF5 SDK!")
endif()

project(accel_read_poll C ASM)

message(STATUS "Toolchain ARM_NONE_EABI_PATH : ${ARM_NONE_EABI_PATH}")

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/toolchain.cmake)

set(SDK_COMPONENTS_DIR ${SDK_ROOT_DIR}/components)
set(SDK_COMPONENT_LIBRARIES_DIR ${SDK_COMPONENTS_DIR}/libraries)
set(SDK_COMPONENT_BLE_LIBRARIES_DIR ${SDK_COMPONENTS_DIR}/ble)
set(SDK_COMPONENT_BRD_LIBRARIES_DIR ${SDK_COMPONENTS_DIR}/boards)
set(SDK_COMPONENT_NFC_LIBRARIES_DIR ${SDK_COMPONENTS_DIR}/nfc)
set(SDK_COMPONENT_SOFTDEVICE_DIR ${SDK_COMPONENTS_DIR}/softdevice)
set(SDK_COMPONENT_TOOLCHAIN_DIR ${SDK_COMPONENTS_DIR}/toolchain)

set(SDK_MODULES_DIR ${SDK_ROOT_DIR}/modules)
set(SDK_NRFX_MODULE_DIR ${SDK_MODULES_DIR}/nrfx)
set(SDK_NRFX_HAL_MODULE_DIR ${SDK_NRFX_MODULE_DIR}/hal)
set(SDK_NRFX_MDK_MODULE_DIR ${SDK_NRFX_MODULE_DIR}/mdk)
set(SDK_NRFX_SOC_MODULE_DIR ${SDK_NRFX_MODULE_DIR}/soc)
set(SDK_NRFX_DRIVERS_MODULE_DIR ${SDK_NRFX_MODULE_DIR}/drivers)
set(SDK_NRFX_DRIVERS_MODULE_DIR_INCLUDE_DIR ${SDK_NRFX_DRIVERS_MODULE_DIR}/include)
set(SDK_NRFX_DRIVERS_MODULE_DIR_SOURCE_DIR ${SDK_NRFX_DRIVERS_MODULE_DIR}/src)

set(SDK_EXTERNAL_DIR ${SDK_ROOT_DIR}/external)

set(SDK_INTEGRATION_DIR ${SDK_ROOT_DIR}/integration)
set(SDK_INTEGRATION_NRFX_DIR ${SDK_INTEGRATION_DIR}/nrfx)
set(SDK_INTEGRATION_NRFX_LEGACY_DIR ${SDK_INTEGRATION_NRFX_DIR}/legacy)

set(PROJECT_CONFIG_DIR ${CMAKE_SOURCE_DIR}/config)

set(SDK_DIR ${CMAKE_SOURCE_DIR}/SDK)

set(ignoreMe "${BOOTLOADER}")

add_compile_definitions(
	APP_TIMER_V2
    APP_TIMER_V2_RTC1_ENABLED
    BL_SETTINGS_ACCESS_ONLY
    BOARD_PCA10056
    BOOTLOADER
    CONFIG_GPIO_AS_PINRESET
    FLOAT_ABI_HARD
    NRF52840_XXAA
    NRF_DFU_SVCI_ENABLED
    NRF_DFU_TRANSPORT_BLE=1
    NRF_SD_BLE_API_VERSION=7
    S140
    SOFTDEVICE_PRESENT
)

add_executable(accel_read_poll ${CMAKE_SOURCE_DIR}/main.c)

set_target_properties(accel_read_poll PROPERTIES C_STANDARD 11)
set_target_properties(accel_read_poll PROPERTIES LINK_DEPENDS ${CMAKE_SOURCE_DIR}/${LINKER_SCRIPT})

target_sources(
    accel_read_poll PUBLIC 
    ${SDK_NRFX_MDK_MODULE_DIR}/gcc_startup_nrf52840.S
    ${CMAKE_SOURCE_DIR}/src/sensor.c
)

target_link_libraries(
	accel_read_poll
	PRIVATE
    m
    c
)

target_include_directories(
    accel_read_poll PUBLIC
    ${CMAKE_SOURCE_DIR}/inc
    ${SDK_COMPONENTS_DIR}
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_advertising
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_db_discovery
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_dtm
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_racp
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_ancs_c
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_ans_c
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_bas
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_bas_c
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_cscs
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_cts_c
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_dfu
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_dis
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_gls
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_hids
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_hrs
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_hrs_c
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_hts
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_ias
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_ias_c
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_lbs
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_lbs_c
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_lls
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_nus
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_nus_c
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_rscs
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_rscs_c
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/ble_services/ble_tps
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/common
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/nrf_ble_gatt
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/nrf_ble_gq
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/nrf_ble_qwr
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/nrf_ble_scan
    ${SDK_COMPONENT_BLE_LIBRARIES_DIR}/peer_manager
    ${SDK_COMPONENT_LIBRARIES_DIR}/atomic
    ${SDK_COMPONENT_LIBRARIES_DIR}/atomic_fifo
    ${SDK_COMPONENT_LIBRARIES_DIR}/atomic_flags
    ${SDK_COMPONENT_LIBRARIES_DIR}/balloc
    ${SDK_COMPONENT_LIBRARIES_DIR}/bootloader
    ${SDK_COMPONENT_LIBRARIES_DIR}/bootloader/dfu
    ${SDK_COMPONENT_LIBRARIES_DIR}/bootloader/ble_dfu
    ${SDK_COMPONENT_LIBRARIES_DIR}/bsp
    ${SDK_COMPONENT_LIBRARIES_DIR}/button
    ${SDK_COMPONENT_LIBRARIES_DIR}/cli
    ${SDK_COMPONENT_LIBRARIES_DIR}/crc16
    ${SDK_COMPONENT_LIBRARIES_DIR}/crc32
    ${SDK_COMPONENT_LIBRARIES_DIR}/crypto
    ${SDK_COMPONENT_LIBRARIES_DIR}/csense
    ${SDK_COMPONENT_LIBRARIES_DIR}/csense_drv
    ${SDK_COMPONENT_LIBRARIES_DIR}/delay
    ${SDK_COMPONENT_LIBRARIES_DIR}/ecc
    ${SDK_COMPONENT_LIBRARIES_DIR}/experimental_section_vars
    ${SDK_COMPONENT_LIBRARIES_DIR}/experimental_task_manager
    ${SDK_COMPONENT_LIBRARIES_DIR}/fds
    ${SDK_COMPONENT_LIBRARIES_DIR}/fstorage
    ${SDK_COMPONENT_LIBRARIES_DIR}/gfx
    ${SDK_COMPONENT_LIBRARIES_DIR}/gpiote
    ${SDK_COMPONENT_LIBRARIES_DIR}/hardfault
    ${SDK_COMPONENT_LIBRARIES_DIR}/hci
    ${SDK_COMPONENT_LIBRARIES_DIR}/led_softblink
    ${SDK_COMPONENT_LIBRARIES_DIR}/log
    ${SDK_COMPONENT_LIBRARIES_DIR}/log/src
    ${SDK_COMPONENT_LIBRARIES_DIR}/low_power_pwm
    ${SDK_COMPONENT_LIBRARIES_DIR}/mem_manager
    ${SDK_COMPONENT_LIBRARIES_DIR}/memobj
    ${SDK_COMPONENT_LIBRARIES_DIR}/mpu
    ${SDK_COMPONENT_LIBRARIES_DIR}/mutex
    ${SDK_COMPONENT_LIBRARIES_DIR}/pwm
    ${SDK_COMPONENT_LIBRARIES_DIR}/pwr_mgmt
    ${SDK_COMPONENT_LIBRARIES_DIR}/queue
    ${SDK_COMPONENT_LIBRARIES_DIR}/ringbuf
    ${SDK_COMPONENT_LIBRARIES_DIR}/scheduler
    ${SDK_COMPONENT_LIBRARIES_DIR}/sdcard
    ${SDK_COMPONENT_LIBRARIES_DIR}/sensorsim
    ${SDK_COMPONENT_LIBRARIES_DIR}/sha256
    ${SDK_COMPONENT_LIBRARIES_DIR}/slip
    ${SDK_COMPONENT_LIBRARIES_DIR}/sortlist
    ${SDK_COMPONENT_LIBRARIES_DIR}/spi_mngr
    ${SDK_COMPONENT_LIBRARIES_DIR}/stack_guard
    ${SDK_COMPONENT_LIBRARIES_DIR}/strerror
    ${SDK_COMPONENT_LIBRARIES_DIR}/svc
    ${SDK_COMPONENT_LIBRARIES_DIR}/timer
    ${SDK_COMPONENT_LIBRARIES_DIR}/twi_mngr
    ${SDK_COMPONENT_LIBRARIES_DIR}/twi_sensor
    ${SDK_COMPONENT_LIBRARIES_DIR}/usbd
    ${SDK_COMPONENT_LIBRARIES_DIR}/usbd/class/audio
    ${SDK_COMPONENT_LIBRARIES_DIR}/usbd/class/cdc
    ${SDK_COMPONENT_LIBRARIES_DIR}/usbd/class/cdc/acm
    ${SDK_COMPONENT_LIBRARIES_DIR}/usbd/class/hid
    ${SDK_COMPONENT_LIBRARIES_DIR}/usbd/class/hid/generic
    ${SDK_COMPONENT_LIBRARIES_DIR}/usbd/class/hid/kbd
    ${SDK_COMPONENT_LIBRARIES_DIR}/usbd/class/hid/mouse
    ${SDK_COMPONENT_LIBRARIES_DIR}/usbd/class/msc
    ${SDK_COMPONENT_LIBRARIES_DIR}/util
    ${SDK_COMPONENT_SOFTDEVICE_DIR}/common
    ${SDK_COMPONENT_SOFTDEVICE_DIR}/s140/headers
    ${SDK_COMPONENT_SOFTDEVICE_DIR}/s140/headers/nrf52
    ${SDK_COMPONENT_TOOLCHAIN_DIR}/cmsis/include
    ${SDK_EXTERNAL_DIR}/fprintf
    ${SDK_EXTERNAL_DIR}/segger_rtt
    ${SDK_EXTERNAL_DIR}/utf_converter
    ${SDK_INTEGRATION_NRFX_DIR}
    ${SDK_INTEGRATION_NRFX_LEGACY_DIR}
    ${SDK_NRFX_MODULE_DIR}
    ${SDK_NRFX_HAL_MODULE_DIR}
    ${SDK_NRFX_MDK_MODULE_DIR}
    ${SDK_NRFX_DRIVERS_MODULE_DIR_INCLUDE_DIR}
    ${PROJECT_CONFIG_DIR}
)

add_subdirectory(${SDK_DIR})

add_custom_command(
    TARGET accel_read_poll
    POST_BUILD
    COMMAND mv accel_read_poll accel_read_poll.elf
    COMMAND arm-none-eabi-objcopy -O ihex accel_read_poll.elf accel_read_poll.hex
    COMMAND arm-none-eabi-objcopy -O binary accel_read_poll.elf accel_read_poll.bin
    COMMAND arm-none-eabi-objdump -D accel_read_poll.elf > accel_read_poll.s
)

#add_custom_command(
#    TARGET accel_read_poll
#    POST_BUILD
#    COMMAND nrfutil pkg generate --application ./accel_read_poll.hex --application-version-string "1.0.0" --hw-version 2 --sd-req 0x0100 --sd-id 0x0100 --key-file ${KEY_DIR}/bootloader_private.key ${DFU_FILENAME}.zip
#)
