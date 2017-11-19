PROJECT_NAME     := ble_sensor_station
TARGETS          := nrf51822_xxaa
OUTPUT_DIRECTORY := _build

SDK_ROOT := ../nRF5_SDK_12.3.0_d7731ad
PROJ_DIR := .

$(OUTPUT_DIRECTORY)/nrf51822_xxaa.out: \
  LINKER_SCRIPT  := nrf51_ble_station_linker.ld

# Source files common to all targets
SRC_FILES += \
  $(PROJ_DIR)/src/main.c \
  $(PROJ_DIR)/src/libs/ble/ble_manager.c \
  $(PROJ_DIR)/src/libs/ble/services/apss/ble_apss.c \
  $(PROJ_DIR)/src/libs/sensing/bmp180_drv/bmp180_drv.c \
  $(PROJ_DIR)/src/libs/sensing/htu21d_drv/htu21d_drv.c \
  $(PROJ_DIR)/src/libs/sensing/tsl2561_drv/tsl2561_drv.c \
  $(PROJ_DIR)/src/libs/sensing/sensor_controller.c \
  $(SDK_ROOT)/components/drivers_nrf/uart/nrf_drv_uart.c \
  $(SDK_ROOT)/components/ble/ble_advertising/ble_advertising.c \
  $(SDK_ROOT)/components/ble/common/ble_advdata.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_params.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_state.c \
  $(SDK_ROOT)/components/ble/common/ble_srv_common.c \
  $(SDK_ROOT)/components/boards/boards.c \
  $(SDK_ROOT)/components/drivers_nrf/clock/nrf_drv_clock.c \
  $(SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
  $(SDK_ROOT)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
  $(SDK_ROOT)/components/drivers_nrf/twi_master/nrf_drv_twi.c \
  $(SDK_ROOT)/components/drivers_nrf/uart/nrf_drv_uart.c \
  $(SDK_ROOT)/components/libraries/crc16/crc16.c \
  $(SDK_ROOT)/components/libraries/hardfault/hardfault_implementation.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_serial.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c \
  $(SDK_ROOT)/components/libraries/scheduler/app_scheduler_serconn.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer_appsh.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/util/sdk_errors.c \
  $(SDK_ROOT)/components/libraries/util/sdk_mapped_flags.c \
  $(SDK_ROOT)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
  $(SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf51.S \
  $(SDK_ROOT)/components/toolchain/system_nrf51.c \


# Include folders common to all targets
INC_FOLDERS += \
  $(PROJ_DIR)/src/ \
  $(PROJ_DIR)/src/libs/sensing \
  $(PROJ_DIR)/src/libs/sensing/bmp180_drv/ \
  $(PROJ_DIR)/src/libs/sensing/htu21d_drv/ \
  $(PROJ_DIR)/src/libs/sensing/tsl2561_drv/ \
  $(PROJ_DIR)/src/libs/ble \
  $(PROJ_DIR)/src/libs/ble/services/apss/ \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/ble/ble_advertising \
  $(SDK_ROOT)/components/ble/ble_dtm \
  $(SDK_ROOT)/components/drivers_nrf/uart \
  $(SDK_ROOT)/components/ble/ble_racp \
  $(SDK_ROOT)/components/ble/common \
  $(SDK_ROOT)/components/ble/nrf_ble_qwr \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/device \
  $(SDK_ROOT)/components/drivers_nrf/clock \
  $(SDK_ROOT)/components/drivers_nrf/common \
  $(SDK_ROOT)/components/drivers_nrf/comp \
  $(SDK_ROOT)/components/drivers_nrf/delay \
  $(SDK_ROOT)/components/drivers_nrf/gpiote \
  $(SDK_ROOT)/components/drivers_nrf/hal \
  $(SDK_ROOT)/components/drivers_nrf/power \
  $(SDK_ROOT)/components/drivers_nrf/swi \
  $(SDK_ROOT)/components/drivers_nrf/timer \
  $(SDK_ROOT)/components/drivers_nrf/twi_master \
  $(SDK_ROOT)/components/libraries/crc16 \
  $(SDK_ROOT)/components/libraries/crc32 \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/libraries/fstorage \
  $(SDK_ROOT)/components/libraries/gpiote \
  $(SDK_ROOT)/components/libraries/hardfault \
  $(SDK_ROOT)/components/libraries/hci \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/libraries/log/src \
  $(SDK_ROOT)/components/libraries/low_power_pwm \
  $(SDK_ROOT)/components/libraries/mem_manager \
  $(SDK_ROOT)/components/libraries/pwm \
  $(SDK_ROOT)/components/libraries/queue \
  $(SDK_ROOT)/components/libraries/scheduler \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/components/libraries/twi \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components/softdevice/common/softdevice_handler \
  $(SDK_ROOT)/components/softdevice/s130/headers \
  $(SDK_ROOT)/components/softdevice/s130/headers/nrf51 \
  $(SDK_ROOT)/components/toolchain \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/components/toolchain/gcc \
  $(SDK_ROOT)/external/segger_rtt \


# Libraries common to all targets
LIB_FILES += \

# C flags common to all targets
CFLAGS += -DBOARD_CUSTOM
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF51
CFLAGS += -DS130
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DNRF51822
CFLAGS += -DNRF_SD_BLE_API_VERSION=2
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs

ifeq ($(DEBUG), 1)
CFLAGS += -DDEBUG
CFLAGS += -O0 -g3
else
CFLAGS +=  -Wall -Os -g3
CFLAGS += -mfloat-abi=soft
endif

# keep every function in separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums 

# C++ flags common to all targets
CXXFLAGS += \

# Assembler flags common to all targets
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DBOARD_CUSTOM
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -DS130
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DNRF51822
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=2

# Linker flags
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

.PHONY: $(TARGETS) default all clean help flash flash_softdevice

# Default target - first one defined
default: nrf51822_xxaa

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo 	nrf51822_xxaa

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc

include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

# Flash the program
flash: $(OUTPUT_DIRECTORY)/nrf51822_xxaa.hex
	@echo Flashing: $<
	nrfjprog --program $< -f nrf51 --sectorerase
	nrfjprog --reset -f nrf51

# Flash softdevice
flash_softdevice:
	@echo Flashing: s130_nrf51_2.0.1_softdevice.hex
	nrfjprog --program $(SDK_ROOT)/components/softdevice/s130/hex/s130_nrf51_2.0.1_softdevice.hex -f nrf51 --sectorerase 
	nrfjprog --reset -f nrf51

erase:
	nrfjprog --eraseall -f nrf52
