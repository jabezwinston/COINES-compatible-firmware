CROSS_COMPILE = arm-none-eabi-

AS = $(CROSS_COMPILE)as
CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++
AR = $(CROSS_COMPILE)ar
OBJCOPY = $(CROSS_COMPILE)objcopy

MCU_SDK_DIR = pico-sdk

SDK_ASM_SRCS += \
$(MCU_SDK_DIR)/src/rp2_common/hardware_divider/divider.S \
$(MCU_SDK_DIR)/src/rp2_common/hardware_irq/irq_handler_chain.S \
$(MCU_SDK_DIR)/src/rp2_common/pico_bit_ops/bit_ops_aeabi.S \
$(MCU_SDK_DIR)/src/rp2_common/pico_double/double_v1_rom_shim.S \
$(MCU_SDK_DIR)/src/rp2_common/pico_float/float_aeabi.S \
$(MCU_SDK_DIR)/src/rp2_common/pico_float/float_v1_rom_shim.S \
$(MCU_SDK_DIR)/src/rp2_common/pico_mem_ops/mem_ops_aeabi.S \
$(MCU_SDK_DIR)/src/rp2_common/pico_standard_link/crt0.S \


SDK_C_SRCS += \
$(MCU_SDK_DIR)/src/rp2_common/hardware_claim/claim.c \
$(MCU_SDK_DIR)/src/rp2_common/hardware_clocks/clocks.c \
$(MCU_SDK_DIR)/src/rp2_common/hardware_gpio/gpio.c \
$(MCU_SDK_DIR)/src/rp2_common/hardware_i2c/i2c.c \
$(MCU_SDK_DIR)/src/rp2_common/hardware_irq/irq.c \
$(MCU_SDK_DIR)/src/rp2_common/hardware_pll/pll.c \
$(MCU_SDK_DIR)/src/rp2_common/hardware_spi/spi.c \
$(MCU_SDK_DIR)/src/rp2_common/hardware_sync/sync.c \
$(MCU_SDK_DIR)/src/rp2_common/hardware_timer/timer.c \
$(MCU_SDK_DIR)/src/rp2_common/hardware_uart/uart.c \
$(MCU_SDK_DIR)/src/rp2_common/hardware_vreg/vreg.c \
$(MCU_SDK_DIR)/src/rp2_common/hardware_watchdog/watchdog.c \
$(MCU_SDK_DIR)/src/rp2_common/hardware_xosc/xosc.c \
$(MCU_SDK_DIR)/src/rp2_common/hardware_flash/flash.c \
$(MCU_SDK_DIR)/src/rp2_common/pico_platform/platform.c \
$(MCU_SDK_DIR)/src/rp2_common/pico_stdlib/stdlib.c \
$(MCU_SDK_DIR)/src/common/pico_time/time.c \
$(MCU_SDK_DIR)/src/common/pico_time/timeout_helper.c \
$(MCU_SDK_DIR)/src/common/pico_sync/sem.c \
$(MCU_SDK_DIR)/src/common/pico_sync/lock_core.c \
$(MCU_SDK_DIR)/src/common/pico_sync/mutex.c \
$(MCU_SDK_DIR)/src/common/pico_sync/critical_section.c \
$(MCU_SDK_DIR)/src/common/pico_util/datetime.c \
$(MCU_SDK_DIR)/src/common/pico_util/pheap.c \
$(MCU_SDK_DIR)/src/common/pico_util/queue.c \
$(MCU_SDK_DIR)/src/rp2_common/pico_unique_id/unique_id.c \
$(MCU_SDK_DIR)/src/rp2_common/pico_runtime/runtime.c \
$(MCU_SDK_DIR)/src/rp2_common/pico_printf/printf.c \
$(MCU_SDK_DIR)/src/rp2_common/pico_bootrom/bootrom.c \
$(MCU_SDK_DIR)/src/rp2_common/pico_double/double_init_rom.c \
$(MCU_SDK_DIR)/src/rp2_common/pico_double/double_math.c \
$(MCU_SDK_DIR)/src/rp2_common/pico_float/float_init_rom.c \
$(MCU_SDK_DIR)/src/rp2_common/pico_float/float_math.c \
$(MCU_SDK_DIR)/src/rp2_common/pico_malloc/pico_malloc.c \
$(MCU_SDK_DIR)/src/rp2_common/pico_standard_link/binary_info.c \
$(MCU_SDK_DIR)/src/rp2_common/pico_stdio/stdio.c \
$(MCU_SDK_DIR)/src/rp2_common/pico_stdio_uart/stdio_uart.c \


SDK_INCLUDE_PATHS += \
$(MCU_SDK_DIR)/src/common/pico_stdlib/include \
$(MCU_SDK_DIR)/src/rp2040/hardware_regs/include \
$(MCU_SDK_DIR)/src/rp2040/hardware_structs/include \
$(MCU_SDK_DIR)/src/rp2_common/pico_platform/include \
$(MCU_SDK_DIR)/src/rp2_common/hardware_base/include \
$(MCU_SDK_DIR)/src/rp2_common/hardware_claim/include \
$(MCU_SDK_DIR)/src/rp2_common/hardware_clocks/include \
$(MCU_SDK_DIR)/src/rp2_common/hardware_divider/include \
$(MCU_SDK_DIR)/src/rp2_common/hardware_gpio/include \
$(MCU_SDK_DIR)/src/rp2_common/hardware_i2c/include \
$(MCU_SDK_DIR)/src/rp2_common/hardware_irq/include \
$(MCU_SDK_DIR)/src/rp2_common/hardware_pll/include \
$(MCU_SDK_DIR)/src/rp2_common/hardware_resets/include \
$(MCU_SDK_DIR)/src/rp2_common/hardware_spi/include \
$(MCU_SDK_DIR)/src/rp2_common/hardware_sync/include \
$(MCU_SDK_DIR)/src/rp2_common/hardware_uart/include \
$(MCU_SDK_DIR)/src/rp2_common/hardware_vreg/include \
$(MCU_SDK_DIR)/src/rp2_common/hardware_watchdog/include \
$(MCU_SDK_DIR)/src/rp2_common/hardware_xosc/include \
$(MCU_SDK_DIR)/src/rp2_common/hardware_flash/include \
$(MCU_SDK_DIR)/src/rp2_common/cmsis/include \
$(MCU_SDK_DIR)/src/rp2_common/hardware_base/include \
$(MCU_SDK_DIR)/src/rp2_common/hardware_timer/include \
$(MCU_SDK_DIR)/src/common/pico_base/include \
$(MCU_SDK_DIR)/src/boards/include \
$(MCU_SDK_DIR)/src/common/pico_time/include \
$(MCU_SDK_DIR)/src/common/pico_sync/include \
$(MCU_SDK_DIR)/src/common/pico_util/include \
$(MCU_SDK_DIR)/src/common/pico_bit_ops/include \
$(MCU_SDK_DIR)/src/common/pico_divider/include \
$(MCU_SDK_DIR)/src/common/pico_binary_info/include \
$(MCU_SDK_DIR)/src/rp2_common/pico_unique_id/include \
$(MCU_SDK_DIR)/src/rp2_common/pico_runtime/include \
$(MCU_SDK_DIR)/src/rp2_common/pico_printf/include \
$(MCU_SDK_DIR)/src/rp2_common/pico_bootrom/include \
$(MCU_SDK_DIR)/src/rp2_common/pico_double/include \
$(MCU_SDK_DIR)/src/rp2_common/pico_int64_ops/include \
$(MCU_SDK_DIR)/src/rp2_common/pico_float/include \
$(MCU_SDK_DIR)/src/rp2_common/pico_malloc/include \
$(MCU_SDK_DIR)/src/rp2_common/boot_stage2/include \
$(MCU_SDK_DIR)/src/rp2_common/pico_stdio/include \
$(MCU_SDK_DIR)/src/rp2_common/pico_stdio_uart/include \


CFLAGS += -c -mcpu=cortex-m0plus -mthumb -ffunction-sections -fdata-sections -g3 -Og
CFLAGS += -Wall -Wno-format -Wno-unused-function -Wno-maybe-uninitialized -D NDEBUG 

CFLAGS += -D LIB_PICO_BIT_OPS=1 -D LIB_PICO_BIT_OPS_PICO=1
CFLAGS += -D LIB_PICO_DIVIDER=1 -D LIB_PICO_DIVIDER_HARDWARE=1
CFLAGS += -D LIB_PICO_DOUBLE=1 -D LIB_PICO_DOUBLE_PICO=1 
CFLAGS += -D LIB_PICO_FLOAT=1 -D LIB_PICO_FLOAT_PICO=1
CFLAGS += -D LIB_PICO_INT64_OPS=1 -D LIB_PICO_INT64_OPS_PICO=1 
CFLAGS += -D LIB_PICO_MALLOC=1 -D LIB_PICO_MEM_OPS=1 -D LIB_PICO_MEM_OPS_PICO=1 
CFLAGS += -D LIB_PICO_PLATFORM=1 
CFLAGS += -D LIB_PICO_PRINTF=1 -D LIB_PICO_PRINTF_PICO=1 
CFLAGS += -D LIB_PICO_RUNTIME=1 -D PICO_CXX_ENABLE_EXCEPTIONS=0 -D LIB_PICO_STANDARD_LINK=1 
CFLAGS += -D LIB_PICO_STDIO=1 -D LIB_PICO_STDIO_UART=1 -D LIB_PICO_STDLIB=1
CFLAGS += -D LIB_PICO_SYNC=1 -D LIB_PICO_SYNC_CORE=1 -D LIB_PICO_SYNC_CRITICAL_SECTION=1 -D LIB_PICO_SYNC_MUTEX=1 -D LIB_PICO_SYNC_SEM=1 
CFLAGS += -D LIB_PICO_TIME=1 -D LIB_PICO_UTIL=1 -D PICO_BOARD=\"pico\" -D PICO_BUILD=1 -D PICO_CMAKE_BUILD_TYPE=\"Release\" 
CFLAGS += -D PICO_COPY_TO_RAM=0 -D PICO_NO_FLASH=0 -D PICO_NO_HARDWARE=0 -D PICO_ON_DEVICE=1 
CFLAGS += -D PICO_PROGRAM_URL=\"https://github.com/jabezwinston/COINES-compatible-firmware\"

ASMFLAGS += $(CFLAGS)

SDK_LD_SCRIPT = $(MCU_SDK_DIR)/src/rp2_common/pico_standard_link/memmap_default.ld

LDFLAGS += -mcpu=cortex-m0plus -mthumb 
LDFLAGS += -Wl,--build-id=none --specs=nosys.specs -Wl,-z,max-page-size=4096 -Wl,--gc-sections
