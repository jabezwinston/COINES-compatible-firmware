CROSS_COMPILE = arm-none-eabi-

AS = $(CROSS_COMPILE)as
CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++
AR = $(CROSS_COMPILE)ar
OBJCOPY = $(CROSS_COMPILE)objcopy

MCU_SDK_DIR = STM32CubeF4

SDK_ASM_SRCS += $(MCU_SDK_DIR)/startup_stm32f401ccux.S

SDK_C_SRCS += \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c \
$(MCU_SDK_DIR)/USB_DEVICE/Target/usbd_conf.c \
$(MCU_SDK_DIR)/USB_DEVICE/App/usb_device.c \
$(MCU_SDK_DIR)/USB_DEVICE/App/usbd_cdc_if.c \
$(MCU_SDK_DIR)/USB_DEVICE/App/usbd_desc.c \
$(MCU_SDK_DIR)/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
$(MCU_SDK_DIR)/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
$(MCU_SDK_DIR)/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
$(MCU_SDK_DIR)/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c

SDK_INCLUDE_PATHS += \
$(MCU_SDK_DIR)/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
$(MCU_SDK_DIR)/Drivers/CMSIS/Include \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Inc \
$(MCU_SDK_DIR)/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy \
$(MCU_SDK_DIR)/Middlewares/ST/STM32_USB_Device_Library/Core/Inc \
$(MCU_SDK_DIR)/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
$(MCU_SDK_DIR)/USB_DEVICE/App \
$(MCU_SDK_DIR)/USB_DEVICE/Target \

SDK_LD_SCRIPT = $(MCU_SDK_DIR)/STM32F401CCUX_FLASH.ld

CFLAGS += -std=gnu11 -c -Os -g3 -Wall -ffunction-sections -fdata-sections --specs=nano.specs
CFLAGS += -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mabi=aapcs 
CFLAGS += -DUSE_HAL_DRIVER -DSTM32F401xC

ASMFLAGS = $(CFLAGS)

LDFLAGS += -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard --specs=nano.specs --specs=nosys.specs 
LDFLAGS += -Wl,--gc-sections -static -Wl,--start-group -lc -lm -Wl,--end-group

