BRD_C_SRCS += \
mcu_pico/mcu_pico.c \
mcu_pico/support/usb_support.c \


BRD_INCLUDE_PATHS += \
mcu_pico \
mcu_pico/support \
mcu_pico/support/generated \

BRD_ASM_SRCS += \
mcu_pico/support/generated/bs2_default_padded_checksummed.S \
