/**
 * Copyright (C) 2022 Jabez Winston
 *
 * SPDX-License-Identifier: MIT
 *
 * @file    mcu_pico.h
 * @brief   COINES API implementation for Raspberry Pi Pico/RP2040 board
 */
#ifndef MCU_PICO_H_
#define MCU_PICO_H_

#include "coines.h"
#include "coines_defs.h"
#include <stdlib.h>
#include <string.h>
#include "usb_support.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "pico/binary_info.h"
#include "coines_fw_cfg.h"

extern bool configured;

uint64_t coines_get_micros();

int16_t coines_set_board_info(struct coines_board_info *data);

typedef void (*isr_cb_t)(void);

void gpio_callback(uint pico_pin, uint32_t events);

#define TIMEOUT_US (100000)

#endif /* MCU_BLUEPILL_H_ */
