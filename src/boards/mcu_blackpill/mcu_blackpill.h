/**
 * Copyright (C) 2021 Jabez Winston
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    mcu_blackpill.h
 * @brief   COINES API declarations for STM32F4x1CCU6 based Blackpill board
 */

#ifndef MCU_BLUEPILL_H_
#define MCU_BLUEPILL_H_

#include "coines.h"
#include "coines_defs.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdlib.h>
#include <string.h>
#include "coines_fw_cfg.h"

#define I2C_TIMEOUT (2000)

uint64_t coines_get_micros();

int16_t coines_set_board_info(struct coines_board_info *data);

typedef void (*isr_cb_t)(void);

void Error_Handler(void);

extern uint8_t UserRxBufferFS[];
extern uint8_t UserTxBufferFS[];

#endif /* MCU_BLUEPILL_H_ */
