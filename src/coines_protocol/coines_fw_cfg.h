/*
 * Copyright (C) 2022 Jabez Winston
 *
 * SPDX-License-Identifier: MIT
 *
 * @file    coines_fw_cfg.h
 * @brief   COINES Firmware configuration
 */

#ifndef COINES_FW_CFG_H
#define COINES_FW_CFG_H

/* Use APP3.0 Board ID */
#define BOARD_ID_APP30 0x05

/* Hardware version - v1.0 */
#define HW_VER 0x10

/* Software version - v1.0 */
#define SW_VER 0x10

/* BMI160 Shuttle ID - 0x38 */
/* Default Shuttle ID can be changed during runtime with 'set-shuttle-board' utility */
#define DEFAULT_SHUTTLE_ID 0x38

#endif
