/**
 * Copyright (C) 2022 Jabez Winston
 *
 * SPDX-License-Identifier: MIT
 *
 * @file    set-shuttle-board.h
 * @brief   Declarations and definitions for set-shuttle-board.c
 */

#ifndef SET_SHUTTLE_BOARD_H_
#define SET_SHUTTLE_BOARD_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "coines.h"
#include "coines_ext.h"

typedef struct
{
	char *name;
	uint16_t id;
} shuttle_board_info_t;

#endif /* SET_SHUTTLE_BOARD_H_ */
