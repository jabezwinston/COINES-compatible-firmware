/**
 * Copyright (C) 2022 Jabez Winston
 *
 * SPDX-License-Identifier: MIT
 *
 * @file    coines_protocol_parser.h
 * @brief   Declarations of COINES protocol handlers
 *
 */

#ifndef COINES_PROTOCOL_PARSER_H_
#define COINES_PROTOCOL_PARSER_H_

#include "coines_defs.h"
#include "coines.h"
#include <stdbool.h>
#include <string.h>

int coines_protocol_parser();

typedef uint8_t (*cmd_handler_t)(uint8_t *data, uint8_t length);

typedef struct
{
    uint8_t cmd_id;
    cmd_handler_t cmd_handler;
} cmd_handler_map_t;

typedef void (*isr_cb_t)(void);

void isr_cb_1(void);
void isr_cb_2(void);

uint8_t coines_cmd_boardinfo_handler(uint8_t *cmd_buff, uint8_t length);

uint8_t coines_cmd_vdd_vddio_cfg_handler(uint8_t *cmd_buff, uint8_t length);

uint8_t coines_cmd_multiio_cfg_handler(uint8_t *cmd_buff, uint8_t length);

uint8_t coines_cmd_interface_cfg_handler(uint8_t *cmd_buff, uint8_t length);

uint8_t coines_cmd_i2c_speed_cfg_handler(uint8_t *cmd_buff, uint8_t length);

uint8_t coines_cmd_spi_settings_cfg_handler(uint8_t *cmd_buff, uint8_t length);

uint8_t coines_cmd_sensor_read_and_write_handler(uint8_t *cmd_buff, uint8_t length);

uint8_t coines_cmd_timer_cfg_handler(uint8_t *cmd_buff, uint8_t length);

/************************** COINES Stream command handlers *********************************/
uint8_t coines_cmd_gen_stream_settings_handler(uint8_t *cmd_buff, uint8_t length);
uint8_t coines_cmd_start_stop_poll_stream_handler(uint8_t *cmd_buff, uint8_t length);
uint8_t coines_cmd_poll_stream_settings_handler(uint8_t *cmd_buff, uint8_t length);
uint8_t coines_cmd_start_stop_int_stream_handler(uint8_t *cmd_buff, uint8_t length);
uint8_t coines_cmd_int_stream_settings_handler(uint8_t *cmd_buff, uint8_t length);

/*******************************!!! COINES Extended !!!*************************************/
int16_t coines_set_board_info(struct coines_board_info *);
uint64_t coines_get_micros();
uint16_t coines_read_intf(enum coines_comm_intf intf, void *buffer, uint16_t len);
void coines_write_intf(enum coines_comm_intf intf, void *buffer, uint16_t len);
/*******************************!!! COINES Extended !!!*************************************/

#endif /* INC_COINES_PROTOCOL_PARSER_H_ */
