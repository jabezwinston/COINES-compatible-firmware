/**
 * Copyright (C) 2022 Jabez Winston
 *
 * SPDX-License-Identifier: MIT
 *
 * @file    coines_ext.c
 * @brief   COINES extended
 */

#include "coines_ext.h"

int16_t coines_set_board_info(struct coines_board_info *data)
{
	int rslt;
	comm_intf_init_command_header(COINES_DD_SET, COINES_CMDID_BOARDINFORMATION);
	comm_intf_put_u16(data->shuttle_id);
	comm_intf_put_u16(data->hardware_id);
	comm_intf_put_u16(data->software_id);
	comm_intf_put_u8(data->board);
	rslt = comm_intf_send_command(NULL);
	return rslt;
}
