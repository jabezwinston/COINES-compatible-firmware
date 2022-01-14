/**
 * Copyright (C) 2022 Jabez Winston
 *
 * SPDX-License-Identifier: MIT
 *
 * @file    main.c
 * @brief   COINES compatible firmware - main()
 */

#include "coines.h"
#include "coines_protocol_parser.h"

int main(void)
{
    coines_open_comm_intf(COINES_COMM_INTF_USB);

    while (true)
    {
        coines_protocol_parser();
    }

    return COINES_E_FAILURE;
}
