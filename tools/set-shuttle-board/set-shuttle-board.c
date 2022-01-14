/**
 * Copyright (C) 2022 Jabez Winston
 *
 * SPDX-License-Identifier: MIT
 *
 * @file    set-shuttle-board.c
 * @brief   Sets shuttle ID
 */

#include "set-shuttle-board.h"

const shuttle_board_info_t shuttle_board_table[] = {
        {"BMI085",  0x46},
        {"BMI088",  0x66},
        {"BMI090L", 0x86},
        {"BMI055",  0x06},
        {"BMI160",  0x38},
        {"BMI260",  0x1B8},
        {"BMI270",  0x1B8},
        {"BMA400",  0x1A1},
        {"BMA456",  0x141},
        {"BMA423",  0x141},
        {"BMA280",  0x17},
        {"BMA253",  0x1A},
        {"BMA490L", 0x9A},
        {"BME280",  0x33},
        {"BME680",  0x93},
        {"BMP280",  0x04},
        {"BMP380",  0xD3},
        {"BMP388",  0xD3},
        {"BMP390",  0x173},
        {"BMM150",  0x07},
        {"BHI160",  0x99},
        {"BHI260",  0x119},
        {"BNO055",  0x19}
};

int main(int argc, char *argv[])
{
    struct coines_board_info board_info;
    uint16_t shuttle_id;

    if(argc == 2)
    {
        if(strcmp(argv[1], "help") == 0)
        {
            printf("\n %s <Shuttle Board name / Shuttle Board ID>", argv[0]);
            printf("\n\n Eg: 1. %s BMI085", argv[0]);
            printf("\n     2. %s 0x46", argv[0]);
            printf("\n     2. %s bmi085", argv[0]);
            printf("\n\nUse \"%s list\" to list shuttle boards.\n", argv[0]);
            exit(EXIT_SUCCESS);
        }

        if(strcmp(argv[1], "list") == 0)
        {
            printf("\nHere are the Shuttle Boards and their respective IDs ...\n");
            printf("\n%-8s\t|\t%s\n","Name", "ID");
            printf("----------------------------------\n");
            for (int i=0 ; i < sizeof(shuttle_board_table)/sizeof(shuttle_board_info_t); i++)
            {
                printf("%-8s\t|\t0x%02X\n", shuttle_board_table[i].name, shuttle_board_table[i].id);
            }
            exit(EXIT_SUCCESS);
        }

        shuttle_id = strtol(argv[1], NULL, 0);
    }
    else
    {
        fprintf(stderr, "Invalid/Insufficient arguments !!\n\n");
        fprintf(stderr, "Use \"%s help\" to print usage.\n", argv[0]);
        fprintf(stderr, "Use \"%s list\" to list shuttle boards.\n", argv[0]);
        exit(-1);
    }

    if (shuttle_id == 0)
    {
        for (int i=0 ; i < sizeof(shuttle_board_table)/sizeof(shuttle_board_info_t); i++)
        {
            if(strcasecmp(argv[1], shuttle_board_table[i].name) == 0)
            {
                shuttle_id = shuttle_board_table[i].id;
                break;
            }
        }

        if(shuttle_id == 0)
        {
            fprintf(stderr, "\nUnable to find ID for %s shuttle board !!\n", argv[1]);
            exit(-2);
        }
    }

    int rslt = coines_open_comm_intf(COINES_COMM_INTF_USB); // Connect to Application board
    if ( rslt < COINES_SUCCESS )
    {
        printf("Unable to connect to Application Board !\n");
        return rslt;
    }

    rslt = coines_get_board_info(&board_info);

    board_info.shuttle_id = shuttle_id;

    if (rslt == COINES_SUCCESS)
        rslt  = coines_set_board_info(&board_info);

    if (rslt == COINES_SUCCESS)
        printf("Shuttle ID set as 0x%x", board_info.shuttle_id);
    else
        fprintf(stderr, "Error setting Shuttle ID !!\n");

    return 0;
}

