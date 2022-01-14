/**
 * Copyright (C) 2022 Jabez Winston
 *
 * SPDX-License-Identifier: MIT
 *
 * @file    coines_protocol_parser.c
 * @brief   Parses COINES protocol commands and calls COINES API
 *
 */

#include "coines_protocol_parser.h"

uint8_t cmd_buff[64];

/*! global communication buffer*/
coines_rsp_buffer_t comm_buf;

static struct coines_streaming_config stream_config[2];
static struct coines_streaming_blocks data_blocks[2];
static uint8_t sensor_id, stream_mode, sensor_count;
static bool stream_in_progress = false;
uint64_t sampling_time_micros;
uint32_t millis_time[2];
uint64_t micros_time[2];
bool data_read_int[2];
static isr_cb_t sensor_int_cb[2] = {isr_cb_1, isr_cb_2};

cmd_handler_map_t normal_cmd_handler[] = {

    {COINES_CMDID_BOARDINFORMATION, coines_cmd_boardinfo_handler},
    {COINES_CMDID_SHUTTLEBOARD_VDD_VDDIO_CONFIGURATION, coines_cmd_vdd_vddio_cfg_handler},
    {COINES_CMDID_MULTIO_CONFIGURATION, coines_cmd_multiio_cfg_handler},
    {COINES_CMDID_INTERFACE, coines_cmd_interface_cfg_handler},
    {COINES_CMDID_I2CSPEED, coines_cmd_i2c_speed_cfg_handler},
    {COINES_CMDID_SPISETTINGS, coines_cmd_spi_settings_cfg_handler},
    {COINES_CMDID_SENSORWRITEANDREAD, coines_cmd_sensor_read_and_write_handler},
    {COINES_CMDID_TIMER_CFG_CMD_ID, coines_cmd_timer_cfg_handler},
};

cmd_handler_map_t stream_cmd_handler[] = {

    {COINES_DD_GENERAL_STREAMING_SETTINGS, coines_cmd_gen_stream_settings_handler},
    {COINES_CMDIDEXT_STARTSTOP_STREAM_POLLING, coines_cmd_start_stop_poll_stream_handler},
    {COINES_CMDIDEXT_STREAM_POLLING, coines_cmd_poll_stream_settings_handler},
    {COINES_CMDIDEXT_STARTSTOP_STREAM_INT, coines_cmd_start_stop_int_stream_handler},
    {COINES_CMDIDEXT_STREAM_INT, coines_cmd_int_stream_settings_handler},
};

/*!
 * @brief This API is used to Initialize the response header
 */
void comm_intf_init_response_header(uint8_t rsp, uint8_t int_feature, uint8_t ret_code)
{
    comm_buf.buffer[0] = COINES_CMD_ID;
    comm_buf.buffer[1] = 0;
    comm_buf.buffer[2] = 1;
    comm_buf.buffer[3] = ret_code;
    comm_buf.buffer[4] = rsp;
    comm_buf.buffer[5] = int_feature;
    comm_buf.buffer_size = 6;
}

/*!
 * @brief This API is used to write the uint8_t data into response buffer
 */
void comm_intf_put_u8(uint8_t data)
{
    comm_buf.buffer[comm_buf.buffer_size++] = data;
}

/*!
 * @brief This API is used to write the uint8_t data into response buffer
 */
void comm_intf_put_u8_arr(uint8_t *data, uint32_t len)
{
    for (int i = 0; i < len; i++)
        comm_buf.buffer[comm_buf.buffer_size++] = data[i];
}

/*!
 * @brief This API is used to write the uint16_t data into response buffer
 */
void comm_intf_put_u16(uint16_t data)
{
    comm_buf.buffer[comm_buf.buffer_size++] = data >> 8;
    comm_buf.buffer[comm_buf.buffer_size++] = data & 0xFF;
}

/*!
 * @brief This API is used to write the uint32_t data into response buffer
 */
void comm_intf_put_u32(uint32_t data)
{
    comm_buf.buffer[comm_buf.buffer_size++] = (data >> 24) & 0xFF;
    comm_buf.buffer[comm_buf.buffer_size++] = (data >> 16) & 0xFF;
    comm_buf.buffer[comm_buf.buffer_size++] = (data >> 8) & 0xFF;
    comm_buf.buffer[comm_buf.buffer_size++] = data & 0xFF;
}

/*!
 * @brief This API is used to send response from board
 */
int16_t comm_intf_send_response(coines_rsp_buffer_t *rsp_buf)
{
    /*if board type is development desktop add line termination characters*/
    comm_intf_put_u8('\r');
    comm_intf_put_u8('\n');
    comm_buf.buffer[1] = comm_buf.buffer_size;

    coines_write_intf(COINES_COMM_INTF_USB, rsp_buf->buffer, rsp_buf->buffer_size);
    return COINES_SUCCESS;
}

/*!
 * @brief Process COINES protocol commands and calls respective command handlers
 */
int coines_protocol_parser()
{
    uint8_t rslt = COINES_SUCCESS;

    coines_read_intf(COINES_COMM_INTF_USB, cmd_buff, sizeof(cmd_buff));

    if (cmd_buff[0] == COINES_CMD_ID)
    {
        cmd_buff[0] = 0xFF;
        if (cmd_buff[2] == COINES_DD_GET || cmd_buff[2] == COINES_DD_SET)
        {
            for (int i = 0; i < sizeof(normal_cmd_handler) / sizeof(cmd_handler_map_t); i++)
            {
                if (cmd_buff[3] == normal_cmd_handler[i].cmd_id)
                {
                    comm_intf_init_response_header(0x40 | cmd_buff[2], cmd_buff[3], 0);
                    rslt = normal_cmd_handler[i].cmd_handler(cmd_buff, cmd_buff[1]);
                    comm_buf.buffer[3] = rslt;
                    comm_intf_send_response(&comm_buf);
                }
            }
        }

        else
        {

            for (int i = 0; i < sizeof(stream_cmd_handler) / sizeof(cmd_handler_map_t); i++)
            {
                if (cmd_buff[2] == stream_cmd_handler[i].cmd_id)
                {
                    comm_intf_init_response_header(0x40 | cmd_buff[2], 0, 0);
                    rslt = stream_cmd_handler[i].cmd_handler(cmd_buff, cmd_buff[1]);
                    comm_buf.buffer[3] = rslt;
                    comm_intf_send_response(&comm_buf);
                }
            }
        }
    }

    /* Time keeping for Polling streaming */
    for (int i = 1; i <= sensor_count; i++)
    {
        switch (stream_config[i - 1].sampling_units)
        {
        case COINES_SAMPLING_TIME_IN_MICRO_SEC:
            if (coines_get_micros() - micros_time[i - 1] > stream_config[i - 1].sampling_time)
            {
                micros_time[i - 1] = coines_get_micros();
                data_read_int[i - 1] = true;
            }
            break;

        case COINES_SAMPLING_TIME_IN_MILLI_SEC:
            if (coines_get_millis() - millis_time[i - 1] > stream_config[i - 1].sampling_time)
            {
                millis_time[i - 1] = coines_get_millis();
                data_read_int[i - 1] = true;
            }
            break;
        }
    }

    if (stream_in_progress == true)
    {
        uint32_t data_len = 0;
        uint32_t offset = 5;

        for (uint8_t i = 1; i <= 2; i++)
        {
            if (data_read_int[i - 1] == true)
            {
                data_read_int[i - 1] = false;
                comm_intf_init_response_header(stream_mode, i, rslt);
                comm_buf.buffer_size--;

                if (stream_mode == COINES_RSPID_INT_STREAMING_DATA)
                {
                    comm_intf_put_u8(i);
                    offset++;
                }

                rslt = coines_read_stream_sensor_data(i, 0, &comm_buf.buffer[offset], &data_len);
                if (rslt == COINES_SUCCESS && data_len != 0)
                {
                    comm_buf.buffer_size += data_len;

                    if (stream_mode == COINES_RSPID_POLLING_STREAMING_DATA)
                    {
                        comm_intf_put_u8(0x00);
                        comm_intf_put_u8(i);
                    }

                    comm_intf_send_response(&comm_buf);
                }
            }
        }
    }

    return 0;
}

/*!
 * @brief Get/Set Board info. using coines_get_board_info()/coines_set_board_info()
 */
uint8_t coines_cmd_boardinfo_handler(uint8_t *cmd_buff, uint8_t length)
{
    struct coines_board_info data;
    uint8_t rslt = COINES_SUCCESS;
    if (cmd_buff[2] == COINES_DD_GET)
    {
        rslt = coines_get_board_info(&data);
        if (rslt == COINES_SUCCESS)
        {
            comm_intf_put_u16(data.shuttle_id);
            comm_intf_put_u16(data.hardware_id);
            comm_intf_put_u16(data.software_id);
            comm_intf_put_u8(data.board);
        }
    }
    /*** !! COINES extended !! ***/
    else if (cmd_buff[2] == COINES_DD_SET)
    {
        data.shuttle_id = cmd_buff[4] << 8 | cmd_buff[5];
        data.hardware_id = cmd_buff[6] << 8 | cmd_buff[7];
        data.software_id = cmd_buff[8] << 8 | cmd_buff[9];
        data.board = cmd_buff[10];
        rslt = coines_set_board_info(&data);
    }

    return rslt;
}

/*!
 * @brief Set VDD, VDDIO voltage using coines_set_shuttleboard_vdd_vddio_config()
 */
uint8_t coines_cmd_vdd_vddio_cfg_handler(uint8_t *cmd_buff, uint8_t length)
{
    uint16_t vdd_millivolt, vddio_millivolt;
    uint8_t rslt = COINES_SUCCESS;

    vdd_millivolt = cmd_buff[4] << 8 | cmd_buff[5];
    vddio_millivolt = cmd_buff[7] << 8 | cmd_buff[8];

    rslt = coines_set_shuttleboard_vdd_vddio_config(vdd_millivolt, vddio_millivolt);

    if (rslt == COINES_SUCCESS)
    {
        comm_intf_put_u16(vdd_millivolt);
        comm_intf_put_u8(vdd_millivolt ? 1 : 0);
        comm_intf_put_u16(vddio_millivolt);
        comm_intf_put_u8(vddio_millivolt ? 1 : 0);
    }

    return rslt;
}

/*!
 * @brief Set/Get IO pin configuration (Input/Output, Low/High) using coines_set_pin_config()/coines_get_pin_config()
 */
uint8_t coines_cmd_multiio_cfg_handler(uint8_t *cmd_buff, uint8_t length)
{
    uint16_t pin_select, pin_value, pin_direction;
    uint8_t rslt = COINES_SUCCESS;

    if (cmd_buff[2] == COINES_DD_SET)
    {
        pin_select = (cmd_buff[4] << 8) | cmd_buff[5];
        pin_direction = (cmd_buff[6] << 8) | cmd_buff[7];
        pin_value = (cmd_buff[8] << 8) | cmd_buff[9];

        for (int i = 0; i < 16; i++)
        {
            if ((pin_select & (1 << i)) != 0)
            {
                pin_select = i;
                pin_direction = (pin_direction & (1 << i)) >> i;
                pin_value = (pin_value & (1 << i)) >> i;
                break;
            }
        }
        rslt = coines_set_pin_config(pin_select, pin_direction, pin_value);
    }
    else if (cmd_buff[2] == COINES_DD_GET)
    {
        pin_select = (cmd_buff[4] << 8) | cmd_buff[5];
        pin_direction = 0;
        pin_value = 0;

        comm_intf_put_u16(pin_select);

        for (int i = 0; i < 16; i++)
        {
            if ((pin_select & (1 << i)) != 0)
            {
                pin_select = i;
                break;
            }
        }

        rslt = coines_get_pin_config(pin_select, (enum coines_pin_direction *)&pin_direction,
                                     (enum coines_pin_value *)&pin_value);

        if (rslt == COINES_SUCCESS)
        {
            pin_direction <<= pin_select;
            pin_value <<= pin_select;
            comm_intf_put_u16(pin_direction);
            comm_intf_put_u16(pin_value);
        }
    }

    return rslt;
}

/*!
 * @brief Set mode of communication - I2C/SPI
 */
uint8_t coines_cmd_interface_cfg_handler(uint8_t *cmd_buff, uint8_t length)
{
    return COINES_SUCCESS;
}

/*!
 * @brief Configure I2C speed using coines_config_i2c_bus()
 */
uint8_t coines_cmd_i2c_speed_cfg_handler(uint8_t *cmd_buff, uint8_t length)
{
    uint8_t rslt = COINES_SUCCESS;
    rslt = coines_config_i2c_bus(cmd_buff[4], cmd_buff[5]);

    if (rslt == COINES_SUCCESS)
    {
        comm_intf_put_u8_arr(&cmd_buff[4], 2);
    }

    return rslt;
}

/*!
 * @brief Configure SPI speed using coines_config_spi_bus()
 */
uint8_t coines_cmd_spi_settings_cfg_handler(uint8_t *cmd_buff, uint8_t length)
{
    uint8_t rslt = COINES_SUCCESS;

    rslt = coines_config_spi_bus(cmd_buff[4], cmd_buff[5], cmd_buff[7]);

    if (rslt == COINES_SUCCESS)
    {
        comm_intf_put_u8_arr(&cmd_buff[4], 4);
    }

    return rslt;
}

/*!
 * @brief Perform read/write on I2C/SPI bus using below APIs
 *        - coines_read_i2c()
 *        - coines_write_i2c()
 *        - coines_read_spi()
 *        - coines_write_spi()
 */
uint8_t coines_cmd_sensor_read_and_write_handler(uint8_t *cmd_buff, uint8_t length)
{
    uint8_t rslt = COINES_SUCCESS;
    uint8_t reg_addr, dev_addr, reg_data[64];
    uint16_t count;

    if (cmd_buff[5] == 0)
    {
        dev_addr = cmd_buff[9];
        reg_addr = cmd_buff[10];
        count = cmd_buff[12];

        if (cmd_buff[2] == COINES_DD_SET)
        {
            memcpy(reg_data, &cmd_buff[16], count);
            rslt = coines_write_i2c(dev_addr, reg_addr, reg_data, count);
        }
        else if (cmd_buff[2] == COINES_DD_GET)
        {
            rslt = coines_read_i2c(dev_addr, reg_addr, reg_data, count);
        }
    }
    else
    {
        dev_addr = cmd_buff[5];
        reg_addr = cmd_buff[10];
        count = cmd_buff[12];

        if (cmd_buff[2] == COINES_DD_SET)
        {
            memcpy(reg_data, &cmd_buff[16], count);
            rslt = coines_write_spi(dev_addr, reg_addr, reg_data, count);
        }
        else if (cmd_buff[2] == COINES_DD_GET)
        {
            rslt = coines_read_spi(dev_addr, reg_addr, reg_data, count);
        }
    }

    if (rslt == COINES_SUCCESS)
    {
        comm_intf_put_u8(cmd_buff[6]);
        comm_intf_put_u8(reg_addr);
        comm_intf_put_u8((uint8_t)count);
        comm_intf_put_u8((uint8_t)count);
        comm_intf_put_u8(0x00);
        comm_intf_put_u8_arr(reg_data, count);
    }

    return rslt;
}

/*!
 * @brief Configure timer using coines_trigger_timer()
 */
uint8_t coines_cmd_timer_cfg_handler(uint8_t *cmd_buff, uint8_t length)
{
    uint8_t rslt = COINES_SUCCESS;
    if (cmd_buff[2] == COINES_DD_SET)
        rslt = coines_trigger_timer(cmd_buff[4], -1);
    else if (cmd_buff[2] == COINES_DD_GET)
        rslt = coines_trigger_timer(-1, cmd_buff[4]);

    return rslt;
}

/*!
 * @brief Configure sampling time for polling streaming
 */
uint8_t coines_cmd_gen_stream_settings_handler(uint8_t *cmd_buff, uint8_t length)
{

    uint8_t sampling_time_unit;
    uint16_t sampling_time;
    sensor_count = cmd_buff[3];
    sampling_time = cmd_buff[5] << 8 | cmd_buff[6];
    sampling_time_unit = cmd_buff[7];
    comm_buf.buffer_size--;

    if (sampling_time_unit == COINES_SAMPLING_TIME_IN_MILLI_SEC)
        sampling_time_micros = sampling_time * 1000;
    else if (sampling_time_unit == COINES_SAMPLING_TIME_IN_MICRO_SEC)
        sampling_time_micros = sampling_time;
    else
        return COINES_E_FAILURE;

    return COINES_SUCCESS;
}

/*!
 * @brief Start/Stop polling streaming using coines_start_stop_streaming()
 */
uint8_t coines_cmd_start_stop_poll_stream_handler(uint8_t *cmd_buff, uint8_t length)
{
    uint8_t rslt = COINES_E_FAILURE;
    if (cmd_buff[3] == 0)
    {
        rslt = coines_start_stop_streaming(COINES_STREAMING_MODE_POLLING, COINES_STREAMING_STOP);
        stream_in_progress = false;
    }
    else if (cmd_buff[3] == COINES_STREAM_INFINITE_SAMPLES)
    {
        rslt = coines_start_stop_streaming(COINES_STREAMING_MODE_POLLING, COINES_STREAMING_START);
        stream_in_progress = true;
    }

    stream_mode = COINES_RSPID_POLLING_STREAMING_DATA;
    comm_buf.buffer_size--;

    return rslt;
}

/*!
 * @brief Configure polling streaming settings using coines_config_streaming()
 */
uint8_t coines_cmd_poll_stream_settings_handler(uint8_t *cmd_buff, uint8_t length)
{
    uint8_t rslt;

    sensor_id = cmd_buff[3] - 1;
    if (cmd_buff[5] == 0)
    {
        stream_config[sensor_id].intf = COINES_SENSOR_INTF_I2C;
        stream_config[sensor_id].i2c_bus = COINES_I2C_BUS_0;
        stream_config[sensor_id].dev_addr = cmd_buff[7] << 8 | cmd_buff[8];
    }
    else
    {
        stream_config[sensor_id].intf = COINES_SENSOR_INTF_SPI;
        stream_config[sensor_id].spi_bus = COINES_SPI_BUS_0;
        if (cmd_buff[5] == 1)
            stream_config[sensor_id].cs_pin = 0;
        else
            stream_config[sensor_id].cs_pin = cmd_buff[5] - 2;
    }
    stream_config[sensor_id].sampling_time = cmd_buff[9] << 8 | cmd_buff[10];
    stream_config[sensor_id].sampling_units = cmd_buff[11];
    data_blocks[sensor_id].no_of_blocks = cmd_buff[13];

    for (int i = 0; i < data_blocks[sensor_id].no_of_blocks; i++)
    {
        data_blocks[sensor_id].reg_start_addr[i] = cmd_buff[14 + 3 * i];
        data_blocks[sensor_id].no_of_data_bytes[i] = cmd_buff[15 + 3 * i] << 8 | cmd_buff[16 + 3 * i];
    }

    rslt = coines_config_streaming(sensor_id + 1, &stream_config[sensor_id], &data_blocks[sensor_id]);
    comm_buf.buffer_size--;
    comm_intf_put_u8(sensor_id + 1);

    return rslt;
}

/*!
 * @brief Start/Stop interrupt streaming using coines_start_stop_streaming()
 */
uint8_t coines_cmd_start_stop_int_stream_handler(uint8_t *cmd_buff, uint8_t length)
{
    uint8_t rslt = COINES_E_FAILURE;
    if (cmd_buff[3] == 0)
    {
        rslt = coines_start_stop_streaming(COINES_STREAMING_MODE_INTERRUPT, COINES_STREAMING_STOP);

        for (int i = 0; i < 2; i++)
            coines_detach_interrupt(stream_config[i].int_pin);

        stream_in_progress = false;
    }
    else if (cmd_buff[3] == COINES_STREAM_INFINITE_SAMPLES)
    {
        rslt = coines_start_stop_streaming(COINES_STREAMING_MODE_INTERRUPT, COINES_STREAMING_START);
        stream_in_progress = true;
    }

    stream_mode = COINES_RSPID_INT_STREAMING_DATA;
    comm_buf.buffer_size--;

    return rslt;
}

/*!
 * @brief Configure interrupt streaming settings using coines_config_streaming()
 */
uint8_t coines_cmd_int_stream_settings_handler(uint8_t *cmd_buff, uint8_t length)
{
    uint8_t rslt;
    sensor_id = cmd_buff[3] - 1;
    stream_config[sensor_id].int_timestamp = cmd_buff[4];
    if (cmd_buff[5] == 0)
    {
        stream_config[sensor_id].intf = COINES_SENSOR_INTF_I2C;
        stream_config[sensor_id].i2c_bus = COINES_I2C_BUS_0;
        stream_config[sensor_id].dev_addr = cmd_buff[7] << 8 | cmd_buff[8];
    }
    else
    {
        stream_config[sensor_id].intf = COINES_SENSOR_INTF_SPI;
        stream_config[sensor_id].spi_bus = COINES_SPI_BUS_0;
        if (cmd_buff[5] == 1)
            stream_config[sensor_id].cs_pin = 0;
        else
            stream_config[sensor_id].cs_pin = cmd_buff[5] - 2;
    }

    stream_config[sensor_id].int_pin = cmd_buff[6];
    data_blocks[sensor_id].no_of_blocks = cmd_buff[10];

    for (int i = 0; i < data_blocks[sensor_id].no_of_blocks; i++)
    {
        data_blocks[sensor_id].reg_start_addr[i] = cmd_buff[11 + 3 * i];
        data_blocks[sensor_id].no_of_data_bytes[i] = cmd_buff[12 + 3 * i] << 8 | cmd_buff[13 + 3 * i];
    }

    rslt = coines_config_streaming(sensor_id + 1, &stream_config[sensor_id], &data_blocks[sensor_id]);
    comm_buf.buffer_size--;
    comm_intf_put_u8(sensor_id + 1);

    /* Configure interrupt*/
    coines_attach_interrupt(stream_config[sensor_id].int_pin,
                            sensor_int_cb[sensor_id],
                            COINES_PIN_INTERRUPT_FALLING_EDGE);

    return rslt;
}

/*!
 * @brief ISR for Interrupt streaming channel 1
 */
void isr_cb_1()
{
    data_read_int[0] = true;
}

/*!
 * @brief ISR for Interrupt streaming channel 2
 */
void isr_cb_2()
{
    data_read_int[1] = true;
}
