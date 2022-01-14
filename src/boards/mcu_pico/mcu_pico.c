/**
 * Copyright (C) 2022 Jabez Winston
 *
 * SPDX-License-Identifier: MIT
 *
 * @file    mcu_pico.c
 * @brief   COINES API implementation for Raspberry Pi Pico/RP2040 board
 */

#include "mcu_pico.h"

struct coines_board_info board_info = {
    .board = BOARD_ID_APP30,
    .hardware_id = HW_VER,
    .shuttle_id = DEFAULT_SHUTTLE_ID,
    .software_id = SW_VER,
};

static struct coines_streaming_config coines_stream_cfg[2];
static struct coines_streaming_blocks coines_data_blks[2];

static isr_cb_t isr_cb[3];
static uint8_t g_stream_mode;
uint32_t packet_counter = 0;

/*!
 * @brief Get hardware pin from SHUTTLE_PIN
 */
static int32_t get_hw_pin(enum coines_multi_io_pin shuttle_pin)
{

#define PIN_MAP(sp, hp) \
    case sp:            \
        return hp

    switch (shuttle_pin)
    {
        /* 
        * Only INT1 and INT2  @ GP2 and GP3 
        * a.k.a COINES_SHUTTLE_PIN_20/21 [APP2.0 Board]
        * COINES_MINI_SHUTTLE_PIN_1_6/1_7 [APP3.0 Board]
        */
        PIN_MAP(COINES_SHUTTLE_PIN_20, 2);
        PIN_MAP(COINES_SHUTTLE_PIN_21, 3);
        PIN_MAP(COINES_MINI_SHUTTLE_PIN_1_6, 2);
        PIN_MAP(COINES_MINI_SHUTTLE_PIN_1_7, 3);

        /* SPI CS */
        PIN_MAP(COINES_SHUTTLE_PIN_7, PICO_DEFAULT_SPI_CSN_PIN);
        PIN_MAP(COINES_MINI_SHUTTLE_PIN_2_1, PICO_DEFAULT_SPI_CSN_PIN);

        /* Generally SDO pin for most shuttle boards */
        PIN_MAP(COINES_SHUTTLE_PIN_15, 6);
        PIN_MAP(COINES_MINI_SHUTTLE_PIN_2_3, 6);

        /* Generally PS for most shuttle boards */
        PIN_MAP(COINES_SHUTTLE_PIN_9, 7);
        PIN_MAP(COINES_MINI_SHUTTLE_PIN_1_4, 7);

    default:
        return COINES_E_NOT_SUPPORTED;
    }
}

/*!
 * @brief This API is used to initialize the communication according to interface type.
 *
 * @param[in] intf_type: Type of interface(USB, COM, or BLE).
 *
 * @return Result of API execution status
 * @retval Zero -> Success
 * @retval Negative -> Error
 */
int16_t coines_open_comm_intf(enum coines_comm_intf intf_type)
{
    /* USB Initialization */
    usb_device_init();
    while (!configured)
        ;

    usb_start_transfer(usb_get_endpoint_configuration(EP2_OUT_ADDR), NULL, 64);

    /* I2C Initializion */
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    i2c_init(i2c_default, 100 * 1000);

    /* SPI Initializion */
    bi_decl(bi_3pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI));
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    spi_init(spi_default, 500 * 1000);

    /* Interrupt pins */
    bi_decl(bi_1pin_with_name(2, "INT1"));
    bi_decl(bi_1pin_with_name(3, "INT2"));

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to get the board information.
 */
int16_t coines_get_board_info(struct coines_board_info *data)
{
    if (data != NULL)
    {
        data->board = board_info.board;
        data->hardware_id = board_info.hardware_id;
        data->shuttle_id = board_info.shuttle_id;
        data->software_id = board_info.software_id;
        return COINES_SUCCESS;
    }
    else
        return COINES_E_NULL_PTR;
}

/*!
 *  @brief This API is used to configure the pin(MULTIIO/SPI/I2C in shuttle board).
 */
int16_t coines_set_pin_config(enum coines_multi_io_pin pin_number, enum coines_pin_direction direction, enum coines_pin_value pin_value)
{
    int32_t hw_pin = get_hw_pin(pin_number);

    if (hw_pin < 0)
        return COINES_E_NOT_SUPPORTED;

    gpio_init(hw_pin);

    if (direction == COINES_PIN_DIRECTION_OUT)
    {
        gpio_set_dir(hw_pin, GPIO_OUT);
        gpio_put(hw_pin, pin_value);
    }
    else if (direction == COINES_PIN_DIRECTION_IN)
    {
        gpio_set_dir(hw_pin, GPIO_IN);

        if (pin_value == COINES_PIN_VALUE_HIGH)
            gpio_pull_up(hw_pin);
        else if (pin_value == COINES_PIN_VALUE_LOW)
            gpio_pull_down(hw_pin);
    }
    else
        return COINES_E_NOT_SUPPORTED;

    return COINES_SUCCESS;
}

/*!
 *  @brief This API function is used to get the pin direction and pin state.
 */
int16_t coines_get_pin_config(enum coines_multi_io_pin pin_number, enum coines_pin_direction *pin_direction, enum coines_pin_value *pin_value)
{
    int32_t hw_pin = get_hw_pin(pin_number);

    if (hw_pin < 0)
        return COINES_E_NOT_SUPPORTED;

    if ((pin_value == NULL) || (pin_direction == NULL))
        return COINES_E_NULL_PTR;

    if (pin_direction != NULL)
        *pin_direction = gpio_get_dir(hw_pin);

    if (pin_value != NULL)
        *pin_value = gpio_get(hw_pin);

    return COINES_SUCCESS;
}

/*!
 *  @brief This API function is used to get the pin direction and pin state.
 */
int16_t coines_set_shuttleboard_vdd_vddio_config(uint16_t vdd_millivolt, uint16_t vddio_millivolt)
{
    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to configure the I2C bus
 */
int16_t coines_config_i2c_bus(enum coines_i2c_bus bus, enum coines_i2c_mode i2c_mode)
{
    if (i2c_mode == COINES_I2C_STANDARD_MODE)
        i2c_set_baudrate(i2c_default, 100 * 1000);
    else
        i2c_set_baudrate(i2c_default, 400 * 1000);

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to write the data in I2C communication.
 */
int8_t coines_write_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    uint8_t tx_buff[count + 1];
    tx_buff[0] = reg_addr;
    memcpy(&tx_buff[1], reg_data, count);

    if (i2c_write_blocking(i2c_default, dev_addr, tx_buff, count + 1, false) < 0)
        return COINES_E_FAILURE;

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to read the data in I2C communication.
 */
int8_t coines_read_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    if (i2c_write_blocking(i2c_default, dev_addr, &reg_addr, 1, true) < 0)
        return COINES_E_FAILURE;

    if (i2c_read_blocking(i2c_default, dev_addr, reg_data, count, false) < 0)
        return COINES_E_FAILURE;

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to configure the SPI bus
 */
int16_t coines_config_spi_bus(enum coines_spi_bus bus, enum coines_spi_speed spi_speed, enum coines_spi_mode spi_mode)
{
    bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_CSN_PIN, "SPI CS"));
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);

    spi_set_baudrate(spi_default, (60 * 1000 * 1000) / spi_speed);

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to read the data in SPI communication.
 */
int8_t coines_write_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);

    if (spi_write_blocking(spi_default, &reg_addr, 1) < 0)
        return COINES_E_FAILURE;
    if (spi_write_blocking(spi_default, reg_data, count) < 0)
        return COINES_E_FAILURE;

    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to read the data in SPI communication.
 */
int8_t coines_read_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);

    if (spi_write_blocking(spi_default, &reg_addr, 1) < 0)
        return COINES_E_FAILURE;
    if (spi_read_blocking(spi_default, 0, reg_data, count) < 0)
        return COINES_E_FAILURE;

    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used for introducing a delay in milliseconds
 */
void coines_delay_msec(uint32_t delay_ms)
{
    sleep_ms(delay_ms);
}

/*!
 * @brief This API is used to send the streaming settings to the board.
 */
int16_t coines_config_streaming(uint8_t channel_id, struct coines_streaming_config *stream_config, struct coines_streaming_blocks *data_blocks)
{
    int16_t rslt = COINES_SUCCESS;

    if ((stream_config != NULL) && (data_blocks != NULL))
    {
        memcpy(&coines_stream_cfg[channel_id - 1], stream_config, sizeof(struct coines_streaming_config));
        memcpy(&coines_data_blks[channel_id - 1], data_blocks, sizeof(struct coines_streaming_blocks));
    }
    else
    {
        rslt = COINES_E_NULL_PTR;
    }
    return rslt;
}

/*!
 * @brief This API is used to send the streaming settings to the board.
 */
int16_t coines_start_stop_streaming(enum coines_streaming_mode stream_mode, uint8_t start_stop)
{
    g_stream_mode = stream_mode;
    return COINES_SUCCESS;
}

/*!
 * @brief This API is used to read the streaming sensor data.
 */
int16_t coines_read_stream_sensor_data(uint8_t sensor_id, uint32_t number_of_samples, uint8_t *data, uint32_t *valid_samples_count)
{
    uint8_t reg_data[64], ts[6], pc[4];
    *valid_samples_count = 0;
    int16_t rslt = COINES_SUCCESS;
    sensor_id--;

    if (coines_stream_cfg[sensor_id].int_timestamp == 1)
    {
        uint64_t time_count = coines_get_micros();

        for (int i = 0; i < 6; i++)
            ts[i] = time_count >> ((5 - i) * 8);
    }

    if (g_stream_mode == COINES_STREAMING_MODE_INTERRUPT)
    {
        packet_counter++;

        for (int i = 0; i < 4; i++)
            pc[i] = packet_counter >> ((3 - i) * 8);

        memcpy(&data[*valid_samples_count], pc, 4);
        *valid_samples_count += 4;
    }

    for (int i = 0; i < coines_data_blks[sensor_id].no_of_blocks; i++)
    {
        if (coines_stream_cfg[sensor_id].intf == COINES_SENSOR_INTF_I2C)
        {
            rslt = coines_read_i2c(coines_stream_cfg[sensor_id].dev_addr,
                                   coines_data_blks[sensor_id].reg_start_addr[i], reg_data,
                                   coines_data_blks[sensor_id].no_of_data_bytes[i]);
        }
        else if (coines_stream_cfg[sensor_id].intf == COINES_SENSOR_INTF_SPI)
        {
            rslt = coines_read_spi(coines_stream_cfg[sensor_id].dev_addr,
                                   coines_data_blks[sensor_id].reg_start_addr[i] | 0x80, reg_data,
                                   coines_data_blks[sensor_id].no_of_data_bytes[i]);
        }

        memcpy(&data[*valid_samples_count], reg_data, coines_data_blks[sensor_id].no_of_data_bytes[i]);
        *valid_samples_count += coines_data_blks[sensor_id].no_of_data_bytes[i];
    }

    if (coines_stream_cfg[sensor_id].int_timestamp == 1)
    {
        memcpy(&data[*valid_samples_count], ts, 6);
        *valid_samples_count += 6;
    }

    return rslt;
}

/*!
 * @brief This API is used to trigger the timer in firmware and enable or disable system time stamp
 */
int16_t coines_trigger_timer(enum coines_timer_config tmr_cfg, enum coines_time_stamp_config ts_cfg)
{

    if (ts_cfg == COINES_TIMESTAMP_ENABLE)
    {
        switch (tmr_cfg)
        {
        case COINES_TIMER_START:
            break;
        case COINES_TIMER_STOP:
            break;
        case COINES_TIMER_RESET:
            break;
        }
    }
    else
    {
    }

    return COINES_SUCCESS;
}

/*!
 * @brief Attaches a interrupt to a Multi-IO pin
 *
 * @param[in] pin_number : Multi-IO pin
 * @param[in] callback : Name of the function to be called on detection of interrupt
 * @param[in] mode : Trigger modes - change,rising edge,falling edge
 *
 * @return void
 */
void coines_attach_interrupt(enum coines_multi_io_pin pin_number,
                             void (*callback)(void),
                             enum coines_pin_interrupt_mode int_mode)
{
    int pico_pin = get_hw_pin(pin_number);

    if (pico_pin < 0)
        return;

    isr_cb[pico_pin - 1] = callback;

    switch (int_mode)
    {
    case COINES_PIN_INTERRUPT_CHANGE:
        gpio_set_irq_enabled_with_callback(pico_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, gpio_callback);
        break;

    case COINES_PIN_INTERRUPT_FALLING_EDGE:
        gpio_set_irq_enabled_with_callback(pico_pin, GPIO_IRQ_EDGE_FALL, true, gpio_callback);
        break;

    case COINES_PIN_INTERRUPT_RISING_EDGE:
        gpio_set_irq_enabled_with_callback(pico_pin, GPIO_IRQ_EDGE_RISE, true, gpio_callback);
        break;
    }
}

/*!
 * @brief Detaches a interrupt from a Multi-IO pin
 *
 * @param[in] pin_number : Multi-IO pin
 *
 * @return void
 */
void coines_detach_interrupt(enum coines_multi_io_pin pin_number)
{
    int pico_pin = get_hw_pin(pin_number);

    if (pico_pin < 0)
        return;

    gpio_set_irq_enabled(pico_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
    isr_cb[pico_pin - 1] = NULL;
}

/*!
 * @brief This API returns the number of milliseconds passed since the program started
 *
 * @return Time in milliseconds
 */
uint32_t coines_get_millis()
{
    return to_ms_since_boot(get_absolute_time());
}

/*******************************!!! COINES Extended !!!*************************************/
/*!
 *  @brief This API is used to set board information.
 */
int16_t coines_set_board_info(struct coines_board_info *data)
{
    if (data != NULL)
    {
        board_info.board = data->board;
        board_info.hardware_id = data->hardware_id;
        board_info.shuttle_id = data->shuttle_id;
        board_info.software_id = data->software_id;
        return COINES_SUCCESS;
    }
    else
        return COINES_E_NULL_PTR;
}

uint64_t coines_get_micros()
{
    return to_us_since_boot(get_absolute_time());
}

void gpio_callback(uint pico_pin, uint32_t events)
{
    isr_cb[pico_pin - 1]();
}

uint16_t coines_read_intf(enum coines_comm_intf intf, void *buffer, uint16_t len)
{
    return usb_receive_data(buffer, len);
}

void coines_write_intf(enum coines_comm_intf intf, void *buffer, uint16_t len)
{
    usb_send_data(buffer, len);
}
