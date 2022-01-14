/**
 * Copyright (C) 2022 Jabez Winston
 *
 * SPDX-License-Identifier: MIT
 *
 * @file    mcu_bluepill.c
 * @brief   COINES API implementation for STM32F103 based Bluepill board
 */

#include "mcu_bluepill.h"

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim2, htim4;

struct coines_board_info board_info = {
    .board = BOARD_ID_APP30,
    .hardware_id = HW_VER,
    .shuttle_id = DEFAULT_SHUTTLE_ID,
    .software_id = SW_VER,
};

static struct coines_streaming_config coines_stream_cfg[2];
static struct coines_streaming_blocks coines_data_blks[2];

volatile uint64_t tim2_count = 0, tim4_count = 0;
static isr_cb_t isr_cb[3];
static uint8_t g_stream_mode;
uint32_t packet_counter = 0;

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 72;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 50000;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 72;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 50000;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
        ;
}

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
        * Only INT1 and INT2  @ PA1 and PA2 
        * a.k.a COINES_SHUTTLE_PIN_20/21 [APP2.0 Board]
        * COINES_MINI_SHUTTLE_PIN_1_6/1_7 [APP3.0 Board]
        */
        PIN_MAP(COINES_SHUTTLE_PIN_20, 1);
        PIN_MAP(COINES_SHUTTLE_PIN_21, 2);
        PIN_MAP(COINES_MINI_SHUTTLE_PIN_1_6, 1);
        PIN_MAP(COINES_MINI_SHUTTLE_PIN_1_7, 2);

        /* SPI CS - PA4 */
        PIN_MAP(COINES_SHUTTLE_PIN_7, 4);
        PIN_MAP(COINES_MINI_SHUTTLE_PIN_2_1, 4);

        /* Generally SDO pin for most shuttle boards - PA0 */
        PIN_MAP(COINES_SHUTTLE_PIN_15, 0);
        PIN_MAP(COINES_MINI_SHUTTLE_PIN_2_3, 0);

        /* Generally PS for most shuttle boards - PA3 */
        PIN_MAP(COINES_SHUTTLE_PIN_9, 3);
        PIN_MAP(COINES_MINI_SHUTTLE_PIN_1_4, 3);

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
    HAL_Init();
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USB_DEVICE_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();

    HAL_TIM_Base_Start_IT(&htim2);

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

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = 1 << hw_pin;

    if (direction == COINES_PIN_DIRECTION_OUT)
    {
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        HAL_GPIO_WritePin(GPIOA, 1 << hw_pin, pin_value);
    }
    else if (direction == COINES_PIN_DIRECTION_IN)
    {
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

        if (pin_value == COINES_PIN_VALUE_HIGH)
            GPIO_InitStruct.Pull = GPIO_PULLUP;
        else if (pin_value == COINES_PIN_VALUE_LOW)
            GPIO_InitStruct.Pull = GPIO_PULLDOWN;

        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = 1 << hw_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    if (hw_pin < 0)
        return COINES_E_NOT_SUPPORTED;

    if ((pin_value == NULL) || (pin_direction == NULL))
        return COINES_E_NULL_PTR;

    if (pin_direction != NULL)
        *pin_direction = COINES_PIN_DIRECTION_IN;

    if (pin_value != NULL)
        *pin_value = HAL_GPIO_ReadPin(GPIOA, 1 << hw_pin);

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
        hi2c1.Init.ClockSpeed = 100000;
    else
        hi2c1.Init.ClockSpeed = 400000;

    if (HAL_I2C_Init(&hi2c1) == HAL_OK)
        return COINES_SUCCESS;
    else
        return COINES_E_FAILURE;
}

/*!
 *  @brief This API is used to write the data in I2C communication.
 */
int8_t coines_write_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    if (HAL_I2C_Mem_Write(&hi2c1, (dev_addr << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, count, I2C_TIMEOUT) == HAL_OK)
        return COINES_SUCCESS;
    else
        return COINES_E_FAILURE;
}

/*!
 *  @brief This API is used to read the data in I2C communication.
 */
int8_t coines_read_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    if (HAL_I2C_Mem_Read(&hi2c1, (dev_addr << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, count, I2C_TIMEOUT) == HAL_OK)
        return COINES_SUCCESS;
    else
        return COINES_E_FAILURE;
}

/*!
 *  @brief This API is used to configure the SPI bus
 */
int16_t coines_config_spi_bus(enum coines_spi_bus bus, enum coines_spi_speed spi_speed, enum coines_spi_mode spi_mode)
{
    hspi1.Init.CLKPhase = spi_mode & 0x1;
    hspi1.Init.CLKPolarity = (spi_mode & 0x2) >> 1;

    switch (spi_speed)
    {
    /* 9 MHz */
    case COINES_SPI_SPEED_10_MHZ:
    case COINES_SPI_SPEED_7_5_MHZ:
        hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
        break;
        break;

    /* 4.5 MHz */
    case COINES_SPI_SPEED_6_MHZ:
    case COINES_SPI_SPEED_5_MHZ:
    case COINES_SPI_SPEED_3_75_MHZ:
        hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
        break;

    /* 2.25 MHz */
    case COINES_SPI_SPEED_3_MHZ:
    case COINES_SPI_SPEED_2_5_MHZ:
    case COINES_SPI_SPEED_2_MHZ:
        hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
        break;

    /* 1.125 MHz */
    case COINES_SPI_SPEED_1_5_MHZ:
    case COINES_SPI_SPEED_1_25_MHZ:
    case COINES_SPI_SPEED_1_2_MHZ:
    case COINES_SPI_SPEED_1_MHZ:
        hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
        break;

    /* 562.5 kHz */
    case COINES_SPI_SPEED_750_KHZ:
    case COINES_SPI_SPEED_600_KHZ:
    case COINES_SPI_SPEED_500_KHZ:
        hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
        break;

    /* 281.25 kHz */
    case COINES_SPI_SPEED_400_KHZ:
    case COINES_SPI_SPEED_300_KHZ:
    case COINES_SPI_SPEED_250_KHZ:
        hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
        break;
    }

    /* SPI CS initialization */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
        return COINES_SUCCESS;
    else
        return COINES_E_FAILURE;
}

/*!
 *  @brief This API is used to read the data in SPI communication.
 */
int8_t coines_write_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    uint8_t tx_buff[64];

    tx_buff[0] = reg_addr;
    memcpy(&tx_buff[1], &reg_data[0], count);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    int8_t rslt = HAL_SPI_Transmit(&hspi1, tx_buff, count + 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    if (rslt == HAL_OK)
        return COINES_SUCCESS;
    else
        return COINES_E_FAILURE;
}

/*!
 *  @brief This API is used to read the data in SPI communication.
 */
int8_t coines_read_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    uint8_t rx_buff[64], tx_buff[64] = {};
    tx_buff[0] = reg_addr;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    int8_t rslt = HAL_SPI_TransmitReceive(&hspi1, tx_buff, rx_buff, count + 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    if (rslt == HAL_OK)
    {
        memcpy(&reg_data[0], &rx_buff[1], count);
        return COINES_SUCCESS;
    }
    else
        return COINES_E_FAILURE;
}

/*!
 *  @brief This API is used for introducing a delay in milliseconds
 */
void coines_delay_msec(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
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
        uint64_t time_count = tim4_count + __HAL_TIM_GET_COUNTER(&htim4);

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
    HAL_StatusTypeDef rslt;

    if (ts_cfg == COINES_TIMESTAMP_ENABLE)
    {
        switch (tmr_cfg)
        {
        case COINES_TIMER_START:
            rslt = HAL_TIM_Base_Start_IT(&htim4);
            break;
        case COINES_TIMER_STOP:
            rslt = HAL_TIM_Base_Stop_IT(&htim4);
            break;
        case COINES_TIMER_RESET:
            rslt = HAL_TIM_Base_Init(&htim4);
            tim4_count = 0;
            break;
        default:
            rslt = HAL_TIM_Base_Start_IT(&htim4);
            tim4_count = 0;
        }
    }
    else
    {
        HAL_TIM_Base_Stop_IT(&htim4);
        rslt = HAL_TIM_Base_Init(&htim4);
        tim2_count = 0;
    }

    if (rslt == HAL_OK)
        return COINES_SUCCESS;
    else
        return COINES_E_FAILURE;
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
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    switch (int_mode)
    {
    case COINES_PIN_INTERRUPT_CHANGE:
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
        break;

    case COINES_PIN_INTERRUPT_FALLING_EDGE:
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
        break;

    case COINES_PIN_INTERRUPT_RISING_EDGE:
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        break;
    }

    int hw_pin = get_hw_pin(pin_number);
    GPIO_InitStruct.Pin = 1 << hw_pin;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(hw_pin + 6, 0, 0);
    HAL_NVIC_EnableIRQ(hw_pin + 6);

    isr_cb[hw_pin] = callback;
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
    int hw_pin = get_hw_pin(pin_number);

    HAL_NVIC_DisableIRQ(hw_pin + 6);
    HAL_GPIO_DeInit(GPIOA, 1 << hw_pin);
    isr_cb[hw_pin] = NULL;
}

/*!
 * @brief This API returns the number of milliseconds passed since the program started
 *
 * @return Time in milliseconds
 */
uint32_t coines_get_millis()
{
    return HAL_GetTick();
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
    uint64_t time_count = tim2_count + __HAL_TIM_GET_COUNTER(&htim2);
    __DSB();
    return time_count;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        tim2_count += 50000;
    }
    else if (htim->Instance == TIM4)
    {
        tim4_count += 50000;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    for (int i = 0; i < 3; i++)
    {
        if (GPIO_Pin & (1 << i))
            isr_cb[i]();
    }
}

uint16_t coines_read_intf(enum coines_comm_intf intf, void *buffer, uint16_t len)
{
    if (UserRxBufferFS[0] == COINES_CMD_ID)
    {
        /* Copy only received bytes (instead of requested bytes) to improve performance ! */
        memcpy(buffer, UserRxBufferFS, UserRxBufferFS[1]);
        UserRxBufferFS[0] = 0xFF;
        return COINES_SUCCESS;
    }
    else
    {
        return COINES_E_FAILURE;
    }
}

void coines_write_intf(enum coines_comm_intf intf, void *buffer, uint16_t len)
{
    while (CDC_Transmit_FS(buffer, len) != USBD_BUSY)
        ;
}
