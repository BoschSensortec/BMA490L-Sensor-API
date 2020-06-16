/**
 * Copyright (C) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "bma490l.h"
#include "bma490l_common.h"

/******************************************************************************/
/*!                Macro definition                 		                  */

/*! Read write length varies based on user requirement */
#define READ_WRITE_LENGTH  UINT8_C(8)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * @brief Function for initialization of I2C bus.
 */
int8_t bma490l_user_i2c_init(void)
{

    /* Implement I2C bus initialization according to the target machine. */
    return 0;
}

/*!
 * @brief Function for initialization of SPI bus.
 */
int8_t bma490l_user_spi_init(void)
{

    /* Implement SPI bus initialization according to the target machine. */
    return 0;
}

/*!
 * @brief This function provides the delay for required time (Microseconds) as per the input provided in some of the
 * APIs.
 */
void user_delay_us(uint32_t period_us, void *intf_ptr)
{
    /* Wait for a period amount of microseconds. */
}

/*!
 * @brief This function provides the delay for required time (Milliseconds) as per the input provided in some of the
 * APIs.
 */
void user_delay_ms(uint32_t period_ms)
{
    /* Wait for a period amount of milliseconds. */
}

/*!
 * @brief This function is for writing the sensor's registers through I2C bus.
 */
int8_t user_i2c_reg_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{

    /* Write to registers using I2C. Return 0 for a successful execution. */
    return 0;
}

/*!
 * @brief This function is for reading the sensor's registers through I2C bus.
 */
int8_t user_i2c_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{

    /* Read from registers using I2C. Return 0 for a successful execution. */
    return 0;
}

/*!
 * @brief This function is for writing the sensor's registers through SPI bus.
 */
int8_t user_spi_reg_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{

    /* Write to registers using SPI. Return 0 for a successful execution. */
    return 0;
}

/*!
 * @brief This function is for reading the sensor's registers through SPI bus.
 */
int8_t user_spi_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{

    /* Read from registers using SPI. Return 0 for a successful execution. */
    return 0;
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bma490l_interface_init(struct bma490l_dev *bma490l_dev, uint8_t intf)
{
    int8_t rslt = BMA490L_OK;

    if (bma490l_dev != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BMA490L_I2C_INTF)
        {
            printf("I2C Interface\n");

            dev_addr = BMA490L_I2C_ADDR_PRIMARY;
            bma490l_dev->bus_read = user_i2c_reg_read;
            bma490l_dev->bus_write = user_i2c_reg_write;
            bma490l_dev->intf = BMA490L_I2C_INTF;
        }

        /* Bus configuration : SPI */
        else if (intf == BMA490L_SPI_INTF)
        {
            printf("SPI Interface\n");

            dev_addr = 0;
            bma490l_dev->bus_read = user_spi_reg_read;
            bma490l_dev->bus_write = user_spi_reg_write;
			bma490l_dev->intf = BMA490L_SPI_INTF;
        }

        /* Holds the I2C device addr or SPI chip selection */
        bma490l_dev->intf_ptr = &dev_addr;

        /* Configure delay in microseconds */
        bma490l_dev->delay_us = user_delay_us;

        /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
        bma490l_dev->read_write_len = READ_WRITE_LENGTH;
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bma490l_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMA490L_OK)
    {
        printf("%s\t", api_name);
        if (rslt & BMA490L_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer\r\n", rslt);
        }
        else if (rslt & BMA490L_E_CONFIG_STREAM_ERROR)
        {
            printf("Error [%d] : Invalid configuration stream\r\n", rslt);
        }
        else if (rslt & BMA490L_E_SELF_TEST_FAIL)
        {
            printf("Error [%d] : Self test failed\r\n", rslt);
        }
        else if (rslt & BMA490L_E_INVALID_SENSOR)
        {
            printf("Error [%d] : Device not found\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}