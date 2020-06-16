/**
 * Copyright (C) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "bma490l.h"
#include "common.h"

/******************************************************************************/
/*!                Macro definition                                           */

/*! Buffer size allocated to store raw FIFO data */
#define BMA490L_FIFO_RAW_DATA_BUFFER_SIZE UINT16_C(1024)

/*! Length of data to be read from FIFO */
#define BMA490L_FIFO_RAW_DATA_USER_LENGTH UINT16_C(1024)

/*! Number of accel frames to be extracted from FIFO
 * Calculation:
 * fifo_buffer = 1024, accel_frame_len = 6.
 * fifo_accel_frame_count = (1024 / 6) = 170 frames
 */
#define BMA490L_FIFO_ACCEL_FRAME_COUNT    UINT8_C(170)

/******************************************************************************/
/*!            Function                                                       */

/* This function starts the execution of program */
int main(void)
{
    /* Status of api are returned to this variable */
    int8_t rslt;

    /* Sensor initialization configuration */
    struct bma490l_dev bma490l_dev;

    /* Accelerometer configuration structure */
    struct bma490l_accel_config acc_conf = { 0 };

    /* Number of accelerometer frames */
    uint16_t accel_length;

    /* Variable to idx bytes */
    uint16_t idx = 0;

    /* Variable to store the available frame length count in FIFO */
    uint8_t frame_count;

    /* Number of bytes of FIFO data */
    uint8_t fifo_data[BMA490L_FIFO_RAW_DATA_BUFFER_SIZE] = { 0 };

    /* Array of accelerometer frames -> Total bytes =
     * 170 * (6 axes bytes) = 1020 bytes */
    struct bma490l_accel fifo_accel_data[BMA490L_FIFO_ACCEL_FRAME_COUNT];

    /* Initialize FIFO frame structure */
    struct bma490l_fifo_frame fifoframe = { 0 };

    /* Variable that contains interrupt status value */
    uint16_t int_status = 0;

    /* Variable to hold the length of FIFO data */
    uint16_t fifo_length = 0;

    /* Interface selection is to be updated as parameter
     * For I2C :  BMA490L_I2C_INTF
     * For SPI :  BMA490L_SPI_INTF
     */
    rslt = bma490l_interface_init(&bma490l_dev, BMA490L_I2C_INTF);
    bma490l_error_codes_print_result("bma490l_interface_selection", rslt);

    /* Initialize BMA490L */
    rslt = bma490l_init(&bma490l_dev);
    bma490l_error_codes_print_result("bma490l_init", rslt);

    /* Enable the accelerometer sensor */
    rslt = bma490l_set_accel_enable(BMA490L_ENABLE, &bma490l_dev);
    bma490l_error_codes_print_result("bma490l_set_accel_enable", rslt);

    /* Accelerometer Configuration Settings */
    acc_conf.odr = BMA490L_OUTPUT_DATA_RATE_100HZ;
    acc_conf.bandwidth = BMA490L_ACCEL_NORMAL_AVG4;
    acc_conf.range = BMA490L_ACCEL_RANGE_2G;

    /* Set the accel configurations */
    rslt = bma490l_set_accel_config(&acc_conf, &bma490l_dev);
    bma490l_error_codes_print_result("bma490l_set_accel_config", rslt);

    /* Disabling advance power save mode as fifo data is not accessible in advance low power mode */
    rslt = bma490l_set_advance_power_save(BMA490L_DISABLE, &bma490l_dev);
    bma490l_error_codes_print_result("bma490l_set_advance_power_save", rslt);

    /* Clear FIFO configuration register */
    rslt = bma490l_set_fifo_config(BMA490L_FIFO_ALL, BMA490L_DISABLE, &bma490l_dev);
    bma490l_error_codes_print_result("bma490l_set_fifo_config", rslt);

    /* Set FIFO configuration by enabling accel.
     * NOTE 1: The header mode is enabled by default.
     * NOTE 2: By default the FIFO operating mode is FIFO mode. */
    rslt = bma490l_set_fifo_config(BMA490L_FIFO_ACCEL, BMA490L_ENABLE, &bma490l_dev);
    bma490l_error_codes_print_result("bma490l_set_fifo_config", rslt);

    /* Update FIFO structure */
    fifoframe.data = fifo_data;
    fifoframe.length = BMA490L_FIFO_RAW_DATA_USER_LENGTH;

    /* To enable headerless mode, disable the header. */
    rslt = bma490l_set_fifo_config(BMA490L_FIFO_HEADER, BMA490L_DISABLE, &bma490l_dev);
    bma490l_error_codes_print_result("bma490l_set_fifo_config", rslt);

    printf("FIFO is configured in headerless mode\n");

    rslt = bma490l_map_interrupt(BMA490L_INTR1_MAP, BMA490L_FIFO_FULL_INT, BMA490L_ENABLE, &bma490l_dev);
    bma490l_error_codes_print_result("bma490l_map_interrupt", rslt);
    while (1)
    {
        rslt = bma490l_read_int_status(&int_status, &bma490l_dev);
        bma490l_error_codes_print_result("bma490l_read_int_status", rslt);

        if ((rslt == BMA490L_OK) && (int_status & BMA490L_FIFO_FULL_INT))
        {
            rslt = bma490l_get_fifo_length(&fifo_length, &bma490l_dev);
            bma490l_error_codes_print_result("bma490l_get_fifo_length", rslt);

            printf("FIFO data bytes available : %d\n", fifo_length);
            printf("FIFO data bytes requested : %d\n", fifoframe.length);

            /* Read FIFO data */
            rslt = bma490l_read_fifo_data(&fifoframe, &bma490l_dev);
            accel_length = BMA490L_FIFO_ACCEL_FRAME_COUNT;

            if (rslt == BMA490L_OK)
            {
                printf("Requested data frames before parsing: %d\n", accel_length);

                /* Parse the FIFO data to extract accelerometer data from the FIFO buffer */
                rslt = bma490l_extract_accel(fifo_accel_data, &accel_length, &fifoframe, &bma490l_dev);
                printf("Parsed accelerometer data frames: %d\n", accel_length);

                /* Calculating the frame count from the available bytes in FIFO
                 * frame_count = (available_fifo_bytes / acc_frame_len) */
                frame_count = (fifo_length / BMA490L_FIFO_ACC_LENGTH);
                printf("Available frame count: %d\n", frame_count);

                /* Print the parsed accelerometer data from the FIFO buffer */
                for (idx = 0; idx < frame_count; idx++)
                {
                    printf("ACCEL[%d] X : %d Y : %d Z : %d\n",
                           idx,
                           fifo_accel_data[idx].x,
                           fifo_accel_data[idx].y,
                           fifo_accel_data[idx].z);
                }
            }

            break;
        }
    }

    return rslt;
}
