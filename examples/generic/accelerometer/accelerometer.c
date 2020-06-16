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

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH      (9.80665f)

/*! Macro that holds the total number of accel x,y and z axes sample counts to be printed */
#define ACCEL_SAMPLE_COUNT UINT8_C(100)

/******************************************************************************/
/*!           Static Function Declaration                                     */

/*!
 *  @brief This internal function converts raw sensor values(LSB) to meters per seconds square.
 *
 *  @param[in] val       : Raw sensor value.
 *  @param[in] g_range   : Accel Range selected (2G, 4G, 8G, 16G).
 *  @param[in] bit_width : Resolution of the sensor.
 *
 *  @return Accel values in meters per second square.
 *
 */
static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width);

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program */
int main(void)
{
    /* Status of api are returned to this variable */
    int8_t rslt;

    /* Sensor initialization configuration */
    struct bma490l_dev bma490l_dev;

    /* Initialize the interrupt status of accel */
    uint16_t int_status = 0;

    /* Variable that holds the accelerometer sample count */
    uint8_t n_data = ACCEL_SAMPLE_COUNT;

    struct bma490l_accel sens_data = { 0 };
    float x = 0, y = 0, z = 0;
    struct bma490l_accel_config accel_conf = { 0 };

    /* Interface selection is to be updated as parameter
     * For I2C :  BMA490L_I2C_INTF
     * For SPI :  BMA490L_SPI_INTF
     */
    rslt = bma490l_interface_init(&bma490l_dev, BMA490L_I2C_INTF);
    bma490l_error_codes_print_result("bma490l_interface_selection", rslt);

    /* Sensor initialization */
    rslt = bma490l_init(&bma490l_dev);
    bma490l_error_codes_print_result("bma490l_init", rslt);

    /* Upload the configuration file to enable the features of the sensor. */
    rslt = bma490l_write_config_file(&bma490l_dev);
    bma490l_error_codes_print_result("bma490l_write_config", rslt);

    /* Enable the accelerometer */
    rslt = bma490l_set_accel_enable(BMA490L_ENABLE, &bma490l_dev);
    bma490l_error_codes_print_result("bma490l_set_accel_enable status", rslt);

    /* Accelerometer Configuration Settings */
    /* Output Data Rate */
    accel_conf.odr = BMA490L_OUTPUT_DATA_RATE_100HZ;

    /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G) */
    accel_conf.range = BMA490L_ACCEL_RANGE_2G;

    /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
     * if it is set to 2, then 2^(bandwidth parameter) samples
     * are averaged, resulting in 4 averaged samples
     * Note1 : For more information, refer the datasheet.
     * Note2 : A higher number of averaged samples will result in a less noisier signal, but
     * this has an adverse effect on the power consumed.
     */
    accel_conf.bandwidth = BMA490L_ACCEL_NORMAL_AVG4;

    /* Enable the filter performance mode where averaging of samples
     * will be done based on above set bandwidth and ODR.
     * There are two modes
     *  0 -> Averaging samples (Default)
     *  1 -> No averaging
     * For more info on No Averaging mode refer datasheet.
     */
    accel_conf.perf_mode = BMA490L_CIC_AVG_MODE;

    /* Set the accel configurations */
    rslt = bma490l_set_accel_config(&accel_conf, &bma490l_dev);
    bma490l_error_codes_print_result("bma490l_set_accel_config status", rslt);

    /* Mapping data ready interrupt with interrupt1 to get interrupt status once getting new accel data */
    rslt = bma490l_map_interrupt(BMA490L_INTR1_MAP, BMA490L_DATA_RDY_INT, BMA490L_ENABLE, &bma490l_dev);
    bma490l_error_codes_print_result("bma490l_map_interrupt status", rslt);

    printf("Ax[m/s2], Ay[m/s2], Az[m/s2]\n");
    while (1)
    {
        /* Read interrupt status */
        rslt = bma490l_read_int_status(&int_status, &bma490l_dev);

        /* Filtering only the accel data ready interrupt */
        if ((rslt == BMA490L_OK) && (int_status & BMA490L_ACCEL_DATA_RDY_INT))
        {
            /* Read the accel x, y, z data */
            rslt = bma490l_read_accel_xyz(&sens_data, &bma490l_dev);

            if (rslt == BMA490L_OK)
            {
                /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range */
                x = lsb_to_ms2(sens_data.x, 2, bma490l_dev.resolution);
                y = lsb_to_ms2(sens_data.y, 2, bma490l_dev.resolution);
                z = lsb_to_ms2(sens_data.z, 2, bma490l_dev.resolution);

                /* Print the data in m/s2 */
                printf("\n%4.2f, %4.2f, %4.2f", x, y, z);
            }

            /* Decrement the count that determines the number of samples to be printed */
            n_data--;

            /* When the count reaches 0, break and exit the loop */
            if (n_data == 0)
            {
                break;
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal function converts raw sensor values(LSB) to meters per seconds square.
 */
static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}
