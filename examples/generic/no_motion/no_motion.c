/**
 * Copyright (C) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "bma490l.h"
#include "common.h"

/******************************************************************************/
/*!            Function                                                       */

/* This function starts the execution of program */
int main(void)
{
    /* Status of api are returned to this variable */
    int8_t rslt;

    /* Sensor initialization configuration */
    struct bma490l_dev bma490l_dev;

    /* Variable to get no-motion interrupt status */
    uint16_t int_status = 0;

    /* Structure to define no-motion configuration */
    struct bma490l_any_no_mot_config no_mot;

    /* Random value for while loop to get the no-motion interrupt */
    uint8_t try = 20;

    /* Interface selection is to be updated as parameter
     * For I2C :  BMA490L_I2C_INTF
     * For SPI :  BMA490L_SPI_INTF
     */
    rslt = bma490l_interface_init(&bma490l_dev, BMA490L_I2C_INTF);
    bma490l_error_codes_print_result("bma490l_interface_selection", rslt);

    /* Sensor initialization */
    rslt = bma490l_init(&bma490l_dev);
    bma490l_error_codes_print_result("bma490l_init", rslt);

    /* Upload the configuration file to enable the features of the sensor */
    rslt = bma490l_write_config_file(&bma490l_dev);
    bma490l_error_codes_print_result("bma490l_write_config", rslt);

    /* Enable the accelerometer */
    rslt = bma490l_set_accel_enable(BMA490L_ENABLE, &bma490l_dev);
    bma490l_error_codes_print_result("bma490l_set_accel_enable status", rslt);

    /* Get no-motion config to get the default values */
    rslt = bma490l_get_no_mot_config(&no_mot, &bma490l_dev);
    bma490l_error_codes_print_result("bma490l_get_no_mot_config status", rslt);

    /* Set configuration parameters for no-motion */
    /* 1LSB equals 20ms. Default is 100ms, setting to 80ms. */
    no_mot.duration = 4;

    /* 1LSB equals to 0.48mg. Default is 83mg, setting to 50mg. */
    no_mot.threshold = 102;

    /* Any-motion enabled for all axis */
    no_mot.axes_en = BMA490L_EN_ALL_AXIS;

    rslt = bma490l_set_no_mot_config(&no_mot, &bma490l_dev);
    bma490l_error_codes_print_result("bma490l_set_no_mot_config status", rslt);

    /* Map the interrupt for no-motion */
    rslt = bma490l_map_interrupt(BMA490L_INTR2_MAP, BMA490L_NO_MOT_INT, BMA490L_ENABLE, &bma490l_dev);
    bma490l_error_codes_print_result("bma490l_map_interrupt status", rslt);

    printf("Do not move the board to get no-motion interrupt\n");

    /* Loop over 20 times to get the interrupt status of no-motion */
    while (try > 0)
    {
        bma490l_dev.delay_us(BMA490L_MS_TO_US(1000), bma490l_dev.intf_ptr);
        int_status = 0;

        /* Read the interrupt status */
        rslt = bma490l_read_int_status(&int_status, &bma490l_dev);

        /* Check if no-motion interrupt is triggered */
        if (int_status & BMA490L_NO_MOT_INT)
        {
            printf("No-motion interrupt is received");
            break;
        }

        try--;
    }

    return rslt;
}
