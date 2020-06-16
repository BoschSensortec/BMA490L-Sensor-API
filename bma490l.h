/**
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file       bma490l.h
 * @date       2020-04-08
 * @version    v0.2.3
 *
 */

/*!
 * @defgroup bma490l BMA490L
 * @brief Sensor driver for BMA490L sensor
 */

#ifndef BMA490L_H_
#define BMA490L_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************/

/*!             Header files
 ****************************************************************************/
#include "bma490l_defs.h"

/***************************************************************************/

/*!                 BMA490L User Interface function prototypes
 ****************************************************************************/

/**
 * \ingroup bma490l
 * \defgroup bma490lApiInit Initialization
 * @brief Initialize the sensor and device structure
 */

/*!
 * \ingroup bma490lApiInit
 * \page bma490l_api_bma490l_init bma490l_init
 * \code
 * int8_t bma490l_init(struct bma490l_dev *dev);
 * \endcode
 * @details This API is the entry point. Call this API before using all other APIs.
 * This API reads the chip-id of the sensor and sets the resolution, feature
 * length and the type of variant.
 *
 * @param[in,out] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_init(struct bma490l_dev *dev);

/**
 * \ingroup bma490l
 * \defgroup bma490lApiConfig ConfigFile
 * @brief Write binary configuration in the sensor
 */

/*!
 * \ingroup bma490lApiConfig
 * \page bma490l_api_bma490l_write_config_file bma490l_write_config_file
 * \code
 * int8_t bma490l_write_config_file(struct bma490l_dev *dev);
 * \endcode
 * @details This API is used to upload the configuration file to enable the
 * features of the sensor.
 *
 * @param[in, out] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_write_config_file(struct bma490l_dev *dev);

/*!
 * \ingroup bma490l
 * \defgroup bma490lApiRegisters Read and Write api
 * @brief Read and Write operation
 */

/*!
 * \ingroup bma490lApiRegisters
 * \page bma490l_api_bma490l_write_regs bma490l_write_regs
 * \code
 * int8_t bma490l_write_regs(uint8_t addr, const uint8_t *data, uint16_t len, struct bma490l_dev *dev);
 * \endcode
 * @details  This API checks whether the write operation requested is for
 * feature configuration or register write and accordingly writes the data in the
 * sensor.
 *
 * @note User has to disable the advance power save mode in the sensor when
 * using this API in burst write mode.
 * bma490l_set_advance_power_save(BMA490L_DISABLE, dev);
 *
 * @param[in] addr : Register address.
 * @param[in] data : Write data buffer.
 * @param[in] len  : No of bytes to write.
 * @param[in] dev  : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_write_regs(uint8_t addr, const uint8_t *data, uint32_t len, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiRegisters
 * \page bma490l_api_bma490l_read_regs bma490l_read_regs
 * \code
 * int8_t bma490l_read_regs(uint8_t addr, uint8_t *data, uint16_t len, struct bma490l_dev *dev);
 * \endcode
 * @details This API checks whether the read operation requested is for
 * feature or register read and accordingly reads the data from the sensor.
 *
 * @param[in] addr : Register address.
 * @param[in] data : Read data buffer.
 * @param[in] len  : No of bytes to read.
 * @param[in] dev  : Structure instance of bma490l_dev.
 *
 * @note For most of the registers auto address increment applies, with the
 * exception of a few special registers, which trap the address. For e.g.,
 * Register address - 0x26, 0x5E.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_read_regs(uint8_t addr, uint8_t *data, uint32_t len, struct bma490l_dev *dev);

/**
 * \ingroup bma490l
 * \defgroup bma490lApiConfiguration Configuration
 * @brief Read / Write advance power mode of accel sensor
 */

/*!
 * \ingroup bma490lApiConfiguration
 * \page bma490l_api_bma490l_set_advance_power_save bma490l_set_advance_power_save
 * \code
 * int8_t bma490l_set_advance_power_save(uint8_t adv_pwr_save, struct bma490l_dev *dev);
 * \endcode
 * @details This API sets the advance power save mode in the sensor.
 *
 * @note If advanced power save is enabled and the accel and/or
 * magnetometer operate in duty cycling mode, the length of the unlatched
 * data ready interrupt pulse is longer than 1/3.2 kHz (312.5 us).
 *
 * @param[in] adv_pwr_save : The value of advance power save mode
 * @param[in] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_set_advance_power_save(uint8_t adv_pwr_save, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiConfiguration
 * \page bma490l_api_bma490l_get_advance_power_save bma490l_get_advance_power_save
 * \code
 * int8_t bma490l_get_advance_power_save(uint8_t adv_pwr_save, struct bma490l_dev *dev);
 * \endcode
 * @details This API reads the status of advance power save mode
 * from the sensor.
 *
 * @note If the advance power save is enabled and the accel and/or
 * magnetometer operate in duty cycling mode, the length of the unlatched
 * DRDY interrupt pulse is longer than 1/3.2 kHz (312.5 us).
 *
 * @param[out] adv_pwr_save : The value of advance power save mode
 * @param[in] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_get_advance_power_save(uint8_t *adv_pwr_save, struct bma490l_dev *dev);

/**
 * \ingroup bma490l
 * \defgroup bma490lApiConfiguration Configuration
 * @brief Enables / Disables accelerometer in the sensor
 */

/*!
 * \ingroup bma490lApiConfiguration
 * \page bma490l_api_bma490l_set_accel_enable bma490l_set_accel_enable
 * \code
 * int8_t bma490l_set_accel_enable(uint8_t accel_en, struct bma490l_dev *dev);
 * \endcode
 * @details This API enables or disables the Accel in the sensor.
 *
 * @note Before reading Accel data, user should call this API.
 *
 * @param[in] accel_en : Variable used to enable or disable the Accel.
 * @param[in] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0  -> Fail
 *
 */
int8_t bma490l_set_accel_enable(uint8_t accel_en, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiConfiguration
 * \page bma490l_api_bma490l_get_accel_enable bma490l_get_accel_enable
 * \code
 * int8_t bma490l_get_accel_enable(uint8_t *accel_en, struct bma490l_dev *dev);
 * \endcode
 * @details This API checks whether Accel is enabled or not in the sensor.
 *
 * @param[out] accel_en : Pointer variable used to store the Accel enable
 *                        status.
 * @param[in] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_get_accel_enable(uint8_t *accel_en, struct bma490l_dev *dev);

/**
 * \ingroup bma490l
 * \defgroup bma490lApiConfiguration Configuration
 * @brief Write available sensor specific commands to the sensor
 */

/*!
 * \ingroup bma490lApiConfiguration
 * \page bma490l_api_bma490l_set_command_register bma490l_set_command_register
 * \code
 * int8_t bma490l_set_command_register(uint8_t command_reg, struct bma490l_dev *dev);
 * \endcode
 * @details This API writes the available sensor specific commands
 * to the sensor.
 *
 *  @param[in] command_reg : The command to write to the command register.
 *
 *@verbatim
 *  Value   |       Description
 *  --------|------------------------------------------------------
 *  0xB6    |       Triggers a soft-reset
 *  0xB0    |       Clears all data in the FIFO, does not change
 *          |       FIFO_CONFIG and FIFO_DOWNS registers
 *  0xF0    |       Reset acceleration data path
 *@endverbatim
 *
 * @param[in] dev : Structure instance of bma490l_dev.
 *
 * @note Register will always read as 0x00.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_set_command_register(uint8_t command_reg, struct bma490l_dev *dev);

/**
 * \ingroup bma490l
 * \defgroup bma490lApiConfiguration Configuration
 * @brief Read / Write configurations of accel sensor
 */

/*!
 * \ingroup bma490lApiConfiguration
 * \page bma490l_api_bma490l_set_accel_config bma490l_set_accel_config
 * \code
 * int8_t bma490l_set_accel_config(struct bma490l_accel_config *accel, struct bma490l_dev *dev);
 * \endcode
 * @details This API sets the Output Data Rate, bandwidth, perf_mode
 * and range of accel.
 *
 * @param[in] accel : Pointer to structure variable which specifies the
 * accel configurations.
 *
 * @note Enums and corresponding values for structure parameters like
 * ODR, Bandwidth and Range are mentioned in the below tables.
 *
 * @verbatim
 *  Value   |       ODR
 *  --------|-----------------------------------------
 *   1      |   BMA490L_OUTPUT_DATA_RATE_0_78HZ
 *   2      |   BMA490L_OUTPUT_DATA_RATE_1_56HZ
 *   3      |   BMA490L_OUTPUT_DATA_RATE_3_12HZ
 *   4      |   BMA490L_OUTPUT_DATA_RATE_6_25HZ
 *   5      |   BMA490L_OUTPUT_DATA_RATE_12_5HZ
 *   6      |   BMA490L_OUTPUT_DATA_RATE_25HZ
 *   7      |   BMA490L_OUTPUT_DATA_RATE_50HZ
 *   8      |   BMA490L_OUTPUT_DATA_RATE_100HZ
 *   9      |   BMA490L_OUTPUT_DATA_RATE_200HZ
 *   10     |   BMA490L_OUTPUT_DATA_RATE_400HZ
 *   11     |   BMA490L_OUTPUT_DATA_RATE_800HZ
 *   12     |   BMA490L_OUTPUT_DATA_RATE_1600HZ
 *@endverbatim
 *
 *@verbatim
 *  Value |  ACCEL_BANDWIDTH
 *  ------|--------------------------
 *    0   |  BMA490L_ACCEL_OSR4_AVG1
 *    1   |  BMA490L_ACCEL_OSR2_AVG2
 *    2   |  BMA490L_ACCEL_NORMAL_AVG4
 *    3   |  BMA490L_ACCEL_CIC_AVG8
 *    4   |  BMA490L_ACCEL_RES_AVG16
 *    5   |  BMA490L_ACCEL_RES_AVG32
 *    6   |  BMA490L_ACCEL_RES_AVG64
 *    7   |  BMA490L_ACCEL_RES_AVG128
 *@endverbatim
 *
 *@verbatim
 *  Value   | G_RANGE
 *  --------|---------------------
 *  0x00    | BMA490L_ACCEL_RANGE_2G
 *  0x01    | BMA490L_ACCEL_RANGE_4G
 *  0x02    | BMA490L_ACCEL_RANGE_8G
 *  0x03    | BMA490L_ACCEL_RANGE_16G
 *@endverbatim
 *
 * @param[in] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 *
 */
int8_t bma490l_set_accel_config(const struct bma490l_accel_config *accel, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiConfiguration
 * \page bma490l_api_bma490l_get_accel_config bma490l_get_accel_config
 * \code
 * int8_t bma490l_get_accel_config(struct bma490l_accel_config *accel, struct bma490l_dev *dev);
 * \endcode
 * @details This API reads the Output Data Rate, Bandwidth, perf_mode
 * and Range of accel.
 *
 * @param[in,out] accel : Address of user passed structure which is used
 *  to store the Accel configurations read from the sensor.
 *
 * @note Enums and corresponding values for structure parameters like
 * ODR, Bandwidth and Range are mentioned in the below tables.
 *
 *@verbatim
 *  Value      |            ODR
 *  -----------|------------------------------------
 *   1         |     BMA490L_OUTPUT_DATA_RATE_0_78HZ
 *   2         |     BMA490L_OUTPUT_DATA_RATE_1_56HZ
 *   3         |     BMA490L_OUTPUT_DATA_RATE_3_12HZ
 *   4         |     BMA490L_OUTPUT_DATA_RATE_6_25HZ
 *   5         |     BMA490L_OUTPUT_DATA_RATE_12_5HZ
 *   6         |     BMA490L_OUTPUT_DATA_RATE_25HZ
 *   7         |     BMA490L_OUTPUT_DATA_RATE_50HZ
 *   8         |     BMA490L_OUTPUT_DATA_RATE_100HZ
 *   9         |     BMA490L_OUTPUT_DATA_RATE_200HZ
 *   10        |     BMA490L_OUTPUT_DATA_RATE_400HZ
 *   11        |     BMA490L_OUTPUT_DATA_RATE_800HZ
 *   12        |     BMA490L_OUTPUT_DATA_RATE_1600HZ
 *@endverbatim
 *
 *@verbatim
 *  Value |  ACCEL_BANDWIDTH
 *  ------|--------------------------
 *    0   |  BMA490L_ACCEL_OSR4_AVG1
 *    1   |  BMA490L_ACCEL_OSR2_AVG2
 *    2   |  BMA490L_ACCEL_NORMAL_AVG4
 *    3   |  BMA490L_ACCEL_CIC_AVG8
 *    4   |  BMA490L_ACCEL_RES_AVG16
 *    5   |  BMA490L_ACCEL_RES_AVG32
 *    6   |  BMA490L_ACCEL_RES_AVG64
 *    7   |  BMA490L_ACCEL_RES_AVG128
 *@endverbatim
 *
 *@verbatim
 *   Value   | G_RANGE
 *   --------|---------------------
 *   0x00    | BMA490L_ACCEL_RANGE_2G
 *   0x01    | BMA490L_ACCEL_RANGE_4G
 *   0x02    | BMA490L_ACCEL_RANGE_8G
 *   0x03    | BMA490L_ACCEL_RANGE_16G
 *@endverbatim
 *
 * @param[in] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_get_accel_config(struct bma490l_accel_config *accel, struct bma490l_dev *dev);

/**
 * \ingroup bma490l
 * \defgroup bma490lApiConfigId ConfigId
 * @brief Get Configuration ID of the sensor
 */

/*!
 * \ingroup bma490lApiConfigId
 * \page bma490l_api_bma490l_get_config_id bma490l_get_config_id
 * \code
 * int8_t bma490l_get_config_id(uint16_t *config_id, struct bma490l_dev *dev);
 * \endcode
 * @details This API is used to get the configuration id of the sensor.
 *
 * @param[out] config_id    : Pointer variable used to store the configuration id.
 * @param[in] dev           : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_get_config_id(uint16_t *config_id, struct bma490l_dev *dev);

/**
 * \ingroup bma490l
 * \defgroup bma490lApiMapInt Interrupt
 * @brief Map / Unmap user provided interrupt to interrupt pin1 or pin2 of the sensor
 */

/*!
 * \ingroup bma490lApiMapInt
 * \page bma490l_api_bma490l_map_interrupt bma490l_map_interrupt
 * \code
 * int8_t bma490l_map_interrupt(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma490l_dev
 * *dev);
 * \endcode
 * @details This API maps/un-maps the user provided interrupt to either interrupt
 * pin1 or pin2 in the sensor.
 *
 * @param[in] int_line  : Variable to select either interrupt pin1 or pin2.
 *
 * @verbatim
 *  int_line    |   Macros
 *  ------------|-------------------
 *      0       | BMA490L_INTR1_MAP
 *      1       | BMA490L_INTR2_MAP
 * @endverbatim
 *
 * @param[in] int_map   : Variable to specify the interrupts.
 * @param[in] enable    : Variable to specify mapping or un-mapping of
 *                        interrupts.
 *
 * @verbatim
 *  Enable  |   Macros
 *----------|-------------------
 *   0x01   |  BMA490L_ENABLE
 *   0x00   |  BMA490L_DISABLE
 * @endverbatim
 *
 * @param[in] dev      : Structure instance of bma490l_dev.
 *
 * @note Below macros specify the feature interrupts.
 *          - BMA490L_ANY_MOT_INT
 *          - BMA490L_NO_MOT_INT
 *          - BMA490L_ERROR_INT
 *
 * @note Below macros specify the hardware interrupts.
 *          - BMA490L_FIFO_FULL_INT
 *          - BMA490L_FIFO_WM_INT
 *          - BMA490L_DATA_RDY_INT
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_map_interrupt(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiMapInt
 * @brief Read interrupt status of the sensor
 * \page bma490l_api_bma490l_read_int_status bma490l_read_int_status
 * \code
 * int8_t bma490l_read_int_status(uint16_t *int_status, struct bma490l_dev *dev);
 * \endcode
 * @details This API reads the bma490l interrupt status from the sensor.
 *
 * @param[out] int_status   : Variable to store the interrupt status
 *                            read from the sensor.
 * @param[in] dev           : Structure instance of bma490l_dev.
 *
 * @note Below macros specify the feature interrupts.
 *          - BMA490L_ANY_MOT_INT
 *          - BMA490L_NO_MOT_INT
 *          - BMA490L_ERROR_INT
 *
 * @note Below macros specify the hardware interrupts.
 *          - BMA490L_FIFO_FULL_INT
 *          - BMA490L_FIFO_WM_INT
 *          - BMA490L_DATA_RDY_INT
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_read_int_status(uint16_t *int_status, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiConfiguration
 * @brief Set / Get x, y and z axis re-mapping in the sensor
 * \page bma490l_api_bma490l_set_remap_axes bma490l_set_remap_axes
 * \code
 * int8_t bma490l_set_remap_axes(const struct bma490l_axes_remap *remap_data, struct bma490l_dev *dev);
 * \endcode
 * @details This API performs x, y and z axis re-mapping in the sensor.
 *
 * @param[in] remap_data    : Pointer to store axes re-mapping data.
 * @param[in] dev           : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0  -> Fail
 *
 */
int8_t bma490l_set_remap_axes(const struct bma490l_axes_remap *remap_data, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiConfiguration
 * \page bma490l_api_bma490l_get_remap_axes bma490l_get_remap_axes
 * \code
 * int8_t bma490l_get_remap_axes(struct bma490l_axes_remap *remap_data, struct bma490l_dev *dev);
 * \endcode
 * @details This API reads the x, y and z axis re-mapped data from the sensor.
 *
 * @param[out] remap_data   : Pointer to store the read axes re-mapped data.
 * @param[in] dev           : Structure instance of bma490l_dev
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_get_remap_axes(struct bma490l_axes_remap *remap_data, struct bma490l_dev *dev);

/**
 * \ingroup bma490l
 * \defgroup bma490lApiFeatureConfig Feature
 * @brief Functions to describe BMA490L features
 */

/*!
 * \ingroup bma490lApiFeatureConfig
 * @brief Functions of Any-motion feature of the sensor
 * \page bma490l_api_bma490l_set_any_motion_config bma490l_set_any_motion_config
 * \code
 * int8_t bma490l_set_any_motion_config(const struct bma490l_anymotion_config *any_motion, struct
 * bma490l_dev *dev);
 * \endcode
 * @details This API sets the configuration of any-motion feature in the sensor.
 * This API enables/disables the any-motion feature according to the axis set.
 *
 * @param[in] any_mot           : Pointer to structure variable to configure
 *                                any-motion.
 *
 * @verbatim
 * -------------------------------------------------------------------------
 *         Structure parameters    |        Description
 * --------------------------------|----------------------------------------
 *                                 |        Defines the number of
 *                                 |        consecutive data points for
 *                                 |        which the threshold condition
 *         duration                |        must be respected, for interrupt
 *                                 |        assertion. It is expressed in
 *                                 |        50 Hz samples (20 ms).
 *                                 |        Range is 0 to 163sec.
 *                                 |        Default value is 5 = 100ms.
 * --------------------------------|----------------------------------------
 *                                 |        Slope threshold  for
 *                                 |        Any-motion detection
 *         threshold               |        in 5.11g format.
 *                                 |        Range is 0 to 1g.
 *                                 |        Default value is 0xAA = 83mg.
 * --------------------------------|----------------------------------------
 *                                 |        Enables the feature on a per-axis
 *         axis_en                 |        basis.
 * ---------------------------------------------------------------------------
 * @endverbatim
 *
 * @verbatim
 *  Value    |  axis_en
 *  ---------|-------------------------
 *  0x00     |  BMA490L_DIS_ALL_AXIS
 *  0x01     |  BMA490L_X_AXIS_EN
 *  0x02     |  BMA490L_Y_AXIS_EN
 *  0x04     |  BMA490L_Z_AXIS_EN
 *  0x07     |  BMA490L_EN_ALL_AXIS
 * @endverbatim
 *
 * @param[in] dev               : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_set_any_mot_config(const struct bma490l_any_no_mot_config *any_mot, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiFeatureConfig
 * \page bma490l_api_bma490l_get_any_motion_config bma490l_get_any_motion_config
 * \code
 * int8_t bma490l_get_any_motion_config(struct bma490l_anymotion_config *any_motion, struct bma490l_dev
 * *dev);
 * \endcode
 * @details This API gets the configuration of any-motion feature from the
 * sensor.
 *
 * @param[out] any_mot        : Pointer to structure variable to configure
 *                              any-motion.
 *
 * @verbatim
 * -------------------------------------------------------------------------
 *         Structure parameters    |        Description
 * --------------------------------|----------------------------------------
 *                                 |        Defines the number of
 *                                 |        consecutive data points for
 *                                 |        which the threshold condition
 *         duration                |        must be respected, for interrupt
 *                                 |        assertion. It is expressed in
 *                                 |        50 Hz samples (20 ms).
 *                                 |        Range is 0 to 163sec.
 *                                 |        Default value is 5 = 100ms.
 * --------------------------------|----------------------------------------
 *                                 |        Slope threshold  for
 *                                 |        Any-motion detection
 *         threshold               |        in 5.11g format.
 *                                 |        Range is 0 to 1g.
 *                                 |        Default value is 0xAA = 83mg.
 * --------------------------------|-----------------------------------------
 *                                 |        Enables the feature on a per-axis
 *         axis_en                 |        basis.
 * ---------------------------------------------------------------------------
 * @endverbatim
 *
 * @verbatim
 *  Value    |  axis_en
 *  ---------|-------------------------
 *  0x00     |  BMA490L_DIS_ALL_AXIS
 *  0x01     |  BMA490L_X_AXIS_EN
 *  0x02     |  BMA490L_Y_AXIS_EN
 *  0x04     |  BMA490L_Z_AXIS_EN
 *  0x07     |  BMA490L_EN_ALL_AXIS
 * @endverbatim
 *
 * @param[in] dev               : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_get_any_mot_config(struct bma490l_any_no_mot_config *any_mot, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiFeatureConfig
 * \page bma490l_api_bma490l_set_no_motion_config bma490l_set_no_motion_config
 * \code
 * int8_t bma490l_set_no_motion_config(const struct bma490l_nomotion_config *no_motion, struct
 * bma490l_dev *dev);
 * \endcode
 * @details This API sets the configuration of no-motion feature in the sensor.
 * This API enables/disables the no-motion feature according to the axis set.
 *
 * @param[in] no_mot                : Pointer to structure variable to configure
 *                                    no-motion.
 *
 * @verbatim
 * -------------------------------------------------------------------------
 *         Structure parameters    |        Description
 * --------------------------------|----------------------------------------
 *                                 |        Defines the number of
 *                                 |        consecutive data points for
 *                                 |        which the threshold condition
 *         duration                |        must be respected, for interrupt
 *                                 |        assertion. It is expressed in
 *                                 |        50 Hz samples (20 ms).
 *                                 |        Range is 0 to 163sec.
 *                                 |        Default value is 5 = 100ms.
 * --------------------------------|----------------------------------------
 *                                 |        Slope threshold  for
 *                                 |        No-motion detection
 *         threshold               |        in 5.11g format.
 *                                 |        Range is 0 to 1g.
 *                                 |        Default value is 0xAA = 83mg.
 * --------------------------------|----------------------------------------
 *                                 |        Enables the feature on a per-axis
 *         axis_en                 |        basis.
 * ---------------------------------------------------------------------------
 * @endverbatim
 *
 * @verbatim
 *  Value    |  axis_en
 *  ---------|-------------------------
 *  0x00     |  BMA490L_DIS_ALL_AXIS
 *  0x01     |  BMA490L_X_AXIS_EN
 *  0x02     |  BMA490L_Y_AXIS_EN
 *  0x04     |  BMA490L_Z_AXIS_EN
 *  0x07     |  BMA490L_EN_ALL_AXIS
 * @endverbatim
 *
 * @param[in] dev               : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_set_no_mot_config(const struct bma490l_any_no_mot_config *no_mot, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiFeatureConfig
 * \page bma490l_api_bma490l_get_no_motion_config bma490l_get_no_motion_config
 * \code
 * int8_t bma490l_get_no_motion_config(struct bma490l_nomotion_config *no_motion, struct bma490l_dev
 * *dev);
 * \endcode
 * @details This API gets the configuration of no-motion feature from the
 * sensor.
 *
 * @param[out] no_mot        : Pointer to structure variable to configure
 *                             no-motion.
 *
 * @verbatim
 * -------------------------------------------------------------------------
 *         Structure parameters    |        Description
 * --------------------------------|----------------------------------------
 *                                 |        Defines the number of
 *                                 |        consecutive data points for
 *                                 |        which the threshold condition
 *         duration                |        must be respected, for interrupt
 *                                 |        assertion. It is expressed in
 *                                 |        50 Hz samples (20 ms).
 *                                 |        Range is 0 to 163sec.
 *                                 |        Default value is 5 = 100ms.
 * --------------------------------|----------------------------------------
 *                                 |        Slope threshold  for
 *                                 |        No-motion detection
 *         threshold               |        in 5.11g format.
 *                                 |        Range is 0 to 1g.
 *                                 |        Default value is 0xAA = 83mg.
 * --------------------------------|-----------------------------------------
 *                                 |        Enables the feature on a per-axis
 *         axis_en                 |        basis.
 * ---------------------------------------------------------------------------
 * @endverbatim
 *
 * @verbatim
 *  Value    |  axis_en
 *  ---------|-------------------------
 *  0x00     |  BMA490L_DIS_ALL_AXIS
 *  0x01     |  BMA490L_X_AXIS_EN
 *  0x02     |  BMA490L_Y_AXIS_EN
 *  0x04     |  BMA490L_Z_AXIS_EN
 *  0x07     |  BMA490L_EN_ALL_AXIS
 * @endverbatim
 *
 * @param[in] dev               : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_get_no_mot_config(struct bma490l_any_no_mot_config *no_mot, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiConfigId
 * @brief This API is used to get the config file major and minor information.
 * \page bma490l_api_bma490l_get_version_config bma490l_get_version_config
 * \code
 * int8_t bma490l_get_version_config(uint16_t *config_major, uint16_t *config_minor, struct bma490l_dev
 * *dev);
 * \endcode
 * @details This API reads the the config file major and minor information.
 *
 * @param[in] dev   : Structure instance of bma490l_dev.
 * @param[out] config_major    : Pointer to data buffer to store the config major.
 * @param[out] config_minor    : Pointer to data buffer to store the config minor.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_get_version_config(uint16_t *config_major, uint16_t *config_minor, struct bma490l_dev *dev);

/**
 * \ingroup bma490l
 * \defgroup bma490lApiFIFO FIFO
 * @brief FIFO Configuration.
 */

/*!
 * \ingroup bma490lApiFIFO
 * @brief Read sensor time of the sensor
 * \page bma490l_api_bma490l_get_sensor_time bma490l_get_sensor_time
 * \code
 * int8_t bma490l_get_sensor_time(uint32_t *sensor_time, struct bma490l_dev *dev);
 * \endcode
 * @details This API reads the sensor time when sensor time gets updated
 *  with every update of data register or FIFO.
 *
 * @param[in] sensor_time : Pointer variable which stores sensor time.
 * @param[in] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_get_sensor_time(uint32_t *sensor_time, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiReadFIFO
 * @brief Read FIFO data of accel and/or Mag sensor
 * \page bma490l_api_bma490l_read_fifo_data bma490l_read_fifo_data
 * \code
 * int8_t bma490l_read_fifo_data(struct bma490l_fifo_frame *fifo, struct bma490l_dev *dev);
 * \endcode
 * @details This API reads the FIFO data of Accel and/or Mag sensor
 *
 * @param[in] fifo : Structure instance of bma490l_fifo_frame.
 * @param dev  : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_read_fifo_data(struct bma490l_fifo_frame *fifo, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiFIFO
 * @brief Parse and Extract Accel frames from FIFO data read
 * \page bma490l_api_bma490l_extract_accel bma490l_extract_accel
 * \code
 * int8_t bma490l_extract_accel(struct bma490l_accel *accel_data, uint16_t *accel_length, struct
 * bma490l_fifo_frame
 **fifo,
 * const struct bma490l_dev *dev);
 * \endcode
 * @details This API parses and extracts the accelerometer frames from
 *  FIFO data read by the "bma490l_read_fifo_data" API and stores it in the
 *  "accel_data" structure instance.
 *
 *  @note The bma490l_extract_accel API should be called only after reading
 *  the FIFO data by calling the bma490l_read_fifo_data() API
 *
 *  @param[in,out] accel_data   : Structure instance of bma490l_accel where
 *   the accelerometer data in FIFO is stored.
 *  @param[in,out] accel_length : Number of accelerometer frames.
 *   (x,y,z axes data)
 *  @param[in,out] fifo         : Structure instance of bma490l_fifo_frame.
 *  @param[in,out] dev          : Structure instance of bma490l_dev.
 *
 *  @note The accel_length has number of accelerometer frames
 *  (1 accel frame   = 6 bytes) which the user needs to extract and store is
 *  provided as input parameter by the user and the number of valid
 *  accelerometer frames extracted and stored is updated in
 *  "accel_length" at the end of execution of this API.
 *
 *  @return Result of API execution status.
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 *
 */
int8_t bma490l_extract_accel(struct bma490l_accel *accel_data,
                             uint16_t *accel_length,
                             struct bma490l_fifo_frame *fifo,
                             const struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiFIFO
 * @brief Read / Write FIFO watermark level in the sensor
 * \page bma490l_api_bma490l_set_fifo_wm bma490l_set_fifo_wm
 * \code
 * int8_t bma490l_set_fifo_wm(uint16_t fifo_wm, struct bma490l_dev *dev);
 * \endcode
 * @details This API sets the FIFO watermark level in the sensor.
 *
 * @note The FIFO watermark is issued when the FIFO fill level is
 * equal or above the watermark level.
 *
 * @param[in]  fifo_wm : Variable used to set the FIFO water mark level
 * @param[out] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma490l_set_fifo_wm(uint16_t fifo_wm, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiFIFO
 * \page bma490l_api_bma490l_get_fifo_wm bma490l_get_fifo_wm
 * \code
 * int8_t bma490l_get_fifo_wm(uint16_t *fifo_wm, struct bma490l_dev *dev);
 * \endcode
 * @details This API reads the FIFO water mark level which is set
 * in the sensor.
 *
 * @note The FIFO watermark is issued when the FIFO fill level is
 * equal or above the watermark level.
 *
 * @param[out]  fifo_wm : Pointer variable to store FIFO water mark level.
 * @param[in] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_get_fifo_wm(uint16_t *fifo_wm, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiFIFO
 * @brief Set / Get Filtered or unfiltered mode of Accel FIFO data
 * \page bma490l_api_bma490l_set_accel_fifo_filter_data bma490l_set_accel_fifo_filter_data
 * \code
 * int8_t bma490l_set_accel_fifo_filter_data(uint8_t accel_fifo_filter, struct bma490l_dev *dev);
 * \endcode
 * @details This API sets the condition of Accel FIFO data either to
 * filtered or unfiltered mode.
 *
 * @param[in] accel_fifo_filter : Variable used to set the filtered or
 * unfiltered condition of Accel FIFO data.
 *
 *@verbatim
 *  Value      |  accel_fifo_filter_data
 *  -----------|-------------------------
 *    0x00     |  Unfiltered data
 *    0x01     |  Filtered data
 *@endverbatim
 *
 * @param[out] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma490l_set_accel_fifo_filter_data(uint8_t accel_fifo_filter, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiFIFO
 * \page bma490l_api_bma490l_get_accel_fifo_filter_data bma490l_get_accel_fifo_filter_data
 * \code
 * int8_t bma490l_get_accel_fifo_filter_data(uint8_t *accel_fifo_filter, struct bma490l_dev *dev);
 * \endcode
 * @details This API checks whether the Accel FIFO data is set for filtered
 * or unfiltered mode.
 *
 * @param[out] accel_fifo_filter : Variable used to check whether the Accel
 * data is filtered or unfiltered.
 *
 *@verbatim
 *  Value    |  accel_fifo_filter
 *  ---------|-------------------------
 *  0x00     |  Unfiltered data
 *  0x01     |  Filtered data
 *@endverbatim
 *
 * @param[in] dev : structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_get_accel_fifo_filter_data(uint8_t *accel_fifo_filter, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiFIFO
 * @brief Read / Write Down sampling rates configured for Accel FIFO Data
 * \page bma490l_api_bma490l_set_fifo_down_accel bma490l_set_fifo_down_accel
 * \code
 * int8_t bma490l_set_fifo_down_accel(uint8_t fifo_down, struct bma490l_dev *dev);
 * \endcode
 * @details This API sets the down-sampling rates for Accel FIFO.
 *
 * @param[in] fifo_down : Variable used to specify the Accel FIFO
 *                        down-sampling rates.
 * @param[in] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma490l_set_fifo_down_accel(uint8_t fifo_down, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiFIFO
 * \page bma490l_api_bma490l_get_fifo_down_accel bma490l_get_fifo_down_accel
 * \code
 * int8_t bma490l_get_fifo_down_accel(uint8_t *fifo_down, struct bma490l_dev *dev);
 * \endcode
 * @details This API reads the down sampling rates which is configured
 * for Accel FIFO data.
 *
 * @param[out] fifo_down : Variable used to specify the Accel FIFO
 *                         down-sampling rates.
 * @param[in] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_get_fifo_down_accel(uint8_t *fifo_down, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiFIFO
 * @brief Read length of FIFO data available in the sensor (unit of bytes)
 * \page bma490l_api_bma490l_get_fifo_length bma490l_get_fifo_length
 * \code
 * int8_t bma490l_get_fifo_length(uint16_t *fifo_length, struct bma490l_dev *dev);
 * \endcode
 * @details This API reads the length of FIFO data available in the sensor
 * in the units of bytes.
 *
 * @note This byte counter is updated each time a complete frame was read
 * or written.
 *
 * @param[in] fifo_length : Pointer variable used to store the value of
 *                          FIFO byte counter.
 * @param[in] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 *
 */
int8_t bma490l_get_fifo_length(uint16_t *fifo_length, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiFIFO
 * @brief Set / Get FIFO Configuration in the sensor
 * \page bma490l_api_bma490l_set_fifo_config bma490l_set_fifo_config
 * \code
 * int8_t bma490l_set_fifo_config(uint8_t config, uint8_t enable, struct bma490l_dev *dev);
 * \endcode
 * @details This API sets the FIFO configurations in the sensor.
 *
 *  @param[in] config : Enum variable used to specify the FIFO
 *  configurations which are to be enabled or disabled in the sensor.
 *
 *  @note : User can set either one or more or all FIFO configurations
 *  by doing OR operation in the below mentioned enums.
 *
 *@verbatim
 *      Config                     |   Value
 *      ---------------------------|---------------------------
 *      BMA490L_FIFO_STOP_ON_FULL  |   0x01
 *      BMA490L_FIFO_TIME          |   0x02
 *      BMA490L_FIFO_TAG_INTR2     |   0x04
 *      BMA490L_FIFO_TAG_INTR1     |   0x08
 *      BMA490L_FIFO_HEADER        |   0x10
 *      BMA490L_FIFO_MAG           |   0x20
 *      BMA490L_FIFO_ACCEL         |   0x40
 *      BMA490L_FIFO_ALL           |   0x7F
 *@endverbatim
 *
 *  @param[in] enable : Parameter used to enable or disable the above
 *                      FIFO configuration.
 *  @param[in] dev : Structure instance of bma490l_dev.
 *
 *  @return Status of bus communication result
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 *
 */
int8_t bma490l_set_fifo_config(uint8_t config, uint8_t enable, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiFIFO
 * \page bma490l_api_bma490l_get_fifo_config bma490l_get_fifo_config
 * \code
 * int8_t bma490l_get_fifo_config(uint8_t *fifo_config, struct bma490l_dev *dev);
 * \endcode
 * @details This API reads the FIFO configurations from the sensor.
 *
 *  @param[in] fifo_config : Enum variable used to get the below fifo
 *                           configuration from the sensor.
 *
 *  @note After calling this function user should do the AND operation with
 *  the enum value populated by this function to know which FIFO
 *  configuration is enabled.
 *
 *@verbatim
 *      fifo_config                 |       Value
 *      ----------------------------|--------------------------
 *      BMA490L_FIFO_STOP_ON_FULL   |       0x01
 *      BMA490L_FIFO_TIME           |       0x02
 *      BMA490L_FIFO_TAG_INTR2      |       0x04
 *      BMA490L_FIFO_TAG_INTR1      |       0x08
 *      BMA490L_FIFO_HEADER         |       0x10
 *      BMA490L_FIFO_MAG            |       0x20
 *      BMA490L_FIFO_ACCEL          |       0x40
 *      BMA490L_FIFO_ALL            |       0x7F
 *@endverbatim
 *
 *  @param[in] dev : Structure instance of bma490l_dev.
 *
 *  @return Status of bus communication function.
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 *
 */
int8_t bma490l_get_fifo_config(uint8_t *fifo_config, struct bma490l_dev *dev);

/**
 * \ingroup bma490l
 * \defgroup bma490lApiErrorStatus Error Status
 * @brief Read error status of the sensor
 */

/*!
 * \ingroup bma490lApiErrorStatus
 * \page bma490l_api_bma490l_get_error_status bma490l_get_error_status
 * \code
 * int8_t bma490l_get_error_status(struct bma490l_err_reg *err_reg, struct bma490l_dev *dev);;
 * \endcode
 * @details This API reads the error status from the sensor.
 *
 * Below table mention the types of error which can occur in the sensor.
 *
 *@verbatim
 *************************************************************************
 *        Error           |       Description
 *************************|***********************************************
 *                        |       Fatal Error, chip is not in operational
 *        fatal           |       state (Boot-, power-system).
 *                        |       This flag will be reset only by
 *                        |       power-on-reset or soft-reset.
 *************************|***********************************************
 *        CMD             |       Command execution failed.
 *************************|***********************************************
 *                        |       Value      Name        Description
 *        error_code      |       000        no_error    no error
 *                        |       001        acc_err     error in
 *                        |                              ACC_CONF
 *************************|***********************************************
 *                        |       Error in FIFO detected: Input data was
 *        FIFO            |       discarded in stream mode. This flag
 *                        |       will be reset when read.
 *************************|***********************************************
 *@endverbatim
 *
 * @param[in,out] err_reg : Pointer to structure variable which stores the
 *                          error status read from the sensor.
 * @param[in] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_get_error_status(struct bma490l_err_reg *err_reg, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiFIFO
 * @brief Read / Write FIFO self wake up functionality in the sensor
 * \page bma490l_api_bma490l_set_fifo_self_wakeup bma490l_set_fifo_self_wakeup
 * \code
 * int8_t bma490l_set_fifo_self_wakeup(uint8_t fifo_self_wakeup, struct bma490l_dev *dev);
 * \endcode
 * @details This API sets the FIFO self wake up functionality in the sensor.
 *
 * @note Functionality related to FIFO self wake up depends upon the
 * advance power save mode. For more information refer data sheet.
 *
 * @param[in] fifo_self_wakeup : Variable used to enable or disable
 *                               FIFO self wake up functionality.
 * @param[in] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_set_fifo_self_wakeup(uint8_t fifo_self_wakeup, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiFIFO
 * \page bma490l_api_bma490l_get_fifo_self_wakeup bma490l_get_fifo_self_wakeup
 * \code
 * int8_t bma490l_get_fifo_self_wakeup(uint8_t *fifo_self_wakeup, struct bma490l_dev *dev);
 * \endcode
 * @details This API gets the status of FIFO self wake up functionality from
 * the sensor.
 *
 * @note Functionality related to FIFO self wake up depends upon the
 * advance power save mode. For more information refer data sheet.
 *
 * @param[out] fifo_self_wake_up : Pointer variable used to store the
 *                                 fifo self wake up status.
 * @param[in] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status.
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma490l_get_fifo_self_wakeup(uint8_t *fifo_self_wake_up, struct bma490l_dev *dev);

/**
 * \ingroup bma490l
 * \defgroup bma490lApiAccelxyz Data read out
 * @brief Read accel xyz data from the sensor
 */

/*!
 * \ingroup bma490lApiAccelxyz
 * \page bma490l_api_bma490l_read_accel_xyz bma490l_read_accel_xyz
 * \code
 * int8_t bma490l_read_accel_xyz(struct bma490l_accel *accel, struct bma490l_dev *dev);
 * \endcode
 * @details This API reads the Accel data for x,y and z-axis from the sensor.
 *  The data units is in LSB format.
 *
 * @param[in] accel : Variable used to store the Accel data which is read
 *                    from the sensor.
 * @param[in] dev : Structure instance of bma490l_dev.
 *
 * @note For setting the Accel configuration, use the below function
 * bma490l_set_accel_config.
 *
 * @return Result of API execution status
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_read_accel_xyz(struct bma490l_accel *accel, struct bma490l_dev *dev);

/**
 * \ingroup bma490l
 * \defgroup bma490lApiSystem System
 */

/*!
 * \ingroup bma490lApiSystem
 * @brief Self-test for accel
 * \page bma490l_api_bma490l_perform_accel_selftest bma490l_perform_accel_selftest
 * \code
 * int8_t bma490l_perform_accel_selftest(struct bma490l_dev *dev);
 * \endcode
 * @details This API checks whether the self-test functionality of the sensor
 *  is working or not.
 *
 *  @param[in] result : Pointer variable used to store the result of self-test
 *  operation.
 *
 *@verbatim
 *  Result   |  Description
 *  ---------|--------------------
 *  0x00     |  BMA4_SELFTEST_PASS
 *  0x01     |  BMA4_SELFTEST_FAIL
 *@endverbatim
 *
 *  @param[in] dev : Structure instance of bma490l_dev.
 *
 *  @return Result of self-test.
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
int8_t bma490l_perform_accel_selftest(struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiSystem
 * \page bma490l_api_bma490l_selftest_config bma490l_selftest_config
 * \code
 * int8_t bma490l_selftest_config(uint8_t sign, struct bma490l_dev *dev);
 * \endcode
 * @details This API performs the steps needed for self-test operation
 *  before reading the Accel Self-test data.
 *
 *  @param[in] sign : Variable used to specify the self-test sign.
 *  @param[in] dev  : Structure instance of bma490l_dev.
 *
 *  @return Result of self-test.
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
int8_t bma490l_selftest_config(uint8_t sign, struct bma490l_dev *dev);

/**
 * \ingroup bma490l
 * \defgroup bma490lApiStatus Status
 * @brief Read sensor status
 */

/*!
 * \ingroup bma490lApiStatus
 * \page bma490l_api_bma4_get_status bma4_get_status
 * \code
 * int8_t bma490l_get_status(uint8_t *status, struct bma4_dev *dev);
 * \endcode
 * @details This API reads the sensor status from the dev sensor.
 *
 * Below table lists the sensor status flags
 *
 * @verbatim
 *                 Status          |       Description
 *     ----------------------------|----------------------------------------
 *     BMA4_MAG_MAN_OP_ONGOING     | Manual Mag. interface operation ongoing
 *     BMA4_CMD_RDY                | Command decoder is ready.
 *     BMA4_MAG_DATA_RDY           | Data ready for Mag.
 *     BMA4_ACC_DATA_RDY           | Data ready for Accel.
 *@endverbatim
 *
 * @param[in] status : Variable used to store the sensor status flags
 * which is read from the sensor.
 * @param[in] dev : Structure instance of bma4_dev.
 *
 * @return Result of API execution status
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bma490l_get_status(uint8_t *status, struct bma490l_dev *dev);

/**
 * \ingroup bma490l
 * \defgroup bma490lApiAccelFoc Accel FOC
 * @brief Performs Fast Offset Compensation for accel
 */

/*!
 * \ingroup bma490lApiAccelFoc
 * \page bma490l_api_bma490l_perform_accel_foc bma490l_perform_accel_foc
 * \code
 * int8_t bma490l_perform_accel_foc(const int32_t accel_g_value[3], struct bma490l_dev *dev);
 * \endcode
 * @details This API performs Fast Offset Compensation for Accel.
 * @param[in] accel_g_value : Array which stores the Accel g units
 *  for x,y and z-axis.
 *
 *@verbatim
 *      accel_g_value             |   Description
 *      --------------------------|---------------------------------------
 *      accel_g_value[0]          |   x-axis g units
 *      accel_g_value[1]          |   y-axis g units
 *      accel_g_value[2]          |   z-axis g units
 *@endverbatim
 *
 *  @param[in] dev : Structure instance of dev.
 *
 *  @return Result of API execution status.
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
int8_t bma490l_perform_accel_foc(const struct bma490l_accel_foc_g_value *accel_g_value, struct bma490l_dev *dev);

/*!
 * \ingroup bma490lApiSystem
 * \page bma490l_api_bma490l_soft_reset bma490l_soft_reset
 * \code
 * int8_t bma490l_soft_reset(struct bma490l_dev *dev);
 * \endcode
 * @details This API commands to do soft-reset.
 *
 *  @param[in] dev : Structure instance of bma490l_dev.
 *
 *  @return Result of API execution status.
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
int8_t bma490l_soft_reset(struct bma490l_dev *dev);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* End of header guard macro */
