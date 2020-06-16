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
 * @file       bma490l_defs.h
 * @date       2020-04-08
 * @version    v0.2.3
 *
 */

/** @file bma490l_defs.h
 * \brief Sensor Driver for BMA490L sensor
 */
#ifndef BMA490L_DEFS_H__
#define BMA490L_DEFS_H__

/*********************************************************************/
/**\ header files */
#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#endif

/*********************************************************************/
/* Macro definitions */

#ifdef __KERNEL__

#if (!defined(UINT8_C) && !defined(INT8_C))
#define INT8_C(x)                                    S8_C(x)
#define UINT8_C(x)                                   U8_C(x)
#endif

#if (!defined(UINT16_C) && !defined(INT16_C))
#define INT16_C(x)                                   S16_C(x)
#define UINT16_C(x)                                  U16_C(x)
#endif

#if (!defined(INT32_C) && !defined(UINT32_C))
#define INT32_C(x)                                   S32_C(x)
#define UINT32_C(x)                                  U32_C(x)
#endif

#if (!defined(INT64_C) && !defined(UINT64_C))
#define INT64_C(x)                                   S64_C(x)
#define UINT64_C(x)                                  U64_C(x)
#endif

#else /* __KERNEL__ */

#if (!defined(UINT8_C) && !defined(INT8_C))
#define INT8_C(x)                                    (x)
#define UINT8_C(x)                                   (x##U)
#endif

#if (!defined(UINT16_C) && !defined(INT16_C))
#define INT16_C(x)                                   (x)
#define UINT16_C(x)                                  (x##U)
#endif

#if (!defined(INT32_C) && !defined(UINT32_C))
#define INT32_C(x)                                   (x)
#define UINT32_C(x)                                  (x##U)
#endif

#if (!defined(INT64_C) && !defined(UINT64_C))
#define INT64_C(x)                                   (x##LL)
#define UINT64_C(x)                                  (x##ULL)
#endif

#endif /* __KERNEL__ */

/**
 * BMA490L_INTF_RET_TYPE is the read/write interface return type which can be overwritten by the build system.
 */
#ifndef BMA490L_INTF_RET_TYPE
#define BMA490L_INTF_RET_TYPE                        int8_t
#endif

/**
 * The last error code from read/write interface is stored in the device structure as intf_rslt.
 */
#ifndef BMA490L_INTF_RET_SUCCESS
#define BMA490L_INTF_RET_SUCCESS                     INT8_C(0)
#endif

/**\name CHIP ID ADDRESS */
#define BMA490L_CHIP_ID_ADDR                         UINT8_C(0x00)

/**\name STATUS ADDRESS */
#define BMA490L_STATUS_ADDR                          UINT8_C(0x03)

/**\name ERROR STATUS */
#define BMA490L_ERROR_ADDR                           UINT8_C(0x02)

/*! @name Chip ID of BMA490L sensor */
#define BMA490L_CHIP_ID                              UINT8_C(0x1A)

/*! @name Sensor feature size */
#define BMA490L_FEATURE_SIZE                         UINT8_C(0x0C)

/*! @name Data Lengths */
#define BMA490L_RD_WR_MIN_LEN                        UINT8_C(0x02)

/*! @name Maximum valid read write length is 1.3kb since config file size is 1.3kb */
#define BMA490L_RD_WR_MAX_LEN                        UINT16_C(0x514)

/*! @name Feature start address */
#define BMA490L_ANY_MOT_START_ADDR                   UINT8_C(0x00)
#define BMA490L_NO_MOT_START_ADDR                    UINT8_C(0x04)
#define BMA490L_CONFIG_ID_START_ADDR                 UINT8_C(0x08)
#define BMA490L_AXES_REMAP_START_ADDR                UINT8_C(0x0A)

/*! @name Mask definitions for axes re-mapping */
#define BMA490L_X_AXIS_MSK                           UINT8_C(0x03)
#define BMA490L_X_AXIS_SIGN_MSK                      UINT8_C(0x04)
#define BMA490L_Y_AXIS_MSK                           UINT8_C(0x18)
#define BMA490L_Y_AXIS_SIGN_MSK                      UINT8_C(0x20)
#define BMA490L_Z_AXIS_MSK                           UINT8_C(0xC0)
#define BMA490L_Z_AXIS_SIGN_MSK                      UINT8_C(0x01)

/*! @name Bit position for axes re-mapping */
#define BMA490L_X_AXIS_SIGN_POS                      UINT8_C(0x02)
#define BMA490L_Y_AXIS_POS                           UINT8_C(0x03)
#define BMA490L_Y_AXIS_SIGN_POS                      UINT8_C(0x05)
#define BMA490L_Z_AXIS_POS                           UINT8_C(0x06)

/*! @name Any/No-motion threshold macros */
#define BMA490L_ANY_NO_MOT_THRES_MSK                 UINT16_C(0x07FF)

/*! @name Any/No-motion enable macros */
#define BMA490L_ANY_NO_MOT_AXIS_EN_MSK               UINT16_C(0xE000)
#define BMA490L_ANY_NO_MOT_AXIS_EN_POS               UINT8_C(0x0D)

/*! @name Any/No-motion duration macros */
#define BMA490L_ANY_NO_MOT_DUR_MSK                   UINT16_C(0x1FFF)

/*! @name Any/No-motion axis enable macros */
#define BMA490L_X_AXIS_EN                            UINT8_C(0x01)
#define BMA490L_Y_AXIS_EN                            UINT8_C(0x02)
#define BMA490L_Z_AXIS_EN                            UINT8_C(0x04)
#define BMA490L_EN_ALL_AXIS                          UINT8_C(0x07)
#define BMA490L_DIS_ALL_AXIS                         UINT8_C(0x00)

/*! @name Interrupt status macros */
#define BMA490L_ANY_MOT_INT                          UINT8_C(0x20)
#define BMA490L_NO_MOT_INT                           UINT8_C(0x40)
#define BMA490L_ERROR_INT                            UINT8_C(0x80)

/*! @name Mask definitions for major and minor configuration */
#define BMA490L_CONFIG_MAJOR_MSK                     UINT16_C(0x3C0)
#define BMA490L_CONFIG_MINOR_MSK                     UINT8_C(0x1F)

/*! @name Bit position for major configuration */
#define BMA490L_CONFIG_MAJOR_POS                     UINT8_C(0x06)

/*! @name Accel enable and data ready macros */
#define BMA490L_ACCEL_EN                             UINT8_C(0x40)
#define BMA490L_ACCEL_DRDY                           UINT8_C(0x80)

/*! @name FOC limit macro */
#define BMA490L_FOC_SAMPLE_LIMIT                     UINT8_C(128)
#define BMA490L_MAX_NOISE_LIMIT(RANGE_VALUE)         (RANGE_VALUE + UINT16_C(255))
#define BMA490L_MIN_NOISE_LIMIT(RANGE_VALUE)         (RANGE_VALUE - UINT16_C(255))

/**\name MAP INTERRUPT 1 and 2 REGISTERS */
#define BMA490L_INT_MAP_1_ADDR                       UINT8_C(0x56)
#define BMA490L_INT_MAP_2_ADDR                       UINT8_C(0x57)
#define BMA490L_INT_MAP_DATA_ADDR                    UINT8_C(0x58)
#define BMA490L_INIT_CTRL_ADDR                       UINT8_C(0x59)

/**\name FEATURE CONFIG RELATED */
#define BMA490L_RESERVED_REG_5B_ADDR                 UINT8_C(0x5B)
#define BMA490L_RESERVED_REG_5C_ADDR                 UINT8_C(0x5C)
#define BMA490L_FEATURE_CONFIG_ADDR                  UINT8_C(0x5E)
#define BMA490L_INTERNAL_ERROR                       UINT8_C(0x5F)

/**\name I2C slave address */
#define BMA490L_I2C_ADDR_PRIMARY                     UINT8_C(0x18)
#define BMA490L_I2C_ADDR_SECONDARY                   UINT8_C(0x19)

/**\name Interface selection macro */
#define BMA490L_SPI_WR_MASK                          UINT8_C(0x7F)
#define BMA490L_SPI_RD_MASK                          UINT8_C(0x80)

/**\name ENABLE/DISABLE BIT VALUES */
#define BMA490L_ENABLE                               UINT8_C(0x01)
#define BMA490L_DISABLE                              UINT8_C(0x00)

/**\name ACCEL PERFORMANCE MODE PARAMETER */
#define BMA490L_CIC_AVG_MODE                         UINT8_C(0)
#define BMA490L_CONTINUOUS_MODE                      UINT8_C(1)

/**\name POWER_CTRL REGISTER */
#define BMA490L_POWER_CONF_ADDR                      UINT8_C(0x7C)
#define BMA490L_POWER_CTRL_ADDR                      UINT8_C(0x7D)

/**\name ADVANCE POWER SAVE POSITION AND MASK */
#define BMA490L_ADVANCE_POWER_SAVE_MSK               UINT8_C(0x01)

/**\name ERROR CODES */
#define BMA490L_OK                                   INT8_C(0)
#define BMA490L_E_NULL_PTR                           INT8_C(-1)
#define BMA490L_E_COM_FAIL                           INT8_C(-2)
#define BMA490L_E_OUT_OF_RANGE                       INT8_C(-3)
#define BMA490L_E_INVALID_SENSOR                     INT8_C(-4)
#define BMA490L_E_CONFIG_STREAM_ERROR                INT8_C(-5)
#define BMA490L_E_SELF_TEST_FAIL                     INT8_C(-6)
#define BMA490L_E_FOC_FAIL                           INT8_C(-7)
#define BMA490L_E_FAIL                               INT8_C(-8)
#define BMA490L_E_INT_LINE_INVALID                   INT8_C(-9)
#define BMA490L_E_RD_WR_LENGTH_INVALID               INT8_C(-10)
#define BMA490L_E_AUX_CONFIG_FAIL                    INT8_C(-11)
#define BMA490L_E_SC_FIFO_HEADER_ERR                 INT8_C(-12)
#define BMA490L_E_SC_FIFO_CONFIG_ERR                 INT8_C(-13)

/**\name INTERRUPT MASKS */
#define BMA490L_FIFO_FULL_INT                        UINT16_C(0x0100)
#define BMA490L_FIFO_WM_INT                          UINT16_C(0x0200)
#define BMA490L_DATA_RDY_INT                         UINT16_C(0x0400)
#define BMA490L_MAG_DATA_RDY_INT                     UINT16_C(0x2000)
#define BMA490L_ACCEL_DATA_RDY_INT                   UINT16_C(0x8000)

/**\name INTERRUPT MAPS */
#define BMA490L_INTR1_MAP                            UINT8_C(0)
#define BMA490L_INTR2_MAP                            UINT8_C(1)

/**\name UTILITY MACROS */
#define BMA490L_SET_LOW_BYTE                         UINT16_C(0x00FF)
#define BMA490L_SET_HIGH_BYTE                        UINT16_C(0xFF00)
#define BMA490L_SET_LOW_NIBBLE                       UINT8_C(0x0F)

/**\name COMMAND REGISTER */
#define BMA490L_CMD_ADDR                             UINT8_C(0x7E)

/**\name SENSOR RESOLUTION */
#define BMA490L_12_BIT_RESOLUTION                    UINT8_C(12)
#define BMA490L_16_BIT_RESOLUTION                    UINT8_C(16)

/**\name AUX/ACCEL DATA BASE ADDRESS REGISTERS */
#define BMA490L_DATA_0_ADDR                          UINT8_C(0x0A)
#define BMA490L_DATA_8_ADDR                          UINT8_C(0x12)

/**\name ACCELEROMETER ENABLE POSITION AND MASK */
#define BMA490L_ACCEL_ENABLE_POS                     UINT8_C(2)
#define BMA490L_ACCEL_ENABLE_MSK                     UINT8_C(0x04)

/**\name ACCEL CONFIGURATION POSITION AND MASK */
#define BMA490L_ACCEL_ODR_MSK                        UINT8_C(0x0F)
#define BMA490L_ACCEL_BW_POS                         UINT8_C(4)
#define BMA490L_ACCEL_BW_MSK                         UINT8_C(0x70)
#define BMA490L_ACCEL_RANGE_MSK                      UINT8_C(0x03)
#define BMA490L_ACCEL_PERFMODE_POS                   UINT8_C(7)
#define BMA490L_ACCEL_PERFMODE_MSK                   UINT8_C(0x80)

/**\name ACCEL CONFIG REGISTERS */
#define BMA490L_ACCEL_CONFIG_ADDR                    UINT8_C(0x40)

/**\name ACCEL RANGE ADDRESS */
#define BMA490L_ACCEL_RANGE_ADDR                     UINT8_C(0x41)

/**\name CONSTANTS */
#define BMA490L_ACCEL_CONFIG_LENGTH                  UINT8_C(2)
#define BMA490L_FIFO_CONFIG_LENGTH                   UINT8_C(2)
#define BMA490L_FIFO_WM_LENGTH                       UINT8_C(2)

/**\name BUS READ AND WRITE LENGTH FOR MAG & ACCEL */
#define BMA490L_ACCEL_DATA_LENGTH                    UINT8_C(6)
#define BMA490L_FIFO_DATA_LENGTH                     UINT8_C(2)

/**\name FIFO MAG DEFINITION */
#define BMA490L_MA_FIFO_A_X_LSB                      UINT8_C(8)

/**\name ARRAY PARAMETER DEFINITIONS */
#define BMA490L_SENSOR_TIME_MSB_BYTE                 UINT8_C(2)
#define BMA490L_SENSOR_TIME_XLSB_BYTE                UINT8_C(1)
#define BMA490L_SENSOR_TIME_LSB_BYTE                 UINT8_C(0)
#define BMA490L_FIFO_LENGTH_MSB_BYTE                 UINT8_C(1)

/**\name ACCEL ODR */
#define BMA490L_OUTPUT_DATA_RATE_0_78HZ              UINT8_C(0x01)
#define BMA490L_OUTPUT_DATA_RATE_1_56HZ              UINT8_C(0x02)
#define BMA490L_OUTPUT_DATA_RATE_3_12HZ              UINT8_C(0x03)
#define BMA490L_OUTPUT_DATA_RATE_6_25HZ              UINT8_C(0x04)
#define BMA490L_OUTPUT_DATA_RATE_12_5HZ              UINT8_C(0x05)
#define BMA490L_OUTPUT_DATA_RATE_25HZ                UINT8_C(0x06)
#define BMA490L_OUTPUT_DATA_RATE_50HZ                UINT8_C(0x07)
#define BMA490L_OUTPUT_DATA_RATE_100HZ               UINT8_C(0x08)
#define BMA490L_OUTPUT_DATA_RATE_200HZ               UINT8_C(0x09)
#define BMA490L_OUTPUT_DATA_RATE_400HZ               UINT8_C(0x0A)
#define BMA490L_OUTPUT_DATA_RATE_800HZ               UINT8_C(0x0B)
#define BMA490L_OUTPUT_DATA_RATE_1600HZ              UINT8_C(0x0C)

/**\name ACCEL BANDWIDTH PARAMETER */
#define BMA490L_ACCEL_OSR4_AVG1                      UINT8_C(0)
#define BMA490L_ACCEL_OSR2_AVG2                      UINT8_C(1)
#define BMA490L_ACCEL_NORMAL_AVG4                    UINT8_C(2)
#define BMA490L_ACCEL_CIC_AVG8                       UINT8_C(3)
#define BMA490L_ACCEL_RES_AVG16                      UINT8_C(4)
#define BMA490L_ACCEL_RES_AVG32                      UINT8_C(5)
#define BMA490L_ACCEL_RES_AVG64                      UINT8_C(6)
#define BMA490L_ACCEL_RES_AVG128                     UINT8_C(7)

/**\name ACCEL RANGE CHECK */
#define BMA490L_ACCEL_RANGE_2G                       UINT8_C(0)
#define BMA490L_ACCEL_RANGE_4G                       UINT8_C(1)
#define BMA490L_ACCEL_RANGE_8G                       UINT8_C(2)
#define BMA490L_ACCEL_RANGE_16G                      UINT8_C(3)

/**\name ERROR STATUS POSITION AND MASK */
#define BMA490L_FATAL_ERR_MSK                        UINT8_C(0x01)
#define BMA490L_CMD_ERR_POS                          UINT8_C(1)
#define BMA490L_CMD_ERR_MSK                          UINT8_C(0x02)
#define BMA490L_ERR_CODE_POS                         UINT8_C(2)
#define BMA490L_ERR_CODE_MSK                         UINT8_C(0x1C)
#define BMA490L_FIFO_ERR_POS                         UINT8_C(6)
#define BMA490L_FIFO_ERR_MSK                         UINT8_C(0x40)

/**\name DELAY DEFINITION IN MSEC */
#define BMA490L_GEN_READ_WRITE_DELAY                 UINT16_C(1000)

/**\name FIFO SELF WAKE UP POSITION AND MASK */
#define BMA490L_FIFO_SELF_WAKE_UP_POS                UINT8_C(1)
#define BMA490L_FIFO_SELF_WAKE_UP_MSK                UINT8_C(0x02)

/**\name FIFO BYTE COUNTER POSITION AND MASK */
#define BMA490L_FIFO_BYTE_COUNTER_MSB_MSK            UINT8_C(0x3F)

/**\name FIFO DATA POSITION AND MASK */
#define BMA490L_FIFO_DATA_POS                        UINT8_C(0)
#define BMA490L_FIFO_DATA_MSK                        UINT8_C(0xFF)

/**\name FIFO FILTER FOR ACCEL POSITION AND MASK */
#define BMA490L_FIFO_DOWN_ACCEL_POS                  UINT8_C(4)
#define BMA490L_FIFO_DOWN_ACCEL_MSK                  UINT8_C(0x70)
#define BMA490L_FIFO_FILTER_ACCEL_POS                UINT8_C(7)
#define BMA490L_FIFO_FILTER_ACCEL_MSK                UINT8_C(0x80)

/**\name FIFO HEADER DATA DEFINITIONS */
#define BMA490L_FIFO_HEADER_ACC                      UINT8_C(0x84)
#define BMA490L_FIFO_HEADER_MAG                      UINT8_C(0x90)
#define BMA490L_FIFO_HEADER_MAG_ACC                  UINT8_C(0x94)
#define BMA490L_FIFO_HEADER_SENSOR_TIME              UINT8_C(0x44)
#define BMA490L_FIFO_HEADER_INPUT_CONFIG             UINT8_C(0x48)
#define BMA490L_FIFO_HEADER_SKIP_FRAME               UINT8_C(0x40)
#define BMA490L_FIFO_HEADER_OVER_READ_MSB            UINT8_C(0x80)
#define BMA490L_FIFO_HEADER_SAMPLE_DROP              UINT8_C(0x50)

/**\name FIFO HEADERLESS MODE DATA ENABLE DEFINITIONS */
#define BMA490L_FIFO_MAG_ACC_ENABLE                  UINT8_C(0x60)
#define BMA490L_FIFO_ACC_ENABLE                      UINT8_C(0x40)
#define BMA490L_FIFO_MAG_ENABLE                      UINT8_C(0x20)

/**\name FIFO CONFIGURATION SELECTION */
#define BMA490L_FIFO_STOP_ON_FULL                    UINT8_C(0x01)
#define BMA490L_FIFO_TIME                            UINT8_C(0x02)
#define BMA490L_FIFO_TAG_INTR2                       UINT8_C(0x04)
#define BMA490L_FIFO_TAG_INTR1                       UINT8_C(0x08)
#define BMA490L_FIFO_HEADER                          UINT8_C(0x10)
#define BMA490L_FIFO_MAG                             UINT8_C(0x20)
#define BMA490L_FIFO_ACCEL                           UINT8_C(0x40)
#define BMA490L_FIFO_ALL                             UINT8_C(0x7F)
#define BMA490L_FIFO_CONFIG_0_MASK                   UINT8_C(0x03)
#define BMA490L_FIFO_CONFIG_1_MASK                   UINT8_C(0xFC)

/**\name FIFO FRAME COUNT DEFINITION */
#define BMA490L_FIFO_LSB_CONFIG_CHECK                UINT8_C(0x00)
#define BMA490L_FIFO_MSB_CONFIG_CHECK                UINT8_C(0x80)
#define BMA490L_FIFO_TAG_INTR_MASK                   UINT8_C(0xFC)

/**\name FIFO DROPPED FRAME DEFINITION */
#define BMA490L_ACCEL_AUX_FIFO_DROP                  UINT8_C(0x05)
#define BMA490L_ACCEL_FIFO_DROP                      UINT8_C(0x01)

/**\name FIFO sensor time length definitions */
#define BMA490L_SENSOR_TIME_LENGTH                   UINT8_C(3)

/**\name FIFO LENGTH DEFINITION */
#define BMA490L_FIFO_ACC_LENGTH                      UINT8_C(6)
#define BMA490L_FIFO_MAG_LENGTH                      UINT8_C(8)
#define BMA490L_FIFO_MAG_ACC_LENGTH                  UINT8_C(14)

/**\name SENSOR TIME REGISTERS */
#define BMA490L_SENSORTIME_0_ADDR                    UINT8_C(0x18)

/**\name INTERRUPT/FEATURE STATUS REGISTERS */
#define BMA490L_INT_STAT_0_ADDR                      UINT8_C(0x1C)

/**\name INTERRUPT/FEATURE STATUS REGISTERS */
#define BMA490L_INT_STAT_1_ADDR                      UINT8_C(0x1D)

/**\name GPIO REGISTERS */
#define BMA490L_INTERNAL_STAT                        UINT8_C(0x2A)

#define BMA490L_CONFIG_STREAM_MESSAGE_MSK            UINT8_C(0x0F)
#define BMA490L_ASIC_INITIALIZED                     UINT8_C(0x01)

/**\name FIFO DOWN SAMPLING REGISTER ADDRESS FOR ACCEL */
#define BMA490L_FIFO_DOWN_ADDR                       UINT8_C(0x45)

/**\name FIFO WATERMARK REGISTER ADDRESS */
#define BMA490L_FIFO_WTM_0_ADDR                      UINT8_C(0x46)

/**\name FIFO CONFIG REGISTERS */
#define BMA490L_FIFO_CONFIG_0_ADDR                   UINT8_C(0x48)
#define BMA490L_FIFO_CONFIG_1_ADDR                   UINT8_C(0x49)

/**\name FIFO REGISTERS */
#define BMA490L_FIFO_LENGTH_0_ADDR                   UINT8_C(0x24)
#define BMA490L_FIFO_DATA_ADDR                       UINT8_C(0x26)

/**\name FOC RELATED MACROS */
#define BMA490L_ACCEL_CONFIG_FOC                     UINT8_C(0xB7)

/**\name ACCEL OFFSET REGISTERS */
#define BMA490L_OFFSET_0_ADDR                        UINT8_C(0x71)
#define BMA490L_OFFSET_1_ADDR                        UINT8_C(0x72)
#define BMA490L_OFFSET_2_ADDR                        UINT8_C(0x73)

/**\name MULTIPLIER */
/*! for handling micro-g values */
#define BMA490L_MULTIPLIER                           UINT32_C(1000000)

/**\name ENABLE/DISABLE SELECTIONS */
#define BMA490L_X_AXIS                               UINT8_C(0)
#define BMA490L_Y_AXIS                               UINT8_C(1)
#define BMA490L_Z_AXIS                               UINT8_C(2)

/**\name SPI,I2C SELECTION REGISTER */
#define BMA490L_NV_CONFIG_ADDR                       UINT8_C(0x70)

/**\name NV_CONFIG POSITION AND MASK*/
/* NV_CONF Description - Reg Addr --> (0x70), Bit --> 3 */
#define BMA490L_NV_ACCEL_OFFSET_POS                  UINT8_C(3)
#define BMA490L_NV_ACCEL_OFFSET_MSK                  UINT8_C(0x08)

/**\name SELF-TEST */
#define BMA490L_SELFTEST_PASS                        UINT8_C(0)
#define BMA490L_SELFTEST_FAIL                        UINT8_C(1)

#define BMA490L_SELFTEST_DIFF_X_AXIS_FAILED          UINT8_C(1)
#define BMA490L_SELFTEST_DIFF_Y_AXIS_FAILED          UINT8_C(2)
#define BMA490L_SELFTEST_DIFF_Z_AXIS_FAILED          UINT8_C(3)
#define BMA490L_SELFTEST_DIFF_X_AND_Y_AXIS_FAILED    UINT8_C(4)
#define BMA490L_SELFTEST_DIFF_X_AND_Z_AXIS_FAILED    UINT8_C(5)
#define BMA490L_SELFTEST_DIFF_Y_AND_Z_AXIS_FAILED    UINT8_C(6)
#define BMA490L_SELFTEST_DIFF_X_Y_AND_Z_AXIS_FAILED  UINT8_C(7)

/**\name ACCEL SELF-TEST POSITION AND MASK */
#define BMA490L_ACCEL_SELFTEST_ENABLE_MSK            UINT8_C(0x01)
#define BMA490L_ACCEL_SELFTEST_SIGN_POS              UINT8_C(2)
#define BMA490L_ACCEL_SELFTEST_SIGN_MSK              UINT8_C(0x04)
#define BMA490L_SELFTEST_AMP_POS                     UINT8_C(3)
#define BMA490L_SELFTEST_AMP_MSK                     UINT8_C(0x08)

/**\name SELF-TEST REGISTER */
#define BMA490L_ACC_SELF_TEST_ADDR                   UINT8_C(0x6D)

/**\name CONDITION CHECK FOR READING AND WRTING DATA */
#define BMA490L_MAX_VALUE_FIFO_FILTER                UINT8_C(1)
#define BMA490L_MAX_VALUE_SPI3                       UINT8_C(1)
#define BMA490L_MAX_VALUE_SELFTEST_AMP               UINT8_C(1)
#define BMA490L_MAX_IF_MODE                          UINT8_C(3)
#define BMA490L_MAX_VALUE_SELFTEST_SIGN              UINT8_C(1)

/* Macros used for Self-test */
/* Self-test: Resulting minimum difference signal in mg for BMA490L */
#define BMA490L_ST_ACC_X_AXIS_SIGNAL_DIFF            UINT16_C(400)
#define BMA490L_ST_ACC_Y_AXIS_SIGNAL_DIFF            UINT16_C(800)
#define BMA490L_ST_ACC_Z_AXIS_SIGNAL_DIFF            UINT16_C(400)

/*! @name Macro to define accelerometer configuration value for FOC */
#define BMA490L_FOC_ACC_CONF_VAL                     UINT8_C(0xB7)

/**\name BOOLEAN TYPES */
#ifndef TRUE
#define TRUE                                         UINT8_C(0x01)
#endif

#ifndef FALSE
#define FALSE                                        UINT8_C(0x00)
#endif

#ifndef NULL
#define NULL                                         UINT8_C(0x00)
#endif

#define BMA490L_SOFT_RESET                           UINT8_C(0xB6)

#define BMA490L_MS_TO_US(X)                          (X * 1000)

#ifndef ABS
#define ABS(a)                                       ((a) > 0 ? (a) : -(a)) /*!< Absolute value */
#endif

/**\name BIT SLICE GET AND SET FUNCTIONS */
#define BMA490L_GET_BITSLICE(regvar, bitname) \
    ((regvar & bitname##_MSK) >> bitname##_POS)

#define BMA490L_SET_BITSLICE(regvar, bitname, val) \
    ((regvar & ~bitname##_MSK) | \
     ((val << bitname##_POS) & bitname##_MSK))

#define BMA490L_GET_DIFF(x, y)                       ((x) - (y))

#define BMA490L_GET_LSB(var)                         (uint8_t)(var & BMA490L_SET_LOW_BYTE)
#define BMA490L_GET_MSB(var)                         (uint8_t)((var & BMA490L_SET_HIGH_BYTE) >> 8)

#define BMA490L_SET_BIT_VAL_0(reg_data, bitname)     (reg_data & ~(bitname##_MSK))

#define BMA490L_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

#define BMA490L_GET_BITS_POS_0(reg_data, bitname)    (reg_data & (bitname##_MSK))

/******************************************************************************/
/*!  @name         TYPEDEF DEFINITIONS                                        */
/******************************************************************************/

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in] reg_addr       : Register address from which data is read.
 * @param[out] read_data     : Pointer to data buffer where read data is stored.
 * @param[in] len            : Number of bytes of data to be read.
 * @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs.
 *
 * @retval BMA4_INTF_RET_SUCCESS -> Success.
 * @retval != BMA4_INTF_RET_SUCCESS -> Fail.
 *
 */
typedef BMA490L_INTF_RET_TYPE (*bma490l_read_fptr_t)(uint8_t reg_addr, uint8_t *read_data, uint32_t len,
                                                     void *intf_ptr);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in] reg_addr      : Register address to which the data is written.
 * @param[in] read_data     : Pointer to data buffer in which data to be written
 *                            is stored.
 * @param[in] len           : Number of bytes of data to be written.
 * @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                            for interface related call backs
 *
 * @retval BMA4_INTF_RET_SUCCESS -> Success.
 * @retval != BMA4_INTF_RET_SUCCESS -> Fail.
 *
 */
typedef BMA490L_INTF_RET_TYPE (*bma490l_write_fptr_t)(uint8_t reg_addr, const uint8_t *read_data, uint32_t len,
                                                      void *intf_ptr);

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *
 */
typedef void (*bma490l_delay_us_fptr_t)(uint32_t period, void *intf_ptr);

/******************************************************************************/
/*!  @name         STRUCTURE DEFINITIONS                                      */
/******************************************************************************/

/*!
 *  @brief
 *  This structure holds asic info. for feature configuration.
 */
struct bma490l_asic_data
{
    /*! Feature config start addr (0-3 bits) */
    uint8_t asic_lsb;

    /*! Feature config start addr (4-11 bits) */
    uint8_t asic_msb;
};

/*!
 *  @brief
 *  Enum to define BMA490L sensor interfaces.
 */
enum bma490l_intf {
    BMA490L_SPI_INTF,
    BMA490L_I2C_INTF
};

/*!
 *  @brief
 *  This structure holds all relevant information about BMA490L
 */
struct bma490l_dev
{
    /*! Chip id of BMA490L */
    uint8_t chip_id;

    /*! Interface function pointer used to enable the device address for I2C and chip selection for SPI */
    void *intf_ptr;

    /*! Interface Selection
     * For SPI, interface = BMA490L_SPI_INTERFACE
     * For I2C, interface = BMA490L_I2C_INTERFACE
     * */
    enum bma490l_intf intf;

    /*! Decide SPI or I2C read mechanism */
    uint8_t dummy_byte;

    /*! Resolution of the sensor */
    uint8_t resolution;

    /*! Config stream data buffer address will be assigned */
    const uint8_t *config_file_ptr;

    /*! Max read/write length (maximum supported length is 32).
     * to be set by the user
     */
    uint16_t read_write_len;

    /*! Feature length */
    uint8_t feature_len;

    /*! Contains asic information */
    struct bma490l_asic_data asic_data;

    /*! Bus read function pointer */
    bma490l_read_fptr_t bus_read;

    /*! Bus write function pointer */
    bma490l_write_fptr_t bus_write;

    /*! Delay(in microsecond) function pointer */
    bma490l_delay_us_fptr_t delay_us;

    /*! Variable to store the size of config file */
    uint16_t config_size;

    /*! Variable to store the status of performance mode */
    uint8_t perf_mode_status;

    /*! Variable to store result of read/write function */
    BMA490L_INTF_RET_TYPE intf_rslt;
};

/*!
 * @brief Accelerometer configuration structure
 */
struct bma490l_accel_config
{
    /*! Output data rate in Hz */
    uint8_t odr;

    /*! Bandwidth parameter, determines filter configuration */
    uint8_t bandwidth;

    /*! Filter performance mode */
    uint8_t perf_mode;

    /*! Gravity range */
    uint8_t range;
};

/*!
 * @brief Accel xyz data structure
 */
struct bma490l_accel
{
    /*! Accel X data */
    int16_t x;

    /*! Accel Y data */
    int16_t y;

    /*! Accel Z data */
    int16_t z;
};

/*!
 *  @brief This structure holds the information for usage of
 *  FIFO by the user.
 */
struct bma490l_fifo_frame
{
    /*! Data buffer of user defined length is to be mapped here */
    uint8_t *data;

    /*! Number of bytes of FIFO to be read as specified by the user */
    uint16_t length;

    /*! Enabling of the FIFO header to stream in header mode */
    uint8_t fifo_header_enable;

    /*! Water-mark level for water-mark interrupt */
    uint16_t wm_lvl;

    /*! Streaming of the accelerometer sensor data or both in FIFO */
    uint8_t fifo_data_enable;

    /*! Will be equal to length when no more frames are there to parse */
    uint16_t accel_byte_start_idx;

    /*! Value of FIFO sensor time time */
    uint32_t sensor_time;

    /*! Value of Skipped frame counts */
    uint8_t skipped_frame_count;

    /*! Value of accel dropped frame count */
    uint8_t accel_dropped_frame_count;
};

/*!
 *  @brief Error Status structure
 */
struct bma490l_err_reg
{
    /*! Indicates fatal error */
    uint8_t fatal_err;

    /*! Indicates command error */
    uint8_t cmd_err;

    /*! Indicates error code */
    uint8_t err_code;

    /*! Indicates fifo error */
    uint8_t fifo_err;
};

/*!
 * @brief ASIC Config structure
 */
struct bma490l_asic_config
{
    /*! Enable/Disable ASIC Wake Up */
    uint8_t asic_en;

    /*! Configure stream_transfer/FIFO mode */
    uint8_t fifo_mode_en;

    /*! Mapping of instance RAM1 */
    uint8_t mem_conf_ram1;

    /*! Mapping of instance RAM2 */
    uint8_t mem_conf_ram2;

    /*! Mapping of instance RAM3 */
    uint8_t mem_conf_ram3;
};

/*! @name Structure to store accelerometer data deviation from ideal value */
struct bma490l_offset_delta
{
    /*! X axis */
    int16_t x;

    /*! Y axis */
    int16_t y;

    /*! Z axis */
    int16_t z;
};

/*!
 * @brief Accel offset xyz structure
 */
struct bma490l_accel_offset
{
    /*! Accel offset X data */
    uint8_t x;

    /*! Accel offset Y data */
    uint8_t y;

    /*! Accel offset Z data */
    uint8_t z;
};

/*!
 * @brief Structure to store temp data values
 */
struct bma490l_accel_temp
{
    /*! Accel X temp data */
    int32_t x;

    /*! Accel Y temp data */
    int32_t y;

    /*! Accel Z temp data */
    int32_t z;
};

/*!  @name Structure to enable an accel axis for FOC */
struct bma490l_accel_foc_g_value
{
    /*! '0' to disable x axis and '1' to enable x axis */
    uint8_t x;

    /*! '0' to disable y axis and '1' to enable y axis */
    uint8_t y;

    /*! '0' to disable z axis and '1' to enable z axis */
    uint8_t z;

    /*! '0' for positive input and '1' for negative input */
    uint8_t sign;
};

/*! @name Structure to store temporary accelerometer */
struct bma490l_foc_temp_value
{
    /*! X data */
    int32_t x;

    /*! Y data */
    int32_t y;

    /*! Z data */
    int32_t z;
};

/*!
 * @brief Accel self-test difference data structure
 */
struct bma490l_self_test_delta_limit
{
    /*! Accel X data */
    int32_t x;

    /*! Accel Y data */
    int32_t y;

    /*! Accel Z data */
    int32_t z;
};

/* Structure to store temporary axes data values */
struct bma490l_temp_axes_val
{
    /*! X data */
    int32_t x;

    /*! Y data */
    int32_t y;

    /*! Z data */
    int32_t z;
};

/*! @name Structure to define any/no-motion configuration */
struct bma490l_any_no_mot_config
{
    /*! Expressed in 50 Hz samples (20 ms) */
    uint16_t duration;

    /*! Threshold value for Any-motion/No-motion detection in
     * 5.11g format
     */
    uint16_t threshold;

    /*! To enable selected axes */
    uint8_t axes_en;
};

/*! @name Structure to store the value of re-mapped axis and its sign */
struct bma490l_axes_remap
{
    /*! Re-mapped x-axis */
    uint8_t x_axis;

    /*! Re-mapped y-axis */
    uint8_t y_axis;

    /*! Re-mapped z-axis */
    uint8_t z_axis;

    /*! Re-mapped x-axis sign */
    uint8_t x_axis_sign;

    /*! Re-mapped y-axis sign */
    uint8_t y_axis_sign;

    /*! Re-mapped z-axis sign */
    uint8_t z_axis_sign;
};

#endif /* End of BMA490L_DEFS_H__ */
