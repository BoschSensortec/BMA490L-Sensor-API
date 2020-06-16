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
 * @file       bma490l.c
 * @date       2020-04-08
 * @version    v0.2.3
 *
 */

/***************************************************************************/

/*!             Header files
 ****************************************************************************/
#include "bma490l.h"

/***************************************************************************/

/*!              Global Variable
 ****************************************************************************/

/*! @name  Global array that stores the configuration file of BMA490L */
const uint8_t bma490l_config_file[] = {
    0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00,
    0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x58, 0x01, 0x80, 0x2e, 0x74, 0x02, 0xb0, 0xf0,
    0x10, 0x30, 0x21, 0x2e, 0x16, 0xf0, 0x80, 0x2e, 0xeb, 0x00, 0x19, 0x50, 0x17, 0x52, 0x01, 0x42, 0x3b, 0x80, 0x41,
    0x30, 0x01, 0x42, 0x3c, 0x80, 0x00, 0x2e, 0x01, 0x40, 0x01, 0x42, 0x21, 0x2e, 0xff, 0xaf, 0xb8, 0x2e, 0x9b, 0x95,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0xfd, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x2e,
    0x55, 0xf0, 0xc0, 0x2e, 0x21, 0x2e, 0x55, 0xf0, 0x80, 0x2e, 0x18, 0x00, 0xfd, 0x2d, 0xaa, 0x00, 0x05, 0x00, 0xaa,
    0x00, 0x05, 0x00, 0x40, 0x48, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x98, 0x2e, 0x80, 0x00, 0x20, 0x26, 0x98, 0x2e, 0xef, 0x00, 0x10, 0x30, 0x21, 0x2e,
    0x59, 0xf0, 0x98, 0x2e, 0x38, 0x00, 0x98, 0x2e, 0x7f, 0x01, 0x98, 0x2e, 0x8e, 0x01, 0x00, 0x2e, 0x00, 0x2e, 0xd0,
    0x2e, 0x98, 0x2e, 0xce, 0x00, 0x01, 0x2e, 0x34, 0x00, 0x00, 0xb2, 0x0d, 0x2f, 0x00, 0x30, 0x21, 0x2e, 0x34, 0x00,
    0x01, 0x50, 0x98, 0x2e, 0x13, 0x01, 0x01, 0x50, 0x03, 0x52, 0x98, 0x2e, 0x00, 0xb0, 0x01, 0x50, 0x05, 0x52, 0x98,
    0x2e, 0x00, 0xb0, 0x98, 0x2e, 0x38, 0x00, 0xe6, 0x2d, 0x13, 0x52, 0x40, 0x30, 0x42, 0x40, 0x90, 0x0a, 0x42, 0x42,
    0x58, 0x84, 0x07, 0x52, 0xa1, 0x42, 0x71, 0x3c, 0x09, 0x56, 0x83, 0x42, 0xa9, 0x84, 0x83, 0x32, 0x84, 0x40, 0x61,
    0x08, 0x4b, 0x0a, 0x81, 0x42, 0x82, 0x82, 0x02, 0x3f, 0x43, 0x40, 0x9a, 0x08, 0x52, 0x42, 0x40, 0x42, 0x7e, 0x80,
    0x61, 0x30, 0x01, 0x42, 0x10, 0x50, 0x01, 0x2e, 0x40, 0xf0, 0x1a, 0x90, 0xfb, 0x7f, 0x20, 0x2f, 0x03, 0x30, 0x0d,
    0x50, 0x34, 0x33, 0x06, 0x30, 0x11, 0x52, 0x0b, 0x54, 0x55, 0x32, 0x1d, 0x1a, 0xe3, 0x22, 0x18, 0x1a, 0x0f, 0x58,
    0xe3, 0x22, 0x04, 0x30, 0xd5, 0x40, 0xb5, 0x0d, 0xe1, 0xbe, 0x6f, 0xbb, 0x80, 0x91, 0xa9, 0x0d, 0x01, 0x89, 0xb5,
    0x23, 0x10, 0xa1, 0xf7, 0x2f, 0xda, 0x0e, 0x34, 0x33, 0xeb, 0x2f, 0x01, 0x2e, 0x25, 0x00, 0x70, 0x1a, 0x00, 0x30,
    0x21, 0x30, 0x02, 0x2c, 0x08, 0x22, 0x30, 0x30, 0x00, 0xb2, 0x06, 0x2f, 0x21, 0x2e, 0x59, 0xf0, 0x98, 0x2e, 0x38,
    0x00, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0xfb, 0x6f, 0xf0, 0x5f, 0xb8, 0x2e, 0x1d, 0x50, 0x05, 0x2e, 0x00, 0xf0,
    0x17, 0x56, 0xd3, 0x0f, 0x01, 0x40, 0xf4, 0x33, 0xcc, 0x08, 0x0d, 0x2f, 0xf4, 0x30, 0x94, 0x08, 0xb9, 0x88, 0x02,
    0xa3, 0x04, 0x2f, 0x1b, 0x58, 0x4c, 0x0a, 0x87, 0xa2, 0x05, 0x2c, 0xcb, 0x22, 0x17, 0x54, 0x4a, 0x0a, 0xf2, 0x3b,
    0xca, 0x08, 0x3c, 0x80, 0x27, 0x2e, 0x59, 0xf0, 0x01, 0x40, 0x01, 0x42, 0xb8, 0x2e, 0x1a, 0x24, 0x26, 0x00, 0x80,
    0x2e, 0x58, 0x00, 0x00, 0x31, 0xc0, 0x2e, 0x21, 0x2e, 0xba, 0xf0, 0x12, 0x30, 0x12, 0x42, 0x02, 0x30, 0x12, 0x42,
    0x12, 0x42, 0x12, 0x42, 0x02, 0x42, 0x03, 0x80, 0x41, 0x84, 0x11, 0x42, 0x02, 0x42, 0xb8, 0x2e, 0x44, 0x47, 0x35,
    0x00, 0x46, 0x00, 0x4f, 0x00, 0xaf, 0x00, 0xff, 0x00, 0xff, 0xb7, 0x00, 0x02, 0x00, 0xb0, 0x05, 0x80, 0xb1, 0xf0,
    0x88, 0x00, 0x80, 0x00, 0x5e, 0xf0, 0xc0, 0x00, 0x59, 0xf0, 0x89, 0xf0, 0x38, 0x00, 0x40, 0x00, 0x42, 0x00, 0x60,
    0x50, 0x03, 0x2e, 0x45, 0x00, 0xe0, 0x7f, 0xf1, 0x7f, 0xdb, 0x7f, 0x30, 0x30, 0x15, 0x54, 0x0a, 0x1a, 0x28, 0x2f,
    0x1a, 0x25, 0x7a, 0x82, 0x00, 0x30, 0x43, 0x30, 0x32, 0x30, 0x05, 0x30, 0x04, 0x30, 0xf6, 0x6f, 0xf2, 0x09, 0xfc,
    0x13, 0xc2, 0xab, 0xb3, 0x09, 0xef, 0x23, 0x80, 0xb3, 0xe6, 0x6f, 0xb7, 0x01, 0x00, 0x2e, 0x8b, 0x41, 0x4b, 0x42,
    0x03, 0x2f, 0x46, 0x40, 0x86, 0x17, 0x81, 0x8d, 0x46, 0x42, 0x41, 0x8b, 0x23, 0xbd, 0xb3, 0xbd, 0x03, 0x89, 0x41,
    0x82, 0x07, 0x0c, 0x43, 0xa3, 0xe6, 0x2f, 0xe1, 0x6f, 0xa2, 0x6f, 0x52, 0x42, 0x00, 0x2e, 0xb2, 0x6f, 0x52, 0x42,
    0x00, 0x2e, 0xc2, 0x6f, 0x42, 0x42, 0x03, 0xb2, 0x06, 0x2f, 0x01, 0x2e, 0x59, 0xf0, 0x01, 0x32, 0x01, 0x0a, 0x21,
    0x2e, 0x59, 0xf0, 0x06, 0x2d, 0x01, 0x2e, 0x59, 0xf0, 0xf1, 0x3d, 0x01, 0x08, 0x21, 0x2e, 0x59, 0xf0, 0xdb, 0x6f,
    0xa0, 0x5f, 0xb8, 0x2e, 0x60, 0x50, 0xc3, 0x7f, 0xd4, 0x7f, 0xe7, 0x7f, 0xf6, 0x7f, 0xb2, 0x7f, 0xa5, 0x7f, 0x36,
    0x30, 0x07, 0x2e, 0x01, 0xf0, 0xbe, 0xbd, 0xbe, 0xbb, 0x1f, 0x58, 0x77, 0x05, 0x01, 0x56, 0x21, 0x54, 0x27, 0x41,
    0x06, 0x41, 0xf8, 0xbf, 0xbe, 0x0b, 0xb5, 0x11, 0xd6, 0x42, 0x03, 0x89, 0x5a, 0x0e, 0xf6, 0x2f, 0x12, 0x30, 0x25,
    0x2e, 0x34, 0x00, 0x02, 0x31, 0x25, 0x2e, 0xb8, 0xf0, 0xd4, 0x6f, 0xc3, 0x6f, 0xe7, 0x6f, 0xb2, 0x6f, 0xa5, 0x6f,
    0xf6, 0x6f, 0xa0, 0x5f, 0xc8, 0x2e, 0x10, 0x50, 0x23, 0x52, 0x03, 0x50, 0xfb, 0x7f, 0x98, 0x2e, 0xf3, 0x00, 0x03,
    0x52, 0x45, 0x82, 0x10, 0x30, 0x50, 0x42, 0x60, 0x30, 0xfb, 0x6f, 0xc0, 0x2e, 0x40, 0x42, 0xf0, 0x5f, 0x10, 0x50,
    0x25, 0x52, 0x05, 0x50, 0xfb, 0x7f, 0x98, 0x2e, 0xf3, 0x00, 0x05, 0x52, 0x45, 0x82, 0x00, 0x30, 0x50, 0x42, 0x70,
    0x30, 0xfb, 0x6f, 0xc0, 0x2e, 0x40, 0x42, 0xf0, 0x5f, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80,
    0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0xfd, 0x2d, 0x46, 0x86,
    0x70, 0x50, 0xe5, 0x40, 0xc3, 0x88, 0x42, 0x84, 0x04, 0x41, 0xc3, 0x40, 0x06, 0x41, 0x6d, 0xbb, 0xc2, 0x7f, 0xf5,
    0x7f, 0x80, 0xb3, 0xe6, 0x7f, 0xd0, 0x7f, 0xb3, 0x7f, 0x12, 0x30, 0x5e, 0x2f, 0x31, 0x25, 0x55, 0x40, 0x41, 0x91,
    0xa1, 0x7f, 0x0f, 0x2f, 0x01, 0x30, 0xc1, 0x42, 0x00, 0x2e, 0xc2, 0x6f, 0x13, 0x40, 0x93, 0x42, 0x00, 0x2e, 0x13,
    0x40, 0x93, 0x42, 0x00, 0x2e, 0x00, 0x40, 0x80, 0x42, 0xbd, 0x80, 0xc0, 0x2e, 0x01, 0x42, 0x90, 0x5f, 0xc7, 0x86,
    0x01, 0x30, 0xc5, 0x40, 0xfb, 0x86, 0x45, 0x41, 0x04, 0x41, 0x43, 0xbe, 0xc3, 0xbb, 0xd5, 0xbe, 0x55, 0xba, 0x97,
    0x7f, 0x05, 0x30, 0xd1, 0x15, 0xf7, 0x09, 0xc0, 0xb3, 0x09, 0x2f, 0x06, 0x40, 0xc7, 0x40, 0xb7, 0x05, 0x07, 0x30,
    0x80, 0xa9, 0xfe, 0x05, 0xb7, 0x23, 0x74, 0x0f, 0x55, 0x23, 0xe6, 0x6f, 0x41, 0x82, 0x01, 0x80, 0xc1, 0x86, 0x43,
    0xa2, 0xec, 0x2f, 0xb0, 0x6f, 0xa4, 0x6f, 0x28, 0x1a, 0xd1, 0x6f, 0xc3, 0x6f, 0x02, 0x2f, 0x02, 0x30, 0x18, 0x2c,
    0x02, 0x43, 0x05, 0x41, 0x6a, 0x29, 0x96, 0x6f, 0x05, 0x43, 0x6e, 0x0e, 0x10, 0x2f, 0xf4, 0x6f, 0x00, 0xb3, 0x03,
    0x2f, 0x3f, 0x89, 0x94, 0x14, 0x25, 0x2e, 0x5e, 0xf0, 0x41, 0x25, 0x23, 0x25, 0x15, 0x41, 0x95, 0x42, 0x00, 0x2e,
    0x15, 0x41, 0x95, 0x42, 0x00, 0x2e, 0x04, 0x41, 0x84, 0x42, 0x00, 0x90, 0x09, 0x2f, 0x50, 0x40, 0xd0, 0x42, 0x00,
    0x2e, 0x50, 0x40, 0xd0, 0x42, 0x00, 0x2e, 0x40, 0x40, 0x02, 0x2c, 0xc0, 0x42, 0x42, 0x42, 0x90, 0x5f, 0xb8, 0x2e,
    0x00, 0x2e, 0x10, 0x24, 0x8a, 0x02, 0x11, 0x24, 0x00, 0x0c, 0x12, 0x24, 0x80, 0x2e, 0x13, 0x24, 0x18, 0x00, 0x12,
    0x42, 0x13, 0x42, 0x41, 0x1a, 0xfb, 0x2f, 0x10, 0x24, 0x50, 0x39, 0x11, 0x24, 0x21, 0x2e, 0x21, 0x2e, 0x10, 0x00,
    0x23, 0x2e, 0x11, 0x00, 0x80, 0x2e, 0x10, 0x00
};

/******************************************************************************/

/*!         Local Function Prototypes
 ******************************************************************************/

/*!
 *  @brief This internal API sets the feature config. data start address in the sensor.
 *
 *  @param[in] dev  : Structure instance of bma490l_dev.
 *
 *  @return Result of API execution status
 *  @return 0 -> Success
 *  @return < 0 value -> Fail
 */
static int8_t set_feature_config_start_addr(struct bma490l_dev *dev);

/*!
 *  @brief This internal API increments the feature config. data address according to the user
 *  provided read/write length in the dev structure.
 *
 *  @param[in] dev  : Structure instance of bma490l_dev.
 *
 *  @return Result of API execution status
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
static int8_t increment_feature_config_addr(struct bma490l_dev *dev);

/*!
 *  @brief This internal API reads the 8-bit data from the given register
 *  in the sensor.
 *
 *  @param[in] addr      : Register address from which data is read.
 *  @param[out] data     : Pointer to data buffer where read data is stored.
 *  @param[in] len       : Number of bytes of data to be read.
 *  @param[in] dev       : Structure instance of bma490l_dev.
 *
 *  @return Result of API execution status
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
static int8_t read_regs(uint8_t addr, uint8_t *data, uint32_t len, struct bma490l_dev *dev);

/*!
 *  @brief This internal API writes the 8-bit data to the given register
 *  in the sensor.
 *
 *  @param[in] addr     : Register address to which the data is written.
 *  @param[in] data     : Pointer to data buffer in which data to be written
 *            is stored.
 *  @param[in] len      : Number of bytes of data to be written.
 *  @param[in] dev      : Structure instance of bma490l_dev
 *
 *  @return Result of API execution status
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
static int8_t write_regs(uint8_t addr, const uint8_t *data, uint32_t len, struct bma490l_dev *dev);

/*!
 *  @brief This internal API is used to write the binary configuration in the sensor.
 *
 *  @param[in] dev      : Structure instance of bma490l_dev
 *
 *  @return Result of API execution status
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
static int8_t config_file_download(struct bma490l_dev *dev);

/*!
 *  @brief This internal API writes the config stream data in memory using burst mode
 *
 *  @param[in] stream_data : Pointer to store data of 1 byte
 *  @param[in] index : Represents value in multiples of 1 byte
 *  @param[in] dev : Structure instance of bma490l_dev.
 *
 *  @return Result of API execution status
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
static int8_t stream_transfer_write(const uint8_t *stream_data, uint16_t index, struct bma490l_dev *dev);

/*!
 *  @brief This internal API sets the feature config. data start address in the sensor.
 *
 *  @param[in] dev      : Structure instance of bma490l_dev.
 *
 *  @return Result of API execution status
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
static int8_t get_feature_config_start_addr(struct bma490l_dev *dev);

/*!
 *  @brief This internal API sets the interrupt to either interrupt1 or
 *  interrupt2 pin in the sensor.
 *
 *  @param[in] int_line : Variable used to select interrupt pin1 or pin2
 *  @param[in] int_map  : Variable used to select a particular interrupt
 *                        in the sensor
 *  @param[in] enable   : Variable used to enable or disable the interrupt
 *  @param[in] dev      : Structure instance of bma490l_dev
 *
 *  @return Result of API execution status
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
static int8_t map_int(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma490l_dev *dev);

/*!
 *  @brief This internal API validates the bandwidth and performance mode
 *  value set by the user.
 *
 *  param bandwidth[in] : Bandwidth value set by the user.
 *  param perf_mode[in] : Performance mode value set by the user.
 *
 *  @return Result of API execution status
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
static int8_t validate_bandwidth_perfmode(uint8_t bandwidth, uint8_t perf_mode);

/*!
 *  @brief This internal API validates the ODR value set by the user.
 *
 *  param bandwidth[in] : ODR for accelerometer.
 *
 */
static int8_t validate_odr(uint8_t odr);

/*!
 *  @brief This internal API is used to reset the FIFO related configurations
 *  in the fifo_frame structure.
 *
 *  @param fifo[in,out]  : Structure instance of bma490l_fifo_frame.
 *
 *  @return Result of API execution status
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
static void reset_fifo_data_structure(struct bma490l_fifo_frame *fifo);

/*!
 *  @brief This internal API computes the number of bytes of accel FIFO data
 *  which is to be parsed in header-less mode.
 *
 *  @param[out] start_idx   : The start index for parsing data.
 *  @param[out] len         : Number of bytes to be parsed.
 *  @param[in]  acc_count   : Number of accelerometer frames to be read.
 *  @param[in]  fifo         : Structure instance of bma490l_fifo_frame.
 *
 */
static void get_accel_len_to_parse(uint16_t *start_idx,
                                   uint16_t *len,
                                   const uint16_t *acc_count,
                                   const struct bma490l_fifo_frame *fifo);

/*!
 *  @brief This internal API checks the fifo read data as empty frame, if it
 *  is empty frame then moves the index to last byte.
 *
 *  @param[in,out] data_index   : The index of the current data to
 *                                be parsed from fifo data
 *  @param[in]  fifo             : Structure instance of bma490l_fifo_frame.
 *
 */
static void check_empty_fifo(uint16_t *data_index, const struct bma490l_fifo_frame *fifo);

/*!
 *  @brief This internal API is used to parse the accelerometer data from the
 *  FIFO data in header mode.
 *
 *  @param[in,out] accel_data   : Structure instance of bma490l_accel where
 *                                the accelerometer data in FIFO is stored.
 *  @param[in,out] accel_length : Number of accelerometer frames
 *                                (x,y,z axes data)
 *  @param[in,out] fifo         : Structure instance of bma490l_fifo_frame.
 *  @param[in,out] dev          : Structure instance of bma490l_dev.
 *
 */
static void extract_accel_header_mode(struct bma490l_accel *accel_data,
                                      uint16_t *accel_length,
                                      struct bma490l_fifo_frame *fifo,
                                      const struct bma490l_dev *dev);

/*!
 *  @brief This internal API is used to parse the accelerometer data from the
 *  FIFO data in both header mode and header-less mode.
 *  It updates the idx value which is used to store the index of
 *  the current data byte which is parsed.
 *
 *  @param[in,out] acc       : Structure instance of bma490l_accel.
 *  @param[in,out] idx       : Index value of number of bytes parsed
 *  @param[in,out] acc_idx   : Index value of accelerometer data
 *                             (x,y,z axes) frame to be parsed
 *  @param[in] frm           : It consists of either fifo_data_enable
 *                             parameter (Accel and/or mag data enabled in FIFO)
 *                             in header-less mode or frame header data
 *                             in header mode
 *  @param[in]  fifo         : Structure instance of bma490l_fifo_frame.
 *  @param[in]  dev          : Structure instance of bma490l_dev.
 *
 */
static void unpack_acc_frm(struct bma490l_accel *acc,
                           uint16_t *idx,
                           uint16_t *acc_idx,
                           uint8_t frm,
                           const struct bma490l_fifo_frame *fifo,
                           const struct bma490l_dev *dev);

/*!
 *  @brief This internal API is used to parse the accelerometer data from the
 *  FIFO data and store it in the instance of the structure bma490l_accel.
 *
 *  @param[out] accel_data        : Structure instance of bma490l_accel where
 *                                  the parsed accel data bytes are stored.
 *  @param[in] data_start_index   : Index value of the accel data bytes
 *                                  which is to be parsed from the fifo data.
 *  @param[in]  fifo              : Structure instance of bma490l_fifo_frame.
 *  @param[in] dev                :  Structure instance of bma490l_dev.
 *
 */
static void unpack_accel_data(struct bma490l_accel *accel_data,
                              uint16_t data_start_index,
                              const struct bma490l_fifo_frame *fifo,
                              const struct bma490l_dev *dev);

/*!
 *  @brief This internal API is used to parse and store the sensor time from the
 *  FIFO data in the structure instance dev.
 *
 *  @param[in,out] data_index : Index of the FIFO data which
 *                              has the sensor time.
 *  @param[in,out]  fifo      : Structure instance of bma490l_fifo_frame.
 *
 */
static void unpack_sensortime_frame(uint16_t *data_index, struct bma490l_fifo_frame *fifo);

/*!
 *  @brief This internal API is used to parse and store the skipped_frame_count from
 *  the FIFO data in the structure instance dev.
 *
 *  @param[in,out] data_index       : Index of the FIFO data which
 *                                    has the skipped frame count.
 *  @param[in,out]  fifo            : Structure instance of bma490l_fifo_frame.
 *
 */
static void unpack_skipped_frame(uint16_t *data_index, struct bma490l_fifo_frame *fifo);

/*!
 *  @brief This internal API is used to parse and store the dropped_frame_count from
 *  the FIFO data in the structure instance dev.
 *
 *  @param[in,out] data_index       : Index of the FIFO data which
 *                                    has the dropped frame data.
 *  @param[in,out]  fifo            : Structure instance of bma490l_fifo_frame.
 *
 */
static void unpack_dropped_frame(uint16_t *data_index, struct bma490l_fifo_frame *fifo);

/*!
 *  @brief This internal API is used to move the data index ahead of the
 *  current_frame_length parameter when unnecessary FIFO data appears while
 *  extracting the user specified data.
 *
 *  @param[in,out] data_index       : Index of the FIFO data which
 *                                    is to be moved ahead of the
 *                                    current_frame_length
 *  @param[in] current_frame_length : Number of bytes in a particular frame
 *  @param[in]  fifo                : Structure instance of bma490l_fifo_frame.
 *
 */
static void move_next_frame(uint16_t *data_index, uint8_t current_frame_length, const struct bma490l_fifo_frame *fifo);

/*!
 * @brief This internal API is used to calculate the power of given
 * base value.
 *
 * @param[in] base : Value of base
 * @param[in] resolution : Resolution of the sensor
 *
 * @return : Return the value of base^resolution
 *
 */
static int32_t power(int16_t base, uint8_t resolution);

/*!
 *  @brief This internal API enables and configures the Accel which is needed
 *  for self-test operation.
 *
 *  @param[in] dev : Structure instance of bma490l_dev
 *
 *  @return results of self-test
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
static int8_t set_accel_selftest_config(struct bma490l_dev *dev);

/*!
 *  @brief This internal API validates accel self-test status from positive and negative axes input
 *
 * @param[in] positive  : Positive accel data.
 * @param[in] negative  : Negative accel data.
 * @param[in/out] accel_data_diff_mg  : Accel data difference data between positive and negative in mg.
 * @param[in] dev  : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status
 *
 * @return 0 -> Success,
 * @retvalue < 0 -> Fail
 *
 */
static int8_t get_accel_data_difference_and_validate(struct bma490l_accel positive,
                                                     struct bma490l_accel negative,
                                                     struct bma490l_self_test_delta_limit *accel_data_diff_mg,
                                                     const struct bma490l_dev *dev);

/*!
 *  @brief This internal API enables or disables the Accel self-test feature in the
 *  sensor.
 *
 *  @param[in] accel_self-test_axis : Variable used to enable or disable
 *  the Accel self-test feature
 *  Value   |  Description
 *  --------|---------------
 *  0x00    | BMA490L_DISABLE
 *  0x01    | BMA490L_ENABLE
 *
 *  @param[in] dev : Structure instance of bma490l_dev.
 *
 *  @return Result of API execution status
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
static int8_t set_accel_selftest_enable(uint8_t accel_selftest_axis, struct bma490l_dev *dev);

/*!
 *  @brief This internal API selects the sign of Accel self-test excitation
 *
 *  @param[in] accel_selftest_sign: Variable used to select the Accel
 *  self-test sign
 *  Value   |  Description
 *  --------|--------------------------
 *  0x00    | BMA490L_DISABLE (negative)
 *  0x01    | BMA490L_ENABLE (positive)
 *
 *  @param[in] dev : Structure instance of bma490l_dev
 *
 *  @return Result of API execution status
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
static int8_t set_accel_selftest_sign(uint8_t accel_selftest_sign, struct bma490l_dev *dev);

/*!
 *  @brief This internal API sets the Accel self-test amplitude in the sensor.
 *
 *  @param[in] accel_selftest_amp : Variable used to specify the Accel self
 *  test amplitude
 *  Value   |  Description
 *  --------|------------------------------------
 *  0x00    | BMA490L_SELFTEST_AMP_LOW
 *  0x01    | BMA490L_SELFTEST_AMP_HIGH
 *
 *  @param[in] dev : Structure instance of bma490l_dev
 *
 *  @return Result of API execution status
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
static int8_t set_accel_selftest_amp(uint8_t accel_selftest_amp, struct bma490l_dev *dev);

/*!
 *  @brief This internal API converts lsb value of axes to mg for self-test
 *
 *  @param[in] accel_data_diff : Pointer variable used to pass accel difference
 *  values in g.
 *  @param[out] accel_data_diff_mg : Pointer variable used to store accel
 *  difference values in mg.
 *  @param[out] dev : Structure instance of bma490l_dev.
 *
 *  @return None
 */
static void convert_lsb_g(const struct bma490l_self_test_delta_limit *accel_data_diff,
                          struct bma490l_self_test_delta_limit *accel_data_diff_mg,
                          const struct bma490l_dev *dev);

/*!
 *  @brief This internal function validates the Accel self-test data and decides the
 *  result of self-test operation.
 *
 *  @param[in] accel_data_diff : Pointer to structure variable which holds
 *  the Accel data difference of self-test operation
 *
 *  @return Result of self-test operation
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
static int8_t validate_selftest(const struct bma490l_self_test_delta_limit *accel_data_diff);

/*!
 * @brief This internal API is used to verify the right position of the sensor before doing accel FOC
 *
 * @param[in] accel_en: Variable to store status of accel.
 * @param[in] accel_g_axis: Accel FOC axis and sign input.
 * @param[in] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
static int8_t verify_foc_position(uint8_t accel_en,
                                  const struct bma490l_accel_foc_g_value *accel_g_axis,
                                  struct bma490l_dev *dev);

/*!
 * @brief This internal API reads and provides average for 128 samples of sensor data for FOC operation
 *
 * @param[in] accel_en : Variable to store status of accel
 * @param[in] temp_foc_data: Store data samples.
 * @param[in] bma490l_dev: Structure instance of bma490l_dev.
 *
 * @return Result of API execution status
 *
 * @return BMA490L_OK
 */
static int8_t get_average_of_sensor_data(uint8_t accel_en,
                                         struct bma490l_foc_temp_value *temp_foc_data,
                                         struct bma490l_dev *dev);

/*!
 * @brief This internal API validates accel FOC position as per the range
 *
 * @param[in] accel_en : Variable to store status of accel
 * @param[in] accel_g_axis : Accel axis to FOC
 * @param[in] avg_foc_data : Average value of sensor sample datas
 * @param[in] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status
 *
 * @retval BMA490L_OK - Success.
 * @retval BMA490L_E_INVALID_INPUT - Error: Invalid input
 */
static int8_t validate_foc_position(uint8_t accel_en,
                                    const struct bma490l_accel_foc_g_value *accel_g_axis,
                                    struct bma490l_accel avg_foc_data,
                                    struct bma490l_dev *dev);

/*!
 * @brief This internal API validates accel FOC axis given as input
 *
 * @param[in] avg_foc_data : Average value of sensor sample datas.
 * @param[in] dev : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
static int8_t validate_foc_accel_axis(int16_t avg_foc_data, struct bma490l_dev *dev);

/*!
 * @brief This internal API saves the configurations before performing FOC.
 *
 * @param[out] acc_cfg      : Accelerometer configuration value
 * @param[out] aps          : Advance power mode value
 * @param[out] acc_en       : Accelerometer enable value
 * @param[in] dev           : Structure instance of bma490l_dev
 *
 * @return Result of API execution status
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
static int8_t save_accel_foc_config(struct bma490l_accel_config *acc_cfg,
                                    uint8_t *aps,
                                    uint8_t *acc_en,
                                    struct bma490l_dev *dev);

/*!
 * @brief This internal API sets configurations for performing accelerometer FOC.
 *
 * @param[in] dev       : Structure instance of bma490l_dev
 *
 * @return Result of API execution status
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
static int8_t set_accel_foc_config(struct bma490l_dev *dev);

/*!
 * @brief This internal API enables/disables the offset compensation for
 * filtered and un-filtered accelerometer data.
 *
 * @param[in] offset_en     : enables/disables offset compensation.
 * @param[in] dev           : Structure instance of bma490l_dev
 *
 * @return Result of API execution status
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
static int8_t set_accel_offset_comp(uint8_t offset_en, struct bma490l_dev *dev);

/*!
 * @brief This internal API performs Fast Offset Compensation for accelerometer.
 *
 * @param[in] accel_g_value : This parameter selects the accel FOC
 * axis to be performed
 *
 * Input format is {x, y, z, sign}. '1' to enable. '0' to disable
 *
 * Eg: To choose x axis  {1, 0, 0, 0}
 * Eg: To choose -x axis {1, 0, 0, 1}
 *
 * @param[in] acc_cfg       : Accelerometer configuration value
 * @param[in] dev           : Structure instance of bma490l_dev.
 *
 * @return Result of API execution status
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 *
 */
static int8_t perform_accel_foc(const struct bma490l_accel_foc_g_value *accel_g_value,
                                const struct bma490l_accel_config *acc_cfg,
                                struct bma490l_dev *dev);

/*!
 * @brief This internal API converts the range value into accelerometer
 * corresponding integer value.
 *
 * @param[in] range_in      : Input range value.
 * @param[out] range_out    : Stores the integer value of range.
 *
 * @return None
 *
 */
static void map_accel_range(uint8_t range_in, uint8_t *range_out);

/*!
 * @brief This internal API compensate the accelerometer data against gravity.
 *
 * @param[in] lsb_per_g     : LSB value pre 1g.
 * @param[in] g_val         : Gravity reference value of all axis.
 * @param[in] data          : Accelerometer data
 * @param[out] comp_data    : Stores the data that is compensated by taking the
 *                            difference in accelerometer data and lsb_per_g
 *                            value.
 *
 */
static void comp_for_gravity(uint16_t lsb_per_g,
                             const struct bma490l_accel_foc_g_value *g_val,
                             const struct bma490l_accel *data,
                             struct bma490l_offset_delta *comp_data);

/*!
 * @brief This internal API scales the compensated accelerometer data according
 * to the offset register resolution.
 *
 * @param[in] range         : Gravity range of the accelerometer.
 * @param[out] comp_data    : Data that is compensated by taking the
 *                            difference in accelerometer data and lsb_per_g
 *                            value.
 * @param[out] data         : Stores offset data
 *
 */
static void scale_accel_offset(uint8_t range,
                               const struct bma490l_offset_delta *comp_data,
                               struct bma490l_accel_offset *data);

/*!
 * @brief This internal API inverts the accelerometer offset data.
 *
 * @param[out] offset_data  : Stores the inverted offset data
 *
 */
static void invert_accel_offset(struct bma490l_accel_offset *offset_data);

/*!
 * @brief This internal API writes the offset data in the offset compensation
 * register.
 *
 * @param[in] offset        : Offset data
 * @param[in] dev           : Structure instance of bma490l_dev
 *
 */
static int8_t write_accel_offset(const struct bma490l_accel_offset *offset, struct bma490l_dev *dev);

/*!
 * @brief This internal API finds the bit position of 3.9mg according to given
 * range and resolution.
 *
 * @param[in] range     : Gravity range of the accelerometer.
 *
 * @return Result of API execution status
 *
 * @return Bit position of 3.9mg
 *
 */
static int8_t get_bit_pos_3_9mg(uint8_t range);

/*!
 * @brief This internal API restores the configurations saved before performing
 * accelerometer FOC.
 *
 * @param[in] acc_cfg       : Accelerometer configuration value
 * @param[in] acc_en        : Accelerometer enable value
 * @param[in] aps           : Advance power mode value
 * @param[in] dev           : Structure instance of bma490l_dev
 *
 * @return Result of API execution status
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
static int8_t restore_accel_foc_config(const struct bma490l_accel_config *acc_cfg,
                                       uint8_t aps,
                                       uint8_t acc_en,
                                       struct bma490l_dev *dev);

/*!
 * @brief This internal API finds the the null error of the device pointer structure
 *
 * @param[in] dev : Structure instance of bma490l_dev.
 *
 *  @return Result of API execution status
 *
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 */
static int8_t null_pointer_check(const struct bma490l_dev *dev);

/***************************************************************************/

/*!                      User Interface Definitions
 ****************************************************************************/

/*!
 * @brief This API is the entry point. Call this API before using all other APIs.
 * This API reads the chip-id of the sensor and sets the resolution, feature
 * length and the type of variant.
 */
int8_t bma490l_init(struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;
    uint8_t data = 0;
    uint8_t dummy_read = 0;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        if (dev->intf == BMA490L_SPI_INTF)
        {
            dev->dummy_byte = 1;
            rslt = bma490l_read_regs(BMA490L_CHIP_ID_ADDR, &dummy_read, 1, dev);
        }
        else
        {
            dev->dummy_byte = 0;
        }

        if (rslt == BMA490L_OK)
        {
            rslt = bma490l_read_regs(BMA490L_CHIP_ID_ADDR, &data, 1, dev);

            if (rslt == BMA490L_OK)
            {
                /* Assign Chip Id */
                dev->chip_id = data;

                if (dev->chip_id == BMA490L_CHIP_ID)
                {
                    /* Resolution of BMA490L sensor is 16 bit */
                    dev->resolution = BMA490L_16_BIT_RESOLUTION;

                    /* Size of configuration file */
                    dev->feature_len = BMA490L_FEATURE_SIZE;

                    dev->config_size = sizeof(bma490l_config_file);
                }
                else
                {
                    rslt = BMA490L_E_INVALID_SENSOR;
                }
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This API checks whether the read operation requested is for feature
 *  or register read and accordingly reads the data from the sensor.
 */
int8_t bma490l_read_regs(uint8_t addr, uint8_t *data, uint32_t len, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;
    uint8_t indx;
    uint32_t loop_count;
    uint16_t overflow;
    uint16_t index;
    uint8_t adv_pwr_save = BMA490L_OK;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (data != NULL))
    {
        if (addr == BMA490L_FEATURE_CONFIG_ADDR)
        {
            /* Disable APS if enabled before reading the feature
             * config register
             */
            rslt = bma490l_get_advance_power_save(&adv_pwr_save, dev);

            if (adv_pwr_save == BMA490L_ENABLE)
            {
                rslt = bma490l_set_advance_power_save(BMA490L_DISABLE, dev);

                /* Wait for sensor time synchronization. Refer
                 * the data-sheet for more information
                 */
                dev->delay_us(450, dev->intf_ptr);
            }

            if (((len % 2) == 0) && (len <= dev->feature_len) && (rslt == BMA490L_OK))
            {
                if (dev->read_write_len < len)
                {
                    /* Calculate the no of writes to be
                     * performed according to the read/write
                     * length
                     */
                    loop_count = len / dev->read_write_len;
                    overflow = len % dev->read_write_len;
                    index = 0;
                    rslt = set_feature_config_start_addr(dev);

                    for (indx = 0; indx < loop_count; indx++)
                    {
                        rslt = read_regs(BMA490L_FEATURE_CONFIG_ADDR, data + index, dev->read_write_len, dev);

                        if (rslt == BMA490L_OK)
                        {
                            rslt = increment_feature_config_addr(dev);
                            index = index + dev->read_write_len;
                        }
                    }

                    if (overflow)
                    {
                        rslt = read_regs(BMA490L_FEATURE_CONFIG_ADDR, data + index, overflow, dev);
                    }

                    if (rslt == BMA490L_OK)
                    {
                        rslt = set_feature_config_start_addr(dev);
                    }
                }
                else
                {
                    rslt = read_regs(BMA490L_FEATURE_CONFIG_ADDR, data, len, dev);
                }
            }
            else
            {
                rslt = BMA490L_E_RD_WR_LENGTH_INVALID;
            }

            if (rslt == BMA490L_OK)
            {
                /* Enable APS if previously enabled */
                if (adv_pwr_save == BMA490L_ENABLE)
                {
                    rslt = bma490l_set_advance_power_save(BMA490L_ENABLE, dev);

                    /* Wait for sensor time synchronization.
                     * Refer the data-sheet for more
                     * information
                     */
                    dev->delay_us(450, dev->intf_ptr);
                }
            }
        }
        else
        {
            rslt = read_regs(addr, data, len, dev);
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API reads the status of advance power save mode
 *  from the sensor.
 */
int8_t bma490l_get_advance_power_save(uint8_t *adv_pwr_save, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;
    uint8_t data = BMA490L_OK;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (adv_pwr_save != NULL))
    {
        rslt = bma490l_read_regs(BMA490L_POWER_CONF_ADDR, &data, 1, dev);

        if (rslt == BMA490L_OK)
        {
            *adv_pwr_save = BMA490L_GET_BITS_POS_0(data, BMA490L_ADVANCE_POWER_SAVE);
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API sets the advance power save mode in the sensor.
 */
int8_t bma490l_set_advance_power_save(uint8_t adv_pwr_save, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;
    uint8_t data = BMA490L_OK;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        rslt = bma490l_read_regs(BMA490L_POWER_CONF_ADDR, &data, 1, dev);

        if (rslt == BMA490L_OK)
        {
            data = BMA490L_SET_BITS_POS_0(data, BMA490L_ADVANCE_POWER_SAVE, adv_pwr_save);
            rslt = bma490l_write_regs(BMA490L_POWER_CONF_ADDR, &data, 1, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API checks whether the write operation requested is for feature
 *  config or register write and accordingly writes the data in the sensor.
 */
int8_t bma490l_write_regs(uint8_t addr, const uint8_t *data, uint32_t len, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;
    uint8_t indx;
    uint32_t loop_count;
    uint16_t overflow;
    uint16_t index;
    uint8_t adv_pwr_save;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (data != NULL))
    {
        if (addr == BMA490L_FEATURE_CONFIG_ADDR)
        {
            /* Disable APS if enabled before writing the feature
             * config register
             */
            rslt = bma490l_get_advance_power_save(&adv_pwr_save, dev);

            if ((adv_pwr_save == BMA490L_ENABLE) && (rslt == BMA490L_OK))
            {
                rslt = bma490l_set_advance_power_save(BMA490L_DISABLE, dev);

                /* Wait for sensor time synchronization. Refer
                 * the data-sheet for more information
                 */
                dev->delay_us(450, dev->intf_ptr);
            }

            if (((len % 2) == 0) && (len <= dev->feature_len) && (rslt == BMA490L_OK))
            {
                if (dev->read_write_len < len)
                {
                    /* Calculate the no of writes to be
                     * performed according to the read/write
                     * length
                     */
                    loop_count = len / dev->read_write_len;
                    overflow = len % dev->read_write_len;
                    index = 0;
                    rslt = set_feature_config_start_addr(dev);

                    for (indx = 0; indx < loop_count; indx++)
                    {
                        rslt = write_regs(BMA490L_FEATURE_CONFIG_ADDR, data + index, dev->read_write_len, dev);

                        if (rslt == BMA490L_OK)
                        {
                            rslt = increment_feature_config_addr(dev);
                            index = index + dev->read_write_len;
                        }
                    }

                    if (overflow)
                    {
                        rslt = write_regs(BMA490L_FEATURE_CONFIG_ADDR, data + index, overflow, dev);
                    }

                    if (rslt == BMA490L_OK)
                    {
                        rslt = set_feature_config_start_addr(dev);
                    }
                }
                else
                {
                    rslt = write_regs(BMA490L_FEATURE_CONFIG_ADDR, data, len, dev);
                }
            }
            else
            {
                rslt = BMA490L_E_RD_WR_LENGTH_INVALID;
            }

            if (rslt == BMA490L_OK)
            {
                /* Enable APS if previously enabled */
                if (adv_pwr_save == BMA490L_ENABLE)
                {
                    rslt = bma490l_set_advance_power_save(BMA490L_ENABLE, dev);

                    /* Wait for sensor time synchronization.
                     * Refer the data-sheet for more
                     * information
                     */
                    dev->delay_us(450, dev->intf_ptr);
                }
            }
        }
        else
        {
            rslt = write_regs(addr, data, len, dev);
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API is used to upload the configuration file to enable the
 * features of the sensor.
 */
int8_t bma490l_write_config_file(struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        /* Validate read/write length */
        if ((dev->read_write_len >= BMA490L_RD_WR_MIN_LEN) && (dev->read_write_len <= BMA490L_RD_WR_MAX_LEN))
        {
            /* Check if even or odd */
            if ((dev->read_write_len % 2) != 0)
            {
                dev->read_write_len = dev->read_write_len - 1;
            }

            /* Assign stream data */
            dev->config_file_ptr = bma490l_config_file;

            /* Load the configuration file */
            rslt = config_file_download(dev);
        }
        else
        {
            rslt = BMA490L_E_RD_WR_LENGTH_INVALID;
        }
    }

    return rslt;
}

/*!
 *  @brief This API enables or disables the Accel in the sensor.
 */
int8_t bma490l_set_accel_enable(uint8_t accel_en, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;
    uint8_t data = BMA490L_OK;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        rslt = bma490l_read_regs(BMA490L_POWER_CTRL_ADDR, &data, 1, dev);

        if (rslt == BMA490L_OK)
        {
            data = BMA490L_SET_BITSLICE(data, BMA490L_ACCEL_ENABLE, accel_en);
            rslt = bma490l_write_regs(BMA490L_POWER_CTRL_ADDR, &data, 1, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API checks whether Accel is enabled or not in the sensor.
 */
int8_t bma490l_get_accel_enable(uint8_t *accel_en, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (accel_en != NULL))
    {
        rslt = bma490l_read_regs(BMA490L_POWER_CTRL_ADDR, &data, 1, dev);

        if (rslt == BMA490L_OK)
        {
            *accel_en = BMA490L_GET_BITSLICE(data, BMA490L_ACCEL_ENABLE);
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API sets the output_data_rate, bandwidth, perf_mode
 *  and range of Accel.
 */
int8_t bma490l_set_accel_config(const struct bma490l_accel_config *accel, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;
    uint8_t accel_config_data[2] = { 0, 0 };

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (accel != NULL))
    {
        /* Check whether the bandwidth and performance mode
         * settings are valid
         */
        rslt = validate_bandwidth_perfmode(accel->bandwidth, accel->perf_mode);

        if (rslt == BMA490L_OK)
        {
            /* Check ODR is valid */
            rslt = validate_odr(accel->odr);

            if (rslt == BMA490L_OK)
            {
                accel_config_data[0] = accel->odr & BMA490L_ACCEL_ODR_MSK;
                accel_config_data[0] |= (uint8_t)(accel->bandwidth << BMA490L_ACCEL_BW_POS);
                accel_config_data[0] |= (uint8_t)(accel->perf_mode << BMA490L_ACCEL_PERFMODE_POS);
                accel_config_data[1] = accel->range & BMA490L_ACCEL_RANGE_MSK;

                /* Flag bit to store the performance mode status */
                dev->perf_mode_status =
                    ((accel_config_data[0] & BMA490L_ACCEL_PERFMODE_MSK) >> BMA490L_ACCEL_PERFMODE_POS);

                rslt = bma490l_write_regs(BMA490L_ACCEL_CONFIG_ADDR, &accel_config_data[0], 1, dev);

                if (rslt == BMA490L_OK)
                {
                    rslt = bma490l_write_regs((BMA490L_ACCEL_CONFIG_ADDR + 1), &accel_config_data[1], 1, dev);
                }
            }
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API reads the Output data rate, Bandwidth, perf_mode
 *  and Range of accel.
 */
int8_t bma490l_get_accel_config(struct bma490l_accel_config *accel, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;
    uint8_t data[2] = { 0 };

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (accel != NULL))
    {
        rslt = bma490l_read_regs(BMA490L_ACCEL_CONFIG_ADDR, data, BMA490L_ACCEL_CONFIG_LENGTH, dev);

        if (rslt == BMA490L_OK)
        {
            /* To get the ODR */
            accel->odr = BMA490L_GET_BITS_POS_0(data[0], BMA490L_ACCEL_ODR);

            /* To get the bandwidth */
            accel->bandwidth = BMA490L_GET_BITSLICE(data[0], BMA490L_ACCEL_BW);

            /* To get the under sampling mode */
            accel->perf_mode = BMA490L_GET_BITSLICE(data[0], BMA490L_ACCEL_PERFMODE);

            /* Read the Accel range */
            accel->range = BMA490L_GET_BITS_POS_0(data[1], BMA490L_ACCEL_RANGE);

            /* Flag bit to store the performance mode status */
            dev->perf_mode_status = accel->perf_mode;
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API is used to get the configuration id of the sensor.
 */
int8_t bma490l_get_config_id(uint16_t *config_id, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    /* Initialize configuration file */
    uint8_t feature_config[BMA490L_FEATURE_SIZE] = { 0 };

    /* Initialize index to get configuration id */
    uint8_t index = BMA490L_CONFIG_ID_START_ADDR;

    /* Variable to define MSB of configuration id */
    uint16_t config_id_lsb = BMA490L_OK;

    /* Variable to define LSB of configuration id */
    uint16_t config_id_msb = BMA490L_OK;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (config_id != NULL))
    {
        /* Read the configuration file */
        rslt = bma490l_read_regs(BMA490L_FEATURE_CONFIG_ADDR, feature_config, BMA490L_FEATURE_SIZE, dev);

        /* Get configuration id */
        if (rslt == BMA490L_OK)
        {
            config_id_lsb = (uint16_t)feature_config[index];
            config_id_msb = ((uint16_t)feature_config[index + 1]) << 8;
            *config_id = config_id_lsb | config_id_msb;
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API writes the available sensor specific commands
 *  to the sensor.
 */
int8_t bma490l_set_command_register(uint8_t command_reg, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        /* Write command register */
        rslt = bma490l_write_regs(BMA490L_CMD_ADDR, &command_reg, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This API maps/un-maps the user provided interrupt to either interrupt
 * pin1 or pin2 in the sensor.
 */
int8_t bma490l_map_interrupt(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    /* Variable to define limit for interrupt line */
    uint16_t limit = 1;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        /* Validate the interrupt line */
        if (int_line <= limit)
        {
            /* Map/Un-map the interrupt */
            rslt = map_int(int_line, int_map, enable, dev);
        }
        else
        {
            rslt = BMA490L_E_INT_LINE_INVALID;
        }
    }

    return rslt;
}

/*!
 *  @brief This API reads the Feature and Hardware interrupt status from the sensor.
 */
int8_t bma490l_read_int_status(uint16_t *int_status, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;
    uint8_t data[2] = { 0 };

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (int_status != NULL))
    {
        rslt = bma490l_read_regs(BMA490L_INT_STAT_0_ADDR, data, 2, dev);

        if (rslt == BMA490L_OK)
        {
            *int_status = data[0];
            *((uint8_t *)int_status + 1) = data[1];
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API performs x, y and z-axis re-mapping in the sensor.
 */
int8_t bma490l_set_remap_axes(const struct bma490l_axes_remap *remap_data, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    /* Initialize configuration file */
    uint8_t feature_config[BMA490L_FEATURE_SIZE] = { 0 };

    /* Initialize index to set re-mapped data */
    uint8_t index = BMA490L_AXES_REMAP_START_ADDR;

    /* Variable to define x-axis to be re-mapped */
    uint8_t x_axis;

    /* Variable to define y-axis to be re-mapped */
    uint8_t y_axis;

    /* Variable to define z-axis to be re-mapped */
    uint8_t z_axis;

    /* Variable to define x-axis sign to be re-mapped */
    uint8_t x_axis_sign;

    /* Variable to define y-axis sign to be re-mapped */
    uint8_t y_axis_sign;

    /* Variable to define z-axis sign to be re-mapped */
    uint8_t z_axis_sign;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (remap_data != NULL))
    {
        /* Read the configuration file */
        rslt = bma490l_read_regs(BMA490L_FEATURE_CONFIG_ADDR, feature_config, BMA490L_FEATURE_SIZE, dev);

        if (rslt == BMA490L_OK)
        {
            /* Get x-axis to be re-mapped */
            x_axis = remap_data->x_axis & BMA490L_X_AXIS_MSK;

            /* Get x-axis sign to be re-mapped */
            x_axis_sign = (remap_data->x_axis_sign << BMA490L_X_AXIS_SIGN_POS) & BMA490L_X_AXIS_SIGN_MSK;

            /* Get y-axis to be re-mapped */
            y_axis = (remap_data->y_axis << BMA490L_Y_AXIS_POS) & BMA490L_Y_AXIS_MSK;

            /* Get y-axis sign to be re-mapped */
            y_axis_sign = (remap_data->y_axis_sign << BMA490L_Y_AXIS_SIGN_POS) & BMA490L_Y_AXIS_SIGN_MSK;

            /* Get z-axis to be re-mapped */
            z_axis = (remap_data->z_axis << BMA490L_Z_AXIS_POS) & BMA490L_Z_AXIS_MSK;

            /* Get z-axis sign to be re-mapped */
            z_axis_sign = remap_data->z_axis_sign & BMA490L_Z_AXIS_SIGN_MSK;

            /* Set the first byte for axis re-mapping */
            feature_config[index] = x_axis | x_axis_sign | y_axis | y_axis_sign | z_axis;

            /* Set the second byte for axis re-mapping */
            feature_config[index + 1] = z_axis_sign;

            /* Set the re-mapped axes */
            rslt = bma490l_write_regs(BMA490L_FEATURE_CONFIG_ADDR, feature_config, BMA490L_FEATURE_SIZE, dev);
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API reads the x, y and z axis re-mapped data from the sensor.
 */
int8_t bma490l_get_remap_axes(struct bma490l_axes_remap *remap_data, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    /* Initialize configuration file */
    uint8_t feature_config[BMA490L_FEATURE_SIZE] = { 0 };

    /* Initialize index to get re-mapped data */
    uint8_t index = BMA490L_AXES_REMAP_START_ADDR;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (remap_data != NULL))
    {
        /* Read the configuration file */
        rslt = bma490l_read_regs(BMA490L_FEATURE_CONFIG_ADDR, feature_config, BMA490L_FEATURE_SIZE, dev);

        if (rslt == BMA490L_OK)
        {
            /* Get re-mapped x-axis */
            remap_data->x_axis = BMA490L_GET_BITS_POS_0(feature_config[index], BMA490L_X_AXIS);

            /* Get re-mapped x-axis sign */
            remap_data->x_axis_sign = BMA490L_GET_BITSLICE(feature_config[index], BMA490L_X_AXIS_SIGN);

            /* Get re-mapped y-axis */
            remap_data->y_axis = BMA490L_GET_BITSLICE(feature_config[index], BMA490L_Y_AXIS);

            /* Get re-mapped y-axis sign */
            remap_data->y_axis_sign = BMA490L_GET_BITSLICE(feature_config[index], BMA490L_Y_AXIS_SIGN);

            /* Get re-mapped z-axis */
            remap_data->z_axis = BMA490L_GET_BITSLICE(feature_config[index], BMA490L_Z_AXIS);

            /* Get re-mapped z-axis sign */
            remap_data->z_axis_sign = BMA490L_GET_BITS_POS_0(feature_config[index + 1], BMA490L_Z_AXIS_SIGN);
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API sets the configuration of any-motion feature in the sensor.
 * This API enables/disables the any-motion feature according to the axis set.
 */
int8_t bma490l_set_any_mot_config(const struct bma490l_any_no_mot_config *any_mot, struct bma490l_dev *dev)
{

    /* Variable to hold execution status */
    int8_t rslt;

    /* Initialize configuration file */
    uint8_t feature_config[BMA490L_FEATURE_SIZE] = { 0 };

    /* Update index to configure any-motion axes */
    uint8_t index = BMA490L_ANY_MOT_START_ADDR;

    /* Variable to define LSB */
    uint16_t lsb = BMA490L_OK;

    /* Variable to define MSB */
    uint16_t msb = BMA490L_OK;

    /* Variable to define LSB and MSB */
    uint16_t lsb_msb = BMA490L_OK;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (any_mot != NULL))
    {
        rslt = bma490l_set_advance_power_save(BMA490L_DISABLE, dev);

        /* Wait for sensor time synchronization. Refer the data-sheet for
         * more information
         */
        dev->delay_us(450, dev->intf_ptr);

        if (rslt == BMA490L_OK)
        {
            /* Get any-motion configuration from the sensor */
            rslt = bma490l_read_regs(BMA490L_FEATURE_CONFIG_ADDR, feature_config, BMA490L_FEATURE_SIZE, dev);

            if (rslt == BMA490L_OK)
            {
                /* Set threshold value in feature configuration array */
                feature_config[index++] = BMA490L_GET_LSB(any_mot->threshold);
                feature_config[index++] = BMA490L_GET_MSB(any_mot->threshold);

                /* Extract the word where duration and axes enable
                 * resides
                 */
                lsb = feature_config[index];
                msb = feature_config[index + 1] << 8;
                lsb_msb = lsb | msb;

                /* Set the duration in the same word */
                lsb_msb = BMA490L_SET_BITS_POS_0(lsb_msb, BMA490L_ANY_NO_MOT_DUR, any_mot->duration);

                /* Set the axes in the same word */
                lsb_msb = BMA490L_SET_BITSLICE(lsb_msb, BMA490L_ANY_NO_MOT_AXIS_EN, any_mot->axes_en);

                /* Assign the word with set duration and axes enable
                 * value back to feature configuration array
                 */
                feature_config[index++] = BMA490L_GET_LSB(lsb_msb);
                feature_config[index] = BMA490L_GET_MSB(lsb_msb);

                /* Set any-motion configuration to the sensor */
                rslt = bma490l_write_regs(BMA490L_FEATURE_CONFIG_ADDR, feature_config, BMA490L_FEATURE_SIZE, dev);
            }
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API gets the configuration of any-motion feature from the
 * sensor.
 */
int8_t bma490l_get_any_mot_config(struct bma490l_any_no_mot_config *any_mot, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    /* Initialize configuration file */
    uint8_t feature_config[BMA490L_FEATURE_SIZE] = { 0 };

    /* Update index to configure any-motion axes */
    uint8_t index = BMA490L_ANY_MOT_START_ADDR;

    /* Variable to define LSB */
    uint16_t lsb = BMA490L_OK;

    /* Variable to define MSB */
    uint16_t msb = BMA490L_OK;

    /* Variable to define LSB and MSB */
    uint16_t lsb_msb = BMA490L_OK;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (any_mot != NULL))
    {
        rslt = bma490l_set_advance_power_save(BMA490L_DISABLE, dev);

        /* Wait for sensor time synchronization. Refer the data-sheet for
         * more information
         */
        dev->delay_us(450, dev->intf_ptr);

        if (rslt == BMA490L_OK)
        {
            /* Get any-motion configuration from the sensor */
            rslt = bma490l_read_regs(BMA490L_FEATURE_CONFIG_ADDR, feature_config, BMA490L_FEATURE_SIZE, dev);

            if (rslt == BMA490L_OK)
            {
                /* Get word to calculate threshold and any-motion
                 * select
                 */
                lsb = (uint16_t)feature_config[index++];
                msb = ((uint16_t)feature_config[index++] << 8);
                lsb_msb = lsb | msb;

                /* Extract threshold value */
                any_mot->threshold = lsb_msb & BMA490L_ANY_NO_MOT_THRES_MSK;

                /* Get word to calculate duration and axes enable */
                lsb = (uint16_t)feature_config[index++];
                msb = ((uint16_t)feature_config[index] << 8);
                lsb_msb = lsb | msb;

                /* Extract duration value */
                any_mot->duration = lsb_msb & BMA490L_ANY_NO_MOT_DUR_MSK;

                /* Extract axes enable value */
                any_mot->axes_en =
                    (uint8_t)((lsb_msb & BMA490L_ANY_NO_MOT_AXIS_EN_MSK) >> BMA490L_ANY_NO_MOT_AXIS_EN_POS);
            }
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API sets the configuration of no-motion feature in the sensor.
 * This API enables/disables the no-motion feature according to the axis set.
 */
int8_t bma490l_set_no_mot_config(const struct bma490l_any_no_mot_config *no_mot, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    /* Initialize configuration file */
    uint8_t feature_config[BMA490L_FEATURE_SIZE] = { 0 };

    /* Update index to configure no-motion axes */
    uint8_t index = BMA490L_NO_MOT_START_ADDR;

    /* Variable to define LSB */
    uint16_t lsb = BMA490L_OK;

    /* Variable to define MSB */
    uint16_t msb = BMA490L_OK;

    /* Variable to define LSB and MSB */
    uint16_t lsb_msb = BMA490L_OK;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (no_mot != NULL))
    {
        /* Get no-motion configuration from the sensor */
        rslt = bma490l_read_regs(BMA490L_FEATURE_CONFIG_ADDR, feature_config, BMA490L_FEATURE_SIZE, dev);

        if (rslt == BMA490L_OK)
        {
            /* Set threshold value in feature configuration array */
            feature_config[index++] = BMA490L_GET_LSB(no_mot->threshold);
            feature_config[index++] = BMA490L_GET_MSB(no_mot->threshold);

            /* Extract the word where duration and axes enable
             * resides
             */
            lsb = feature_config[index];
            msb = feature_config[index + 1] << 8;
            lsb_msb = lsb | msb;

            /* Set the duration in the same word */
            lsb_msb = BMA490L_SET_BITS_POS_0(lsb_msb, BMA490L_ANY_NO_MOT_DUR, no_mot->duration);

            /* Set the axes in the same word */
            lsb_msb = BMA490L_SET_BITSLICE(lsb_msb, BMA490L_ANY_NO_MOT_AXIS_EN, no_mot->axes_en);

            /* Assign the word with set duration and axes enable
             * value back to feature configuration array
             */
            feature_config[index++] = BMA490L_GET_LSB(lsb_msb);
            feature_config[index] = BMA490L_GET_MSB(lsb_msb);

            /* Set no-motion configuration to the sensor */
            rslt = bma490l_write_regs(BMA490L_FEATURE_CONFIG_ADDR, feature_config, BMA490L_FEATURE_SIZE, dev);
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API gets the configuration of no-motion feature from the
 * sensor.
 */
int8_t bma490l_get_no_mot_config(struct bma490l_any_no_mot_config *no_mot, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    /* Initialize configuration file */
    uint8_t feature_config[BMA490L_FEATURE_SIZE] = { 0 };

    /* Update index to configure no-motion axes */
    uint8_t index = BMA490L_NO_MOT_START_ADDR;

    /* Variable to define LSB */
    uint16_t lsb = BMA490L_OK;

    /* Variable to define MSB */
    uint16_t msb = BMA490L_OK;

    /* Variable to define LSB and MSB */
    uint16_t lsb_msb = BMA490L_OK;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (no_mot != NULL))
    {
        /* Get no-motion configuration from the sensor */
        rslt = bma490l_read_regs(BMA490L_FEATURE_CONFIG_ADDR, feature_config, BMA490L_FEATURE_SIZE, dev);

        if (rslt == BMA490L_OK)
        {
            /* Get word to calculate threshold and no-motion
             * select
             */
            lsb = (uint16_t)feature_config[index++];
            msb = ((uint16_t)feature_config[index++] << 8);
            lsb_msb = lsb | msb;

            /* Extract threshold value */
            no_mot->threshold = lsb_msb & BMA490L_ANY_NO_MOT_THRES_MSK;

            /* Get word to calculate duration and axes enable */
            lsb = (uint16_t)feature_config[index++];
            msb = ((uint16_t)feature_config[index] << 8);
            lsb_msb = lsb | msb;

            /* Extract duration value */
            no_mot->duration = lsb_msb & BMA490L_ANY_NO_MOT_DUR_MSK;

            /* Extract axes enable value */
            no_mot->axes_en = (uint8_t)((lsb_msb & BMA490L_ANY_NO_MOT_AXIS_EN_MSK) >> BMA490L_ANY_NO_MOT_AXIS_EN_POS);
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API is used to get the config file major and minor information.
 */
int8_t bma490l_get_version_config(uint16_t *config_major, uint16_t *config_minor, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    /* Initialize configuration file */
    uint8_t feature_config[BMA490L_FEATURE_SIZE] = { 0 };

    /* Update index to config file version */
    uint8_t index = BMA490L_CONFIG_ID_START_ADDR;

    /* Variable to define LSB */
    uint8_t lsb = BMA490L_OK;

    /* Variable to define MSB */
    uint8_t msb = BMA490L_OK;

    /* Variable to define LSB and MSB */
    uint16_t lsb_msb = BMA490L_OK;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (config_major != NULL) && (config_minor != NULL))
    {
        rslt = bma490l_set_advance_power_save(BMA490L_DISABLE, dev);

        /* Wait for sensor time synchronization. Refer the data-sheet for
         * more information
         */
        dev->delay_us(450, dev->intf_ptr);

        if (rslt == BMA490L_OK)
        {
            /* Get config file identification from the sensor */
            rslt = bma490l_read_regs(BMA490L_FEATURE_CONFIG_ADDR, feature_config, BMA490L_FEATURE_SIZE, dev);

            if (rslt == BMA490L_OK)
            {
                /* Get word to calculate config file identification */
                lsb = feature_config[index++];
                msb = feature_config[index++];
                lsb_msb = (uint16_t)(msb << 8 | lsb);

                /* Get major and minor version */
                *config_major = BMA490L_GET_BITSLICE(lsb_msb, BMA490L_CONFIG_MAJOR);
                *config_minor = BMA490L_GET_BITS_POS_0(lsb, BMA490L_CONFIG_MINOR);
            }
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API reads the sensor time when sensor time gets updated
 *  with every update of data register or FIFO.
 */
int8_t bma490l_get_sensor_time(uint32_t *sensor_time, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint8_t data[BMA490L_SENSOR_TIME_LENGTH] = { 0 };
    uint8_t msb = 0;
    uint8_t xlsb = 0;
    uint8_t lsb = 0;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (sensor_time != NULL))
    {
        rslt = bma490l_read_regs(BMA490L_SENSORTIME_0_ADDR, data, BMA490L_SENSOR_TIME_LENGTH, dev);

        if (rslt == BMA490L_OK)
        {
            msb = data[BMA490L_SENSOR_TIME_MSB_BYTE];
            xlsb = data[BMA490L_SENSOR_TIME_XLSB_BYTE];
            lsb = data[BMA490L_SENSOR_TIME_LSB_BYTE];
            *sensor_time = (uint32_t)((msb << 16) | (xlsb << 8) | lsb);
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API reads the FIFO data of Accel and/or Mag sensor
 */
int8_t bma490l_read_fifo_data(struct bma490l_fifo_frame *fifo, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint8_t data = 0;
    uint8_t addr = BMA490L_FIFO_DATA_ADDR;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (fifo != NULL) && (fifo->data != NULL))
    {
        /* Reset the fifo data structure */
        reset_fifo_data_structure(fifo);

        /* Read the fifo data */
        rslt = bma490l_read_regs(addr, fifo->data, fifo->length, dev);

        if (rslt == BMA490L_OK)
        {
            /* Read fifo frame content configuration */
            rslt = bma490l_read_regs(BMA490L_FIFO_CONFIG_1_ADDR, &data, 1, dev);
        }

        /* Filter fifo header enabled status */
        fifo->fifo_header_enable = data & BMA490L_FIFO_HEADER;

        /* Filter accel/mag data enabled status */
        fifo->fifo_data_enable = data & BMA490L_FIFO_MAG_ACC_ENABLE;
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API parses and extracts the accelerometer frames from
 *  FIFO data read by the "bma490l_read_fifo_data" API and stores it in the
 *  "accel_data" structure instance.
 */
int8_t bma490l_extract_accel(struct bma490l_accel *accel_data,
                             uint16_t *accel_length,
                             struct bma490l_fifo_frame *fifo,
                             const struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint16_t data_index = 0;
    uint16_t accel_index = 0;
    uint16_t data_read_length = 0;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (fifo != NULL) && (fifo->data != NULL) && (accel_length != NULL) &&
        (accel_data != NULL))
    {
        /* Parsing the FIFO data in header-less mode */
        if (fifo->fifo_header_enable == 0)
        {
            get_accel_len_to_parse(&data_index, &data_read_length, accel_length, fifo);

            for (; data_index < data_read_length;)
            {
                unpack_acc_frm(accel_data, &data_index, &accel_index, fifo->fifo_data_enable, fifo, dev);

                /* Checks for the availability of next
                 * two bytes of FIFO data
                 */
                check_empty_fifo(&data_index, fifo);
            }

            /* Update number of accel data read */
            *accel_length = accel_index;

            /* Update the accel byte index */
            fifo->accel_byte_start_idx = data_index;
        }
        else
        {
            /* Parsing the FIFO data in header mode */
            extract_accel_header_mode(accel_data, accel_length, fifo, dev);
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API reads the FIFO water mark level which is set
 *  in the sensor.
 */
int8_t bma490l_get_fifo_wm(uint16_t *fifo_wm, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint8_t data[2] = { 0, 0 };

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (fifo_wm != NULL))
    {
        /* Read the FIFO water mark level */
        rslt = bma490l_read_regs(BMA490L_FIFO_WTM_0_ADDR, data, BMA490L_FIFO_WM_LENGTH, dev);

        if (rslt == BMA490L_OK)
        {
            *fifo_wm = (data[1] << 8) | (data[0]);
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API sets the FIFO watermark level in the sensor.
 */
int8_t bma490l_set_fifo_wm(uint16_t fifo_wm, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint8_t data[2] = { 0, 0 };

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        data[0] = BMA490L_GET_LSB(fifo_wm);
        data[1] = BMA490L_GET_MSB(fifo_wm);

        /* Consecutive write is not possible in suspend mode hence
         * separate write is used with delay of 1 ms
         */

        /* Write the fifo watermark level */
        rslt = bma490l_write_regs(BMA490L_FIFO_WTM_0_ADDR, &data[0], 1, dev);
        dev->delay_us(BMA490L_GEN_READ_WRITE_DELAY, dev->intf_ptr);

        if (rslt == BMA490L_OK)
        {
            rslt = bma490l_write_regs((BMA490L_FIFO_WTM_0_ADDR + 1), &data[1], 1, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API checks whether the Accel FIFO data is set for filtered
 *  or unfiltered mode.
 */
int8_t bma490l_get_accel_fifo_filter_data(uint8_t *accel_fifo_filter, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint8_t data = 0;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (accel_fifo_filter != NULL))
    {
        /* Read the Accel FIFO filter data */
        rslt = bma490l_read_regs(BMA490L_FIFO_DOWN_ADDR, &data, 1, dev);

        if (rslt == BMA490L_OK)
        {
            *accel_fifo_filter = BMA490L_GET_BITSLICE(data, BMA490L_FIFO_FILTER_ACCEL);
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API sets the condition of Accel FIFO data either to
 *  filtered or unfiltered mode.
 */
int8_t bma490l_set_accel_fifo_filter_data(uint8_t accel_fifo_filter, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint8_t data = 0;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        if (accel_fifo_filter <= BMA490L_MAX_VALUE_FIFO_FILTER)
        {
            rslt = bma490l_read_regs(BMA490L_FIFO_DOWN_ADDR, &data, 1, dev);

            if (rslt == BMA490L_OK)
            {
                /* Write Accel FIFO filter data */
                data = BMA490L_SET_BITSLICE(data, BMA490L_FIFO_FILTER_ACCEL, accel_fifo_filter);
                rslt = bma490l_write_regs(BMA490L_FIFO_DOWN_ADDR, &data, 1, dev);
            }
        }
        else
        {
            rslt = BMA490L_E_OUT_OF_RANGE;
        }
    }

    return rslt;
}

/*!
 *  @brief This API reads the down sampling rates which is configured
 *  for Accel FIFO data.
 */
int8_t bma490l_get_fifo_down_accel(uint8_t *fifo_down, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint8_t data = 0;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (fifo_down != NULL))
    {
        /* Read the Accel FIFO down data */
        rslt = bma490l_read_regs(BMA490L_FIFO_DOWN_ADDR, &data, 1, dev);

        if (rslt == BMA490L_OK)
        {
            *fifo_down = BMA490L_GET_BITSLICE(data, BMA490L_FIFO_DOWN_ACCEL);
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API sets the down-sampling rates for Accel FIFO.
 */
int8_t bma490l_set_fifo_down_accel(uint8_t fifo_down, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint8_t data = 0;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        /* Write the Accel FIFO down data */
        rslt = bma490l_read_regs(BMA490L_FIFO_DOWN_ADDR, &data, 1, dev);

        if (rslt == BMA490L_OK)
        {
            data = BMA490L_SET_BITSLICE(data, BMA490L_FIFO_DOWN_ACCEL, fifo_down);
            rslt = bma490l_write_regs(BMA490L_FIFO_DOWN_ADDR, &data, 1, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API reads the length of FIFO data available in the sensor
 *  in the units of bytes.
 */
int8_t bma490l_get_fifo_length(uint16_t *fifo_length, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint8_t index = 0;
    uint8_t data[BMA490L_FIFO_DATA_LENGTH] = { 0, 0 };

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (fifo_length != NULL))
    {
        /* Read FIFO length */
        rslt = bma490l_read_regs(BMA490L_FIFO_LENGTH_0_ADDR, data, BMA490L_FIFO_DATA_LENGTH, dev);

        if (rslt == BMA490L_OK)
        {
            index = BMA490L_FIFO_LENGTH_MSB_BYTE;
            data[index] = BMA490L_GET_BITS_POS_0(data[index], BMA490L_FIFO_BYTE_COUNTER_MSB);
            *fifo_length = ((data[index] << 8) | data[index - 1]);
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*! @brief This API sets the FIFO configuration in the sensor. */
int8_t bma490l_set_fifo_config(uint8_t config, uint8_t enable, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint8_t data[2] = { 0, 0 };
    uint8_t fifo_config_0 = config & BMA490L_FIFO_CONFIG_0_MASK;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        rslt = bma490l_read_regs(BMA490L_FIFO_CONFIG_0_ADDR, data, BMA490L_FIFO_CONFIG_LENGTH, dev);

        if (rslt == BMA490L_OK)
        {
            if (fifo_config_0 > 0)
            {
                if (enable == TRUE)
                {
                    data[0] = data[0] | fifo_config_0;
                }
                else
                {
                    data[0] = data[0] & (~fifo_config_0);
                }
            }

            if (enable == TRUE)
            {
                data[1] = data[1] | (config & BMA490L_FIFO_CONFIG_1_MASK);
            }
            else
            {
                data[1] = data[1] & (~(config & BMA490L_FIFO_CONFIG_1_MASK));
            }

            /* Burst write is not possible in suspend mode hence
             * separate write is used with delay of 1 ms
             */
            rslt = bma490l_write_regs(BMA490L_FIFO_CONFIG_0_ADDR, &data[0], 1, dev);
            dev->delay_us(BMA490L_GEN_READ_WRITE_DELAY, dev->intf_ptr);

            if (rslt == BMA490L_OK)
            {
                rslt = bma490l_write_regs((BMA490L_FIFO_CONFIG_0_ADDR + 1), &data[1], 1, dev);
            }
        }
    }

    return rslt;
}

/*! @brief This API reads the FIFO configuration from the sensor.
 */
int8_t bma490l_get_fifo_config(uint8_t *fifo_config, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint8_t data[2] = { 0, 0 };

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (fifo_config != NULL))
    {
        rslt = bma490l_read_regs(BMA490L_FIFO_CONFIG_0_ADDR, data, BMA490L_FIFO_CONFIG_LENGTH, dev);

        if (rslt == BMA490L_OK)
        {
            *fifo_config = ((uint8_t)((data[0] & BMA490L_FIFO_CONFIG_0_MASK) | (data[1])));
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API reads the error status from the sensor.
 */
int8_t bma490l_get_error_status(struct bma490l_err_reg *err_reg, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint8_t data = 0;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (err_reg != NULL))
    {
        /* Read the error codes */
        rslt = bma490l_read_regs(BMA490L_ERROR_ADDR, &data, 1, dev);

        if (rslt == BMA490L_OK)
        {
            /* Fatal error */
            err_reg->fatal_err = BMA490L_GET_BITS_POS_0(data, BMA490L_FATAL_ERR);

            /* Cmd error */
            err_reg->cmd_err = BMA490L_GET_BITSLICE(data, BMA490L_CMD_ERR);

            /* User error */
            err_reg->err_code = BMA490L_GET_BITSLICE(data, BMA490L_ERR_CODE);

            /* FIFO error */
            err_reg->fifo_err = BMA490L_GET_BITSLICE(data, BMA490L_FIFO_ERR);
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API sets the FIFO self wake up functionality in the sensor.
 */
int8_t bma490l_set_fifo_self_wakeup(uint8_t fifo_self_wakeup, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint8_t data = 0;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        rslt = bma490l_read_regs(BMA490L_POWER_CONF_ADDR, &data, 1, dev);

        if (rslt == BMA490L_OK)
        {
            data = BMA490L_SET_BITSLICE(data, BMA490L_FIFO_SELF_WAKE_UP, fifo_self_wakeup);
            rslt = bma490l_write_regs(BMA490L_POWER_CONF_ADDR, &data, 1, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API gets the status of FIFO self wake up functionality from
 *  the sensor.
 */
int8_t bma490l_get_fifo_self_wakeup(uint8_t *fifo_self_wake_up, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint8_t data = 0;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (fifo_self_wake_up != NULL))
    {
        rslt = bma490l_read_regs(BMA490L_POWER_CONF_ADDR, &data, 1, dev);

        if (rslt == BMA490L_OK)
        {
            *fifo_self_wake_up = BMA490L_GET_BITSLICE(data, BMA490L_FIFO_SELF_WAKE_UP);
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API reads the Accel data for x,y and z-axis from the sensor.
 *  The data units is in LSB format.
 */
int8_t bma490l_read_accel_xyz(struct bma490l_accel *accel, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint16_t lsb = 0;
    uint16_t msb = 0;
    uint8_t data[BMA490L_ACCEL_DATA_LENGTH] = { 0 };

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (accel != NULL))
    {
        rslt = bma490l_read_regs(BMA490L_DATA_8_ADDR, data, BMA490L_ACCEL_DATA_LENGTH, dev);

        if (rslt == BMA490L_OK)
        {
            msb = data[1];
            lsb = data[0];

            /* Accel data x-axis */
            accel->x = (int16_t)((msb << 8) | lsb);
            msb = data[3];
            lsb = data[2];

            /* Accel data y-axis */
            accel->y = (int16_t)((msb << 8) | lsb);
            msb = data[5];
            lsb = data[4];

            /* Accel data z-axis */
            accel->z = (int16_t)((msb << 8) | lsb);

            if (dev->resolution == BMA490L_12_BIT_RESOLUTION)
            {
                accel->x = (accel->x / 0x10);
                accel->y = (accel->y / 0x10);
                accel->z = (accel->z / 0x10);
            }
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API checks whether the self-test functionality of the sensor
 *  is working or not.
 *  The following parameter of struct bma490l_dev, should have the valid value
 *  before performing the self-test,
 *  1.  Variant and 2. Resolution
 */
int8_t bma490l_perform_accel_selftest(struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    struct bma490l_accel positive = { 0, 0, 0 };
    struct bma490l_accel negative = { 0, 0, 0 };

    /*! Structure for difference of accel values in mg */
    struct bma490l_self_test_delta_limit accel_data_diff_mg = { 0, 0, 0 };

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        rslt = set_accel_selftest_config(dev);

        /* Wait for 2ms after accel self-test config please refer data sheet data sheet 4.9. sensor self-test */
        dev->delay_us(BMA490L_MS_TO_US(2), dev->intf_ptr);

        if (rslt == BMA490L_OK)
        {
            rslt = bma490l_selftest_config(BMA490L_ENABLE, dev);
        }

        if (rslt == BMA490L_OK)
        {
            /* Taking positive data */

            /* User should wait 50ms before interpreting the acceleration data.
             * please refer data sheet 4.10. sensor self-test
             */
            dev->delay_us(BMA490L_MS_TO_US(50), dev->intf_ptr);
            rslt = bma490l_read_accel_xyz(&positive, dev);
            if (rslt == BMA490L_OK)
            {
                rslt = bma490l_selftest_config(BMA490L_DISABLE, dev);
            }

            if (rslt == BMA490L_OK)
            {
                /* User should wait 50ms before interpreting the acceleration data.
                 * please refer data sheet 4.10. sensor self-test
                 */
                dev->delay_us(BMA490L_MS_TO_US(50), dev->intf_ptr);
                rslt = bma490l_read_accel_xyz(&negative, dev);
                if (rslt == BMA490L_OK)
                {
                    rslt = get_accel_data_difference_and_validate(positive, negative, &accel_data_diff_mg, dev);
                }

                if (rslt == BMA490L_OK)
                {
                    /* Triggers a soft-reset */
                    rslt = bma490l_soft_reset(dev);
                    dev->delay_us(BMA490L_MS_TO_US(200), dev->intf_ptr);
                }
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This API performs the steps needed for self-test operation
 *  before reading the Accel self-test data.
 */
int8_t bma490l_selftest_config(uint8_t sign, struct bma490l_dev *dev)
{
    /* Variable to define the error */
    int8_t rslt;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        rslt = set_accel_selftest_enable(BMA490L_ENABLE, dev);

        if (rslt == BMA490L_OK)
        {
            rslt = set_accel_selftest_sign(sign, dev);

            /* Set self-test amplitude based on variant */
            if (rslt == BMA490L_OK)
            {
                /* Set self-test amplitude to high for BMA490L */
                rslt = set_accel_selftest_amp(BMA490L_ENABLE, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API gets the data ready status of accelerometer,
 * auxiliary, command decoder and busy status of auxiliary.
 */
int8_t bma490l_get_status(uint8_t *status, struct bma490l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (status != NULL))
    {
        rslt = bma490l_read_regs(BMA490L_STATUS_ADDR, status, 1, dev);
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API performs Fast Offset Compensation for accelerometer.
 */
int8_t bma490l_perform_accel_foc(const struct bma490l_accel_foc_g_value *accel_g_value, struct bma490l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Structure to define the accelerometer configurations */
    struct bma490l_accel_config acc_cfg = { 0, 0, 0, 0 };

    /* Variable to store status of advance power save */
    uint8_t aps = 0;

    /* Variable to store status of accelerometer enable */
    uint8_t acc_en = 0;

    uint8_t accel_en = 0;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (accel_g_value != NULL))
    {
        /* Check for input validity */
        if ((((ABS(accel_g_value->x)) + (ABS(accel_g_value->y)) + (ABS(accel_g_value->z))) == 1) &&
            ((accel_g_value->sign == 1) || (accel_g_value->sign == 0)))
        {
            rslt = bma490l_set_accel_enable(BMA490L_ENABLE, dev);

            if (rslt == BMA490L_OK)
            {
                rslt = bma490l_get_accel_enable(&accel_en, dev);
            }

            if (rslt == BMA490L_OK)
            {
                rslt = verify_foc_position(BMA490L_ACCEL_EN, accel_g_value, dev);
            }

            if (rslt == BMA490L_OK)
            {

                /* Save accelerometer configurations, accelerometer
                 * enable status and advance power save status
                 */
                rslt = save_accel_foc_config(&acc_cfg, &aps, &acc_en, dev);
            }

            /* Set configurations for FOC */
            if (rslt == BMA490L_OK)
            {
                rslt = set_accel_foc_config(dev);
            }

            /* Perform accelerometer FOC */
            if (rslt == BMA490L_OK)
            {
                rslt = perform_accel_foc(accel_g_value, &acc_cfg, dev);
            }

            /* Restore the saved configurations */
            if (rslt == BMA490L_OK)
            {
                rslt = restore_accel_foc_config(&acc_cfg, aps, acc_en, dev);
            }
        }
        else
        {
            rslt = BMA490L_E_OUT_OF_RANGE;
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API does soft-reset
 */
int8_t bma490l_soft_reset(struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint8_t command_reg = BMA490L_SOFT_RESET;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        /* Write command register */
        rslt = bma490l_write_regs(BMA490L_CMD_ADDR, &command_reg, 1, dev);
    }

    return rslt;
}

/***************************************************************************/

/*!         Local Function Definitions
 ****************************************************************************/

/*!
 *  @brief This internal API reads the 8-bit data from the given register
 *  in the sensor.
 */
static int8_t read_regs(uint8_t addr, uint8_t *data, uint32_t len, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint32_t temp_len = len + dev->dummy_byte;
    uint16_t indx;
    uint8_t temp_buff[temp_len];

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (data != NULL))
    {
        if (dev->intf == BMA490L_SPI_INTF)
        {
            /* SPI mask added */
            addr = addr | BMA490L_SPI_RD_MASK;
        }

        /* Read the data from the register */
        dev->intf_rslt = dev->bus_read(addr, temp_buff, temp_len, dev->intf_ptr);

        if (dev->intf_rslt == BMA490L_INTF_RET_SUCCESS)
        {
            for (indx = 0; indx < len; indx++)
            {
                /* Parsing and storing the valid data */
                data[indx] = temp_buff[indx + dev->dummy_byte];
            }
        }
        else
        {
            rslt = BMA490L_E_COM_FAIL;
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This internal API increments the feature config. data address according to the user
 *  provided read/write length in the dev structure.
 */
static int8_t increment_feature_config_addr(struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint16_t asic_addr;
    uint8_t asic_lsb;
    uint8_t asic_msb;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        /* Read the asic address from the sensor */
        rslt = read_regs(BMA490L_RESERVED_REG_5B_ADDR, &asic_lsb, 1, dev);

        if (rslt == BMA490L_OK)
        {
            rslt = read_regs(BMA490L_RESERVED_REG_5C_ADDR, &asic_msb, 1, dev);

            if (rslt == BMA490L_OK)
            {
                /* Get the asic address */
                asic_addr = (asic_msb << 4) | (asic_lsb & 0x0F);

                /* Sum the asic address with read/write length after converting from
                 * byte to word
                 */
                asic_addr = asic_addr + (dev->read_write_len / 2);

                /* Split the asic address */
                asic_lsb = asic_addr & 0x0F;
                asic_msb = (uint8_t)(asic_addr >> 4);

                /* Write the asic address in the sensor */
                rslt = write_regs(BMA490L_RESERVED_REG_5B_ADDR, &asic_lsb, 1, dev);

                if (rslt == BMA490L_OK)
                {
                    rslt = write_regs(BMA490L_RESERVED_REG_5C_ADDR, &asic_msb, 1, dev);
                }
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This internal API sets the feature config. data start address in the sensor.
 */
static int8_t set_feature_config_start_addr(struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        rslt = write_regs(BMA490L_RESERVED_REG_5B_ADDR, &dev->asic_data.asic_lsb, 1, dev);

        if (rslt == BMA490L_OK)
        {
            rslt = write_regs(BMA490L_RESERVED_REG_5C_ADDR, &dev->asic_data.asic_msb, 1, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This internal API writes the 8-bit data to the given register
 *  in the sensor.
 */
static int8_t write_regs(uint8_t addr, const uint8_t *data, uint32_t len, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (data != NULL))
    {
        if (dev->intf == BMA490L_SPI_INTF)
        {
            addr = addr & BMA490L_SPI_WR_MASK;
        }

        /* Writes data in the register */
        dev->intf_rslt = dev->bus_write(addr, data, len, dev->intf_ptr);

        if (dev->intf_rslt != BMA490L_INTF_RET_SUCCESS)
        {
            rslt = BMA490L_E_COM_FAIL;
        }
        else
        {
            /* After write operation 2us delay is required when device operates in performance mode whereas 450us
             * is required when the device operates in suspend and low power mode.
             * NOTE: For more information refer datasheet section 6.6 */
            if (dev->perf_mode_status == BMA490L_ENABLE)
            {
                dev->delay_us(2, dev->intf_ptr);
            }
            else
            {
                dev->delay_us(450, dev->intf_ptr);
            }
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This internal API is used to write the binary configuration in the sensor
 */
static int8_t config_file_download(struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint8_t config_load = BMA490L_OK;
    uint16_t index = BMA490L_OK;
    uint8_t config_stream_status = BMA490L_OK;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        /* Disable advanced power save */
        rslt = bma490l_set_advance_power_save(BMA490L_DISABLE, dev);

        /* Wait for sensor time synchronization. Refer the data-sheet for
         * more information
         */
        dev->delay_us(450, dev->intf_ptr);

        if (rslt == BMA490L_OK)
        {
            /* Disable config loading */
            rslt = bma490l_write_regs(BMA490L_INIT_CTRL_ADDR, &config_load, 1, dev);

            /* Write the config stream */
            for (index = 0;
                 ((index < dev->config_size) && (rslt == BMA490L_OK));
                 index += dev->read_write_len)
            {
                rslt = stream_transfer_write((dev->config_file_ptr + index), index, dev);
            }

            /* Enable config loading and FIFO mode */
            config_load = 0x01;
            rslt = bma490l_write_regs(BMA490L_INIT_CTRL_ADDR, &config_load, 1, dev);

            /* Wait till ASIC is initialized. Refer the data-sheet for
             * more information
             */
            dev->delay_us(BMA490L_MS_TO_US(150), dev->intf_ptr);

            if (rslt == BMA490L_OK)
            {
                /* Read the status of config stream operation */
                rslt = bma490l_read_regs(BMA490L_INTERNAL_STAT, &config_stream_status, 1, dev);

                config_stream_status = config_stream_status & BMA490L_CONFIG_STREAM_MESSAGE_MSK;

                if (config_stream_status != BMA490L_ASIC_INITIALIZED)
                {
                    rslt = BMA490L_E_CONFIG_STREAM_ERROR;
                }
                else
                {
                    /* Enable advanced power save */
                    rslt = bma490l_set_advance_power_save(BMA490L_ENABLE, dev);

                    if (rslt == BMA490L_OK)
                    {
                        rslt = get_feature_config_start_addr(dev);
                    }
                }
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This internal API writes the config stream data in memory using burst mode
 *  @note index value should be even number.
 */
static int8_t  stream_transfer_write(const uint8_t *stream_data, uint16_t index, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint8_t asic_msb = (uint8_t)((index / 2) >> 4);
    uint8_t asic_lsb = ((index / 2) & 0x0F);

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (stream_data != NULL))
    {
        rslt = bma490l_write_regs(BMA490L_RESERVED_REG_5B_ADDR, &asic_lsb, 1, dev);

        if (rslt == BMA490L_OK)
        {
            rslt = bma490l_write_regs(BMA490L_RESERVED_REG_5C_ADDR, &asic_msb, 1, dev);

            if (rslt == BMA490L_OK)
            {
                rslt = write_regs(BMA490L_FEATURE_CONFIG_ADDR, (uint8_t *)stream_data, dev->read_write_len, dev);
            }
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This internal API sets the feature config. data start address in the sensor.
 */
static int8_t get_feature_config_start_addr(struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint8_t asic_lsb = BMA490L_OK;
    uint8_t asic_msb = BMA490L_OK;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        rslt = read_regs(BMA490L_RESERVED_REG_5B_ADDR, &asic_lsb, 1, dev);

        if (rslt == BMA490L_OK)
        {
            rslt = read_regs(BMA490L_RESERVED_REG_5C_ADDR, &asic_msb, 1, dev);

            if (rslt == BMA490L_OK)
            {
                /* Store asic info in dev structure */
                dev->asic_data.asic_lsb = asic_lsb & 0x0F;
                dev->asic_data.asic_msb = asic_msb;
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This internal API sets the interrupt to either interrupt1 or
 *  interrupt2 pin in the sensor.
 */
static int8_t map_int(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint8_t data[3] = { 0, 0, 0 };
    uint8_t index[2] = { BMA490L_INT_MAP_1_ADDR, BMA490L_INT_MAP_2_ADDR };

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        rslt = bma490l_read_regs(BMA490L_INT_MAP_1_ADDR, data, 3, dev);

        if (rslt == BMA490L_OK)
        {
            if (enable == TRUE)
            {
                /* Feature interrupt mapping */
                data[int_line] = (uint8_t)(int_map & (0x00FF));

                /* Hardware interrupt mapping */
                data[2] = (uint8_t)((int_map & (0xFF00)) >> 8);
            }
            else
            {
                /* Feature interrupt un-mapping */
                data[int_line] &= (~(uint8_t)(int_map & (0x00FF)));

                /* Hardware interrupt un-mapping */
                data[2] &= (~(uint8_t)((int_map & (0xFF00)) >> 8));
            }

            rslt = bma490l_write_regs(index[int_line], &data[int_line], 1, dev);

            if (rslt == BMA490L_OK)
            {
                rslt = bma490l_write_regs(BMA490L_INT_MAP_DATA_ADDR, &data[2], 1, dev);
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This internal API validates the bandwidth and perf_mode
 *  value set by the user.
 */
static int8_t validate_bandwidth_perfmode(uint8_t bandwidth, uint8_t perf_mode)
{
    /* Variable to hold execution status */
    int8_t rslt = BMA490L_OK;

    if (perf_mode == BMA490L_CONTINUOUS_MODE)
    {
        if (bandwidth > BMA490L_ACCEL_NORMAL_AVG4)
        {
            /* Invalid bandwidth error for continuous mode */
            rslt = BMA490L_E_OUT_OF_RANGE;
        }
    }
    else if (perf_mode == BMA490L_CIC_AVG_MODE)
    {
        if (bandwidth > BMA490L_ACCEL_RES_AVG128)
        {
            /* Invalid bandwidth error for CIC avg. mode */
            rslt = BMA490L_E_OUT_OF_RANGE;
        }
    }
    else
    {
        rslt = BMA490L_E_OUT_OF_RANGE;
    }

    return rslt;
}

/*!
 *  @brief This internal API validates the ODR value set by the user.
 */
static int8_t validate_odr(uint8_t odr)
{
    /* Variable to hold execution status */
    int8_t rslt = BMA490L_OK;

    if ((odr < BMA490L_OUTPUT_DATA_RATE_0_78HZ) || (odr > BMA490L_OUTPUT_DATA_RATE_1600HZ))
    {
        /* If ODR is not valid return error */
        rslt = BMA490L_E_OUT_OF_RANGE;
    }

    return rslt;
}

/*!
 *  @brief This internal API is used to reset the FIFO related configurations
 *  in the fifo_frame structure.
 */
static void reset_fifo_data_structure(struct bma490l_fifo_frame *fifo)
{
    /* Prepare for next FIFO read by resetting FIFO's
     * internal data structures
     */
    fifo->accel_byte_start_idx = 0;
    fifo->sensor_time = 0;
    fifo->skipped_frame_count = 0;
    fifo->accel_dropped_frame_count = 0;
}

/*!
 *  @brief This internal API computes the number of bytes of accel FIFO data
 *  which is to be parsed in header-less mode
 */
static void get_accel_len_to_parse(uint16_t *start_idx,
                                   uint16_t *len,
                                   const uint16_t *acc_count,
                                   const struct bma490l_fifo_frame *fifo)
{
    /* Data start index */
    *start_idx = fifo->accel_byte_start_idx;

    if (fifo->fifo_data_enable == BMA490L_FIFO_ACC_ENABLE)
    {
        /* Len has the number of bytes to loop for */
        *len = (uint16_t)(((*acc_count) * BMA490L_FIFO_ACC_LENGTH));
    }
    else if (fifo->fifo_data_enable == BMA490L_FIFO_MAG_ACC_ENABLE)
    {
        /* Len has the number of bytes to loop for */
        *len = (uint16_t)(((*acc_count) * BMA490L_FIFO_MAG_ACC_LENGTH));
    }
    else
    {
        /* Only aux. sensor or no sensor is enabled in FIFO,
         * so there will be no accel data.
         * Update the data index as complete.
         */
        *start_idx = fifo->length;
    }

    if ((*len) > fifo->length)
    {
        /* Handling the case where more data is requested
         * than available.
         */
        *len = fifo->length;
    }
}

/*!
 *  @brief This internal API checks the fifo read data as empty frame, if it
 *  is empty frame then moves the index to last byte.
 */
static void check_empty_fifo(uint16_t *data_index, const struct bma490l_fifo_frame *fifo)
{
    if ((*data_index + 2) < fifo->length)
    {
        /* Check if FIFO is empty */
        if ((fifo->data[*data_index] == BMA490L_FIFO_MSB_CONFIG_CHECK) &&
            (fifo->data[*data_index + 1] == BMA490L_FIFO_LSB_CONFIG_CHECK))
        {
            /* Updates the data index as complete */
            *data_index = fifo->length;
        }
    }
}

/*!
 *  @brief This internal API is used to parse the accelerometer data from the
 *  FIFO data in header mode.
 */
static void extract_accel_header_mode(struct bma490l_accel *accel_data,
                                      uint16_t *accel_length,
                                      struct bma490l_fifo_frame *fifo,
                                      const struct bma490l_dev *dev)
{
    uint8_t frame_header = 0;
    uint16_t data_index;
    uint16_t accel_index = 0;
    uint16_t frame_to_read = *accel_length;

    for (data_index = fifo->accel_byte_start_idx; data_index < fifo->length;)
    {
        /* Header byte is stored in the variable frame_header */
        frame_header = fifo->data[data_index];

        /* Get the frame details from header */
        frame_header = frame_header & BMA490L_FIFO_TAG_INTR_MASK;

        /* Index is moved to next byte where the data is starting */
        data_index++;

        switch (frame_header)
        {
            /* Accel frame */
            case BMA490L_FIFO_HEADER_ACC:
            case BMA490L_FIFO_HEADER_MAG_ACC:
                unpack_acc_frm(accel_data, &data_index, &accel_index, frame_header, fifo, dev);
                break;

            /* Aux sensor frame */
            case BMA490L_FIFO_HEADER_MAG:
                move_next_frame(&data_index, BMA490L_FIFO_MAG_LENGTH, fifo);
                break;

            /* Sensor time frame */
            case BMA490L_FIFO_HEADER_SENSOR_TIME:
                unpack_sensortime_frame(&data_index, fifo);
                break;

            /* Skip frame */
            case BMA490L_FIFO_HEADER_SKIP_FRAME:
                unpack_skipped_frame(&data_index, fifo);
                break;

            /* Input config frame */
            case BMA490L_FIFO_HEADER_INPUT_CONFIG:
                move_next_frame(&data_index, 1, fifo);
                break;

            /* Sample drop frame */
            case BMA490L_FIFO_HEADER_SAMPLE_DROP:
                unpack_dropped_frame(&data_index, fifo);
                break;

            /* Over read FIFO data */
            case BMA490L_FIFO_HEADER_OVER_READ_MSB:

                /* Update the data index as complete */
                data_index = fifo->length;
                break;
            default:
                break;
        }

        if (frame_to_read == accel_index)
        {
            /* Number of frames to read completed */
            break;
        }
    }

    /* Updates number of accel data read */
    *accel_length = accel_index;

    /* Updates the accel frame index */
    fifo->accel_byte_start_idx = data_index;
}

/*!
 *  @brief This internal API is used to parse the accelerometer data from the
 *  FIFO data in both header mode and header-less mode.
 *  It updates the idx value which is used to store the index of
 *  the current data byte which is parsed.
 */
static void unpack_acc_frm(struct bma490l_accel *acc,
                           uint16_t *idx,
                           uint16_t *acc_idx,
                           uint8_t frm,
                           const struct bma490l_fifo_frame *fifo,
                           const struct bma490l_dev *dev)
{

    switch (frm)
    {
        case BMA490L_FIFO_HEADER_ACC:
        case BMA490L_FIFO_ACC_ENABLE:

            /* Partial read, then skip the data */
            if ((*idx + BMA490L_FIFO_ACC_LENGTH) > fifo->length)
            {
                /* Update the data index as complete */
                *idx = fifo->length;
                break;
            }

            /* Unpack the data array into the structure instance "acc" */
            unpack_accel_data(&acc[*acc_idx], *idx, fifo, dev);

            /* Moves the data index */
            *idx = *idx + BMA490L_FIFO_ACC_LENGTH;
            (*acc_idx)++;
            break;
        case BMA490L_FIFO_HEADER_MAG_ACC:
        case BMA490L_FIFO_MAG_ACC_ENABLE:

            /* Partial read, then skip the data */
            if ((*idx + BMA490L_FIFO_MAG_ACC_LENGTH) > fifo->length)
            {
                /* Update the data index as complete */
                *idx = fifo->length;
                break;
            }

            /* Unpack the data array into structure instance "acc" */
            unpack_accel_data(&acc[*acc_idx], *idx + BMA490L_MA_FIFO_A_X_LSB, fifo, dev);

            /* Moves the data index */
            *idx = *idx + BMA490L_FIFO_MAG_ACC_LENGTH;
            (*acc_idx)++;
            break;

        /* Aux sensor frame */
        case BMA490L_FIFO_HEADER_MAG:
        case BMA490L_FIFO_MAG_ENABLE:
            (*idx) = (*idx) + BMA490L_FIFO_MAG_LENGTH;
            break;
        default:
            break;
    }
}

/*!
 *  @brief This internal API is used to parse the accelerometer data from the
 *  FIFO data and store it in the instance of the structure bma490l_accel and bma490l_fifo_frame.
 */
static void unpack_accel_data(struct bma490l_accel *accel_data,
                              uint16_t data_start_index,
                              const struct bma490l_fifo_frame *fifo,
                              const struct bma490l_dev *dev)
{
    uint16_t data_lsb;
    uint16_t data_msb;

    /* Accel raw x data */
    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    accel_data->x = (int16_t)((data_msb << 8) | data_lsb);

    /* Accel raw y data */
    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    accel_data->y = (int16_t)((data_msb << 8) | data_lsb);

    /* Accel raw z data */
    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    accel_data->z = (int16_t)((data_msb << 8) | data_lsb);

    if (dev->resolution == BMA490L_12_BIT_RESOLUTION)
    {
        accel_data->x = (accel_data->x / 0x10);
        accel_data->y = (accel_data->y / 0x10);
        accel_data->z = (accel_data->z / 0x10);
    }
}

/*!
 *  @brief This internal API is used to parse and store the sensor time from the
 *  FIFO data in the structure instance fifo.
 *
 */
static void unpack_sensortime_frame(uint16_t *data_index, struct bma490l_fifo_frame *fifo)
{
    uint32_t sensor_time_byte3 = 0;
    uint16_t sensor_time_byte2 = 0;
    uint8_t sensor_time_byte1 = 0;

    /* Partial read, then move the data index to last data */
    if ((*data_index + BMA490L_SENSOR_TIME_LENGTH) > fifo->length)
    {
        /* Update the data index as complete */
        *data_index = fifo->length;
    }
    else
    {
        sensor_time_byte3 = fifo->data[(*data_index) + BMA490L_SENSOR_TIME_MSB_BYTE] << 16;
        sensor_time_byte2 = fifo->data[(*data_index) + BMA490L_SENSOR_TIME_XLSB_BYTE] << 8;
        sensor_time_byte1 = fifo->data[(*data_index)];

        /* Sensor time */
        fifo->sensor_time = (uint32_t)(sensor_time_byte3 | sensor_time_byte2 | sensor_time_byte1);
        *data_index = (*data_index) + BMA490L_SENSOR_TIME_LENGTH;
    }
}

/*!
 *  @brief This internal API is used to parse and store the skipped_frame_count from
 *  the FIFO data in the structure instance fifo.
 */
static void unpack_skipped_frame(uint16_t *data_index, struct bma490l_fifo_frame *fifo)
{
    /* Partial read, then move the data index to last data */
    if (*data_index >= fifo->length)
    {
        /* Update the data index as complete */
        *data_index = fifo->length;
    }
    else
    {
        fifo->skipped_frame_count = fifo->data[*data_index];

        /* Move the data index */
        *data_index = (*data_index) + 1;
    }
}

/*!
 *  @brief This internal API is used to parse and store the dropped_frame_count from
 *  the FIFO data in the structure instance fifo.
 */
static void unpack_dropped_frame(uint16_t *data_index, struct bma490l_fifo_frame *fifo)
{
    uint8_t dropped_frame = 0;

    /* Partial read, then move the data index to last data */
    if (*data_index >= fifo->length)
    {
        /* Update the data index as complete */
        *data_index = fifo->length;
    }
    else
    {
        /* Extract accel and mag dropped frame count */
        dropped_frame = fifo->data[*data_index] & BMA490L_ACCEL_AUX_FIFO_DROP;

        /* Move the data index and update the dropped frame count */
        switch (dropped_frame)
        {
            case BMA490L_ACCEL_FIFO_DROP:
                *data_index = (*data_index) + BMA490L_FIFO_ACC_LENGTH;
                fifo->accel_dropped_frame_count = fifo->accel_dropped_frame_count + 1;
                break;
            default:
                break;
        }
    }
}

/*!
 *  @brief This internal API is used to move the data index ahead of the
 *  current_frame_length parameter when unnecessary FIFO data appears while
 *  extracting the user specified data.
 */
static void move_next_frame(uint16_t *data_index, uint8_t current_frame_length, const struct bma490l_fifo_frame *fifo)
{
    /* Partial read, then move the data index to last data */
    if ((*data_index + current_frame_length) > fifo->length)
    {
        /* Update the data index as complete */
        *data_index = fifo->length;
    }
    else
    {
        /* Move the data index to next frame */
        *data_index = *data_index + current_frame_length;
    }
}

/*!
 *  @brief This internal function enables and configures the Accel which is needed
 *  for self-test operation.
 */
static int8_t set_accel_selftest_config(struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    struct bma490l_accel_config accel = { 0, 0, 0, 0 };

    accel.odr = BMA490L_OUTPUT_DATA_RATE_1600HZ;
    accel.bandwidth = BMA490L_ACCEL_NORMAL_AVG4;
    accel.perf_mode = BMA490L_ENABLE;
    accel.range = BMA490L_ACCEL_RANGE_8G;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        rslt = bma490l_set_accel_enable(BMA490L_ENABLE, dev);

        if (rslt == BMA490L_OK)
        {
            dev->delay_us(BMA490L_MS_TO_US(1), dev->intf_ptr);

            rslt = bma490l_set_accel_config(&accel, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This internal API validates accel self-test status from positive and negative axes input
 */
static int8_t get_accel_data_difference_and_validate(struct bma490l_accel positive,
                                                     struct bma490l_accel negative,
                                                     struct bma490l_self_test_delta_limit *accel_data_diff_mg,
                                                     const struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    /* Structure for difference of accel values in g */
    struct bma490l_self_test_delta_limit accel_data_diff = { 0, 0, 0 };

    accel_data_diff.x = ABS(positive.x - negative.x);
    accel_data_diff.y = ABS(positive.y - negative.y);
    accel_data_diff.z = ABS(positive.z - negative.z);

    /* Converting LSB of the differences of accel values to mg */
    convert_lsb_g(&accel_data_diff, accel_data_diff_mg, dev);

    /* Validating self-test for accel values in mg */
    rslt = validate_selftest(accel_data_diff_mg);

    return rslt;
}

/*!
 *  @brief This internal API enables or disables the Accel self-test feature in the
 *  sensor.
 */
static int8_t set_accel_selftest_enable(uint8_t accel_selftest_enable, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    uint8_t data = 0;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        /* Read the self-test register */
        rslt = bma490l_read_regs(BMA490L_ACC_SELF_TEST_ADDR, &data, 1, dev);

        if (rslt == BMA490L_OK)
        {
            data = BMA490L_SET_BITS_POS_0(data, BMA490L_ACCEL_SELFTEST_ENABLE, accel_selftest_enable);
            rslt = bma490l_write_regs(BMA490L_ACC_SELF_TEST_ADDR, &data, 1, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This internal API selects the sign of Accel self-test excitation.
 */
static int8_t set_accel_selftest_sign(uint8_t accel_selftest_sign, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        if (accel_selftest_sign <= BMA490L_MAX_VALUE_SELFTEST_SIGN)
        {
            /* Read the Accel self-test sign */
            rslt = bma490l_read_regs(BMA490L_ACC_SELF_TEST_ADDR, &data, 1, dev);

            if (rslt == BMA490L_OK)
            {
                data = BMA490L_SET_BITSLICE(data, BMA490L_ACCEL_SELFTEST_SIGN, accel_selftest_sign);
                rslt = bma490l_write_regs(BMA490L_ACC_SELF_TEST_ADDR, &data, 1, dev);
            }
        }
        else
        {
            rslt = BMA490L_E_OUT_OF_RANGE;
        }
    }

    return rslt;
}

/*!
 *  @brief This internal API sets the Accel self-test amplitude in the sensor.
 */
static int8_t set_accel_selftest_amp(uint8_t accel_selftest_amp, struct bma490l_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        if (accel_selftest_amp <= BMA490L_MAX_VALUE_SELFTEST_AMP)
        {
            /* Write self-test amplitude */
            rslt = bma490l_read_regs(BMA490L_ACC_SELF_TEST_ADDR, &data, 1, dev);

            if (rslt == BMA490L_OK)
            {
                data = BMA490L_SET_BITSLICE(data, BMA490L_SELFTEST_AMP, accel_selftest_amp);
                rslt = bma490l_write_regs(BMA490L_ACC_SELF_TEST_ADDR, &data, 1, dev);
            }
        }
        else
        {
            rslt = BMA490L_E_OUT_OF_RANGE;
        }
    }

    return rslt;
}

/*!
 *  @brief This internal API converts lsb value of axes to mg for self-test
 */
static void convert_lsb_g(const struct bma490l_self_test_delta_limit *accel_data_diff,
                          struct bma490l_self_test_delta_limit *accel_data_diff_mg,
                          const struct bma490l_dev *dev)
{
    uint32_t lsb_per_g;

    /* Range considered for self-test is 8g */
    uint8_t range = 8;

    /* lsb_per_g for the respective resolution and 8g range */
    lsb_per_g = (uint32_t)(power(2, dev->resolution) / (2 * range));

    /* Accel x value in mg */
    accel_data_diff_mg->x = (accel_data_diff->x / (int32_t)lsb_per_g) * 1000;

    /* Accel y value in mg */
    accel_data_diff_mg->y = (accel_data_diff->y / (int32_t)lsb_per_g) * 1000;

    /* Accel z value in mg */
    accel_data_diff_mg->z = (accel_data_diff->z / (int32_t)lsb_per_g) * 1000;
}

/*!
 *  @brief This internal function validates the Accel self-test data and decides the
 *  result of self-test operation.
 */
static int8_t validate_selftest(const struct bma490l_self_test_delta_limit *accel_data_diff)
{
    /* Variable to hold execution status */
    int8_t rslt;

    /* Check the parameter is NULL or not */
    if (accel_data_diff != NULL)
    {
        /* Validating accel data by comparing with minimum value of the
         * axes in mg. For BMA42x - > x axis limit 400mg, y axis limit
         * 800mg and z axis limit 400mg
         */
        if ((accel_data_diff->x <= BMA490L_ST_ACC_X_AXIS_SIGNAL_DIFF) &&
            (accel_data_diff->y <= BMA490L_ST_ACC_Y_AXIS_SIGNAL_DIFF) &&
            (accel_data_diff->z <= BMA490L_ST_ACC_Z_AXIS_SIGNAL_DIFF))
        {
            rslt = BMA490L_SELFTEST_DIFF_X_Y_AND_Z_AXIS_FAILED;
        }
        else if ((accel_data_diff->x <= BMA490L_ST_ACC_X_AXIS_SIGNAL_DIFF) &&
                 (accel_data_diff->y <= BMA490L_ST_ACC_Y_AXIS_SIGNAL_DIFF))
        {
            rslt = BMA490L_SELFTEST_DIFF_X_AND_Y_AXIS_FAILED;
        }
        else if ((accel_data_diff->x <= BMA490L_ST_ACC_X_AXIS_SIGNAL_DIFF) &&
                 (accel_data_diff->z <= BMA490L_ST_ACC_Z_AXIS_SIGNAL_DIFF))
        {
            rslt = BMA490L_SELFTEST_DIFF_X_AND_Z_AXIS_FAILED;
        }
        else if ((accel_data_diff->y <= BMA490L_ST_ACC_Y_AXIS_SIGNAL_DIFF) &&
                 (accel_data_diff->z <= BMA490L_ST_ACC_Z_AXIS_SIGNAL_DIFF))
        {
            rslt = BMA490L_SELFTEST_DIFF_Y_AND_Z_AXIS_FAILED;
        }
        else if (accel_data_diff->x <= BMA490L_ST_ACC_X_AXIS_SIGNAL_DIFF)
        {
            rslt = BMA490L_SELFTEST_DIFF_X_AXIS_FAILED;
        }
        else if (accel_data_diff->y <= BMA490L_ST_ACC_Y_AXIS_SIGNAL_DIFF)
        {
            rslt = BMA490L_SELFTEST_DIFF_Y_AXIS_FAILED;
        }
        else if (accel_data_diff->z <= BMA490L_ST_ACC_Z_AXIS_SIGNAL_DIFF)
        {
            rslt = BMA490L_SELFTEST_DIFF_Z_AXIS_FAILED;
        }
        else
        {
            rslt = BMA490L_SELFTEST_PASS;
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API is used to calculate the power of 2
 */
static int32_t power(int16_t base, uint8_t resolution)
{
    uint8_t indx = 1;

    /* Initialize variable to store the power of 2 value */
    int32_t value = 1;

    for (; indx <= resolution; indx++)
    {
        value = (int32_t)(value * base);
    }

    return value;
}

/*!
 * @brief This internal API verifies and allows only the correct position to do Fast Offset Compensation for
 * accelerometer.
 */
static int8_t verify_foc_position(uint8_t accel_en,
                                  const struct bma490l_accel_foc_g_value *accel_g_axis,
                                  struct bma490l_dev *dev)
{
    int8_t rslt;
    struct bma490l_accel avg_foc_data = { 0 };
    struct bma490l_foc_temp_value temp_foc_data = { 0 };

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (accel_g_axis != NULL))
    {
        rslt = get_average_of_sensor_data(BMA490L_ACCEL_EN, &temp_foc_data, dev);

        if (rslt == BMA490L_OK)
        {
            if (accel_en == BMA490L_ACCEL_EN)
            {

                /* Taking modulus to make negative values as positive */
                if ((accel_g_axis->x == 1) && (accel_g_axis->sign == 1))
                {
                    temp_foc_data.x = temp_foc_data.x * -1;
                }
                else if ((accel_g_axis->y == 1) && (accel_g_axis->sign == 1))
                {
                    temp_foc_data.y = temp_foc_data.y * -1;
                }
                else if ((accel_g_axis->z == 1) && (accel_g_axis->sign == 1))
                {
                    temp_foc_data.z = temp_foc_data.z * -1;
                }
            }

            /* Typecasting into 16bit */
            avg_foc_data.x = (int16_t)(temp_foc_data.x);
            avg_foc_data.y = (int16_t)(temp_foc_data.y);
            avg_foc_data.z = (int16_t)(temp_foc_data.z);

            rslt = validate_foc_position(BMA490L_ACCEL_EN, accel_g_axis, avg_foc_data, dev);
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API reads and provides average for 128 samples of sensor data for FOC operation
 */
static int8_t get_average_of_sensor_data(uint8_t accel_en,
                                         struct bma490l_foc_temp_value *temp_foc_data,
                                         struct bma490l_dev *dev)
{
    int8_t rslt;
    struct bma490l_accel sensor_data = { 0 };
    uint8_t sample_count = 0;
    uint8_t datardy_try_cnt;
    uint8_t drdy_status = 0;
    uint8_t sensor_drdy = 0;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (temp_foc_data != NULL))
    {
        if (accel_en == BMA490L_ACCEL_EN)
        {
            sensor_drdy = BMA490L_ACCEL_DRDY;
        }

        /* Read sensor values before FOC */
        while (sample_count < BMA490L_FOC_SAMPLE_LIMIT)
        {
            datardy_try_cnt = 5;
            do
            {
                dev->delay_us(BMA490L_MS_TO_US(20), dev->intf_ptr);
                rslt = bma490l_get_status(&drdy_status, dev);
                datardy_try_cnt--;
            } while ((rslt == BMA490L_OK) && (!(drdy_status & sensor_drdy)) && (datardy_try_cnt));

            if ((rslt != BMA490L_OK) || (datardy_try_cnt == 0))
            {
                rslt = BMA490L_E_FAIL;
                break;
            }

            rslt = bma490l_read_accel_xyz(&sensor_data, dev);

            if (rslt == BMA490L_OK)
            {
                temp_foc_data->x += sensor_data.x;
                temp_foc_data->y += sensor_data.y;
                temp_foc_data->z += sensor_data.z;
            }
            else
            {
                return rslt;
            }

            sample_count++;
        }

        if (rslt == BMA490L_OK)
        {
            temp_foc_data->x = (temp_foc_data->x / BMA490L_FOC_SAMPLE_LIMIT);
            temp_foc_data->y = (temp_foc_data->y / BMA490L_FOC_SAMPLE_LIMIT);
            temp_foc_data->z = (temp_foc_data->z / BMA490L_FOC_SAMPLE_LIMIT);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API validates accel FOC position as per the range
 */
static int8_t validate_foc_position(uint8_t accel_en,
                                    const struct bma490l_accel_foc_g_value *accel_g_axis,
                                    struct bma490l_accel avg_foc_data,
                                    struct bma490l_dev *dev)
{
    int8_t rslt;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (accel_g_axis != NULL))
    {
        if (accel_en == BMA490L_ACCEL_EN)
        {
            if (accel_g_axis->x == 1)
            {
                rslt = validate_foc_accel_axis(avg_foc_data.x, dev);
            }
            else if (accel_g_axis->y == 1)
            {
                rslt = validate_foc_accel_axis(avg_foc_data.y, dev);
            }
            else
            {
                rslt = validate_foc_accel_axis(avg_foc_data.z, dev);
            }
        }
        else
        {
            rslt = BMA490L_E_FAIL;
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API validates depends on accel FOC access input
 */
static int8_t validate_foc_accel_axis(int16_t avg_foc_data, struct bma490l_dev *dev)
{
    struct bma490l_accel_config sens_cfg = { 0 };
    uint8_t range;
    int8_t rslt;
    uint16_t range_2g = 0;
    uint16_t range_4g = 0;
    uint16_t range_8g = 0;
    uint16_t range_16g = 0;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        /* Get configurations for accel */
        rslt = bma490l_get_accel_config(&sens_cfg, dev);
        range = sens_cfg.range;

        /* Calculation. Eg. Range = 2G, Resolution = 12 bit.
         * Value(i.e range_2g) = 2^(Resolution - 1) / Range
         *                     = 2^(12-1) / 2 = 1024
         */
        if (dev->resolution == BMA490L_12_BIT_RESOLUTION)
        {
            range_2g = 1024;
            range_4g = 512;
            range_8g = 256;
            range_16g = 128;
        }
        else if (dev->resolution == BMA490L_16_BIT_RESOLUTION)
        {
            range_2g = 16384;
            range_4g = 8192;
            range_8g = 4096;
            range_16g = 2048;
        }

        /* Reference LSB value of 2G */
        if ((range == BMA490L_ACCEL_RANGE_2G) && (avg_foc_data > BMA490L_MIN_NOISE_LIMIT(range_2g)) &&
            (avg_foc_data < BMA490L_MAX_NOISE_LIMIT(range_2g)))
        {
            rslt = BMA490L_OK;
        }
        /* Reference LSB value of 4G */
        else if ((range == BMA490L_ACCEL_RANGE_4G) && (avg_foc_data > BMA490L_MIN_NOISE_LIMIT(range_4g)) &&
                 (avg_foc_data < BMA490L_MAX_NOISE_LIMIT(range_4g)))
        {
            rslt = BMA490L_OK;
        }
        /* Reference LSB value of 8G */
        else if ((range == BMA490L_ACCEL_RANGE_8G) && (avg_foc_data > BMA490L_MIN_NOISE_LIMIT(range_8g)) &&
                 (avg_foc_data < BMA490L_MAX_NOISE_LIMIT(range_8g)))
        {
            rslt = BMA490L_OK;
        }
        /* Reference LSB value of 16G */
        else if ((range == BMA490L_ACCEL_RANGE_16G) && (avg_foc_data > BMA490L_MIN_NOISE_LIMIT(range_16g)) &&
                 (avg_foc_data < BMA490L_MAX_NOISE_LIMIT(range_16g)))
        {
            rslt = BMA490L_OK;
        }
        else
        {
            rslt = BMA490L_E_FOC_FAIL;
        }
    }

    return rslt;
}

/*!
 * @brief This internal API saves the configurations before performing FOC.
 */
static int8_t save_accel_foc_config(struct bma490l_accel_config *acc_cfg,
                                    uint8_t *aps,
                                    uint8_t *acc_en,
                                    struct bma490l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to get the status from PWR_CTRL register */
    uint8_t pwr_ctrl_data = 0;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (acc_cfg != NULL))
    {
        /* Get accelerometer configurations to be saved */
        rslt = bma490l_get_accel_config(acc_cfg, dev);

        if (rslt == BMA490L_OK)
        {
            /* Get accelerometer enable status to be saved */
            rslt = bma490l_read_regs(BMA490L_POWER_CTRL_ADDR, &pwr_ctrl_data, 1, dev);

            if (rslt == BMA490L_OK)
            {
                *acc_en = BMA490L_GET_BITSLICE(pwr_ctrl_data, BMA490L_ACCEL_ENABLE);
            }

            /* Get advance power save mode to be saved */
            if (rslt == BMA490L_OK)
            {
                rslt = bma490l_get_advance_power_save(aps, dev);
            }
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API sets configurations for performing accelerometer FOC.
 */
static int8_t set_accel_foc_config(struct bma490l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to set the accelerometer configuration value */
    uint8_t acc_conf_data = BMA490L_FOC_ACC_CONF_VAL;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        /* Disabling offset compensation */
        rslt = set_accel_offset_comp(BMA490L_DISABLE, dev);
        if (rslt == BMA490L_OK)
        {
            /* Set accelerometer configurations to 50Hz, continuous mode, CIC mode */
            rslt = bma490l_write_regs(BMA490L_ACCEL_CONFIG_ADDR, &acc_conf_data, 1, dev);

            if (rslt == BMA490L_OK)
            {
                /* Set accelerometer to normal mode by enabling it */
                rslt = bma490l_set_accel_enable(BMA490L_ENABLE, dev);

                if (rslt == BMA490L_OK)
                {
                    /* Disable advance power save mode */
                    rslt = bma490l_set_advance_power_save(BMA490L_DISABLE, dev);
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal API enables/disables the offset compensation for
 * filtered and un-filtered accelerometer data.
 */
static int8_t set_accel_offset_comp(uint8_t offset_en, struct bma490l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t data = 0;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if (rslt == BMA490L_OK)
    {
        /* Enable/Disable offset compensation */
        rslt = bma490l_read_regs(BMA490L_NV_CONFIG_ADDR, &data, 1, dev);

        if (rslt == BMA490L_OK)
        {
            data = BMA490L_SET_BITSLICE(data, BMA490L_NV_ACCEL_OFFSET, offset_en);
            rslt = bma490l_write_regs(BMA490L_NV_CONFIG_ADDR, &data, 1, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API performs Fast Offset Compensation for accelerometer.
 */
static int8_t perform_accel_foc(const struct bma490l_accel_foc_g_value *accel_g_value,
                                const struct bma490l_accel_config *acc_cfg,
                                struct bma490l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define count */
    uint8_t loop;

    /* Variable to store status read from the status register */
    uint8_t reg_status = 0;

    /* Array of structure to store accelerometer data */
    struct bma490l_accel accel_value[128] = { { 0 } };

    /* Structure to store accelerometer data temporarily */
    struct bma490l_foc_temp_value temp = { 0, 0, 0 };

    /* Structure to store the average of accelerometer data */
    struct bma490l_accel accel_avg = { 0, 0, 0 };

    /* Variable to define LSB per g value */
    uint16_t lsb_per_g = 0;

    /* Variable to define range */
    uint8_t range = 0;

    /* Structure to store accelerometer data deviation from ideal value */
    struct bma490l_offset_delta delta = { 0, 0, 0 };

    /* Structure to store accelerometer offset values */
    struct bma490l_accel_offset offset = { 0, 0, 0 };

    /* Variable tries max 5 times for interrupt then generates timeout */
    uint8_t try_cnt;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (accel_g_value != NULL) && (acc_cfg != NULL))
    {
        for (loop = 0; loop < BMA490L_FOC_SAMPLE_LIMIT; loop++)
        {
            try_cnt = 5;
            while (try_cnt && (!(reg_status & BMA490L_ACCEL_DRDY)))
            {
                /* 20ms delay for 50Hz ODR */
                dev->delay_us(BMA490L_MS_TO_US(20), dev->intf_ptr);
                rslt = bma490l_get_status(&reg_status, dev);
                try_cnt--;
            }

            if ((rslt == BMA490L_OK) && (reg_status & BMA490L_ACCEL_DRDY))
            {
                rslt = bma490l_read_accel_xyz(&accel_value[loop], dev);
            }

            if (rslt == BMA490L_OK)
            {
                rslt = bma490l_read_accel_xyz(&accel_value[loop], dev);
            }

            if (rslt == BMA490L_OK)
            {
                /* Store the data in a temporary structure */
                temp.x = temp.x + (int32_t)accel_value[loop].x;
                temp.y = temp.y + (int32_t)accel_value[loop].y;
                temp.z = temp.z + (int32_t)accel_value[loop].z;
            }
            else
            {
                break;
            }
        }

        if (rslt == BMA490L_OK)
        {
            /* Take average of x, y and z data for lesser noise */
            accel_avg.x = (int16_t)(temp.x / 128);
            accel_avg.y = (int16_t)(temp.y / 128);
            accel_avg.z = (int16_t)(temp.z / 128);

            /* Get the exact range value */
            map_accel_range(acc_cfg->range, &range);

            /* Get the smallest possible measurable acceleration level given the range and
             * resolution */
            lsb_per_g = (uint16_t)(power(2, dev->resolution) / (2 * range));

            /* Compensate acceleration data against gravity */
            comp_for_gravity(lsb_per_g, accel_g_value, &accel_avg, &delta);

            /* Scale according to offset register resolution */
            scale_accel_offset(range, &delta, &offset);

            /* Invert the accelerometer offset data */
            invert_accel_offset(&offset);

            /* Write offset data in the offset compensation register */
            rslt = write_accel_offset(&offset, dev);

            /* Enable offset compensation */
            if (rslt == BMA490L_OK)
            {
                rslt = set_accel_offset_comp(BMA490L_ENABLE, dev);
            }
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API converts the accelerometer range value into
 * corresponding integer value.
 */
static void map_accel_range(uint8_t range_in, uint8_t *range_out)
{
    switch (range_in)
    {
        case BMA490L_ACCEL_RANGE_2G:
            *range_out = 2;
            break;
        case BMA490L_ACCEL_RANGE_4G:
            *range_out = 4;
            break;
        case BMA490L_ACCEL_RANGE_8G:
            *range_out = 8;
            break;
        case BMA490L_ACCEL_RANGE_16G:
            *range_out = 16;
            break;
        default:

            /* By default RANGE 8G is set */
            *range_out = 8;
            break;
    }
}

/*!
 * @brief This internal API compensate the accelerometer data against gravity.
 */
static void comp_for_gravity(uint16_t lsb_per_g,
                             const struct bma490l_accel_foc_g_value *g_val,
                             const struct bma490l_accel *data,
                             struct bma490l_offset_delta *comp_data)
{
    /* Array to store the accelerometer values in LSB */
    int16_t accel_value_lsb[3] = { 0 };

    /* Convert g-value to LSB */
    accel_value_lsb[BMA490L_X_AXIS] = (int16_t)(lsb_per_g * g_val->x);
    accel_value_lsb[BMA490L_Y_AXIS] = (int16_t)(lsb_per_g * g_val->y);
    accel_value_lsb[BMA490L_Z_AXIS] = (int16_t)(lsb_per_g * g_val->z);

    /* Get the compensated values for X, Y and Z axis */
    comp_data->x = (data->x - accel_value_lsb[BMA490L_X_AXIS]);
    comp_data->y = (data->y - accel_value_lsb[BMA490L_Y_AXIS]);
    comp_data->z = (data->z - accel_value_lsb[BMA490L_Z_AXIS]);
}

/*!
 * @brief This internal API scales the compensated accelerometer data according
 * to the offset register resolution.
 *
 * @note The bit position is always greater than 0 since accelerometer data is
 * 16 bit wide.
 */
static void scale_accel_offset(uint8_t range,
                               const struct bma490l_offset_delta *comp_data,
                               struct bma490l_accel_offset *data)
{
    /* Variable to store the position of bit having 3.9mg resolution */
    int8_t bit_pos_3_9mg;

    /* Variable to store the position previous of bit having 3.9mg resolution */
    int8_t bit_pos_3_9mg_prev_bit;

    /* Variable to store the round-off value */
    uint8_t round_off;

    /* Find the bit position of 3.9mg */
    bit_pos_3_9mg = get_bit_pos_3_9mg(range);

    /* Round off, consider if the next bit is high */
    bit_pos_3_9mg_prev_bit = bit_pos_3_9mg - 1;
    round_off = (uint8_t)(power(2, ((uint8_t) bit_pos_3_9mg_prev_bit)));

    /* Scale according to offset register resolution */
    data->x = (uint8_t)((comp_data->x + round_off) / power(2, ((uint8_t) bit_pos_3_9mg)));
    data->y = (uint8_t)((comp_data->y + round_off) / power(2, ((uint8_t) bit_pos_3_9mg)));
    data->z = (uint8_t)((comp_data->z + round_off) / power(2, ((uint8_t) bit_pos_3_9mg)));
}

/*!
 * @brief This internal API inverts the accelerometer offset data.
 */
static void invert_accel_offset(struct bma490l_accel_offset *offset_data)
{
    /* Get the offset data */
    offset_data->x = (uint8_t)((offset_data->x) * (-1));
    offset_data->y = (uint8_t)((offset_data->y) * (-1));
    offset_data->z = (uint8_t)((offset_data->z) * (-1));
}

/*!
 * @brief This internal API writes the offset data in the offset compensation
 * register.
 */
static int8_t write_accel_offset(const struct bma490l_accel_offset *offset, struct bma490l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store the offset data */
    uint8_t data_array[3] = { 0 };

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (offset != NULL))
    {
        data_array[0] = offset->x;
        data_array[1] = offset->y;
        data_array[2] = offset->z;

        /* Offset values are written in the offset register */
        rslt = bma490l_write_regs(BMA490L_OFFSET_0_ADDR, data_array, 3, dev);
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API finds the bit position of 3.9mg according to given
 * range and resolution.
 */
static int8_t get_bit_pos_3_9mg(uint8_t range)
{
    /* Variable to store the bit position of 3.9mg resolution */
    int8_t bit_pos_3_9mg;

    /* Variable to shift the bits according to the resolution  */
    uint32_t divisor = 1;

    /* Scaling factor to get the bit position of 3.9 mg resolution */
    int16_t scale_factor = -1;

    /* Variable to store temporary value */
    uint16_t temp;

    /* Shift left by the times of resolution */
    divisor = divisor << BMA490L_16_BIT_RESOLUTION;

    /* Get the bit position to be shifted */
    temp = (uint16_t)(divisor / (range * 256));

    /* Get the scaling factor until bit position is shifted to last bit */
    while (temp != 1)
    {
        scale_factor++;
        temp = temp >> 1;
    }

    /* Scaling factor is the bit position of 3.9 mg resolution */
    bit_pos_3_9mg = (int8_t) scale_factor;

    return bit_pos_3_9mg;
}

/*!
 * @brief This internal API restores the configurations saved before performing
 * accelerometer FOC.
 */
static int8_t restore_accel_foc_config(const struct bma490l_accel_config *acc_cfg,
                                       uint8_t aps,
                                       uint8_t acc_en,
                                       struct bma490l_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to get the status from PWR_CTRL register */
    uint8_t pwr_ctrl_data = 0;

    /* Check the dev structure parameter is NULL or not */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA490L_OK) && (acc_cfg != NULL))
    {
        /* Restore the saved accelerometer configurations */
        rslt = bma490l_set_accel_config(acc_cfg, dev);

        if (rslt == BMA490L_OK)
        {
            /* Restore the saved accelerometer enable status */
            rslt = bma490l_read_regs(BMA490L_POWER_CTRL_ADDR, &pwr_ctrl_data, 1, dev);

            if (rslt == BMA490L_OK)
            {
                pwr_ctrl_data = BMA490L_SET_BITSLICE(pwr_ctrl_data, BMA490L_ACCEL_ENABLE, acc_en);
                rslt = bma490l_write_regs(BMA490L_POWER_CTRL_ADDR, &pwr_ctrl_data, 1, dev);

                /* Restore the saved advance power save */
                if (rslt == BMA490L_OK)
                {
                    rslt = bma490l_set_advance_power_save(aps, dev);
                }
            }
        }
    }
    else
    {
        rslt = BMA490L_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API checks null pointer error
 */
static int8_t null_pointer_check(const struct bma490l_dev *dev)
{
    int8_t rslt = BMA490L_OK;

    if ((dev == NULL) || (dev->bus_read == NULL) || (dev->bus_write == NULL) || (dev->intf_ptr == NULL))
    {
        rslt = BMA490L_E_NULL_PTR;
    }
    else
    {
        rslt = BMA490L_OK;
    }

    return rslt;
}
