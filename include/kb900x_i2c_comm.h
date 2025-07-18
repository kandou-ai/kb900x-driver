/*
 * Copyright (c) Kandou-AI.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _KB_I2C_COMM_H
#define _KB_I2C_COMM_H

#include "kb900x_utils.h"
#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define KB900X_FILENAME_MAX_LENGTH (20)

// Global variable to store the selected slave address
extern uint8_t kb900x_i2c_slave_addr;

/** \brief open an i2c connection
 *
 * \param[in] i2c_id the i2c device id (/dev/i2c-{i2c_id})
 *
 * \return the handler of the I2C connection
 */
int kb900x_i2c_open(int i2c_id);

/** \brief close an i2c connection
 *
 * \param[in] handle the i2c connection handle
 */
void kb900x_i2c_close(int handle);

/** \brief select the I2C slave address
 *
 * \param[in] handle the I2C connection handle
 * \param[in] slave_addr the I2C slave address
 *
 * \return the result code
 */
int kb900x_i2c_select_slave_addr(int handle, uint8_t slave_addr);

/** \brief write an I2C block
 *
 * \param[in] config the config context
 * \param[in] address the address
 * \param[in] address_size the size of the address in bytes
 * \param[in] value the value to write at the address
 *
 * \return the result code
 */
int kb900x_i2c_write(const kb900x_config_t *config, const uint32_t address,
                     const uint8_t address_size, const uint32_t value);

/** \brief read an I2C block
 *
 * \param[in] config the config context
 * \param[in] address the address
 * \param[in] address_size the size of the address in bytes
 * \param[out] value pointer to store the value read
 *
 * \return the result code
 */
int kb900x_i2c_read(const kb900x_config_t *config, const uint32_t address,
                    const uint8_t address_size, uint32_t *value);

#endif // _KB_I2C_COMM_H
