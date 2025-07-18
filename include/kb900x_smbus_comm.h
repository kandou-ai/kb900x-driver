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

#ifndef _KB_SMBUS_COMM_H
#define _KB_SMBUS_COMM_H

#include "kb900x_utils.h"
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/ioctl.h>

/** \brief check if the functionality is supported by the slave device
 *
 * \param[in] handle the I2C handler
 * \param[in] func the functionality (ex: I2C_PEC)
 *
 * \return 0: not supported, anything else: supported
 */
int kb900x_smbus_check_supported_func(int handle, unsigned long func);

/** \brief enable/disable PEC verification
 *
 * \param[in] handle the I2C handler
 * \param[in] enabled true = enabled, false = disabled
 *
 * \return the result code
 */
int kb900x_smbus_pec(int handle, bool enabled);

/** \brief write an SMBus block
 *
 * \note This function uses the I2C_SMBUS_BLOCK_DATA command code.
 *
 * \param[in] config the config context
 * \param[in] address the address
 * \param[in] address_size the size of the address in bytes
 * \param[in] value the value to write at the address
 *
 * \return the result code
 */
int kb900x_smbus_write_block(const kb900x_config_t *config, const uint32_t address,
                             const uint8_t address_size, const uint32_t value);

/** \brief write an SMBus block
 *
 * \note This function uses the I2C_SMBUS_I2C_BLOCK_DATA command code.
 *
 * \param[in] config the config context
 * \param[in] address the address
 * \param[in] address_size the size of the address in bytes
 * \param[in] value the value to write at the address
 *
 * \return the result code
 */
int kb900x_smbus_write_i2c(const kb900x_config_t *config, const uint32_t address,
                           const uint8_t address_size, const uint32_t value);

/** \brief write an SMBus read request and reads the response.
 *
 * \note This function uses the I2C_SMBUS_BLOCK_DATA command code.
 *
 * \param[in] config the config context
 * \param[in] address the address
 * \param[in] address_size the size of the address in bytes
 * \param[out] value pointer to store the value read
 *
 * \return the result code
 */
int kb900x_smbus_read_block(const kb900x_config_t *config, const uint32_t address,
                            const uint8_t address_size, uint32_t *value);

/** \brief write an SMBus read request and reads the response.
 *
 * \note This function uses the I2C_SMBUS_I2C_BLOCK_DATA command code.
 *
 * \param[in] config the config context
 * \param[in] address the address
 * \param[in] address_size the size of the address in bytes
 * \param[out] value pointer to store the value read
 *
 * \return the result code
 */
int kb900x_smbus_read_i2c(const kb900x_config_t *config, const uint32_t address,
                          const uint8_t address_size, uint32_t *value);

#endif // _KB_SMBUS_COMM_H
