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

#ifndef _KB_BIC_COMM_H
#define _KB_BIC_COMM_H

#include "kb900x_utils.h"

/** \brief Write an SMBus block through BIC.
 *
 * \param[in] config the library communication configuration
 * \param[in] address the address (big-endian)
 * \param[in] address_size the size of the address in bytes
 * \param[in] value the payload (big-endian)
 *
 * \return the result code
 */
int kb900x_bic_write(const kb900x_config_t *config, const uint32_t address,
                     const uint8_t address_size, const uint32_t value);

/** \brief Read an SMBus block through BIC.
 *
 * \param[in] config the library communication configuration
 * \param[in] address the address (big-endian)
 * \param[in] address_size the size of the address in bytes
 * \param[out] value pointer to the uint32_t that will store the data read (big-endian)
 *
 * \return the result code
 */
int kb900x_bic_read(const kb900x_config_t *config, const uint32_t address,
                    const uint8_t address_size, uint32_t *value);

#endif // _KB_BIC_COMM_H
