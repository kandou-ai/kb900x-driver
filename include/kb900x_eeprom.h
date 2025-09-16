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

#ifndef _KB_EEPROM_H
#define _KB_EEPROM_H

#include "kb900x_utils.h"
#include <stdint.h>
#include <stdlib.h>

/**
 * @brief Configuration structure for EEPROM.
 *
 * This structure holds the configuration parameters required
 * for initializing and interacting with the EEPROM device.
 */
typedef struct {
    /**
     * @brief The I2C address of the EEPROM slave.
     *
     * This is the 7-bit I2C slave address used to communicate
     * with the EEPROM.
     */
    uint8_t slave_addr;

    /**
     * @brief The total size of the EEPROM in bytes.
     *
     * Specifies the total memory capacity of the EEPROM.
     */
    uint32_t eeprom_size;

    /**
     * @brief The page size of the EEPROM in bytes.
     *
     * Specifies the size of a page in the EEPROM. This value
     * is used to determine the maximum amount of data that can
     * be written in a single write operation.
     */
    uint32_t page_size;

    /**
     * @brief Write cycle time for EEPROM in milliseconds.
     *
     * Specifies the time needed to complete one write cycle to
     * the EEPROM in milliseconds. It is important to respect
     * this timing to ensure proper data writing.
     */
    unsigned write_cycle_time_ms;
} kb900x_eeprom_config_t;

/** \brief Write data to the EEPROM.
 *
 * \param[in] config the config context
 * \param[in] addr the start address to write to
 * \param[in] payload the data to write
 * \param[in] payload_size the size of the data to write
 * \param[in] eeprom_config the eeprom config structure
 *
 * \return 0 if no error, else the error code
 */
int kb900x_eeprom_write(const kb900x_config_t *config, const uint32_t addr, const uint8_t *payload,
                        const size_t payload_size, const kb900x_eeprom_config_t *eeprom_config);

/** \brief Read data from the EEPROM.
 *
 * \param[in] config the config context
 * \param[in] addr the start address to read from
 * \param[in] length the size of the data to read (in bytes)
 * \param[out] result the buffer to store the read data
 * \param[in] eeprom_config the eeprom config structure
 *
 * \return 0 if no error, else the error code
 */
int kb900x_eeprom_read(const kb900x_config_t *config, const uint32_t addr, const size_t length,
                       uint8_t *result, const kb900x_eeprom_config_t *eeprom_config);

#endif // _KB_EEPROM_H
