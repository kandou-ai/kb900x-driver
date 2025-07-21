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

#include "kb900x_eeprom.h"
#include "kb900x_i2c_master.h"
#include "kb900x_utils.h"
#include <string.h>
#include <time.h>

// FIXME too coupled with firmware
int kb900x_eeprom_write(const kb900x_config_t *config, const uint16_t addr, const uint8_t *payload,
                        const size_t payload_size, const kb900x_eeprom_config_t *eeprom_config)
{
    if (payload == NULL || config == NULL || eeprom_config == NULL) {
        KANDOU_ERR("Invalid parameters: make sure payload and config are not null");
        return -EINVAL;
    }
    const size_t QUARTER_SIZE_BYTES = 1 << 16; // If 16 bits addressing
    const size_t i2c_max_write_size = eeprom_config->page_size < KB900X_TX_FIFO_DEPTH
                                          ? eeprom_config->page_size
                                          : KB900X_TX_FIFO_DEPTH;
    // Making sure the page size is at least 3 bytes (2 address bytes and one data byte)
    if (i2c_max_write_size <= 2) {
        KANDOU_ERR("Invalid page size");
        return -EINVAL;
    }
    if ((addr + payload_size) > eeprom_config->eeprom_size) {
        KANDOU_ERR("Write out of memory bounds! addr = 0x%02x length=%zu", addr, payload_size);
        return -EINVAL;
    }
    size_t bytes_written = 0;
    while (bytes_written < payload_size) {
        size_t global_addr = addr + bytes_written;
        uint8_t sa = eeprom_config->slave_addr + (global_addr / QUARTER_SIZE_BYTES);
        uint16_t page_addr = global_addr % QUARTER_SIZE_BYTES;

        size_t write_size = ((i2c_max_write_size - 2) < (payload_size - bytes_written))
                                ? (i2c_max_write_size - 2)
                                : (payload_size - bytes_written);

        if (global_addr / QUARTER_SIZE_BYTES <
            (global_addr + write_size - 1) / QUARTER_SIZE_BYTES) {
            write_size = QUARTER_SIZE_BYTES - (global_addr % QUARTER_SIZE_BYTES);
        }
        if (global_addr / eeprom_config->page_size <
            (global_addr + write_size - 1) / eeprom_config->page_size) {
            write_size = eeprom_config->page_size - (global_addr % eeprom_config->page_size);
        }

        uint8_t data_bytes[write_size];
        for (size_t i = 0; i < write_size; i++) {
            data_bytes[i] = payload[i + bytes_written];
        }
        uint8_t bytes_to_write[write_size + 2];
        const uint8_t mask = 0xFF;
        bytes_to_write[0] = page_addr >> BITS_IN_BYTE;
        bytes_to_write[1] = page_addr & mask;
        for (size_t i = 0; i < write_size; i++) {
            bytes_to_write[i + 2] = data_bytes[i];
        }
        int ret = kb900x_i2c_master_write(config, sa, bytes_to_write, write_size + 2, true);
        CHECK_SUCCESS_MSG(ret, "Failed to write to EEPROM");

        // Sleep for transaction to complete in EEPROM
        wait_ms(eeprom_config->write_cycle_time_ms);

        // check bytes written if first or second iteration
        // to assert working communication
        if (bytes_written <= write_size) {
            uint8_t result[write_size];
            ret = kb900x_i2c_master_read(config, sa, bytes_to_write, 2, write_size, result, true,
                                         false);
            CHECK_SUCCESS_MSG(ret, "Something went wrong while reading from EEPROM");
            for (size_t i = 0; i < write_size; i++) {
                if (result[i] != data_bytes[i]) {
                    KANDOU_ERR("Error while writing the firmware, does not match addr = 0x%02x%02x",
                               bytes_to_write[0], bytes_to_write[1]);
                    // Buffer for logging
                    const int nb_char_per_byte = 6; // "0x00 \0"
                    char buf[write_size * nb_char_per_byte];
                    buf[0] = '\0';
                    for (size_t j = 0; j < write_size; j++) {
                        char temp[nb_char_per_byte];
                        snprintf(temp, sizeof(temp), "0x%02x ", data_bytes[j]); // NOLINT
                        strncat(buf, temp, sizeof(buf) - strlen(buf) - 1);      // NOLINT
                    }
                    KANDOU_ERR("%s", buf);
                    KANDOU_ERR(" VS ");
                    buf[0] = '\0';
                    for (size_t j = 0; j < write_size; j++) {
                        char temp[nb_char_per_byte];
                        snprintf(temp, sizeof(temp), "0x%02x ", result[j]); // NOLINT
                        strncat(buf, temp, sizeof(buf) - strlen(buf) - 1);  // NOLINT
                    }
                    KANDOU_ERR("%s", buf);
                    return -EILSEQ;
                }
            }
        }
        bytes_written += write_size;
        KANDOU_DEBUG("%zu / %zu bytes written", bytes_written, payload_size);
    }

    return KB900X_E_OK;
}

int kb900x_eeprom_read(const kb900x_config_t *config, const uint16_t addr, const size_t length,
                       uint8_t *result, const kb900x_eeprom_config_t *eeprom_config)
{
    if (result == NULL || config == NULL || eeprom_config == NULL) {
        KANDOU_ERR("Invalid parameters: make sure config, result and eeprom_config are not null");
        return -EINVAL;
    }
    const size_t QUARTER_SIZE_BYTES = 1 << 16; // If 16 bits addressing
    const size_t i2c_max_read_size = eeprom_config->page_size < KB900X_RX_FIFO_DEPTH
                                         ? eeprom_config->page_size
                                         : KB900X_RX_FIFO_DEPTH;

    if ((addr + length) > eeprom_config->eeprom_size) {
        KANDOU_ERR("Read out of memory bounds! addr = 0x%02x length=%zu", addr, length);
        return -EINVAL;
    }
    size_t bytes_read = 0;
    uint8_t read_buffer[i2c_max_read_size];
    const uint8_t mask = 0xFF;
    uint8_t current_address[2];
    int ret;
    // Start address
    current_address[0] = addr >> BITS_IN_BYTE;
    current_address[1] = addr & mask;
    while (bytes_read < length) {
        size_t global_addr = addr + bytes_read;
        uint8_t sa = eeprom_config->slave_addr + (global_addr / QUARTER_SIZE_BYTES);
        uint16_t page_addr = global_addr % QUARTER_SIZE_BYTES;

        size_t read_size = ((i2c_max_read_size) < (length - bytes_read)) ? (i2c_max_read_size)
                                                                         : (length - bytes_read);

        if (global_addr / QUARTER_SIZE_BYTES < (global_addr + read_size - 1) / QUARTER_SIZE_BYTES) {
            read_size = QUARTER_SIZE_BYTES - (global_addr % QUARTER_SIZE_BYTES);
        }
        current_address[0] = page_addr >> BITS_IN_BYTE;
        current_address[1] = page_addr & mask;
        ret = kb900x_i2c_master_read(config, sa, current_address, 2, read_size, read_buffer, true,
                                     false);
        CHECK_SUCCESS_MSG(ret, "Something went wrong while reading from EEPROM");
        for (size_t i = 0; i < read_size; i++) {
            result[i + bytes_read] = read_buffer[i];
        }
        bytes_read += read_size;
    }
    return KB900X_E_OK;
}
