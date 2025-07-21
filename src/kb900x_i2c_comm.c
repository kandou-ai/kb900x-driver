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

#include "kb900x_i2c_comm.h"
#include "kb900x_utils.h"

uint8_t kb900x_i2c_slave_addr = 0x00;

int kb900x_i2c_open(int i2c_id)
{
    int handle = 0;
    char filename[KB900X_FILENAME_MAX_LENGTH];
    // Try to open /dev/i2c/{i2c_id}
    snprintf(filename, KB900X_FILENAME_MAX_LENGTH, "/dev/i2c/%d", i2c_id); // NOLINT
    handle = open(filename, O_RDWR);
    if (handle < 0 && (errno == ENOENT)) {
        // Try to open /dev/i2c-{i2c_id}
        snprintf(filename, KB900X_FILENAME_MAX_LENGTH, "/dev/i2c-%d", i2c_id); // NOLINT
        handle = open(filename, O_RDWR);
    }
    // If the connection failed
    if (handle < 0) {
        if (errno == ENOENT) {
            KANDOU_ERR("Error: Could not open file "
                       "`/dev/i2c-%d' or `/dev/i2c/%d': %s\n",
                       i2c_id, i2c_id, strerror(ENOENT));
        }
        else {
            KANDOU_ERR("Error: Could not open file "
                       "`%s': %s\n",
                       filename, strerror(errno));
            if (errno == EACCES) {
                KANDOU_ERR("Run as root?");
            }
        }
        // Forward error above
        return -errno;
    }
    return handle;
}

void kb900x_i2c_close(int handle)
{
    close(handle);
}

int kb900x_i2c_select_slave_addr(int handle, uint8_t slave_addr)
{
    kb900x_i2c_slave_addr = slave_addr;
    int ret = ioctl(handle, I2C_SLAVE, slave_addr);
    CHECK_IOCTL_MSG(ret, "Unable to select I2C slave address %d, err code: %d - %s", slave_addr,
                    errno, strerror(errno));
    return KB900X_E_OK;
}

int kb900x_i2c_write(const kb900x_config_t *config, const uint32_t address,
                     const uint8_t address_size, const uint32_t value)
{
    // We only use 4 bytes addresses with TWI interface
    if (address_size != 4) {
        KANDOU_ERR("Address size must be 4 bytes");
        return -EINVAL;
    }
    // As we don't care about the APB and tile number (only accessing tile0)
    const uint32_t mask = 0x0CFFFFFF;
    uint32_t converted_addr = address & mask;

    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[1];
    const uint8_t payload_size = 4;
    uint8_t buffer[payload_size + address_size]; // 4 bytes for address + payload size

    // Copy the 4-byte address to the beginning of the buffer
    for (size_t i = 0; i < address_size; i++) {
        buffer[i] = converted_addr >> ((address_size - 1 - i) * BITS_IN_BYTE) & 0xFF;
    }

    // Copy the payload after the address
    for (size_t i = 0; i < payload_size; i++) {
        buffer[i + address_size] = value >> ((payload_size - 1 - i) * BITS_IN_BYTE) & 0xFF;
    }

    messages[0].addr = kb900x_i2c_slave_addr;
    messages[0].flags = 0; // 0 means write
    messages[0].len = payload_size + address_size;
    messages[0].buf = buffer;

    packets.msgs = messages;
    packets.nmsgs = 1;

    int ret = ioctl(config->handle, I2C_RDWR, &packets);
    CHECK_IOCTL_MSG(ret, "Unable to write I2C data, err code: %d - %s", errno, strerror(errno));
    return KB900X_E_OK;
}

int kb900x_i2c_read(const kb900x_config_t *config, const uint32_t address,
                    const uint8_t address_size, uint32_t *value)
{
    // We only use 4 bytes addresses with TWI interface
    if (address_size != 4) {
        KANDOU_ERR("Address size must be 4 bytes\n");
        return -EINVAL;
    }
    // As we don't care about the APB and tile number (only accessing tile0)
    const uint32_t mask = 0x0CFFFFFF;
    uint32_t converted_addr = address & mask;

    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];
    uint8_t addr_buf[address_size];

    // Copy the 4-byte address into the address buffer
    for (size_t i = 0; i < address_size; i++) {
        addr_buf[i] = converted_addr >> ((address_size - 1 - i) * BITS_IN_BYTE) & 0xFF;
    }

    // First message to write the 4-byte address
    messages[0].addr = kb900x_i2c_slave_addr;
    messages[0].flags = 0; // 0 means write (for sending the address)
    messages[0].len = address_size;
    messages[0].buf = addr_buf;

    // Second message to read the data from the device
    const uint8_t result_size = 4;
    uint8_t rx_buf[result_size];
    messages[1].addr = kb900x_i2c_slave_addr;
    messages[1].flags = I2C_M_RD; // I2C_M_RD means read
    messages[1].len = result_size;
    messages[1].buf = rx_buf;

    packets.msgs = messages;
    packets.nmsgs = 2;

    int ret = ioctl(config->handle, I2C_RDWR, &packets);
    CHECK_IOCTL_MSG(ret, "Unable to read I2C data, err code: %d - %s", errno, strerror(errno));

    *value = 0;
    for (int i = 0; i < result_size; i++) {
        *value = *value | (rx_buf[i] << ((result_size - 1 - i) * BITS_IN_BYTE));
    }

    return KB900X_E_OK;
}
