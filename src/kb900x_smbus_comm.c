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

#include "kb900x_smbus_comm.h"
#include "kb900x_i2c_comm.h"
#include <errno.h>
#include <string.h>

unsigned long SUPPORTED_FUNCS = 0;

// As per doc, the kernel SMBus driver takes care of adding the PEC when writing
// and checking it when reading (if using I2C_SMBUS_BLOCK_DATA). When using
// I2C_SMBUS_I2C_BLOCK_DATA, the PEC is not handled by the kernel, we have to take care of it.
int kb900x_smbus_check_supported_func(int handle, unsigned long func)
{
    if (SUPPORTED_FUNCS == 0) {
        int ret = ioctl(handle, I2C_FUNCS, &SUPPORTED_FUNCS);
        CHECK_IOCTL_MSG(ret, "Unable to get supported functions, err code : %d - %s", errno,
                        strerror(errno));
    }
    // 0 not supported
    return (SUPPORTED_FUNCS & func) == func;
}

int kb900x_smbus_pec(int handle, bool enabled)
{
    int ret = kb900x_smbus_check_supported_func(handle, I2C_FUNC_SMBUS_PEC);
    if (ret == 0) {
        // Not supported
        KANDOU_WARN("PEC functionality not supported");
        // return -KB900X_E_PEC_NOT_SUPPORTED;
    }
    else if (ret < 0) {
        // IOCTL errno - error
        return ret;
    }
    ret = ioctl(handle, I2C_PEC, enabled);
    CHECK_IOCTL_MSG(ret, "Unable to set PEC, err code : %d - %s", errno, strerror(errno));
    return KB900X_E_OK;
}

int get_smbus_command_code(uint8_t address_size, uint8_t *command_code_start,
                           uint8_t *command_code_stop)
{
    if (address_size == 4) {
        *command_code_start = CCODE_START_READ_FUNC2;
        *command_code_stop = CCODE_END_READ_FUNC2;
    }
    else if (address_size == 2) {
        *command_code_start = CCODE_START_READ_FUNC0;
        *command_code_stop = CCODE_END_READ_FUNC0;
    }
    else {
        KANDOU_ERR("Address size not supported. Must be 2 or 4 bytes.");
        return -EINVAL;
    }
    return KB900X_E_OK;
}

int kb900x_smbus_write_block(const kb900x_config_t *config, const uint32_t address,
                             const uint8_t address_size, const uint32_t value)
{
    // We only use 4 bytes addresses with vendor defined SMBus write register
    if (address_size != 4) {
        KANDOU_ERR("Address size must be 4 bytes");
        return -EINVAL;
    }
    struct i2c_smbus_ioctl_data blk;
    union i2c_smbus_data i2c_data;

    // Copy the 4-byte address to the beginning of the buffer
    const uint8_t payload_size = 4;
    i2c_data.block[0] = payload_size + address_size;
    for (size_t i = 0; i < address_size; i++) {
        // As SMBus expect address in little endian
        // We reverse the address to match the expected format
        i2c_data.block[i + 1] = address >> (i * BITS_IN_BYTE) & 0xFF;
    }

    // Copy the payload after the address
    for (size_t i = 0; i < payload_size; i++) {
        // As SMBus expect address in little endian
        // We reverse the payload to match the expected format
        i2c_data.block[i + 1 + address_size] = value >> (i * BITS_IN_BYTE) & 0xFF;
    }

    blk.read_write = I2C_SMBUS_WRITE;
    blk.command = CCODE_START_END_WRITE_FUNC3;
    blk.size = I2C_SMBUS_BLOCK_DATA;
    blk.data = &i2c_data;
    int ret = ioctl(config->handle, I2C_SMBUS, &blk);
    CHECK_IOCTL_MSG(ret, "Unable to write I2C data, err code : %d - %s", errno, strerror(errno));
    return KB900X_E_OK;
}

int kb900x_smbus_write_i2c(const kb900x_config_t *config, const uint32_t address,
                           const uint8_t address_size, const uint32_t value)
{
    // We only use 4 bytes addresses with vendor defined SMBus write register
    if (address_size != 4) {
        KANDOU_ERR("Address size must be 4 bytes");
        return -EINVAL;
    }
    struct i2c_smbus_ioctl_data blk;
    union i2c_smbus_data i2c_data;
    // Populate payload
    const uint8_t pec_size = 1;
    const uint8_t bytecnt_size = 1;

    const uint8_t payload_size = 4;
    i2c_data.block[0] = bytecnt_size + address_size + payload_size + pec_size;
    uint8_t bytecnt = address_size + payload_size;
    i2c_data.block[1] = bytecnt;
    // Copy the 4-byte address to the beginning of the buffer
    for (size_t i = 0; i < address_size; i++) {
        // As SMBus expect address in little endian
        // We reverse the address to match the expected format
        i2c_data.block[i + 2] = address >> (i * BITS_IN_BYTE) & 0xFF;
    }

    // Copy the payload after the address
    for (size_t i = 0; i < payload_size; i++) {
        // As SMBus expect address in little endian
        // We reverse the payload to match the expected format
        i2c_data.block[i + 2 + address_size] = value >> (i * BITS_IN_BYTE) & 0xFF;
    }
    // Add the PEC
    uint8_t data_to_sign[address_size + payload_size + 3];
    data_to_sign[0] = kb900x_i2c_slave_addr << 1; // Write
    data_to_sign[1] = CCODE_START_END_WRITE_FUNC3;
    for (int i = 0; i < bytecnt_size + address_size + payload_size; i++) {
        data_to_sign[i + 2] = i2c_data.block[i + 1];
    }
    i2c_data.block[bytecnt_size + address_size + payload_size + 1] =
        cal_crc8(data_to_sign, address_size + payload_size + 3);

    blk.read_write = I2C_SMBUS_WRITE;
    blk.command = CCODE_START_END_WRITE_FUNC3;
    blk.size = I2C_SMBUS_I2C_BLOCK_DATA;
    blk.data = &i2c_data;
    int ret = ioctl(config->handle, I2C_SMBUS, &blk);
    CHECK_IOCTL_MSG(ret, "Unable to write I2C data, err code : %d - %s", errno, strerror(errno));
    return KB900X_E_OK;
}

int kb900x_smbus_read_block(const kb900x_config_t *config, const uint32_t address,
                            const uint8_t address_size, uint32_t *value)
{
    struct i2c_smbus_ioctl_data blk;
    union i2c_smbus_data i2c_data;

    uint8_t command_code_start;
    uint8_t command_code_stop;
    int ret = get_smbus_command_code(address_size, &command_code_start, &command_code_stop);
    CHECK_SUCCESS(ret);

    int retries = 0;
    const int max_retries = 3;
    const uint8_t bytecnt_size = 1;
    const uint8_t result_size = 4;

    while (retries < max_retries) {
        retries++;
        // Populate payload
        i2c_data.block[0] = address_size;
        for (size_t i = 0; i < address_size; i++) {
            // As SMBus expect address in little endian
            // We reverse the address to match the expected format
            i2c_data.block[i + 1] = address >> (i * BITS_IN_BYTE) & 0xFF;
        }
        blk.read_write = I2C_SMBUS_WRITE;
        blk.command = command_code_start;
        blk.size = I2C_SMBUS_BLOCK_DATA;
        blk.data = &i2c_data;
        // Send write
        // while(retries < max_retries) {
        ret = ioctl(config->handle, I2C_SMBUS, &blk);
        if (ret < 0) {
            if (errno == EBADMSG) {
                // Retry
                continue;
            }
            else {
                CHECK_IOCTL_MSG(ret, "Unable to write (prepare read) I2C data, err code : %d - %s",
                                errno, strerror(errno));
            }
        }
        // READ
        // When using I2C_SMBUS_BLOCK_DATA the first byte of the response is the number of bytes
        // following When using I2C_SMBUS_I2C_BLOCK_DATA the two first bytes of the response are the
        // total number of bytes received and the number of data bytes
        blk.read_write = I2C_SMBUS_READ;
        blk.command = command_code_stop;
        blk.size = I2C_SMBUS_BLOCK_DATA;
        i2c_data.block[0] = result_size + address_size + bytecnt_size;
        blk.data = &i2c_data;
        // Send read
        ret = ioctl(config->handle, I2C_SMBUS, &blk);
        if (ret < 0) {
            if (errno == EBADMSG) {
                // Retry
                continue;
            }
            else {
                CHECK_IOCTL_MSG(ret, "Unable to read I2C data, err code : %d - %s", errno,
                                strerror(errno));
            }
        }
        // Check address
        uint32_t returned_address = 0;
        if (address_size == 4) {
            returned_address = i2c_data.block[1] | (i2c_data.block[2] << BITS_IN_BYTE) |
                               (i2c_data.block[3] << (2 * BITS_IN_BYTE)) |
                               (i2c_data.block[4] << (3 * BITS_IN_BYTE));
        }
        else if (address_size == 2) {
            returned_address = i2c_data.block[1] | (i2c_data.block[2] << BITS_IN_BYTE);
        }
        else {
            KANDOU_ERR("Address size not supported. Must be 2 or 4 bytes.");
            return -EINVAL;
        }
        if ((returned_address << 8) != (address << 8)) {
            KANDOU_WARN(
                "Address mismatch: expected 0x%08x, got 0x%08x - potential collision - retrying...",
                address, returned_address);
            continue;
        }
        break;
    }
    // The data are two of four bytes of address and 4 bytes of data
    const uint8_t nb_of_data_received = i2c_data.block[0];
    // Parse response
    if (result_size < nb_of_data_received - address_size) {
        KANDOU_ERR("Buffer size too small: received %d expected %d",
                   nb_of_data_received - address_size, result_size);
        return -EINVAL;
    }
    // Check if the read register address is invalid (0xFFFFFFFF) and data is invalid (0xDEADBEEF)
    const uint8_t data_offset = 2;
    if (address_size == 4) {
        const uint8_t INVALID_ADDR_BYTES[] = {0xFF, 0xFF, 0xFF, 0xFF};
        const uint8_t INVALID_DATA_BYTES[] = {0xEF, 0xBE, 0xAD, 0xDE}; // Little endian
        bool is_invalid_addr =
            !memcmp(&i2c_data.block[data_offset], INVALID_ADDR_BYTES, address_size);
        bool is_invalid_data =
            !memcmp(&i2c_data.block[data_offset + address_size], INVALID_DATA_BYTES, address_size);
        if (is_invalid_addr && is_invalid_data) {
            KANDOU_ERR("Invalid register address 0x%04x", address);
            return -EINVAL;
        }
    }

    *value = 0;
    for (size_t i = 0; i < result_size; i++) {
        // result is little endian -> reverse
        *value = *value | ((i2c_data.block[bytecnt_size + address_size + i] << (i * BITS_IN_BYTE)));
    }

    return ret;
}

int kb900x_smbus_read_i2c(const kb900x_config_t *config, const uint32_t address,
                          const uint8_t address_size, uint32_t *value)
{
    struct i2c_smbus_ioctl_data blk;
    union i2c_smbus_data i2c_data;

    const uint8_t data_offset = 2;
    uint8_t command_code_start;
    uint8_t command_code_stop;
    uint8_t bytecnt;
    int ret;
    int retries = 0;
    const int max_retries = 3;
    const uint8_t result_size = 4;

    while (retries < max_retries) {
        retries++;
        // WRITE (prepare read)
        // Populate payload
        const uint8_t pec_size = 1;
        const uint8_t bytecnt_size = 1;

        i2c_data.block[0] = bytecnt_size + address_size + pec_size; // Total data to send
        bytecnt = address_size;
        i2c_data.block[1] = bytecnt;
        for (size_t i = 0; i < address_size; i++) {
            // As SMBus expect address in little endian
            // We reverse the address to match the expected format
            i2c_data.block[i + 2] = address >> (i * BITS_IN_BYTE) & 0xFF;
        }
        ret = get_smbus_command_code(address_size, &command_code_start, &command_code_stop);
        CHECK_SUCCESS(ret);

        // Add the PEC
        uint8_t data_to_sign[address_size + 3];
        data_to_sign[0] = kb900x_i2c_slave_addr << 1; // Write
        data_to_sign[1] = command_code_start;
        for (int i = 0; i < bytecnt_size + address_size; i++) {
            data_to_sign[i + 2] = i2c_data.block[i + 1];
        }
        i2c_data.block[bytecnt_size + address_size + 1] = cal_crc8(data_to_sign, address_size + 3);
        blk.read_write = I2C_SMBUS_WRITE;
        blk.command = command_code_start;
        blk.size = I2C_SMBUS_I2C_BLOCK_DATA;
        blk.data = &i2c_data;
        // Send write
        ret = ioctl(config->handle, I2C_SMBUS, &blk);
        if (ret < 0) {
            continue;
        }
        // READ
        blk.read_write = I2C_SMBUS_READ;
        blk.command = command_code_stop;
        blk.size = I2C_SMBUS_I2C_BLOCK_DATA;
        const int nb_data_to_read = result_size + address_size + bytecnt_size + pec_size;
        i2c_data.block[0] = nb_data_to_read;
        blk.data = &i2c_data;
        // Send read
        ret = ioctl(config->handle, I2C_SMBUS, &blk);
        if (ret < 0) {
            continue;
        }
        // Check address
        uint32_t returned_address = 0;
        if (address_size == 4) {
            returned_address = i2c_data.block[data_offset] |
                               (i2c_data.block[data_offset + 1] << BITS_IN_BYTE) |
                               (i2c_data.block[data_offset + 2] << (2 * BITS_IN_BYTE)) |
                               (i2c_data.block[data_offset + 3] << (3 * BITS_IN_BYTE));
        }
        else if (address_size == 2) {
            returned_address =
                i2c_data.block[data_offset] | (i2c_data.block[data_offset + 1] << BITS_IN_BYTE);
        }
        else {
            KANDOU_ERR("Address size not supported. Must be 2 or 4 bytes.");
            return -EINVAL;
        }
        // FIXME fw seems to return the same addr but on tile 0 only (0xe11c8434 -> 0xe01c8434)
        if ((returned_address << 8) != (address << 8)) {
            KANDOU_WARN(
                "Address mismatch: expected 0x%08x, got 0x%08x - potential collision - retrying...",
                address, returned_address);
            continue;
        }
        // Check PEC
        const uint8_t max_expected_size = 8;
        const uint8_t min_expected_size = 6;
        bytecnt = i2c_data.block[1];
        if (bytecnt != min_expected_size && bytecnt != max_expected_size) {
            continue;
        }
        uint8_t pec = i2c_data.block[data_offset + address_size + result_size];
        uint8_t data_to_check[address_size + result_size + 4];
        data_to_check[0] = (kb900x_i2c_slave_addr << 1) | 0; // Write
        data_to_check[1] = command_code_stop;
        data_to_check[2] = (kb900x_i2c_slave_addr << 1) | 1; // Read
        data_to_check[3] = bytecnt;
        for (int i = 0; i < bytecnt; i++) {
            data_to_check[i + 4] = i2c_data.block[data_offset + i];
        }
        if (pec != cal_crc8(data_to_check, address_size + result_size + 4)) {
            for (int i = 0; i < address_size + result_size + 4; i++) {
                KANDOU_DEBUG("data_to_check[%d] = %02x", i, data_to_check[i]);
            }
            KANDOU_ERR("PEC mismatch - received %02x expected %02x", pec,
                       cal_crc8(data_to_check, address_size + result_size + 4));
            continue;
        }
        break;
    }
    CHECK_IOCTL_MSG(ret, "Unable to read SMBus data");
    if (retries >= max_retries) {
        KANDOU_ERR("Unable to read SMBus data");
        return -ECOMM;
    }
    // Parse response
    // Check result buffer size FIXME
    if (result_size < bytecnt - address_size) {
        KANDOU_ERR("Buffer size too small: received %d expected %d", bytecnt - address_size,
                   result_size);
        return -EINVAL;
    }

    // Check if the read register address is invalid (0xFFFFFFFF) and data is invalid (0xDEADBEEF)
    if (address_size == 4) {
        const uint8_t INVALID_ADDR_BYTES[] = {0xFF, 0xFF, 0xFF, 0xFF};
        const uint8_t INVALID_DATA_BYTES[] = {0xEF, 0xBE, 0xAD, 0xDE}; // Little endian
        bool is_invalid_addr =
            !memcmp(&i2c_data.block[data_offset], INVALID_ADDR_BYTES, address_size);
        bool is_invalid_data =
            !memcmp(&i2c_data.block[data_offset + address_size], INVALID_DATA_BYTES, address_size);
        if (is_invalid_addr && is_invalid_data) {
            KANDOU_ERR("Invalid register address: 0x%04x", address);
            return -EINVAL;
        }
    }

    *value = 0;
    for (size_t i = 0; i < result_size; i++) {
        // result is little endian -> reverse
        *value = *value | ((i2c_data.block[data_offset + address_size + i] << (i * BITS_IN_BYTE)));
    }

    return ret;
}
