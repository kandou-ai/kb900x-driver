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

#include "kb900x_bic_comm.h"
#include "kb900x_log.h"

// #include <facebook/bic_xfer.h>
// Predeclaration for build without yocto (local)
extern int bic_data_send(uint8_t slot_id, uint8_t netfn, uint8_t cmd, uint8_t *tbuf, uint8_t tlen,
                         uint8_t *rbuf, uint8_t *rlen, uint8_t intf);
#define NETFN_APP_REQ 0x06
#define CMD_APP_MASTER_WRITE_READ 0x52

static int get_smbus_command_code(uint8_t address_size, uint8_t *command_code_start,
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

int kb900x_bic_write(const kb900x_config_t *config, const uint32_t address,
                     const uint8_t address_size, const uint32_t value)
{
    // We only use 4 bytes addresses with vendor defined SMBus write register
    if (address_size != 4) {
        KANDOU_ERR("Address size must be 4 bytes");
        return -EINVAL;
    }
    uint8_t nb_retry = 5;
    while (nb_retry > 0) {
        uint8_t tbuf[64] = {0x00};
        uint8_t rbuf[64] = {0x00};
        uint8_t tlen = 0;
        uint8_t rlen = 0;

        const uint8_t payload_size = 4;
        const uint8_t smbus_tx_length =
            payload_size + address_size +
            3; // Write length = ByteCount + command code + payload size + address size + PEC
        tbuf[0] = (config->bus_id << 1) + 1;
        tbuf[1] = config->retimer_addr << 1;
        tbuf[2] = 0x00; // Read count = 0
        tbuf[3] = CCODE_START_END_WRITE_FUNC3;
        tbuf[4] = address_size + payload_size; // Address + Data

        // Copy address to the beginning of the buffer
        for (size_t i = 0; i < address_size; i++) {
            // As SMBus expect address in little endian
            // We reverse the address to match the expected format
            tbuf[i + 5] = address >> (i * BITS_IN_BYTE) & 0xFF;
        }
        // Copy the payload after the address
        for (size_t i = 0; i < payload_size; i++) {
            // As SMBus expect payload in little endian
            // We reverse the payload to match the expected format
            tbuf[i + 5 + address_size] = value >> (i * BITS_IN_BYTE) & 0xFF;
        }

        // Add the PEC
        uint8_t data_to_sign[smbus_tx_length];
        data_to_sign[0] = config->retimer_addr << 1; // Write
        memcpy(&(data_to_sign[1]), &(tbuf[3]), smbus_tx_length - 1);
        tbuf[smbus_tx_length + 2] = cal_crc8(data_to_sign, smbus_tx_length);
        tlen = smbus_tx_length + 3; // bus_id + retimer_addr + tx_len + I2C data
        int ret = bic_data_send(config->slot_id, NETFN_APP_REQ, CMD_APP_MASTER_WRITE_READ, tbuf,
                                tlen, rbuf, &rlen, config->intf);
        if (ret != 0) {
            KANDOU_ERR("bic_data_wrapper failed with error code: %d", ret);
            nb_retry--;
            continue;
        }
        // NB: Sometimes if the BIC is busy a write can be ignored.
        // As the bic_data_send function doesn't always returns an error if a write is ignored
        // the only way to know if a write has been ignored is to read the register written to
        // ensure the write was successful NB: Doesn't work for FIFO registers as they're reset to
        // 0x00 once the data written has been pushed to the FIFO
        break;
    }
    if (nb_retry <= 0) {
        KANDOU_ERR("Failed to communicate after 5 attempts");
        return -KB900X_E_ERR;
    }

    return KB900X_E_OK;
}

int kb900x_bic_read(const kb900x_config_t *config, const uint32_t address,
                    const uint8_t address_size, uint32_t *value)
{
    if (address_size != 2 && address_size != 4) {
        KANDOU_ERR("Address size must be 2 or 4 bytes");
        return -EINVAL;
    }
    if (value == NULL) {
        KANDOU_ERR("Value pointer is NULL");
        return -EINVAL;
    }
    uint8_t nb_retry = 10;
    while (nb_retry > 0) {
        uint8_t tlen = 0;
        uint8_t rlen = 0;
        uint8_t tbuf[64] = {0x00};
        uint8_t rbuf[64] = {0x00};

        // Get SMBus Command Code
        uint8_t command_code_start;
        uint8_t command_code_stop;
        int ret = get_smbus_command_code(address_size, &command_code_start, &command_code_stop);
        CHECK_SUCCESS(ret);
        // First we need to write the address to the retimer
        const uint8_t smbus_tx_length =
            address_size + 3; // Write length = Command Code + ByteCount + address size + PEC
        tbuf[0] = (config->bus_id << 1) + 1;
        tbuf[1] = config->retimer_addr << 1; // Write
        tbuf[2] = 0x00;
        tbuf[3] = command_code_start;
        tbuf[4] = address_size;

        // Copy address to the beginning of the buffer
        for (size_t i = 0; i < address_size; i++) {
            // As SMBus expect address in little endian
            // We reverse the address to match the expected format
            tbuf[i + 5] = address >> (i * BITS_IN_BYTE) & 0xFF;
        }
        // Add the PEC
        uint8_t data_to_sign[smbus_tx_length];
        data_to_sign[0] = config->retimer_addr << 1; // Write
        memcpy(&(data_to_sign[1]), &(tbuf[3]), smbus_tx_length - 1);
        tbuf[smbus_tx_length + 2] = cal_crc8(data_to_sign, smbus_tx_length);

        tlen = smbus_tx_length + 3; // bus_id + retimer_addr + tx_len + I2C data
        ret = bic_data_send(config->slot_id, NETFN_APP_REQ, CMD_APP_MASTER_WRITE_READ, tbuf, tlen,
                            rbuf, &rlen, config->intf);
        if (ret != 0) {
            KANDOU_WARN("bic_data_wrapper failed with error code: %d", ret);
            nb_retry--;
            continue;
        }
        // Then we read the result from the retimer
        memset(tbuf, 0, sizeof(tbuf));
        memset(rbuf, 0, sizeof(rbuf));
        tlen = 0;
        rlen = 0;

        tbuf[0] = (config->bus_id << 1) + 1;
        tbuf[1] = config->retimer_addr << 1;
        tbuf[2] = address_size + 6; // A read operation will always return 4 bytes data, the
                                    // address, the bytecount and the PEC
        tbuf[3] = command_code_stop;
        tlen = 4;
        ret = bic_data_send(config->slot_id, NETFN_APP_REQ, CMD_APP_MASTER_WRITE_READ, tbuf, tlen,
                            rbuf, &rlen, config->intf);
        if (ret != 0 || rlen == 0) {
            KANDOU_WARN("bic_data_wrapper failed with error code: %d retrying...", ret);
            nb_retry--;
            continue;
        }
        // Checking for the number of bytes received
        const uint8_t bytecnt = rbuf[0];
        if (bytecnt < 6 || rlen < (bytecnt + 1)) {
            KANDOU_DEBUG("Invalid number of bytes received (bytecount): %d with command : 0x%08x "
                         "retrying...",
                         bytecnt, address);
            nb_retry--;
            continue;
        }
        // Check address
        uint32_t returned_address = 0;
        if (address_size == 4) {
            returned_address = rbuf[1] | (rbuf[2] << 8) | (rbuf[3] << 16) | (rbuf[4] << 24);
        }
        else if (address_size == 2) {
            returned_address = rbuf[1] | (rbuf[2] << 8);
        }
        // FIXME fw seems to return the same addr but on tile 0 only (0xe11c8434 -> 0xe01c8434)
        if ((returned_address << 8) != (address << 8)) {
            KANDOU_WARN(
                "Address mismatch: expected 0x%08x, got 0x%08x - potential collision - retrying...",
                address, returned_address);
            nb_retry--;
            continue;
        }
        // Check PEC
        const uint8_t data_offset = 1; // First byte is the bytecount
        const uint8_t result_size = 4;
        uint8_t pec = rbuf[rlen - 1];
        uint8_t data_to_check[address_size + result_size + 4];
        data_to_check[0] = config->retimer_addr << 1; // Write
        data_to_check[1] = command_code_stop;
        data_to_check[2] = (config->retimer_addr << 1) | 1; // Read
        data_to_check[3] = bytecnt;
        for (size_t i = 0; i < bytecnt; i++) {
            data_to_check[i + 4] = rbuf[data_offset + i];
        }
        if (pec != cal_crc8(data_to_check, bytecnt + 4)) {
            KANDOU_WARN("PEC mismatch: expected 0x%02x, got 0x%02x retrying...",
                        cal_crc8(data_to_check, bytecnt + 4), pec); // FIXME logging level
            nb_retry--;
            continue;
        }
        // Copy the result to the result buffer
        *value = 0;
        for (size_t i = 0; i < result_size; i++) {
            // As SMBus expect payload in little endian
            // We reverse the payload to match the expected format
            *value = *value | ((rbuf[bytecnt - result_size + i + 1] << (i * BITS_IN_BYTE)));
        }
        break;
    }
    if (nb_retry <= 0) {
        KANDOU_ERR("Failed to communicate after 10 attempts, enable warning logs for more details");
        return -KB900X_E_ERR;
    }

    return KB900X_E_OK;
}
