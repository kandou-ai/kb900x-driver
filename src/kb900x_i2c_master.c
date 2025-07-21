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

#include "kb900x_i2c_master.h"
#include "kb900x.h"

uint8_t kb900x_i2c_master_slave_addr = 0x00;

int kb900x_i2c_master_init(const kb900x_config_t *config, const uint8_t slave_addr)
{
    // Enable clock for I2C Master
    uint32_t payload = 0x1F;
    int ret = kb900x_write_register(config, kb900x_cpu_periph_clk_gate_en, payload);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while enabling the I2C master clock");
    // Disable before configuring
    ret = kb900x_i2c_master_enable(config, false, false);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while disalbing the I2C master");
    payload = 0x63;
    ret = kb900x_write_register(config, KB900X_ee_IC_CON, payload);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while configuring the I2C master");

    // Clear all interrupts to not disrupt FW
    payload = 0;
    ret = kb900x_write_register(config, KB900X_ee_IC_INTR_MASK, payload);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while clearing the interrupts");

    // Set slave address
    ret = kb900x_i2c_master_set_slave_address(config, slave_addr);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while setting the slave address");
    kb900x_i2c_master_slave_addr = slave_addr;

    ret = kb900x_i2c_master_enable(config, true, false);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while enabling the I2C master");
    return KB900X_E_OK;
}

int kb900x_i2c_master_write(const kb900x_config_t *config, const uint8_t slave_addr,
                            const uint8_t *data, const size_t data_size, const bool check)
{
    if (data_size > KB900X_TX_FIFO_DEPTH || data_size <= 0 || data == NULL) {
        KANDOU_ERR("Invalid parameters: make sure data is not null and KB900X_TX_FIFO_DEPTH < "
                   "data_size <= 0");
        return -EINVAL;
    }
    int ret = KB900X_E_ERR;
    // Change slave address if needed
    if (slave_addr != kb900x_i2c_master_slave_addr) {
        ret = kb900x_i2c_master_set_slave_address(config, slave_addr);
        CHECK_SUCCESS_MSG(ret, "Something went wrong while setting the slave address");
        kb900x_i2c_master_slave_addr = slave_addr;
    }

    // Block FIFO for pre-fill
    ret = kb900x_i2c_master_enable(config, true, true);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while blocking the FIFO");

    // Push data to FIFO
    for (size_t i = 0; i < data_size; i++) {
        uint32_t payload = data[i];
        ret = kb900x_write_register(config, KB900X_ee_IC_DATA_CMD, payload);
        CHECK_SUCCESS_MSG(ret, "Something went wrong while writing to TX FIFO");
    }

    // Release FIFO
    ret = kb900x_i2c_master_enable(config, true, false);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while releasing the FIFO");

    // Wait until transmission is complete
    ret = kb900x_i2c_master_wait_for_inactivity(config);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while waiting for inactivity");

    // Check status if requested
    if (check) {
        ret = kb900x_i2c_master_check_status(config, slave_addr, data);
        CHECK_SUCCESS_MSG(ret, "Something went wrong while checking the status");
    }
    return KB900X_E_OK;
}

int kb900x_i2c_master_read(const kb900x_config_t *config, const uint8_t slave_addr,
                           const uint8_t *addr, const uint8_t addr_size, const size_t length,
                           uint8_t *result, const bool check, const bool skip_addr)
{
    if (length > KB900X_TX_FIFO_DEPTH || length > KB900X_RX_FIFO_DEPTH || addr == NULL ||
        result == NULL) {
        KANDOU_ERR(
            "Invalid parameters: make sure addr and result are not null and KB900X_TX_FIFO_DEPTH "
            "<= length <= KB900X_RX_FIFO_DEPTH");
        return -EINVAL;
    }
    int ret;
    if (!skip_addr) {
        // Send address
        ret = kb900x_i2c_master_write(config, slave_addr, addr, addr_size, check);
        CHECK_SUCCESS_MSG(ret, "Something went wrong while sending the address");
    }

    // Block FIFO for pre-fill
    ret = kb900x_i2c_master_enable(config, true, true);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while blocking the FIFO");

    // Push length "read commands" to TX FIFO
    const uint32_t payload = 1 << 8;
    for (size_t i = 0; i < length; i++) {
        ret = kb900x_write_register(config, KB900X_ee_IC_DATA_CMD, payload);
        CHECK_SUCCESS_MSG(ret, "Something went wrong while writing to TX FIFO");
    }

    // Release FIFO
    ret = kb900x_i2c_master_enable(config, true, false);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while releasing the FIFO");

    // Wait untime transmission is complete
    ret = kb900x_i2c_master_wait_for_inactivity(config);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while waiting for inactivity");

    // Check status if requested
    if (check) {
        ret = kb900x_i2c_master_check_status(config, slave_addr, addr);
        CHECK_SUCCESS_MSG(ret, "Something went wrong while checking the status");
    }

    // Read length bytes from RX FIFO
    uint32_t tmp_val;
    for (size_t i = 0; i < length; i++) {
        ret = kb900x_read_register(config, KB900X_ee_IC_DATA_CMD, &tmp_val);
        CHECK_SUCCESS_MSG(ret, "Something went wrong while reading RX FIFO");
        result[i] = tmp_val & 0xFF;
    }
    return KB900X_E_OK;
}

int kb900x_i2c_master_set_slave_address(const kb900x_config_t *config, const uint8_t slave_address)
{
    int ret = kb900x_i2c_master_enable(config, false, false);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while disabling the I2C master");
    const uint32_t payload = slave_address;
    ret = kb900x_write_register(config, KB900X_ee_IC_TAR, payload);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while writing the field to select slave address");
    ret = kb900x_i2c_master_enable(config, true, false);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while enabling the I2C master");
    return KB900X_E_OK;
}

int kb900x_i2c_master_enable(const kb900x_config_t *config, const bool enable,
                             const bool block_fifo)
{
    const uint32_t value = ((uint32_t)(block_fifo) << 2) | (uint32_t)(enable);
    int ret = kb900x_write_register(config, KB900X_ee_IC_ENABLE, value);
    CHECK_SUCCESS_MSG(
        ret, "Something went wrong while writing register to enable/disable the I2C master");
    return KB900X_E_OK;
}

int kb900x_i2c_master_wait_for_inactivity(const kb900x_config_t *config)
{
    const size_t nb_retry = 1000;
    uint32_t result;
    const uint32_t mask = 0x20;
    size_t retry = 0;
    for (; retry < nb_retry; retry++) {
        int ret = kb900x_read_register(config, KB900X_ee_IC_STATUS, &result);
        CHECK_SUCCESS_MSG(ret, "Something went wrong while reading the activity register");
        if ((result & mask) == 0) {
            return KB900X_E_OK;
        }
    }
    KANDOU_ERR("TIMEOUT while waiting for inactivity!");
    return -ETIME;
}

int kb900x_i2c_master_check_status(const kb900x_config_t *config, const uint8_t slave_address,
                                   const uint8_t *data)
{
    uint32_t result;
    const uint32_t mask = 0x40;
    int ret = kb900x_read_register(config, KB900X_ee_IC_RAW_INTR_STAT, &result);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while reading the status register")
    if ((result & mask) != 0) {
        // FIXME determine if needed
        // Try to determine source of abort needed?
        (void)slave_address;
        (void)data;
        // Clear TX_ABRT interrupt
        uint32_t tmp;
        kb900x_read_register(config, kb900x_tx_abrt_clr, &tmp);
        KANDOU_ERR("TX ABORT detected");
        return -KB900X_E_TX_ABORT;
    }
    return KB900X_E_OK;
}

int kb900x_write_field(const kb900x_config_t *config, const uint32_t addr,
                       const uint8_t field_width, const uint8_t field_lsb, const uint32_t value)
{
    uint32_t reg_val;
    int ret = kb900x_read_register(config, addr, &reg_val);
    CHECK_SUCCESS_MSG(ret, "Read register (read/modify) failed, err code : %d - %s", errno,
                      strerror(errno));

    uint32_t mask = (1 << field_width) - 1;

    // Set field's bits to 0
    reg_val &= ~(mask << field_lsb);

    // Set value to field's bits
    reg_val |= (value & mask) << field_lsb;
    ret = kb900x_write_register(config, addr, reg_val);
    CHECK_SUCCESS_MSG(ret, "Write register (read/modify) failed, err code : %d - %s", errno,
                      strerror(errno));
    return KB900X_E_OK;
}

int kb900x_read_field(const kb900x_config_t *config, const uint32_t addr, const uint8_t field_width,
                      const uint8_t field_lsb, uint32_t *value)
{
    if (value == NULL) {
        KANDOU_ERR("Invalid parameters: make sure value is not null");
        return -EINVAL;
    }
    uint32_t result;
    int ret = kb900x_read_register(config, addr, &result);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while reading field");
    uint32_t mask = (1 << field_width) - 1;
    *value = result >> field_lsb & mask;
    return KB900X_E_OK;
}

int kb900x_unlock_eeprom(const kb900x_config_t *config)
{
    const uint8_t nb_ticks_to_send = 11;
    const uint8_t field_width = 6;
    const uint8_t snd_field_position = 16;
    const uint8_t shift = 5;
    uint32_t prev_val_1;
    uint32_t prev_val_2;
    // Save previous values
    int ret = kb900x_read_register(config, kb900x_cfg_top_vd_bump_0, &prev_val_1);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while interracting with the I2C master interface");
    ret = kb900x_read_register(config, kb900x_cfg_top_vd_bump_1, &prev_val_2);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while interracting with the I2C master interface");
    // Configure
    ret = kb900x_write_field(config, kb900x_cfg_top_vd_bump_0, field_width, 0, 1 << shift);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while interracting with the I2C master interface");
    ret = kb900x_write_field(config, kb900x_cfg_top_vd_bump_0, field_width, snd_field_position,
                             1 << shift);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while interracting with the I2C master interface");
    // Send ticks
    for (uint8_t i = 0; i < nb_ticks_to_send; i++) {
        ret = kb900x_write_field(config, kb900x_cfg_top_vd_bump_1, field_width, 0, 1 << shift);
        CHECK_SUCCESS_MSG(ret, "Something went wrong while sending ticks HIGH");
        ret = kb900x_write_field(config, kb900x_cfg_top_vd_bump_1, field_width, 0, 0 << shift);
        CHECK_SUCCESS_MSG(ret, "Something went wrong while sending ticks LOW");
    }
    // Deconfigure
    ret = kb900x_write_register(config, kb900x_cfg_top_vd_bump_0, prev_val_1);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while interracting with the I2C master interface");
    ret = kb900x_write_register(config, kb900x_cfg_top_vd_bump_1, prev_val_2);
    CHECK_SUCCESS_MSG(ret, "Something went wrong while interracting with the I2C master interface");
    return KB900X_E_OK;
}
