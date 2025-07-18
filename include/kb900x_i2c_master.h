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

#ifndef _KB_I2C_MASTER_H
#define _KB_I2C_MASTER_H

#include "kb900x_utils.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define KB900X_TX_FIFO_DEPTH (24)
#define KB900X_RX_FIFO_DEPTH (24)

// I2C Master interface registers
#define KB900X_ee_IC_CON 0xe0081000
#define KB900X_ee_IC_TAR 0xe0081004
#define KB900X_ee_IC_DATA_CMD 0xe0081010
#define KB900X_ee_IC_INTR_MASK 0xe0081030
#define KB900X_ee_IC_RAW_INTR_STAT 0xe0081034
#define KB900X_ee_IC_ENABLE 0xe008106c
#define KB900X_ee_IC_STATUS 0xe0081070
#define KB900X_ee_IC_TX_ABRT_SOURCE 0xe0081080
#define kb900x_cfg_top_vd_bump_0 0xe048018c
#define kb900x_cfg_top_vd_bump_1 0xe0480190

// Other registers
#define kb900x_cpu_periph_clk_gate_en                                                              \
    0xe009005c                        // I2C Master clock - Clock gate enables per peripheral
#define kb900x_tx_abrt_clr 0xe0081054 // Clear interrupts

// Global variable to store the selected slave address
extern uint8_t kb900x_i2c_master_slave_addr;

/**
 * \brief Initialize I2C master interface.
 *
 * \param[in] config the config context
 * \param[in] slave_addr the I2C slave address to write to (EEPROM or Flash)
 *
 * \return 0 if no error, else the error code
 */
int kb900x_i2c_master_init(const kb900x_config_t *config, uint8_t slave_addr);

/**
 * \brief Send I2C write operation from the KB900x I2C master interface
 *
 * \param[in] config the config context
 * \param[in] slave_addr the I2C slave address to write to
 * \param[in] data the payload to write
 * \param[in] data_size the payload size
 * \param[in] check can be used to disable the automatic status check after the write
 *
 * \return 0 if no error, else the error code
 */
int kb900x_i2c_master_write(const kb900x_config_t *config, const uint8_t slave_addr,
                            const uint8_t *data, const size_t data_size, const bool check);

/**
 * \brief Send I2C read operation from the KB900x I2C master interface
 *
 * \param[in] config the config context
 * \param[in] slave_addr the I2C slave address to read from
 * \param[in] addr the list that will form the 1st part of the I2C payload (Usually the single or
 * multibyte address of the register within the device) \param[in] addr_size the size of the addr
 * list \param[in] length the number of bytes to read \param[out] result the pointer to the array
 * used to store the result \param[in] check can be used to disable the automatic status check after
 * the write \param[in] skip_addr can be used to avoid sending a write packet to setup the address
 * before the read
 *
 * \return 0 if no error, else the error code
 */
int kb900x_i2c_master_read(const kb900x_config_t *config, const uint8_t slave_addr,
                           const uint8_t *addr, const uint8_t addr_size, const size_t length,
                           uint8_t *result, const bool check, const bool skip_addr);

/**
 * \brief Set the I2C master interface slave address
 *
 * \param[in] config the config context
 * \param[in] slave_address the I2C slave address to set
 *
 * \return 0 if no error, else the error code
 */
int kb900x_i2c_master_set_slave_address(const kb900x_config_t *config, const uint8_t slave_address);

/**
 * \brief Enable/Disable I2C (TX/RX FIFO population)
 *
 * \param[in] config the config context
 * \param[in] enable true to enable, false to disable
 * \param[in] block_fifo true to lock the FIFO for writing else false
 *
 * \return 0 if no error, else the error code
 */
int kb900x_i2c_master_enable(const kb900x_config_t *config, const bool enable,
                             const bool block_fifo);

/**
 * \brief Wait for I2C inactivity
 *
 * \param[in] config the config context
 *
 * \return 0 if no error, else the error code
 */
int kb900x_i2c_master_wait_for_inactivity(const kb900x_config_t *config);

/**
 * \brief Check the I2C interface status
 *
 * \param[in] config the config context
 * \param[in] slave_address the slave_address to communicate with
 * \param[in] data the data that we sent on the bus, for logging purpose
 *
 * \return 0 if no error, else the error code
 */
int kb900x_i2c_master_check_status(const kb900x_config_t *config, const uint8_t slave_address,
                                   const uint8_t *data);

/**
 * \brief Write a field (part of a 32 bits register)
 *
 * \param[in] config the config context
 * \param[in] addr the register address
 * \param[in] field_width the width of the field to write
 * \param[in] field_lsb the least significant bit of the field to write
 * \param[in] value the value of the field to write
 *
 * \return 0 if no error, else the error code
 */
int kb900x_write_field(const kb900x_config_t *config, const uint32_t addr,
                       const uint8_t field_width, const uint8_t field_lsb, const uint32_t value);

/**
 * \brief Read a field (part of a 32 bits register)
 *
 * \param[in] config the config context
 * \param[in] addr the register address
 * \param[in] field_width the width of the field to read
 * \param[in] field_lsb the least significant bit of the field to read
 * \param[out] value a pointer to the uint32_t used to store the result
 *
 * \return 0 if no error, else the error code
 */
int kb900x_read_field(const kb900x_config_t *config, const uint32_t addr, const uint8_t field_width,
                      const uint8_t field_lsb, uint32_t *value);

/**
 * \brief Unlock EEPROM - KB900x communication
 *
 *  If the load of the EEPROM has been interrupted the bus can be locked (I2C stuck low bug).
 *  This function will unlock the EEPROM by sending clock ticks from the master to the EEPROM.
 *
 * \param[in] config the config context
 *
 * \return 0 if no error, else the error code
 */
int kb900x_unlock_eeprom(const kb900x_config_t *config);

#endif // _KB_I2C_MASTER_H
