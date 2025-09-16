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

#include "kb900x.h"
#include "kb900x_addresses.h"
#include "kb900x_i2c_master.h"
#include "kb900x_reg_dump.h"
#include <string.h>
#include <time.h>
#ifdef BIC_COMMUNICATION
#include "kb900x_bic_comm.h"
#endif

#ifdef BIC_COMMUNICATION
// Set BIC mode by default
KB900X_IO io = {kb900x_bic_write, kb900x_bic_read};
// Save the current comms mode
kb900x_communication_mode_t current_mode = KB900X_COMM_BIC;
#else
// Set I2C mode by default
KB900X_IO io = {kb900x_i2c_write, kb900x_i2c_read};
// Save the current comms mode
kb900x_communication_mode_t current_mode = KB900X_COMM_TWI;
#endif

const kb9003_mapping_t kb9003_mapping = {
    .a_rx =
        {
            {
                .tile_id = 0,
                .phy_id = 0,
                .phy_lane_id = 1,
                .rpcs_id = 0,
            },
            {
                .tile_id = 0,
                .phy_id = 0,
                .phy_lane_id = 0,
                .rpcs_id = 0,
            },
            {
                .tile_id = 0,
                .phy_id = 1,
                .phy_lane_id = 1,
                .rpcs_id = 0,
            },
            {
                .tile_id = 0,
                .phy_id = 1,
                .phy_lane_id = 0,
                .rpcs_id = 0,
            },
            {
                .tile_id = 1,
                .phy_id = 3,
                .phy_lane_id = 0,
                .rpcs_id = 0,
            },
            {
                .tile_id = 1,
                .phy_id = 3,
                .phy_lane_id = 1,
                .rpcs_id = 0,
            },
            {
                .tile_id = 1,
                .phy_id = 2,
                .phy_lane_id = 0,
                .rpcs_id = 0,
            },
            {
                .tile_id = 1,
                .phy_id = 2,
                .phy_lane_id = 1,
                .rpcs_id = 0,
            },
            {
                .tile_id = 2,
                .phy_id = 0,
                .phy_lane_id = 1,
                .rpcs_id = 0,
            },
            {
                .tile_id = 2,
                .phy_id = 0,
                .phy_lane_id = 0,
                .rpcs_id = 0,
            },
            {
                .tile_id = 2,
                .phy_id = 1,
                .phy_lane_id = 1,
                .rpcs_id = 0,
            },
            {
                .tile_id = 2,
                .phy_id = 1,
                .phy_lane_id = 0,
                .rpcs_id = 0,
            },
            {
                .tile_id = 3,
                .phy_id = 3,
                .phy_lane_id = 0,
                .rpcs_id = 0,
            },
            {
                .tile_id = 3,
                .phy_id = 3,
                .phy_lane_id = 1,
                .rpcs_id = 0,
            },
            {
                .tile_id = 3,
                .phy_id = 2,
                .phy_lane_id = 0,
                .rpcs_id = 0,
            },
            {
                .tile_id = 3,
                .phy_id = 2,
                .phy_lane_id = 1,
                .rpcs_id = 0,
            },
        },
    .b_rx =
        {
            {
                .tile_id = 0,
                .phy_id = 2,
                .phy_lane_id = 1,
                .rpcs_id = 0,
            },
            {
                .tile_id = 0,
                .phy_id = 2,
                .phy_lane_id = 0,
                .rpcs_id = 0,
            },
            {
                .tile_id = 0,
                .phy_id = 3,
                .phy_lane_id = 1,
                .rpcs_id = 0,
            },
            {
                .tile_id = 0,
                .phy_id = 3,
                .phy_lane_id = 0,
                .rpcs_id = 0,
            },
            {
                .tile_id = 1,
                .phy_id = 1,
                .phy_lane_id = 0,
                .rpcs_id = 0,
            },
            {
                .tile_id = 1,
                .phy_id = 1,
                .phy_lane_id = 1,
                .rpcs_id = 0,
            },
            {
                .tile_id = 1,
                .phy_id = 0,
                .phy_lane_id = 0,
                .rpcs_id = 0,
            },
            {
                .tile_id = 1,
                .phy_id = 0,
                .phy_lane_id = 1,
                .rpcs_id = 0,
            },
            {
                .tile_id = 2,
                .phy_id = 2,
                .phy_lane_id = 1,
                .rpcs_id = 0,
            },
            {
                .tile_id = 2,
                .phy_id = 2,
                .phy_lane_id = 0,
                .rpcs_id = 0,
            },
            {
                .tile_id = 2,
                .phy_id = 3,
                .phy_lane_id = 1,
                .rpcs_id = 0,
            },
            {
                .tile_id = 2,
                .phy_id = 3,
                .phy_lane_id = 0,
                .rpcs_id = 0,
            },
            {
                .tile_id = 3,
                .phy_id = 1,
                .phy_lane_id = 0,
                .rpcs_id = 0,
            },
            {
                .tile_id = 3,
                .phy_id = 1,
                .phy_lane_id = 1,
                .rpcs_id = 0,
            },
            {
                .tile_id = 3,
                .phy_id = 0,
                .phy_lane_id = 0,
                .rpcs_id = 0,
            },
            {
                .tile_id = 3,
                .phy_id = 0,
                .phy_lane_id = 1,
                .rpcs_id = 0,
            },
        },
};
// Global (const) used to store the RTSSM states strings
const char *KB900X_STATE_STRING[] = {KB900X_FOREACH_STATE(KB900X_GENERATE_STRING)};

// Global (const) used to store the PCIe gen strings
const char *KB900X_GEN_STRING[] = {KB900X_FOREACH_GEN(KB900X_GENERATE_STRING)};

// DO NOT MODIFY - this is set in kb900x_open
kb900x_fw_version_t _fw_version = {
    .major = 0,
    .minor = 0,
    .patch = 0,
    .suffix = 0,
};

// DO NOT MODIFY, this is set by a dedicated function
bool _skip_fw_version_check = true;

void skip_fw_version_check(bool skip)
{
    _skip_fw_version_check = skip;
}

int init_fw_version(const kb900x_config_t *config)
{
    return kb900x_get_firmware_version(config, &_fw_version);
}

#define KB900X_ENSURE_MINIMAL_FW_VERSION(min_major, min_minor, min_patch)                          \
    do {                                                                                           \
        if (!_skip_fw_version_check) {                                                             \
            kb900x_communication_mode_t mode;                                                      \
            int ret = kb900x_get_communication_mode(&mode);                                        \
            if (ret != KB900X_E_OK) {                                                              \
                return ret;                                                                        \
            }                                                                                      \
            if (mode == KB900X_COMM_SMBUS) {                                                       \
                bool is_version_ok;                                                                \
                kb900x_fw_version_t min_fw_version = {.major = (min_major),                        \
                                                      .minor = (min_minor),                        \
                                                      .patch = (min_patch),                        \
                                                      .suffix = 0};                                \
                ret = check_fw_version(&_fw_version, &min_fw_version, &is_version_ok);             \
                if (ret != KB900X_E_OK) {                                                          \
                    return ret;                                                                    \
                }                                                                                  \
                if (!is_version_ok) {                                                              \
                    KANDOU_ERR("Unsupported FW version: %d.%d.%d.%d", _fw_version.major,           \
                               _fw_version.minor, _fw_version.patch, _fw_version.suffix);          \
                    KANDOU_ERR("Minimum required FW version: %d.%d.%d.*", (min_major),             \
                               (min_minor), (min_patch));                                          \
                    return -KB900X_E_OP_NOT_SUPPORTED_BY_FW;                                       \
                }                                                                                  \
            }                                                                                      \
        }                                                                                          \
    } while (0)

/**
 * \brief Check the order of two FW versions.
 *
 * \param[in] version A pointer to a FW version.
 * \param[in] min_version A pointer to a FW version.
 * \param[out] is_version_ok true if version >= min_version, false otherwise
 *
 * \return 0 if no error, otherwise an error code
 */
int check_fw_version(kb900x_fw_version_t *version, kb900x_fw_version_t *min_version,
                     bool *is_version_ok)
{
    if (version == NULL || min_version == NULL || is_version_ok == NULL) {
        KANDOU_ERR("Version, min_version and is_version_ok cannot be NULL.");
        return -EINVAL;
    }

    // Ignore version suffix in comparison
    uint8_t version_array[] = {version->major, version->minor, version->patch};
    uint8_t min_version_array[] = {min_version->major, min_version->minor, min_version->patch};

    *is_version_ok = true;
    for (size_t i = 0; i < 3; ++i) {
        if (version_array[i] < min_version_array[i]) {
            *is_version_ok = false;
            break;
        }
        else if (version_array[i] < min_version_array[i]) {
            break;
        }
        // If these numbers are equal, check smaller numbers.
        // If all numbers are equal, the version check is successful.
    }

    return KB900X_E_OK;
}

int kb900x_open(kb900x_config_t *config)
{
    // Open the I2C interface
    config->handle = kb900x_i2c_open(config->bus_id);

    // Set the slave address
    int ret = kb900x_i2c_select_slave_addr(config->handle, config->retimer_addr);
    CHECK_SUCCESS(ret);

    // Enable PEC
    ret = kb900x_smbus_pec(config->handle, true);
    CHECK_SUCCESS(ret);

    return KB900X_E_OK;
}

void kb900x_close(kb900x_config_t *config)
{
    kb900x_i2c_close(config->handle);
    config->handle = -1;
}

int kb900x_set_communication_mode(const kb900x_config_t *config, kb900x_communication_mode_t mode)
{
    if (mode == KB900X_COMM_SMBUS) {
        if (!kb900x_smbus_check_supported_func(config->handle, I2C_FUNC_SMBUS_BLOCK_DATA)) {
            KANDOU_WARN("Unsupported operation: I2C_FUNC_SMBUS_BLOCK_DATA - Trying "
                        "I2C_FUNC_SMBUS_I2C_BLOCK");
            if (!kb900x_smbus_check_supported_func(config->handle, I2C_FUNC_SMBUS_I2C_BLOCK)) {
                KANDOU_ERR("Unsupported operation: I2C_FUNC_SMBUS_I2C_BLOCK");
                return -EOPNOTSUPP;
            }
            else {
                io = (KB900X_IO){kb900x_smbus_write_i2c, kb900x_smbus_read_i2c};
                current_mode = mode;
                return init_fw_version(config);
            }
        }
        else {
            io = (KB900X_IO){kb900x_smbus_write_block, kb900x_smbus_read_block};
            current_mode = mode;
            return init_fw_version(config);
        }
    }
    else if (mode == KB900X_COMM_TWI) {
        if (!kb900x_smbus_check_supported_func(config->handle, I2C_FUNC_I2C)) {
            KANDOU_ERR("Unsupported operation: I2C_FUNC_I2C");
            return -EOPNOTSUPP;
        }
        else {
            io = (KB900X_IO){kb900x_i2c_write, kb900x_i2c_read};
            current_mode = mode;
            return KB900X_E_OK;
        }
    }
#ifdef BIC_COMMUNICATION
    else if (mode == KB900X_COMM_BIC) {
        io = (KB900X_IO){kb900x_bic_write, kb900x_bic_read};
        current_mode = mode;
        return KB900X_E_OK;
    }
#endif
    KANDOU_ERR("Mode not supported: %d", mode);
    return -EINVAL;
}

int kb900x_get_communication_mode(kb900x_communication_mode_t *mode)
{
    if (!mode) {
        return -EINVAL;
    }

    *mode = current_mode;
    return KB900X_E_OK;
}

int kb900x_detect_communication_mode(const kb900x_config_t *config,
                                     kb900x_communication_mode_t *mode)
{
    // Try to detect the communication mode
    int ret = KB900X_E_OK;
    uint32_t result = 0;

    KANDOU_DEBUG("Trying SMBUS");
    kb900x_set_communication_mode(config, KB900X_COMM_SMBUS);
    ret = kb900x_get_revid(config, &result);
    if (ret >= KB900X_E_OK && (result == KB900X_REVID_B0 || result == KB900X_REVID_B1)) {
        *mode = KB900X_COMM_SMBUS;
        return KB900X_E_OK;
    }
    KANDOU_DEBUG("Value read: 0x%08x - rc : %d - %s", result, ret, kb900x_strerror(ret));

    kb900x_set_communication_mode(config, KB900X_COMM_TWI);
    KANDOU_DEBUG("Trying raw i2c");
    ret = kb900x_get_revid(config, &result);
    CHECK_SUCCESS_MSG(ret, "Error: Unable to read REVID register to detect communication mode");

    // Fix for bitshifting
    const uint32_t bitshifting_val = 0xFF;
    const uint8_t offset = 24;
    uint8_t byte;
    while (result >> offset == bitshifting_val) {
        KANDOU_DEBUG("Bitshifting detected");
        if (read(config->handle, &byte, 1) != 1) {
            KANDOU_ERR("Failed to read from the I2C device");
        }
        ret = kb900x_get_revid(config, &result);
    }

    if (ret >= KB900X_E_OK && (result == KB900X_REVID_B0 || result == KB900X_REVID_B1)) {
        *mode = KB900X_COMM_TWI;
        return KB900X_E_OK;
    }
    KANDOU_DEBUG("Value read: 0x%08x - rc : %d - %s", result, ret, kb900x_strerror(ret));
    KANDOU_ERR("No communication mode available...");
    return -ECOMM;
}

int kb900x_switch_communication_mode(const kb900x_config_t *config,
                                     kb900x_communication_mode_t mode)
{
#ifdef BIC_COMMUNICATION
    // If we are in BIC mode, we only need to set the library mode to BIC mode,
    // because the BIC will already have configured the retimer to SMBus mode.
    if (mode == KB900X_COMM_BIC) {
        int ret = kb900x_set_communication_mode(config, mode);
        CHECK_SUCCESS_MSG(ret, "Error: Unable to set library communication mode");
        return KB900X_E_OK;
    }
#else
    if (mode == KB900X_COMM_BIC) {
        KANDOU_ERR("Mode not supported: %d", mode);
        return -EINVAL;
    }
#endif

    if (mode < 0 || mode > 1) {
        KANDOU_DEBUG("Invalid communication mode: %d", mode);
        return -EINVAL;
    }
    // Switch the communication mode
    // Read previous value
    const uint32_t mux_addr = 0xe0480008;
    uint32_t mux_value = 0;
    int ret = kb900x_read_register(config, mux_addr, &mux_value);
    CHECK_SUCCESS_MSG(ret, "Error: Unable to read MUX register");
    // Update it
    if (mode == KB900X_COMM_SMBUS) {
        const uint32_t mask = 0x2;
        mux_value = mux_value | mask; // Set second bit to 1
    }
    else if (mode == KB900X_COMM_TWI) {
        KANDOU_ERR("Switch to I2C is not implemented");
        return -KB900X_E_NOT_IMPLEMENTED;
    }
    // Send new value
    ret = kb900x_write_register(config, mux_addr, mux_value);
    CHECK_SUCCESS_MSG(ret, "Error: Unable to write MUX register");
    KANDOU_DEBUG("Communication mode switched to %d", mode);

    // Wait for firmware to boot
    sleep(2);

    ret = kb900x_set_communication_mode(config, mode);
    CHECK_SUCCESS_MSG(ret, "Error: Unable to set library communication mode");
    return KB900X_E_OK;
}

int kb900x_get_lane_temperature(const kb900x_config_t *config, int side, int lane,
                                float *temperature)
{
    KB900X_ENSURE_MINIMAL_FW_VERSION(2, 0, 3);

    if (lane >= KB9003_NUM_LANES || lane < 0 || side < 0) {
        KANDOU_ERR("Invalid lane or side: %d, %d - Number of lanes : ", lane, side,
                   KB9003_NUM_LANES);
        return -EINVAL;
    }
    int ret = 0;

    uint16_t temp_addr = KB900X_ADDR_TEMPERATURE;
    temp_addr &= 0xFF00;
    temp_addr |= (side == KB900X_SIDE_B_RX) ? KB900X_ADDR_TEMP_1ST_LANE_B_RX
                                            : KB900X_ADDR_TEMP_1ST_LANE_A_RX;
    temp_addr += (lane / 2) * 4;
    uint32_t value = 0;
    ret = io.read(config, temp_addr, KB900X_SMBUS_REGISTER_ADDR_SIZE, &value);
    CHECK_SUCCESS_MSG(ret, "Unable to read lane temperature, err code : %d - %s", errno,
                      strerror(errno));

    const int precision = 1 << FLOAT_PRECISION;
    uint8_t rx_buf[KB900X_SMBUS_REGISTER_SIZE] = {
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF,
    };
    *temperature = ((float)(rx_buf[3] + (rx_buf[2] << 8) + (rx_buf[1] << 16) + (rx_buf[0] << 24)) /
                    precision) +
                   ABSOLUTE_ZERO;
    return KB900X_E_OK;
}

int kb900x_get_temperature(const kb900x_config_t *config, float *temperature)
{
    KB900X_ENSURE_MINIMAL_FW_VERSION(2, 0, 3);

    int ret = KB900X_E_ERR;
    const int nb_ports = 2;
    const int max_reasonable_temp = 200;
    const int min_reasonable_temp = -20;
    float max_temperature = ABSOLUTE_ZERO;
    float tmp_temperature = 0;
    for (int port = 0; port < nb_ports; port++) {
        for (int lane = 0; lane < KB9003_NUM_LANES; lane++) {
            ret = kb900x_get_lane_temperature(config, port, lane, &tmp_temperature);
            CHECK_SUCCESS_MSG(ret, "Error: Unable to get temperature for lane %d, port %d", lane,
                              port);
            if (tmp_temperature > min_reasonable_temp && tmp_temperature < max_reasonable_temp) {
                max_temperature =
                    tmp_temperature > max_temperature ? tmp_temperature : max_temperature;
            }
            else {
                KANDOU_WARN("Error: temperature out of range for lane %d, port %d, temperature "
                            "measured : %f",
                            lane, port, tmp_temperature);
            }
        }
    }
    *temperature = max_temperature;
    return KB900X_E_OK;
}

int kb900x_get_vendor_id(const kb900x_config_t *config, uint32_t *vendor_id)
{
    KB900X_ENSURE_MINIMAL_FW_VERSION(2, 0, 3);

    int ret = 0;
    uint32_t value = 0;
    ret = io.read(config, KB900X_ADDR_VID, KB900X_SMBUS_REGISTER_ADDR_SIZE, &value);
    CHECK_SUCCESS_MSG(ret, "Unable to read vendor id, err code : %d - %s", ret, strerror(ret));
    *vendor_id = ((value >> 16) & 0xFF) + (((value >> 24) & 0xFF) << 8);
    return KB900X_E_OK;
}

int kb900x_get_firmware_version(const kb900x_config_t *config, kb900x_fw_version_t *fw_version)
{
    int ret = 0;
    uint32_t value = 0;
    ret = io.read(config, KB900X_ADDR_FW_VERSION, KB900X_SMBUS_REGISTER_ADDR_SIZE, &value);
    CHECK_SUCCESS_MSG(ret, "Unable to read firmware version, err code : %d - %s", errno,
                      strerror(errno));

    fw_version->major = (value >> 24) & 0xFF;
    fw_version->minor = (value >> 16) & 0xFF;
    fw_version->patch = (value >> 8) & 0xFF;
    fw_version->suffix = value & 0xFF;
    return KB900X_E_OK;
}

int kb900x_get_firmware_health(const kb900x_config_t *config, kb900x_fw_health_t *firmware_health)
{
    KB900X_ENSURE_MINIMAL_FW_VERSION(2, 0, 3);

    int ret = 0;
    uint32_t value = 0;
    ret = io.read(config, KB900X_ADDR_FIRMWARE_HEALTH, KB900X_SMBUS_REGISTER_ADDR_SIZE, &value);
    CHECK_SUCCESS_MSG(ret, "Unable to read firmware health, err code : %d - %s", errno,
                      strerror(errno));

    firmware_health->liveliness = ((value >> 24) & 0xFF) >> 4;
    firmware_health->fw_is_initialized = (value & 0xFF) & 0x01;
    return KB900X_E_OK;
}

// Comparison function for qsort in kb900x_get_sw_rtssm_log
static int compare(const void *a, const void *b)
{
    // Cast the pointers to uint32_t pointers and extract timestamps
    // If the timestamp is 0 (not populated), set it to UINT32_MAX to be at the bottom of the array
    const uint32_t timeA = ((const uint32_t *)a)[2] == 0 ? UINT32_MAX : ((const uint32_t *)a)[2];
    const uint32_t timeB = ((const uint32_t *)b)[2] == 0 ? UINT32_MAX : ((const uint32_t *)b)[2];
    return (timeA > timeB) - (timeA < timeB);
}

int kb900x_get_sds_addr(const kb900x_config_t *config, uint32_t *sds_addr)
{
    if (!sds_addr) {
        KANDOU_ERR("sds_addr pointer cannot be NULL.");
        return -EINVAL;
    }

    uint32_t revid;
    int ret = kb900x_get_revid(config, &revid);
    CHECK_SUCCESS_MSG(ret, "Could not read the KB900x revision ID.");

    switch (revid) {
        case KB900X_REVID_A0:
            *sds_addr = KB900X_DCCM_END_ADDR_A0 - KB900X_SIZEOF_SW_SHARED_DATA;
            break;
        case KB900X_REVID_B0:
        case KB900X_REVID_B1:
            *sds_addr = KB900X_DCCM_END_ADDR_B0_B1 - KB900X_SIZEOF_SW_SHARED_DATA;
            break;
        default:
            KANDOU_ERR("Unknown or unsupported revision ID: 0x%x", revid);
            return -KB900X_E_UNKNOWN_REVID;
    }

    return KB900X_E_OK;
}

int kb900x_get_sw_rtssm_log(const kb900x_config_t *config, kb900x_sw_rtssm_logs_t *logs)
{
    if (!logs) {
        KANDOU_ERR("Buffer argument must not be NULL");
        return -EINVAL;
    }

    kb900x_communication_mode_t mode;
    int ret = kb900x_get_communication_mode(&mode);
    CHECK_SUCCESS_MSG(ret, "Could not get communication mode.");
    if (mode == KB900X_COMM_TWI) {
        KANDOU_ERR("Cannot read RTSSM dump in TWI mode");
        return -EOPNOTSUPP;
    }

    uint32_t sds_addr;
    ret = kb900x_get_sds_addr(config, &sds_addr);
    CHECK_SUCCESS_MSG(ret, "Could not determine SDS address.");

    // Read data
    uint32_t buffer[KB900X_SIZEOF_SW_SHARED_DATA / BYTES_IN_U32] = {0};
    uint32_t chunk;
    for (size_t i = 0; i < (KB900X_SIZEOF_SW_SHARED_DATA >> 2); ++i) {
        const uint32_t address = sds_addr + i * 4;
        ret = kb900x_read_register(config, address, &chunk);
        CHECK_SUCCESS_MSG(ret, "Failed reading RTSSM dump data. Address: 0x%08x", address);
        buffer[i] = chunk;
    }

    // Parsing Header
    const uint32_t SDS_MAGIC_VALUE = 0x5353424b;
    const uint32_t header_size = 8;
    const uint32_t entries_step = 3;
    const uint32_t rtssm_data_size = KB900X_SIZEOF_SW_SHARED_DATA / BYTES_IN_U32;

    struct {
        uint32_t magic_value;
        uint16_t lock_val;
        uint16_t counter;
        uint32_t conflict_counter;
        uint32_t cmd;
        uint32_t parm0;
        uint32_t parm1;
        uint32_t parm2;
        uint32_t status;
    } ALIGN_PACKED(4) kb900x_sw_rtssm_header;

    memcpy(&kb900x_sw_rtssm_header, buffer, sizeof(kb900x_sw_rtssm_header)); // NOLINT

    if (kb900x_sw_rtssm_header.magic_value != SDS_MAGIC_VALUE) {
        KANDOU_ERR("Invalid magic value in RTSSM dump.");
        return -KB900X_E_ERR;
    }
    // FIXME is there anything else to check?

    // Sort data by timestamp
    // Data format: 0 address, 1 data, 2 timestamp
    const size_t entries_num = (rtssm_data_size - header_size) / entries_step;
    qsort(&buffer[header_size], entries_num, entries_step * sizeof(uint32_t), compare);

    // Parsing data
    struct {
        uint32_t curr_state : 6;
        uint32_t prev_state : 6;
        uint32_t prev_prev_state : 6;
        uint32_t speed : 3;
        uint32_t : 11;
    } ALIGN_PACKED(4) kb900x_sw_rtssm;

    const uint32_t valid_registers[8] = {0x4C0604, 0x4C07C4, 0x4C0984, 0x4C0B44,
                                         0x4C0D04, 0x4C0EC4, 0x4C1084, 0x4C1244};
    for (uint32_t i = header_size; (i + 2) < rtssm_data_size; i += entries_step) {
        uint32_t entry_idx = (i - header_size) / entries_step;
        // Check entry validity
        uint8_t rpcs_id = 0;
        logs->entries[entry_idx].is_valid = 0;
        for (int j = 0; j < 8; j++) {
            if ((buffer[i] & 0xFFFFFF) == valid_registers[j]) {
                rpcs_id = j;
                logs->entries[entry_idx].is_valid = 1;
                break;
            }
        }
        memcpy(&kb900x_sw_rtssm, &buffer[i + 1], sizeof(kb900x_sw_rtssm)); // NOLINT
        logs->entries[entry_idx].curr_state = kb900x_sw_rtssm.curr_state;
        logs->entries[entry_idx].prev_state = kb900x_sw_rtssm.prev_state;
        logs->entries[entry_idx].prev_prev_state = kb900x_sw_rtssm.prev_prev_state;
        logs->entries[entry_idx].speed = kb900x_sw_rtssm.speed;
        logs->entries[entry_idx].rpcs_id = rpcs_id;
        logs->entries[entry_idx].timestamp = buffer[i + 2];
    }

    return KB900X_E_OK;
}

int kb900x_get_link_status(const kb900x_config_t *config, int link_id,
                           kb900x_link_status_t *link_status)
{
    KB900X_ENSURE_MINIMAL_FW_VERSION(2, 0, 3);

    if (link_status == NULL) {
        KANDOU_ERR("link_status cannot be NULL");
        return -EINVAL;
    }

    if (link_id >= KB9003_MAX_NUM_LINKS || link_id < 0) {
        KANDOU_ERR("Invalid link id: %d - Number of links (link_id starts at 0): %d", link_id,
                   KB9003_MAX_NUM_LINKS);
        return -EINVAL;
    }

    // Ask to prepare data
    uint32_t value = 0;
    int ret =
        io.read(config, KB900X_ADDR_LINK_STATUS_GATHER, KB900X_SMBUS_REGISTER_ADDR_SIZE, &value);
    CHECK_SUCCESS_MSG(ret, "Unable to trigger link status info gathering, err code : %d - %s",
                      errno, strerror(errno));
    // Wait for data to be ready
    size_t timeout = KB900X_LINK_STATUS_TIMEOUT;
    bool ready = false;
    while (timeout > 0 && !ready) {
        // Wait for 2 ms before reading the operation status again.
        wait_ms(2);

        ret =
            io.read(config, KB900X_ADDR_LINK_STATUS_READY, KB900X_SMBUS_REGISTER_ADDR_SIZE, &value);
        CHECK_SUCCESS_MSG(ret, "Unable to read link status readiness, err code : %d - %s", errno,
                          strerror(errno));
        ready = value & 0x01;
        timeout--;
    }
    if (timeout <= 0) {
        KANDOU_ERR("Timeout while waiting for link status info to be ready");
        return -ETIME;
    }
    // Fetch data
    const uint8_t link_steps = 0x04;
    uint16_t link_status_addr = KB900X_ADDR_LINK_STATUS + (link_id * link_steps);
    ret = io.read(config, link_status_addr, KB900X_SMBUS_REGISTER_ADDR_SIZE, &value);
    CHECK_SUCCESS_MSG(ret, "Unable to read link status, err code : %d - %s", errno,
                      strerror(errno));
    link_status->raw = value;

    // Check for data validity
    if (link_status->link_invalid) {
        KANDOU_INFO("No valid data for link id %d", link_id);
    }
    return KB900X_E_OK;
}

int kb900x_write_register(const kb900x_config_t *config, const uint32_t address,
                          const uint32_t payload)
{
    int ret = io.write(config, address, KB900X_REGLI_REGISTER_ADDR_SIZE, payload);
    CHECK_SUCCESS_MSG(ret, "Unable to write I2C data, err code : %d - %s", errno, strerror(errno));
    return KB900X_E_OK;
}

int kb900x_read_register(const kb900x_config_t *config, const uint32_t address, uint32_t *result)
{
    int ret = io.read(config, address, KB900X_REGLI_REGISTER_ADDR_SIZE, result);
    CHECK_SUCCESS_MSG(ret, "Unable to read I2C data, err code : %d - %s", errno, strerror(errno));
    return KB900X_E_OK;
}

int kb900x_flash_firmware(const kb900x_config_t *config, const uint8_t *buffer,
                          const uint32_t buffer_size, const kb900x_eeprom_config_t *eeprom_config)
{
    if (eeprom_config == NULL || config == NULL || buffer == NULL || buffer_size < 1 ||
        eeprom_config->eeprom_size < 1) {
        KANDOU_ERR("Invalid parameters");
        return -EINVAL;
    }
    // Adapt buffer to fit full EEPROM size
    uint8_t padded_buffer[eeprom_config->eeprom_size];
    if (buffer_size > eeprom_config->eeprom_size) {
        KANDOU_ERR("Data size is too big for the EEPROM size! Data size = %u", buffer_size);
        return -EINVAL;
    }
    else if (buffer_size < eeprom_config->eeprom_size) {
        const uint8_t padding_value = 0xff;
        memset( // NOLINT boundaries already checked with surrounding if conditions
            padded_buffer, padding_value, eeprom_config->eeprom_size);
        memcpy( // NOLINT boundaries already checked with surrounding if conditions
            padded_buffer, buffer, buffer_size);
    }
    else {
        memcpy( // NOLINT boundaries already checked with surrounding if conditions
            padded_buffer, buffer, buffer_size);
    }

    int ret =
        kb900x_i2c_master_init(config, eeprom_config->slave_addr); // FIXME PASSE slave addr here?
    CHECK_SUCCESS_MSG(ret, "Failed to initialize I2C master");

    ret = kb900x_eeprom_write(config, 0x0000, padded_buffer, eeprom_config->eeprom_size,
                              eeprom_config);
    CHECK_SUCCESS_MSG(ret, "Failed to flash firmware");

    return KB900X_E_OK;
}

int kb900x_get_config_region(const kb900x_config_t *config,
                             const kb900x_eeprom_config_t *eeprom_config, uint32_t *start_addr,
                             uint32_t *length)
{
    if (config == NULL || eeprom_config == NULL || eeprom_config->eeprom_size < 1) {
        KANDOU_ERR("Invalid parameters");
        return -EINVAL;
    }
    // Parse region table to deduce config area start address and length
    const uint16_t region_table_addr = 0x100;
    const size_t region_table_len = 0x100;
    uint8_t region_table[region_table_len];
    int ret = kb900x_eeprom_read(config, region_table_addr, region_table_len, region_table,
                                 eeprom_config);
    CHECK_SUCCESS_MSG(ret, "Failed to read region table");
    // Find the configuration region
    const uint8_t region_table_entry_len = 48; // Each entry is 48 bytes
    *start_addr = 0;
    *length = 0;
    for (uint8_t i = 0; i < region_table_len; i += region_table_entry_len) {
        if (region_table[i] == 0x72 && region_table[i + 1] == 0x74) {
            // Found a region table entry
            if (region_table[i + 2] == 0x03) { // Configuration region
                *start_addr = region_table[i + 4] | (region_table[i + 5] << 8) |
                              (region_table[i + 6] << 16) | (region_table[i + 7] << 24);
                *length = region_table[i + 8] | (region_table[i + 9] << 8) |
                          (region_table[i + 10] << 16) | (region_table[i + 11] << 24);
                KANDOU_DEBUG("Found configuration region at 0x%08X with length 0x%08X", *start_addr,
                             *length);
                break;
            }
        }
    }
    // Check if we found the configuration region
    if (*start_addr == 0 || *length == 0) {
        KANDOU_ERR("Configuration region not found in region table");
        return -KB900X_E_EEPROM_FORMAT_ERROR;
    }
    return KB900X_E_OK;
}

int kb900x_configure_firmware(const kb900x_config_t *config,
                              const kb900x_eeprom_config_t *eeprom_config,
                              const uint8_t *config_payload, const size_t config_payload_size)
{
    if (config == NULL || eeprom_config == NULL || config_payload == NULL ||
        eeprom_config->eeprom_size < 1) {
        KANDOU_ERR("Invalid parameters");
        return -EINVAL;
    }

    int ret = kb900x_i2c_master_init(config, eeprom_config->slave_addr);
    CHECK_SUCCESS_MSG(ret, "Failed to initialize I2C master");

    // Find the configuration region
    uint32_t config_region_start = 0;
    uint32_t config_region_length = 0;
    ret = kb900x_get_config_region(config, eeprom_config, &config_region_start,
                                   &config_region_length);
    CHECK_SUCCESS_MSG(ret, "Failed to get configuration region informations");
    // Check if the configuration payload size is valid
    if (config_payload_size > config_region_length) {
        KANDOU_ERR("Configuration payload size is bigger than the configuration region size in "
                   "the EEPROM : "
                   "Payload size = %zu, Region size = %u",
                   config_payload_size, config_region_length);
        return -EINVAL;
    }
    // Erase old configuration region
    uint8_t reset_payload[config_region_length];
    for (uint32_t i = 0; i < config_region_length; i++) {
        reset_payload[i] = 0xFF;
    }
    ret = kb900x_eeprom_write(config, config_region_start, reset_payload, config_region_length,
                              eeprom_config);
    CHECK_SUCCESS_MSG(ret, "Failed to reset configuration region");
    // Write the new configuration
    ret = kb900x_eeprom_write(config, config_region_start, config_payload, config_region_length,
                              eeprom_config);
    CHECK_SUCCESS_MSG(ret, "Failed to configure firmware");
    return KB900X_E_OK;
}

int kb900x_check_firmware(const kb900x_config_t *config, const uint8_t *buffer,
                          const uint32_t buffer_size, const kb900x_eeprom_config_t *eeprom_config)
{
    if (config == NULL || buffer == NULL || buffer_size < 1 || eeprom_config->eeprom_size < 1) {
        KANDOU_ERR("Invalid parameters");
        return -EINVAL;
    }
    // Adapt buffer to fit full EEPROM size
    uint8_t padded_buffer[eeprom_config->eeprom_size];
    if (buffer_size > eeprom_config->eeprom_size) {
        KANDOU_ERR("Data size is too big for the EEPROM size! Data size = %u", buffer_size);
        return -EINVAL;
    }
    else if (buffer_size < eeprom_config->eeprom_size) {
        const uint8_t padding_value = 0xff;
        memset( // NOLINT boundaries already checked with surrounding if conditions
            padded_buffer, padding_value, eeprom_config->eeprom_size);
        memcpy( // NOLINT boundaries already checked with surrounding if conditions
            padded_buffer, buffer, buffer_size);
    }
    else {
        memcpy( // NOLINT boundaries already checked with surrounding if conditions
            padded_buffer, buffer, buffer_size);
    }

    uint8_t buffer_read[eeprom_config->eeprom_size];
    // FIXME there is more logic in bombinicore here
    // is the porting needed?
    int ret =
        kb900x_i2c_master_init(config, eeprom_config->slave_addr); // FIXME PASSE slave addr here?
    CHECK_SUCCESS_MSG(ret, "Failed to initialize I2C master");
    ret =
        kb900x_eeprom_read(config, 0x0000, eeprom_config->eeprom_size, buffer_read, eeprom_config);
    CHECK_SUCCESS_MSG(ret, "Failed to read firmware");

    for (unsigned i = 0; i < eeprom_config->eeprom_size; i++) {
        if (padded_buffer[i] != buffer_read[i]) {
            KANDOU_ERR("Error: Firmware does not match at index %d", i);
            KANDOU_ERR("Original: 0x%02x, Read: 0x%02x", padded_buffer[i], buffer_read[i]);
            return -EILSEQ;
        }
    }

    return KB900X_E_OK;
}

int kb900x_reset(const kb900x_config_t *config)
{
    KB900X_ENSURE_MINIMAL_FW_VERSION(2, 0, 3);

    const uint32_t reset_value = 0;
    int ret = kb900x_write_register(config, kb900x_cpu_reset_vector_addr, reset_value);
    CHECK_SUCCESS_MSG(ret, "Unable to reset the vector addr");
    ret = kb900x_write_register(config, kb900x_cpu_system, reset_value);
    CHECK_SUCCESS_MSG(ret, "Unable to reset the system");
    ret = kb900x_write_register(config, kb900x_cpu_scratchpad, reset_value);
    CHECK_SUCCESS_MSG(ret, "Unable to reset the scratchpad");
    ret = kb900x_write_register(config, kb900x_cpu_reset, reset_value);
    CHECK_SUCCESS_MSG(ret, "Unable to reset the cpu");
    return KB900X_E_OK;
}

int kb900x_get_boot_status(const kb900x_config_t *config, kb900x_boot_status_t *boot_status)
{
    KB900X_ENSURE_MINIMAL_FW_VERSION(2, 0, 3);

    const uint32_t mask_28_27 = 0x18000000;
    const uint32_t offset_28_27 = 27;
    const uint32_t mask_30_29 = 0x60000000;
    const uint32_t offset_30_29 = 29;
    int ret = KB900X_E_OK;
    uint32_t result = 0;
    ret = kb900x_read_register(config, kb900x_cpu_system, &result);
    CHECK_SUCCESS_MSG(ret, "Unable to read boot status");
    *boot_status = (result & mask_28_27) >> offset_28_27;
    kb900x_boot_entity_t entity = (result & mask_30_29) >> offset_30_29;
    // Check validity of status
    if (*boot_status != KB900X_STATE_READY || entity != KB900X_FW) {
        KANDOU_ERR("Error: Invalid boot status %d or entity %d", *boot_status, entity);
        KANDOU_DEBUG("Raw value: 0x%08x", result);
        return -KB900X_E_BOOT_STATUS;
    }
    return KB900X_E_OK;
}

int kb900x_get_revid(const kb900x_config_t *config, uint32_t *revid)
{
    if (!revid) {
        KANDOU_ERR("revid pointer cannot be NULL.");
        return -EINVAL;
    }

    int ret = kb900x_read_register(config, kb900x_cfg_top_revid, revid);
    CHECK_SUCCESS_MSG(ret, "Could not read the revision id register.");
    return KB900X_E_OK;
}

int kb900x_get_tx_presets(const kb900x_config_t *config, kb900x_all_presets_t *presets)
{
    int ret = 0;
    uint32_t operation_status = 0;
    uint32_t dccm_buff_addr;
    uint32_t dccm_buff_length;

    // Read request
    uint32_t value = 0;
    ret = io.read(config, KB900X_ADDR_PRESET_REQ, KB900X_SMBUS_REGISTER_ADDR_SIZE, &value);
    CHECK_SUCCESS_MSG(ret, "Unable to get the tx presets (request), err code : %d - %s", errno,
                      strerror(errno));

    // Read operation status
    uint8_t retry = 0;
    while (operation_status != KB900X_FEAT_REQ_STATUS_SUCCESS && retry < KB900X_FEAT_REQ_RETRIES) {
        // Wait for 2 ms before reading the operation status again.
        wait_ms(2);

        ret = io.read(config, KB900X_ADDR_PRESET_STATUS, KB900X_SMBUS_REGISTER_ADDR_SIZE,
                      &operation_status);
        CHECK_SUCCESS_MSG(ret, "Unable to get the tx presets (request status), err code : %d - %s",
                          errno, strerror(errno));

        // Break if the request failed
        if (operation_status == KB900X_FEAT_REQ_STATUS_FAILURE) {
            break;
        }

        retry++;
    }

    // Check the operation status
    if (operation_status != KB900X_FEAT_REQ_STATUS_SUCCESS) {
        return -KB900X_E_FEATURE_REQ_FAILED;
    }

    // Read DCCM start address
    ret = io.read(config, KB900X_ADDR_PRESET_START_ADDR, KB900X_SMBUS_REGISTER_ADDR_SIZE,
                  &dccm_buff_addr);
    CHECK_SUCCESS_MSG(ret, "Unable to get the tx presets (get buffer address), err code : %d - %s",
                      errno, strerror(errno));

    // Read DCCM length
    ret = io.read(config, KB900X_ADDR_PRESET_LENGTH, KB900X_SMBUS_REGISTER_ADDR_SIZE,
                  &dccm_buff_length);
    CHECK_SUCCESS_MSG(ret, "Unable to get the tx presets (get buffer length), err code : %d - %s",
                      errno, strerror(errno));

    // Check if the buffer length is valid
    if (dccm_buff_length > sizeof(kb900x_all_presets_t)) {
        KANDOU_ERR("Invalid DCCM buffer length: %u, max_size: %u", dccm_buff_length,
                   sizeof(kb900x_all_presets_t));
        return -EOVERFLOW;
    }

    // Read dump
    value = 0x0;
    for (uint32_t idx = 0; idx < dccm_buff_length / 4; idx++) {
        kb900x_read_register(config, dccm_buff_addr + (4 * idx), &value);
        ((uint32_t *)presets)[idx] = value;
    }

    return KB900X_E_OK;
}

uint32_t kb900x_delta_to_us(uint8_t delta)
{
    // Check overflow
    if (KB900X_DELTA_IS_OVERFLOW(delta)) {
        // Overflow (delta > 200ms)
        return KB900X_DELTA_OVERFLOW;
    }
    if (KB900X_DELTA_IS_RESOLUTION_1US(delta)) {
        // 1us resolution
        return delta;
    }
    if (KB900X_DELTA_IS_RESOLUTION_32US(delta)) {
        // 32us resolution
        return (delta & 0x1F) * 32;
    }
    if (KB900X_DELTA_IS_RESOLUTION_512US(delta)) {
        // 512us resolution
        return (delta & 0xF) * 512;
    }
    if (KB900X_DELTA_IS_RESOLUTION_8192US(delta)) {
        // 8192us resolution
        return (delta & 0x7) * 8192;
    }
    if (KB900X_DELTA_IS_RESOLUTION_32768US(delta)) {
        // 32768us resolution
        return (delta & 0x7) * 32768;
    }

    // Invalid
    return KB900X_DELTA_OVERFLOW;
}

int kb900x_get_hw_rtssm_log(const kb900x_config_t *config, kb900x_hw_rtssm_logs_t *log)
{
    KB900X_ENSURE_MINIMAL_FW_VERSION(2, 0, 4);

    int ret = 0;
    uint32_t operation_status = 0;
    uint32_t dccm_buff_addr;
    uint32_t dccm_buff_length;

    // Read request
    uint32_t value = 0;
    ret = io.read(config, KB900X_ADDR_RTSSM_REQ, KB900X_SMBUS_REGISTER_ADDR_SIZE, &value);
    CHECK_SUCCESS_MSG(ret, "Unable to get the tx presets (request), err code : %d - %s", errno,
                      strerror(errno));

    // Read operation status
    uint8_t retry = 0;
    while (operation_status != KB900X_FEAT_REQ_STATUS_SUCCESS && retry < KB900X_FEAT_REQ_RETRIES) {
        // Wait for 2 ms before reading the operation status again.
        wait_ms(2);

        ret = io.read(config, KB900X_ADDR_RTSSM_STATUS, KB900X_SMBUS_REGISTER_ADDR_SIZE,
                      &operation_status);
        CHECK_SUCCESS_MSG(ret, "Unable to get the tx presets (request status), err code : %d - %s",
                          errno, strerror(errno));

        // Break if the request failed
        if (operation_status == KB900X_FEAT_REQ_STATUS_FAILURE) {
            break;
        }

        retry++;
    }

    // Check the operation status
    if (operation_status != KB900X_FEAT_REQ_STATUS_SUCCESS) {
        return -KB900X_E_FEATURE_REQ_FAILED;
    }

    // Read DCCM start address
    ret = io.read(config, KB900X_ADDR_RTSSM_START_ADDR, KB900X_SMBUS_REGISTER_ADDR_SIZE,
                  &dccm_buff_addr);
    CHECK_SUCCESS_MSG(ret, "Unable to get the tx presets (get buffer address), err code : %d - %s",
                      errno, strerror(errno));

    // Read DCCM length
    ret = io.read(config, KB900X_ADDR_RTSSM_LENGTH, KB900X_SMBUS_REGISTER_ADDR_SIZE,
                  &dccm_buff_length);
    CHECK_SUCCESS_MSG(ret, "Unable to get the tx presets (get buffer length), err code : %d - %s",
                      errno, strerror(errno));

    // Check if the buffer length is valid
    if (dccm_buff_length > sizeof(kb900x_hw_rtssm_logs_t)) {
        KANDOU_ERR("Invalid DCCM buffer length: %u, max_size: %u", dccm_buff_length,
                   sizeof(kb900x_hw_rtssm_logs_t));
        return -EOVERFLOW;
    }

    // Read dump
    value = 0x0;
    for (uint32_t idx = 0; idx < dccm_buff_length / 4; idx++) {
        kb900x_read_register(config, dccm_buff_addr + (4 * idx), &value);
        ((uint32_t *)log)[idx] = value;
    }

    return KB900X_E_OK;
}

int kb900x_get_fom(const kb900x_config_t *config, kb900x_fom_t *fom)
{
    uint32_t addr;
    uint32_t val;
    kb900x_lane_mapping_t mapping;
    int ret;

    for (uint8_t lane = 0; lane < KB9003_NUM_LANES; lane++) {
        // A RX | B TX side
        mapping = kb9003_mapping.a_rx[lane];

        // Startup FOM
        addr = KB900X_RX_STARTUP_FOM_ADDR(mapping.tile_id, mapping.phy_id, mapping.phy_lane_id);
        ret = kb900x_read_register(config, addr, &val);
        CHECK_SUCCESS_MSG(ret, "Error: Unable to read STARTUP FOM register (A RX | B TX side)");
        fom->startup_a_rx[lane] = val & 0xFF;

        // Mission mode FOM
        addr = KB900X_RX_MM_FOM_ADDR(mapping.tile_id, mapping.phy_id, mapping.phy_lane_id);
        ret = kb900x_read_register(config, addr, &val);
        CHECK_SUCCESS_MSG(ret, "Error: Unable to read MM FOM register (A RX | B TX side)");
        fom->mm_a_rx[lane] = val & 0xFF;

        // A TX | B RX side
        mapping = kb9003_mapping.b_rx[lane];

        // Startup FOM
        addr = KB900X_RX_STARTUP_FOM_ADDR(mapping.tile_id, mapping.phy_id, mapping.phy_lane_id);
        ret = kb900x_read_register(config, addr, &val);
        CHECK_SUCCESS_MSG(ret, "Error: Unable to read STARTUP FOM register (A TX | B RX side)");
        fom->startup_b_rx[lane] = val & 0xFF;

        // Mission mode FOM
        addr = KB900X_RX_MM_FOM_ADDR(mapping.tile_id, mapping.phy_id, mapping.phy_lane_id);
        ret = kb900x_read_register(config, addr, &val);
        CHECK_SUCCESS_MSG(ret, "Error: Unable to read MM FOM register (A TX | B RX side)");
        fom->mm_b_rx[lane] = val & 0xFF;
    }
    return KB900X_E_OK;
}

/**
 * \brief Local function to get the bank of the phy (depends on the PCIe gen)
 *
 * \param[in] config the config context
 * \param[out] bank a pointer to a uint8_t to store the bank (0 to 2)
 * \param[in] phy_mapping the mapping used to select the phy
 */
static int kb900x_get_bank(const kb900x_config_t *config, uint8_t *bank,
                           kb900x_lane_mapping_t phy_mapping)
{
    int ret;
    uint32_t addr;
    uint32_t val;
    uint32_t rate;

    uint8_t tile_id = phy_mapping.tile_id;
    uint8_t phy_id = phy_mapping.phy_id;
    uint8_t phy_lane_id = phy_mapping.phy_lane_id;

    addr = KB900X_RX_OVRD_IN_1_ADDR(tile_id, phy_id, phy_lane_id);
    ret = kb900x_read_register(config, addr, &val);
    CHECK_SUCCESS_MSG(ret, "Error: Unable to read KB900X_RX_OVRD_IN_1 register");

    uint32_t rate_override_enable = (val >> 11) & 0x1;

    if (rate_override_enable) {
        rate = (val >> 8) & 0x7;
    }
    else {
        addr = KB900X_RX_ASIC_IN_0_ADDR(tile_id, phy_id, phy_lane_id);
        ret = kb900x_read_register(config, addr, &val);
        CHECK_SUCCESS_MSG(ret, "Error: Unable to read KB900X_RX_ASIC_IN_0 register");
        rate = (val >> 7) & 0x7;
    }

    if (rate == 0) {
        addr = KB900X_RX_PCS_IN_5_ADDR(tile_id, phy_id, phy_lane_id);
        ret = kb900x_read_register(config, addr, &val);
        CHECK_SUCCESS_MSG(ret, "Error: Unable to read KB900X_RX_PCS_IN_5 register");

        uint32_t adapt_sel = (val >> 3) & 0x3;

        if (adapt_sel == 1) {
            *bank = 1;
        }
        else {
            *bank = 2;
        }
    }
    else {
        *bank = 0;
    }

    return KB900X_E_OK;
}

/**
 * \brief Local function to get the phy info for a single lane
 *
 * \param[in] config the config context
 * \param[out] phy_info a pointer to kb9003_single_phy_info_t to store the phy info
 * \param[in] phy_mapping the mapping used to select the phy
 */
static int kb900x_get_single_phy_info(const kb900x_config_t *config,
                                      kb9003_single_phy_info_t *phy_info,
                                      kb900x_lane_mapping_t phy_mapping)
{
    int ret;
    uint32_t addr;
    uint32_t val;
    uint8_t bank;

    uint8_t tile_id = phy_mapping.tile_id;
    uint8_t phy_id = phy_mapping.phy_id;
    uint8_t phy_lane_id = phy_mapping.phy_lane_id;

    // Get bank
    ret = kb900x_get_bank(config, &bank, phy_mapping);
    CHECK_SUCCESS_MSG(ret, "Error: Unable to get the bank");

    // KB900X_RX_ATT
    addr = KB900X_RX_ATT_ADDR(tile_id, phy_id, phy_lane_id, bank);
    ret = kb900x_read_register(config, addr, &val);
    CHECK_SUCCESS_MSG(ret, "Error: Unable to read KB900X_RX_ATT register");
    phy_info->rx_att = val;

    // KB900X_RX_CTLE
    addr = KB900X_RX_CTLE_ADDR(tile_id, phy_id, phy_lane_id, bank);
    ret = kb900x_read_register(config, addr, &val);
    CHECK_SUCCESS_MSG(ret, "Error: Unable to read KB900X_RX_CTLE register");
    phy_info->rx_ctle = val;

    // KB900X_RX_VGA
    addr = KB900X_RX_VGA_ADDR(tile_id, phy_id, phy_lane_id, bank);
    ret = kb900x_read_register(config, addr, &val);
    CHECK_SUCCESS_MSG(ret, "Error: Unable to read KB900X_RX_VGA register");
    phy_info->rx_vga = val;

    // KB900X_AFE_RTRIM
    addr = KB900X_AFE_RTRIM_ADDR(tile_id, phy_id, phy_lane_id);
    ret = kb900x_read_register(config, addr, &val);
    CHECK_SUCCESS_MSG(ret, "Error: Unable to read KB900X_AFE_RTRIM register");
    phy_info->afe_rtrim = val;

    // KB900X_RX_AFE_RATE
    addr = KB900X_RX_AFE_RATE_ADDR(tile_id, phy_id, phy_lane_id, bank);
    ret = kb900x_read_register(config, addr, &val);
    CHECK_SUCCESS_MSG(ret, "Error: Unable to read KB900X_RX_AFE_RATE register");
    phy_info->rx_afe_rate = val;

    // KB900X_RX_AFE_CONFIG
    addr = KB900X_RX_AFE_CONFIG_ADDR(tile_id, phy_id, phy_lane_id, bank);
    ret = kb900x_read_register(config, addr, &val);
    CHECK_SUCCESS_MSG(ret, "Error: Unable to read KB900X_RX_AFE_CONFIG register");
    phy_info->rx_afe_config = val;

    // RX_DFE_TAP
    for (uint8_t tap = 0; tap < 8; tap++) {
        addr = KB900X_RX_DFE_TAP_ADDR(tile_id, phy_id, phy_lane_id, bank, tap);
        ret = kb900x_read_register(config, addr, &val);
        CHECK_SUCCESS_MSG(ret, "Error: Unable to read RX_DFE_TAP register");
        phy_info->rx_dfe_taps[tap] = val;
    }

    // RX_DFE_FLOAT_TAP
    for (uint8_t tap = 0; tap < 4; tap++) {
        addr = KB900X_RX_DFE_FLOAT_TAP_ADDR(tile_id, phy_id, phy_lane_id, bank, tap);
        ret = kb900x_read_register(config, addr, &val);
        CHECK_SUCCESS_MSG(ret, "Error: Unable to read RX_DFE_FLOAT_TAP register");
        phy_info->rx_dfe_float_taps[tap] = val;
    }

    // KB900X_RX_DFE_FLOAT_TAP_POS
    addr = KB900X_RX_DFE_FLOAT_TAP_POS_ADDR(tile_id, phy_id, phy_lane_id, bank);
    ret = kb900x_read_register(config, addr, &val);
    CHECK_SUCCESS_MSG(ret, "Error: Unable to read KB900X_RX_DFE_FLOAT_TAP_POS register");
    phy_info->rx_dfe_float_tap_pos = val;

    // KB900X_RX_DPLL_FREQ
    addr = KB900X_RX_DPLL_FREQ_ADDR(tile_id, phy_id, phy_lane_id);
    ret = kb900x_read_register(config, addr, &val);
    CHECK_SUCCESS_MSG(ret, "Error: Unable to read KB900X_RX_DPLL_FREQ register");
    phy_info->rx_dpll_freq = val;

    // KB900X_TX_ASIC_IN_1
    addr = KB900X_TX_ASIC_IN_1_ADDR(tile_id, phy_id, phy_lane_id);
    ret = kb900x_read_register(config, addr, &val);
    CHECK_SUCCESS_MSG(ret, "Error: Unable to read KB900X_TX_ASIC_IN_1 register");
    phy_info->tx_asic_in_1 = val;

    // KB900X_TX_ASIC_IN_2
    addr = KB900X_TX_ASIC_IN_2_ADDR(tile_id, phy_id, phy_lane_id);
    ret = kb900x_read_register(config, addr, &val);
    CHECK_SUCCESS_MSG(ret, "Error: Unable to read KB900X_TX_ASIC_IN_2 register");
    phy_info->tx_asic_in_2 = val;

    // Startup FOM
    addr = KB900X_RX_STARTUP_FOM_ADDR(tile_id, phy_id, phy_lane_id);
    ret = kb900x_read_register(config, addr, &val);
    CHECK_SUCCESS_MSG(ret, "Error: Unable to read STARTUP FOM register");
    phy_info->rx_startup_fom = val & 0xFF;

    // Mission mode FOM
    addr = KB900X_RX_MM_FOM_ADDR(tile_id, phy_id, phy_lane_id);
    ret = kb900x_read_register(config, addr, &val);
    CHECK_SUCCESS_MSG(ret, "Error: Unable to read MM FOM register");
    phy_info->rx_mm_fom = val & 0xFF;

    return KB900X_E_OK;
}

int kb900x_get_phy_info(const kb900x_config_t *config, kb9003_phy_info_t *phy_info)
{
    int ret;
    kb900x_lane_mapping_t mapping;

    for (uint8_t lane = 0; lane < KB9003_NUM_LANES; lane++) {
        // A RX | B TX side
        mapping = kb9003_mapping.a_rx[lane];

        ret = kb900x_get_single_phy_info(config, &(phy_info->a_rx[lane]), mapping);
        CHECK_SUCCESS_MSG(ret, "Error: Unable to read phy info (A TX | B RX side)");

        // A TX | B RX side
        mapping = kb9003_mapping.b_rx[lane];

        ret = kb900x_get_single_phy_info(config, &(phy_info->b_rx[lane]), mapping);
        CHECK_SUCCESS_MSG(ret, "Error: Unable to read phy info (A RX | B TX side)");
    }

    return KB900X_E_OK;
}

int kb900x_get_firmware_trace(const kb900x_config_t *config, kb900x_register_dump_t *trace)
{
    if (config == NULL) {
        KANDOU_ERR("Config pointer cannot be NULL");
        return -EINVAL;
    }
    if (trace == NULL) {
        KANDOU_ERR("Firmware logs pointer cannot be NULL");
        return -EINVAL;
    }
    if (trace->num_records < KB900X_FW_TRACE_SIZE) {
        KANDOU_ERR("Trace buffer size is too small, expected at least %u, got %u",
                   KB900X_FW_TRACE_SIZE, trace->num_records);
        return -EINVAL;
    }
    // Read the firmware trace
    for (uint32_t i = 0; i < KB900X_FW_TRACE_SIZE; i++) {
        uint32_t addr = KB900X_FW_TRACE_START_ADDR + (i * 4);
        trace->records[i].address = addr;
        int ret = kb900x_read_register(config, addr, &trace->records[i].value);
        CHECK_SUCCESS_MSG(ret, "Error: Unable to read firmware trace at address 0x%08X", addr);
    }
    return KB900X_E_OK;
}

int kb900x_get_phy_rpcs_registers(const kb900x_config_t *config, kb900x_register_dump_t *rpcs_dump)
{
    if (config == NULL) {
        KANDOU_ERR("Config pointer cannot be NULL");
        return -EINVAL;
    }
    if (rpcs_dump == NULL) {
        KANDOU_ERR("RPCS dump pointer cannot be NULL");
        return -EINVAL;
    }
    if (rpcs_dump->num_records < KB900X_PHY_RPCS_OUTPUT_SIZE) {
        KANDOU_ERR("RPCS buffer size is too small, expected at least %u, got %u",
                   KB900X_PHY_RPCS_OUTPUT_SIZE, rpcs_dump->num_records);
        return -EINVAL;
    }
    // Read the RPCS registers
    for (uint8_t tile = 0; tile < KB9003_NUM_TILES; tile++) {
        for (uint32_t i = 0; i < KB900X_NUM_PHY_RPCS_REGISTERS; i++) {
            const uint32_t addr = ((KB900X_PHY_RPCS_REG_ADDR[i] << 8) >> 8) | ((0xe0 + tile) << 24);
            const uint32_t index = i + tile * KB900X_NUM_PHY_RPCS_REGISTERS;
            int ret = kb900x_read_register(config, addr, &rpcs_dump->records[index].value);
            CHECK_SUCCESS(ret);
            rpcs_dump->records[index].address = addr;
        }
    }

    return KB900X_E_OK;
}

int kb900x_get_rpcs_dbg_counter(const kb900x_config_t *config,
                                kb900x_rpcs_debug_counter_t *rpcs_dbg)
{
    if (config == NULL) {
        KANDOU_ERR("Config pointer cannot be NULL");
        return -EINVAL;
    }
    if (rpcs_dbg == NULL) {
        KANDOU_ERR("RPCS debug counter pointer cannot be NULL");
        return -EINVAL;
    }

    // Fetch the RPCS debug counter registers
    const uint8_t addr_read_offset = 4;
    for (uint8_t tile = 0; tile < KB9003_NUM_TILES; tile++) {
        for (uint8_t rpcs = 0; rpcs < KB900X_NUM_RPCS; rpcs++) {
            const uint32_t addr_conf = KB900X_RPCS_DBG_COUNTER_ADDR_CONF(tile, rpcs);
            const uint32_t addr_read = KB900X_RPCS_DBG_COUNTER_ADDR_READ(addr_conf);
            uint32_t original_value;
            // Store the original value
            int ret = kb900x_read_register(config, addr_conf, &original_value);
            CHECK_SUCCESS_MSG(ret,
                              "Error: Unable to read RPCS debug counter configuration at "
                              "address 0x%08X",
                              addr_conf);
            for (uint8_t event_code = 0; event_code < KB900X_NUM_RPCS_EVENTS; event_code++) {
                const uint32_t index = (tile * KB900X_NUM_RPCS * KB900X_NUM_RPCS_EVENTS) +
                                       (rpcs * KB900X_NUM_RPCS_EVENTS) + event_code;
                // Configure
                const uint32_t conf_value = 0x00FF8007 | (event_code << 8) | original_value;
                ret = kb900x_write_register(config, addr_conf, conf_value);
                CHECK_SUCCESS_MSG(ret,
                                  "Error: Unable to configure RPCS debug counter at address "
                                  "0x%08X",
                                  addr_conf);
                // Release clear flag
                ret = kb900x_write_register(config, addr_conf, conf_value & 0xFFFFFFFD);
                CHECK_SUCCESS_MSG(ret,
                                  "Error: Unable to reset RPCS debug counter at address "
                                  "0x%08X",
                                  addr_conf + addr_read_offset);
                // Read
                uint32_t value;
                ret = kb900x_read_register(config, addr_read, &value);
                CHECK_SUCCESS_MSG(ret,
                                  "Error: Unable to read RPCS debug counter at address "
                                  "0x%08X",
                                  addr_read);
                rpcs_dbg->entries[index].tile_id = tile;
                rpcs_dbg->entries[index].rpcs_id = rpcs;
                rpcs_dbg->entries[index].event_code = event_code;
                rpcs_dbg->entries[index].value = value;
            }
            // Restore
            ret = kb900x_write_register(config, addr_conf, original_value);
            CHECK_SUCCESS_MSG(ret,
                              "Error: Unable to restore RPCS debug counter configuration at "
                              "address 0x%08X",
                              addr_conf);
        }
    }

    return KB900X_E_OK;
}

int kb900x_log_firmware_trace(kb900x_register_dump_t *trace, const char *filename)
{
    if (trace == NULL) {
        KANDOU_ERR("Trace buffer pointer cannot be NULL");
        return -EINVAL;
    }
    if (filename == NULL) {
        KANDOU_ERR("Firmware trace filename cannot be NULL");
        return -KB900X_E_ERR;
    }
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        KANDOU_ERR("Error while trying to create/open log file: %s", filename);
        return -KB900X_E_ERR;
    }

    // First two entries are index and count
    fprintf(file, "Current index: %u\n", trace->records[0].value);
    fprintf(file, "Count: %u\n", trace->records[1].value);
    // We ignore the first two entries
    // They are meta entries, giving info about the log index and number of valid entries
    for (uint32_t i = 2; i < KB900X_FW_TRACE_SIZE; i++) {
        const char *error_type =
            ((trace->records[i].value & 0x70000000) >> 28) == 0 ? "Info" : "Error";
        uint16_t file_id = (trace->records[i].value & 0x0FF00000) >> 20;
        uint16_t line = (trace->records[i].value & 0x000FFF00) >> 8;
        uint16_t code = (trace->records[i].value & 0x00FF);
        fprintf(file,
                "Index: %03u, Value: 0x%08x, Type: %s, FileId: %u, Line number: %u, Code: 0x%02x\n",
                (i - 1), trace->records[i].value, error_type, file_id, line, code);
    }

    fclose(file);

    return KB900X_E_OK;
}

int kb900x_log_hw_rtssm(const kb900x_hw_rtssm_logs_t *data, const char *filename)
{
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        KANDOU_ERR("Error while trying to create/open log file: %s", filename);
        return -KB900X_E_ERR;
    }

    const uint8_t nb_loggers = 8;
    const uint8_t nb_entries = 32;

    fprintf(file, "{\n");
    fprintf(file, "    \"logs\": [\n");

    for (int i = 0; i < nb_loggers; i++) {
        fprintf(file, "        {\n");
        fprintf(file, "            \"log_map_info\": {\n");
        fprintf(file, "                \"tile_id\": %u,\n", data->logs[i].log_map_info.tile_id);
        fprintf(file, "                \"rpcs_id\": %u,\n", data->logs[i].log_map_info.rpcs_id);
        fprintf(file, "                \"rtssm_cfg_id\": %u,\n",
                data->logs[i].log_map_info.rtssm_cfg_id);
        fprintf(file, "                \"curr_pos\": %u\n", data->logs[i].log_map_info.curr_pos);
        fprintf(file, "            },\n");
        fprintf(file, "            \"entries\": [\n");

        for (int j = 0; j < nb_entries; j++) {
            if (data->logs[i].entries[j].data_rate == 7) {
                // Skip invalid entries
                continue;
            }

            fprintf(file, "                {\n");
            fprintf(file, "                    \"rtssm\": %u,\n", data->logs[i].entries[j].rtssm);
            fprintf(file, "                    \"rtssm_str\": \"%s\",\n",
                    KB900X_STATE_TO_STRING(data->logs[i].entries[j].rtssm));
            fprintf(file, "                    \"data_rate\": %u,\n",
                    data->logs[i].entries[j].data_rate);
            fprintf(file, "                    \"data_rate_str\": \"%s\",\n",
                    KB900X_GEN_TO_STRING(data->logs[i].entries[j].data_rate));
            fprintf(file, "                    \"delta_us\": %u,\n",
                    kb900x_delta_to_us(data->logs[i].entries[j].delta));
            fprintf(file, "                    \"raw\": %u\n", data->logs[i].entries[j].raw);
            fprintf(file, "                },\n");
        }
        fseek(file, -2, SEEK_END); // Move the file pointer back to overwrite the last comma
        fprintf(file, "            ]\n");
        fprintf(file, "        }%s\n", (i < (nb_loggers - 1)) ? "," : "");
    }

    fprintf(file, "    ]\n");
    fprintf(file, "}\n");

    fclose(file);
    return KB900X_E_OK;
}

int kb900x_log_tx_presets(const kb900x_all_presets_t *data, const char *filename)
{
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        KANDOU_ERR("Error while trying to create/open log file: %s", filename);
        return -KB900X_E_ERR;
    }

    fprintf(file, "{\n");
    fprintf(file, "    \"lanes\": [\n");

    for (int i = 0; i < KB9003_NUM_LANES; i++) {
        fprintf(file, "        {\n");
        fprintf(file, "            \"config\": {\n");
        fprintf(file, "                \"lane_id\": %u,\n", data->lanes[i].config.lane_id);
        fprintf(file, "                \"data_rate\": %u,\n", data->lanes[i].config.data_rate);
        fprintf(file, "                \"data_rate_str\": \"%s\"\n",
                KB900X_GEN_TO_STRING(data->lanes[i].config.data_rate));
        fprintf(file, "            },\n");
        fprintf(file, "            \"data\": {\n");
        fprintf(file, "                \"rt_rc\": %u,\n", data->lanes[i].data.rt_rc);
        fprintf(file, "                \"rt_ep\": %u,\n", data->lanes[i].data.rt_ep);
        fprintf(file, "                \"rc_rt\": %u,\n", data->lanes[i].data.rc_rt);
        fprintf(file, "                \"ep_rt\": %u\n", data->lanes[i].data.ep_rt);
        fprintf(file, "            }\n");
        fprintf(file, "        }%s\n", (i < KB9003_NUM_LANES - 1) ? "," : "");
    }

    fprintf(file, "    ]\n");
    fprintf(file, "}\n");

    fclose(file);
    return KB900X_E_OK;
}

int kb900x_log_fom(const kb900x_fom_t *data, const char *filename)
{
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        KANDOU_ERR("Error while trying to create/open log file: %s", filename);
        return -KB900X_E_ERR;
    }

    uint8_t nb_lanes = KB9003_NUM_LANES;

    fprintf(file, "{\n");

    // Write startup_a_rx array
    fprintf(file, "    \"startup_a_rx\": [");
    for (int i = 0; i < nb_lanes; i++) {
        fprintf(file, "%u%s", data->startup_a_rx[i], (i < nb_lanes - 1) ? "," : "");
    }
    fprintf(file, "],\n");

    // Write mm_a_rx array
    fprintf(file, "    \"mm_a_rx\": [");
    for (int i = 0; i < nb_lanes; i++) {
        fprintf(file, "%u%s", data->mm_a_rx[i], (i < nb_lanes - 1) ? "," : "");
    }
    fprintf(file, "],\n");

    // Write startup_b_rx array
    fprintf(file, "    \"startup_b_rx\": [");
    for (int i = 0; i < nb_lanes; i++) {
        fprintf(file, "%u%s", data->startup_b_rx[i], (i < nb_lanes - 1) ? "," : "");
    }
    fprintf(file, "],\n");

    // Write mm_b_rx array
    fprintf(file, "    \"mm_b_rx\": [");
    for (int i = 0; i < nb_lanes; i++) {
        fprintf(file, "%u%s", data->mm_b_rx[i], (i < nb_lanes - 1) ? "," : "");
    }
    fprintf(file, "]\n");

    fprintf(file, "}\n");

    fclose(file);
    return KB900X_E_OK;
}

int kb900x_log_phy_info(const kb9003_phy_info_t *data, const char *filename)
{
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        KANDOU_ERR("Error while trying to create/open log file: %s", filename);
        return -KB900X_E_ERR;
    }

    fprintf(file, "{\n");

    // Write a_rx array
    fprintf(file, "    \"a_rx\": [\n");
    for (int i = 0; i < 16; i++) {
        fprintf(file, "        {\n");
        fprintf(file, "            \"rx_att\": %u,\n", data->a_rx[i].rx_att);
        fprintf(file, "            \"rx_ctle\": %u,\n", data->a_rx[i].rx_ctle);
        fprintf(file, "            \"rx_vga\": %u,\n", data->a_rx[i].rx_vga);
        fprintf(file, "            \"afe_rtrim\": %u,\n", data->a_rx[i].afe_rtrim);
        fprintf(file, "            \"rx_afe_rate\": %u,\n", data->a_rx[i].rx_afe_rate);
        fprintf(file, "            \"rx_afe_config\": %u,\n", data->a_rx[i].rx_afe_config);
        fprintf(file, "            \"rx_dfe_taps\": [%u, %u, %u, %u, %u, %u, %u, %u],\n",
                data->a_rx[i].rx_dfe_taps[0], data->a_rx[i].rx_dfe_taps[1],
                data->a_rx[i].rx_dfe_taps[2], data->a_rx[i].rx_dfe_taps[3],
                data->a_rx[i].rx_dfe_taps[4], data->a_rx[i].rx_dfe_taps[5],
                data->a_rx[i].rx_dfe_taps[6], data->a_rx[i].rx_dfe_taps[7]);
        fprintf(file, "            \"rx_dfe_float_taps\": [%u, %u, %u, %u],\n",
                data->a_rx[i].rx_dfe_float_taps[0], data->a_rx[i].rx_dfe_float_taps[1],
                data->a_rx[i].rx_dfe_float_taps[2], data->a_rx[i].rx_dfe_float_taps[3]);
        fprintf(file, "            \"rx_dfe_float_tap_pos\": %u,\n",
                data->a_rx[i].rx_dfe_float_tap_pos);
        fprintf(file, "            \"rx_dpll_freq\": %u,\n", data->a_rx[i].rx_dpll_freq);
        fprintf(file, "            \"tx_asic_in_1\": %u,\n", data->a_rx[i].tx_asic_in_1);
        fprintf(file, "            \"tx_asic_in_2\": %u,\n", data->a_rx[i].tx_asic_in_2);
        fprintf(file, "            \"rx_startup_fom\": %u,\n", data->a_rx[i].rx_startup_fom);
        fprintf(file, "            \"rx_mm_fom\": %u\n", data->a_rx[i].rx_mm_fom);
        fprintf(file, "        }%s\n", (i < 15) ? "," : "");
    }
    fprintf(file, "    ],\n");

    // Write b_rx array
    fprintf(file, "    \"b_rx\": [\n");
    for (int i = 0; i < 16; i++) {
        fprintf(file, "        {\n");
        fprintf(file, "            \"rx_att\": %u,\n", data->b_rx[i].rx_att);
        fprintf(file, "            \"rx_ctle\": %u,\n", data->b_rx[i].rx_ctle);
        fprintf(file, "            \"rx_vga\": %u,\n", data->b_rx[i].rx_vga);
        fprintf(file, "            \"afe_rtrim\": %u,\n", data->b_rx[i].afe_rtrim);
        fprintf(file, "            \"rx_afe_rate\": %u,\n", data->b_rx[i].rx_afe_rate);
        fprintf(file, "            \"rx_afe_config\": %u,\n", data->b_rx[i].rx_afe_config);
        fprintf(file, "            \"rx_dfe_taps\": [%u, %u, %u, %u, %u, %u, %u, %u],\n",
                data->b_rx[i].rx_dfe_taps[0], data->b_rx[i].rx_dfe_taps[1],
                data->b_rx[i].rx_dfe_taps[2], data->b_rx[i].rx_dfe_taps[3],
                data->b_rx[i].rx_dfe_taps[4], data->b_rx[i].rx_dfe_taps[5],
                data->b_rx[i].rx_dfe_taps[6], data->b_rx[i].rx_dfe_taps[7]);
        fprintf(file, "            \"rx_dfe_float_taps\": [%u, %u, %u, %u],\n",
                data->b_rx[i].rx_dfe_float_taps[0], data->b_rx[i].rx_dfe_float_taps[1],
                data->b_rx[i].rx_dfe_float_taps[2], data->b_rx[i].rx_dfe_float_taps[3]);
        fprintf(file, "            \"rx_dfe_float_tap_pos\": %u,\n",
                data->b_rx[i].rx_dfe_float_tap_pos);
        fprintf(file, "            \"rx_dpll_freq\": %u,\n", data->b_rx[i].rx_dpll_freq);
        fprintf(file, "            \"tx_asic_in_1\": %u,\n", data->b_rx[i].tx_asic_in_1);
        fprintf(file, "            \"tx_asic_in_2\": %u,\n", data->b_rx[i].tx_asic_in_2);
        fprintf(file, "            \"rx_startup_fom\": %u,\n", data->b_rx[i].rx_startup_fom);
        fprintf(file, "            \"rx_mm_fom\": %u\n", data->b_rx[i].rx_mm_fom);
        fprintf(file, "        }%s\n", (i < 15) ? "," : "");
    }
    fprintf(file, "    ]\n");

    fprintf(file, "}\n");

    fclose(file);

    return KB900X_E_OK;
}

int kb900x_log_sw_rtssm(const kb900x_sw_rtssm_logs_t *data, const char *filename)
{
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        KANDOU_ERR("Error while trying to create/open log file: %s", filename);
        return -KB900X_E_ERR;
    }

    fprintf(file, "{\n    \"sw_rtssm_logs\": [\n");
    for (int i = 0; i < KB900X_SW_RTSSM_SIZE; i++) {
        // Skip invalid or uninitialized entries
        if (!data->entries[i].is_valid) {
            continue;
        }

        fprintf(file, "        {\n");
        fprintf(file, "            \"curr_state\": %u,\n", data->entries[i].curr_state);
        fprintf(file, "            \"curr_state_str\": \"%s\",\n",
                KB900X_STATE_TO_STRING(data->entries[i].curr_state));
        fprintf(file, "            \"prev_state\": %u,\n", data->entries[i].prev_state);
        fprintf(file, "            \"prev_state_str\": \"%s\",\n",
                KB900X_STATE_TO_STRING(data->entries[i].prev_state));
        fprintf(file, "            \"prev_prev_state\": %u,\n", data->entries[i].prev_prev_state);
        fprintf(file, "            \"prev_prev_state_str\": \"%s\",\n",
                KB900X_STATE_TO_STRING(data->entries[i].prev_prev_state));
        fprintf(file, "            \"speed\": %u,\n", data->entries[i].speed);
        fprintf(file, "            \"speed_str\": \"%s\",\n",
                KB900X_GEN_TO_STRING(data->entries[i].speed));
        fprintf(file, "            \"rpcs_id\": %u,\n", data->entries[i].rpcs_id);
        fprintf(file, "            \"timestamp\": %u\n", data->entries[i].timestamp);
        fprintf(file, "        },\n");
    }
    fseek(file, -2, SEEK_END); // Move the file pointer back to overwrite the last comma
    fprintf(file, "\n    ]\n}\n");

    fclose(file);
    return KB900X_E_OK;
}

int kb900x_log_registers_record(const kb900x_register_dump_t *data, const char *filename)
{
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        KANDOU_ERR("Error while trying to create/open log file: %s", filename);
        return -KB900X_E_ERR;
    }

    fprintf(file, "{\n");
    for (uint32_t i = 0; i < data->num_records; i++) {
        fprintf(file, "        \"0x%08x\": %u,\n", data->records[i].address,
                data->records[i].value);
    }
    fseek(file, -2, SEEK_END); // Move the file pointer back to overwrite the last comma
    fprintf(file, "\n   }\n");

    fclose(file);
    return KB900X_E_OK;
}

int kb900x_log_phy_rpcs_registers(const kb900x_register_dump_t *data, const char *filename)
{
    if (data == NULL || filename == NULL) {
        KANDOU_ERR("Error: config or filename is NULL");
        return -EINVAL;
    }
    // Log the data
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        KANDOU_ERR("Error while trying to create/open log file: %s", filename);
        return -KB900X_E_ERR;
    }
    fprintf(file, "address, value\n");
    for (uint32_t i = 0; i < data->num_records; i++) {
        fprintf(file, "0x%08X, 0x%08X\n", data->records[i].address, data->records[i].value);
    }
    fclose(file);
    return KB900X_E_OK;
}

int kb900x_log_rpcs_dbg_counter(const kb900x_rpcs_debug_counter_t *data, const char *filename)
{
    if (data == NULL || filename == NULL) {
        KANDOU_ERR("Error: data or filename is NULL");
        return -EINVAL;
    }
    // Log the data
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        KANDOU_ERR("Error while trying to create/open log file: %s", filename);
        return -KB900X_E_ERR;
    }
    const uint32_t nb_entries = KB9003_NUM_TILES * KB900X_NUM_RPCS * KB900X_NUM_RPCS_EVENTS;
    fprintf(file, "[\n");
    for (uint32_t i = 0; i < nb_entries; i++) {
        fprintf(file, "    {\n");
        fprintf(file, "        \"tile_id\": %u,\n", data->entries[i].tile_id);
        fprintf(file, "        \"rpcs_id\": %u,\n", data->entries[i].rpcs_id);
        fprintf(file, "        \"event_code\": %u,\n", data->entries[i].event_code);
        fprintf(file, "        \"value\": %u\n", data->entries[i].value);
        fprintf(file, "    }%s\n", (i < nb_entries - 1) ? "," : "");
    }
    fprintf(file, "]\n");

    fclose(file);
    return KB900X_E_OK;
}

int kb900x_error_dump(const kb900x_config_t *config, const char *filename)
{
    if (config == NULL) {
        KANDOU_ERR("Error: config is NULL");
        return -EINVAL;
    }
    if (filename == NULL) {
        if (config->intf == 0x05) {
            filename = "kandou_1ou_retimer_error_dump.log";
        }
        else if (config->intf == 0x25) {
            filename = "kandou_3ou_retimer_error_dump.log";
        }
        else {
            KANDOU_ERR(
                "Warning: Unknown interface, possible value for config.intf are 0x05 or 0x25");
            return -EINVAL;
        }
    }
    printf("Getting the error dump data from Kandou Retimer ...\n");
    // Empty the file if it already exists
    FILE *main_file = fopen(filename, "w");
    if (main_file == NULL) {
        KANDOU_ERR("Error while trying to create/open log file: %s", filename);
        return -KB900X_E_ERR;
    }
    fprintf(main_file, "{\n");
    fclose(main_file);

    typedef struct {
        int (*data_function)(const kb900x_config_t *, void *); // Function to get the data
        int (*log_function)(const void *, const char *);       // Function to log the data in a file
        const char *name;                                      // Name of the feature to log
        void *data; // Pointer to the data structure to store the data
    } LogFunction;

    kb900x_fom_t fom = {0};
    kb900x_all_presets_t tx_presets = {0};
    kb900x_sw_rtssm_logs_t sw_rtssm = {0};
    kb900x_hw_rtssm_logs_t hw_rtssm = {0};
    kb9003_phy_info_t phy_info = {0};
    kb900x_register_record_t trace_records[KB900X_FW_TRACE_SIZE] = {0};
    kb900x_register_dump_t trace = {
        .num_records = KB900X_FW_TRACE_SIZE,
        .records = trace_records,
    };
    kb900x_register_record_t phy_rpcs_records[KB900X_PHY_RPCS_OUTPUT_SIZE] = {0};
    kb900x_register_dump_t phy_rpcs = {
        .num_records = KB900X_PHY_RPCS_OUTPUT_SIZE,
        .records = phy_rpcs_records,
    };

    const LogFunction features_to_log[] = {
        {(int (*)(const kb900x_config_t *, void *))kb900x_get_fom,
         (int (*)(const void *, const char *))kb900x_log_fom, "fom", &fom},
        {(int (*)(const kb900x_config_t *, void *))kb900x_get_tx_presets,
         (int (*)(const void *, const char *))kb900x_log_tx_presets, "tx_presets", &tx_presets},
        {(int (*)(const kb900x_config_t *, void *))kb900x_get_sw_rtssm_log,
         (int (*)(const void *, const char *))kb900x_log_sw_rtssm, "sw_rtssm", &sw_rtssm},
        {(int (*)(const kb900x_config_t *, void *))kb900x_get_hw_rtssm_log,
         (int (*)(const void *, const char *))kb900x_log_hw_rtssm, "hw_rtssm", &hw_rtssm},
        {(int (*)(const kb900x_config_t *, void *))kb900x_get_phy_info,
         (int (*)(const void *, const char *))kb900x_log_phy_info, "phy_info", &phy_info},
        {(int (*)(const kb900x_config_t *, void *))kb900x_get_firmware_trace,
         (int (*)(const void *, const char *))kb900x_log_registers_record, "firmware_trace",
         &trace},
        {(int (*)(const kb900x_config_t *, void *))kb900x_get_phy_rpcs_registers,
         (int (*)(const void *, const char *))kb900x_log_registers_record, "phy_rpcs", &phy_rpcs},
    };

    for (size_t i = 0; i < sizeof(features_to_log) / sizeof(features_to_log[0]); i++) {
        char filename_tmp[32];
        snprintf(filename_tmp, sizeof(filename_tmp), "%s_tmp.json", // NOLINT
                 features_to_log[i].name);
        // Fetch data
        int ret = features_to_log[i].data_function(config, features_to_log[i].data);
        CHECK_SUCCESS_MSG(ret, "Unable to get %s, err code : %d - %s", features_to_log[i].name,
                          errno, strerror(ret));
        // Log data
        ret = features_to_log[i].log_function(features_to_log[i].data, filename_tmp);
        CHECK_SUCCESS_MSG(ret, "Unable to log %s, err code : %d - %s", features_to_log[i].name,
                          errno, strerror(ret));

        // Open the main file in append mode
        main_file = fopen(filename, "a");
        if (main_file == NULL) {
            KANDOU_ERR("Error while trying to open log file: %s", filename);
            return -KB900X_E_ERR;
        }
        // Check if the main file is empty to determine if we need a comma
        fseek(main_file, 0, SEEK_END);
        long file_size = ftell(main_file);
        bool needs_comma = (file_size > 2);
        // Open the temporary file for reading
        FILE *temp_file = fopen(filename_tmp, "r");
        if (temp_file == NULL) {
            KANDOU_ERR("Error opening temporary file: %s", filename_tmp);
            fclose(main_file);
            return -KB900X_E_ERR;
        }
        // If the main file is not empty, add a comma and newline before appending
        if (needs_comma) {
            fprintf(main_file, ",\n");
        }
        // Print JSON key for the feature
        fprintf(main_file, "\"%s\": ", features_to_log[i].name);
        // Copy the content from the temporary file to the main file
        int ch;
        while ((ch = fgetc(temp_file)) != EOF) {
            fputc(ch, main_file);
        }

        // Close both files
        fclose(temp_file);
        fclose(main_file);

        // Remove the temporary file
        if (remove(filename_tmp) != 0) {
            KANDOU_ERR("Error removing temporary file: %s", filename_tmp);
        }
    }
    main_file = fopen(filename, "a");
    if (main_file == NULL) {
        KANDOU_ERR("Error while trying to open log file: %s", filename);
        return -KB900X_E_ERR;
    }
    // Close the JSON object
    fprintf(main_file, "}\n");
    fclose(main_file);

    printf("Kandou Retimer Error report dumped successfully !\n");
    printf("Dump file : %s\n", filename);

    return KB900X_E_OK;
}
