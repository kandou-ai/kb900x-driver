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

#ifndef _KB_REGLI_H
#define _KB_REGLI_H

#include <assert.h> // For static_assert

#include "kb900x_eeprom.h"
#include "kb900x_i2c_comm.h"
#include "kb900x_smbus_comm.h"

// Constants
#define KB900X_REGLI_REGISTER_ADDR_SIZE (4)
#define KB900X_REGLI_REGISTER_SIZE (4)
#define KB900X_SMBUS_REGISTER_ADDR_SIZE (2)
#define KB900X_SMBUS_REGISTER_SIZE (4)
#define KB900X_LINK_STATUS_TIMEOUT (500) // FIXME magic number - How much do we have to wait?
#define KB900X_SIZEOF_SW_SHARED_DATA                                                               \
    (0x400 * 4) // NOTE: MUST be a multiple of 4, because we can only read in 4B chunks
#define KB9003_NUM_TILES (4)
#define KB900X_NUM_RPCS (8)
#define KB900X_NUM_RPCS_EVENTS (128)
#define KB9003_NUM_LANES (16)
#define KB9003_MAX_NUM_LINKS (8)
#define KB900X_DCCM_END_ADDR_A0 (0x80008000)
#define KB900X_DCCM_END_ADDR_B0_B1 (0x80010000)
#define KB900X_SDS_MAGIC_HEADER (0x4B425353) // 'KBSS' big-endian
#define KB900X_SW_RTSSM_SIZE (341)
#define KB900X_FEAT_REQ_RETRIES (10)
#define KB900X_FW_TRACE_START_ADDR (0x8000e5ec)
#define KB900X_FW_TRACE_SIZE (637)

// HW RTSSM delta conversion constants
#define KB900X_DELTA_OVERFLOW (0xFFFFFFFF)
#define KB900X_DELTA_IS_OVERFLOW(delta) (delta >= 0x7F)
#define KB900X_DELTA_IS_RESOLUTION_1US(delta) ((delta >> 6) == 0x0)     // no leading 1
#define KB900X_DELTA_IS_RESOLUTION_32US(delta) ((delta >> 5) == 0x2)    // 1 leading 1
#define KB900X_DELTA_IS_RESOLUTION_512US(delta) ((delta >> 4) == 0x6)   // 2 leading 1
#define KB900X_DELTA_IS_RESOLUTION_8192US(delta) ((delta >> 3) == 0xE)  // 3 leading 1
#define KB900X_DELTA_IS_RESOLUTION_32768US(delta) ((delta >> 3) == 0xF) // 4 leading 1

// Macros
#define KB900X_STATE_TO_STRING(state) (KB900X_STATE_STRING[state])
#define KB900X_GEN_TO_STRING(gen) (KB900X_GEN_STRING[gen])

#define KB900X_GENERATE_ENUM(ENUM) ENUM,
#define KB900X_GENERATE_STRING(STRING) #STRING,

#define KB900X_FOREACH_GEN(GEN)                                                                    \
    GEN(GEN1)                                                                                      \
    GEN(GEN2)                                                                                      \
    GEN(GEN3)                                                                                      \
    GEN(GEN4)                                                                                      \
    GEN(GEN5)                                                                                      \
    GEN(UNKNOWN1)                                                                                  \
    GEN(UNKNOWN2)                                                                                  \
    GEN(INVALID_ENTRY)

#define KB900X_FOREACH_STATE(STATE)                                                                \
    STATE(RESET)                                                                                   \
    STATE(WAIT_RX_DET)                                                                             \
    STATE(WAIT_DET_ALL)                                                                            \
    STATE(IDLE)                                                                                    \
    STATE(SPEED_DET)                                                                               \
    STATE(EIE_DET)                                                                                 \
    STATE(F_S_EIE_NDET_A)                                                                          \
    STATE(EIE_WO_IMP)                                                                              \
    STATE(F_RX_DET_B_R2)                                                                           \
    STATE(F_RX_DET_B_R4)                                                                           \
    STATE(F_S_EIE_DET_B)                                                                           \
    STATE(F_SPEED_CHNG)                                                                            \
    STATE(F_SPEED_CH_2)                                                                            \
    STATE(F_DATA_CHK)                                                                              \
    STATE(F_DATA_TS)                                                                               \
    STATE(F_DATA_UP)                                                                               \
    STATE(F_DATA_N_TS)                                                                             \
    STATE(F_HOT_RST)                                                                               \
    STATE(F_DISABLED)                                                                              \
    STATE(F_HR_DIS)                                                                                \
    STATE(F_L1_ENTRY)                                                                              \
    STATE(F_L1_IDLE)                                                                               \
    STATE(F_L1_IDLE_EXIT)                                                                          \
    STATE(F_N_TS_NM)                                                                               \
    STATE(F_TS_NM)                                                                                 \
    STATE(E_EQ_IDLE)                                                                               \
    STATE(E_UP_PH2_A)                                                                              \
    STATE(E_UP_PH2_P)                                                                              \
    STATE(E_UP_PH3_RE)                                                                             \
    STATE(E_UP_PH3)                                                                                \
    STATE(E_DWN_PH2_RE)                                                                            \
    STATE(E_DWN_PH2)                                                                               \
    STATE(E_DWN_PH3_A)                                                                             \
    STATE(E_DWN_PH3_P)                                                                             \
    STATE(E_FRC_TO)                                                                                \
    STATE(E_FRC_TO_EXT)                                                                            \
    STATE(E_EQ_DONE)                                                                               \
    STATE(E_SL_ENTRY)                                                                              \
    STATE(E_SL_PARK)                                                                               \
    STATE(E_SL_SPEED)                                                                              \
    STATE(E_SL_ACTIVE)                                                                             \
    STATE(E_SL_EXIT)                                                                               \
    STATE(E_SL_EQ_PH1_D)                                                                           \
    STATE(E_SL_EQ_PH0)                                                                             \
    STATE(E_SL_EQ_PH1_U)                                                                           \
    STATE(F_LB_PRE_SYNC)                                                                           \
    STATE(F_LB)                                                                                    \
    STATE(F_LB_EXIT)                                                                               \
    STATE(F_LB_SPEED)                                                                              \
    STATE(F_LB_SPEED_DONE)                                                                         \
    STATE(F_LB_DIR)                                                                                \
    STATE(F_COMPL_ENTRY)                                                                           \
    STATE(F_COMPL_LOCK)                                                                            \
    STATE(F_COMPL_PTRN)                                                                            \
    STATE(F_COMPL_EXT)                                                                             \
    STATE(E_CLB_ENTRY)                                                                             \
    STATE(E_CLB_CHG_DR)                                                                            \
    STATE(E_CLB_PTRN)                                                                              \
    STATE(E_CLB_EXT)                                                                               \
    STATE(NA_1)                                                                                    \
    STATE(NA_2)                                                                                    \
    STATE(NA_3)                                                                                    \
    STATE(NA_4)                                                                                    \
    STATE(NA_5)                                                                                    \
    STATE(STATE_COUNT)

// Enums
// Revision ID
typedef enum {
    KB900X_REVID_A0 = 0x00000000,
    KB900X_REVID_B0 = 0x00000010,
    KB900X_REVID_B1 = 0x00000011,
} kb900x_revid_t;

// Mailbox status constants
typedef enum {
    /** status_not set */
    KB900X_FEAT_REQ_STATUS_NOT_SET = 0,
    /** Request succeeded */
    KB900X_FEAT_REQ_STATUS_SUCCESS = 1,
    /** Request in progress */
    KB900X_FEAT_REQ_STATUS_IN_PROGRESS = 2,
    /** Request failed */
    KB900X_FEAT_REQ_STATUS_FAILURE = 3,
} kb900x_feature_req_status_t;

// NOTE: these comments are used to denote what must be included in the headerfile for the CFFI
//! CFFI
// Communication modes
typedef enum {
    KB900X_COMM_SMBUS = 0,
    KB900X_COMM_TWI = 1,
    KB900X_COMM_BIC = 2,
} kb900x_communication_mode_t;
// NOTE: these comments are used to denote what must be included in the headerfile for the CFFI
//! CFFI END

// Boot Status
typedef enum {
    KB900X_UNKNOWN = 0,
    KB900X_BOOT_ROM = 1,
    KB900X_SBL = 2,
    KB900X_FW = 3
} kb900x_boot_entity_t;

typedef enum {
    KB900X_STATE_INIT = 0,
    KB900X_STATE_READY = 1,
    KB900X_STATE_ERROR = 2,
} kb900x_boot_status_t;

// RTSSM states
enum KB900X_RTSSM_STATES { KB900X_FOREACH_STATE(KB900X_GENERATE_ENUM) };

// PCIe gen
enum KB900X_PCIE_GEN { KB900X_FOREACH_GEN(KB900X_GENERATE_ENUM) };

// Types
/**
 * \brief This struct is used to hold the firmware version (kb900x_get_firmware_version)
 *
 * The format of the FW version is :
 * major.minor.patch.suffix
 *
 */
typedef union {
    struct {
        /** Major version (int) */
        uint8_t major;
        /** Minor version (int) */
        uint8_t minor;
        /** Patch (int) */
        uint8_t patch;
        /** Suffix (int) */
        uint8_t suffix;
    };
    uint32_t raw;
} kb900x_fw_version_t;
static_assert(sizeof(kb900x_fw_version_t) == sizeof(uint32_t),
              "kb900x_fw_version_t size mismatch!");

/**
 * \brief This struct is used to hold the firmware health (kb900x_get_firmware_health)
 */
typedef union {
    struct {
        /* liveliness counter */
        uint32_t liveliness : 4;
        /* unused */
        uint32_t : 27;
        /* Firmware is initialized */
        uint32_t fw_is_initialized : 1;
    };
    uint32_t raw;
} kb900x_fw_health_t;
static_assert(sizeof(kb900x_fw_health_t) == sizeof(uint32_t), "kb900x_fw_health_t size mismatch!");

/**
 * \brief This struct is used to hold the status of a link (kb900x_get_link_status)
 */
typedef union {
    struct {
        uint32_t link_invalid : 1;
        uint32_t l0_reached : 1;
        uint32_t link_number : 9;
        uint32_t rtssm_state : 5;
        uint32_t rtssm_speed : 3;
        uint32_t link_width : 5;
        /* unused */
        uint32_t : 8;
    };
    uint32_t raw;
} ALIGN_PACKED(4) kb900x_link_status_t;
static_assert(sizeof(kb900x_link_status_t) == sizeof(uint32_t),
              "kb900x_link_status_t size mismatch!");

/**
 * \brief This struct stores a register address and value tuple, for register dumps.
 */
typedef struct {
    uint32_t address;
    uint32_t value;
} ALIGN_PACKED(4) kb900x_register_record_t;
static_assert(sizeof(kb900x_register_record_t) == 2 * sizeof(uint32_t),
              "kb900x_link_status_t size mismatch!");

/**
 * \brief This struct is used to hold the register dump (kb900x_get_register_dump)
 *
 * It contains a list of register records, the number of records and a name.
 */
typedef struct {
    /** The number of records in this dump */
    uint32_t num_records;
    /** The register records */
    kb900x_register_record_t *records;
} ALIGN_PACKED(4) kb900x_register_dump_t;

/**
 * \brief Encode a range of register addresses to dump.
 */
typedef struct {
    /** The base address of this range. */
    uint32_t base_address;
    /** The number of registers in this range. */
    uint32_t num_registers;
} kb900x_register_range_t;

/**
 * \brief The preset config structure
 */
typedef struct {
    /** The lane ID in range 0 to 15*/
    uint8_t lane_id;
    /** The data rate in range 0 to 4 (PCIe Gen 5 = 4, PCIe Gen4 = 3, ...)*/
    uint8_t data_rate;
    /** Unused */
    uint16_t pad;
} ALIGN_PACKED(4) kb900x_preset_config_t;
static_assert(sizeof(kb900x_preset_config_t) == sizeof(uint32_t),
              "kb900x_preset_config_t size mismatch!");

/**
 * \brief The preset config structure
 *
 * It contains the different requested presets.
 */
typedef struct {
    /** Local upstream preset (retimer -> root complex) in range 0 to 10 */
    uint8_t rt_rc;
    /** Local downstream preset (retimer -> endpoint) in range 0 to 10 */
    uint8_t rt_ep;
    /** Partner upstream preset (root compex -> retimer) in range 0 to 10 */
    uint8_t rc_rt;
    /** Partner downstream preset (endpoint -> retimer) in range 0 to 10 */
    uint8_t ep_rt;
} ALIGN_PACKED(4) kb900x_preset_data_t;
static_assert(sizeof(kb900x_preset_data_t) == sizeof(uint32_t),
              "kb900x_preset_data_t size mismatch!");

/**
 * \brief The presets for a single lane structure
 */
typedef struct {
    kb900x_preset_config_t config; // preset config
    kb900x_preset_data_t data;     // config info
} ALIGN_PACKED(4) kb900x_presets_t;
static_assert(sizeof(kb900x_presets_t) == 2 * sizeof(uint32_t), "kb900x_presets_t size mismatch!");

/**
 * \brief The presets for all lanes structure
 */
typedef struct {
    kb900x_presets_t lanes[KB9003_NUM_LANES];
} ALIGN_PACKED(4) kb900x_all_presets_t;
static_assert(sizeof(kb900x_all_presets_t) == 32 * sizeof(uint32_t),
              "kb900x_all_presets_t size mismatch!");

/**
 * \brief Type representing a single entry in a log.
 *
 * \note The delta parameter must be interpreted using the delta_to_ms function.
 */
typedef union {
    struct {
        /** The RTSSM state */
        uint16_t rtssm : 6;
        /** The data rate in range 0 to 4 (invalid =7, PCIe Gen 5 = 4, PCIe Gen4 = 3, ...)*/
        uint16_t data_rate : 3;
        /** The time delta */
        uint16_t delta : 7;
    };
    uint16_t raw;
} ALIGN_PACKED(2) kb900x_rtssm_entry_t;
static_assert(sizeof(kb900x_rtssm_entry_t) == sizeof(uint16_t),
              "kb900x_rtssm_entry_t size mismatch!");

/**
 * \brief Type representing a log map info.
 */
typedef union {
    struct {
        /** The tile ID */
        uint32_t tile_id : 8;
        /** The RPCS ID */
        uint32_t rpcs_id : 8;
        /** The RTSSM config ID */
        uint32_t rtssm_cfg_id : 8;
        /** The current position */
        uint32_t curr_pos : 4;
        /** The era */
        uint32_t era : 4;
    };
    uint32_t raw;
} ALIGN_PACKED(4) kb900x_rtssm_log_map_info_t;
static_assert(sizeof(kb900x_rtssm_log_map_info_t) == sizeof(uint32_t),
              "kb900x_rtssm_log_map_info_t size mismatch!");

/**
 * \brief Struct representing all RTSSM entries of a single RPCS
 */
typedef struct {
    kb900x_rtssm_log_map_info_t log_map_info;
    kb900x_rtssm_entry_t entries[32];
} ALIGN_PACKED(4) kb900x_rtssm_log_t;
static_assert(sizeof(kb900x_rtssm_log_t) == 17 * sizeof(uint32_t),
              "kb900x_rtssm_log_t size mismatch!");

/**
 * \brief Struct representing all the RTSSM logs on a device
 */
typedef struct {
    kb900x_rtssm_log_t logs[8]; // indexed log contents (some may be empty)
} ALIGN_PACKED(4) kb900x_hw_rtssm_logs_t;
static_assert(sizeof(kb900x_hw_rtssm_logs_t) == 8 * 17 * sizeof(uint32_t),
              "kb900x_hw_rtssm_logs_t size mismatch!");

/**
 * \brief Struct representing all the FOM (figure of merit) values on a device
 *
 * \note The FOM values are only available from PCIe gen4
 */
typedef struct {
    /** Startup FOM on side A RX | B TX */
    uint8_t startup_a_rx[16];
    /** Mission mode FOM on side A RX | B TX */
    uint8_t mm_a_rx[16];
    /** Startup FOM on side A TX | B RX */
    uint8_t startup_b_rx[16];
    /** Mission mode FOM on side A TX | B RX */
    uint8_t mm_b_rx[16];
} ALIGN_PACKED(4) kb900x_fom_t;
static_assert(sizeof(kb900x_fom_t) == 16 * sizeof(uint32_t), "kb900x_fom_t size mismatch!");

/**
 * \brief Struct used to map a lane to HW block IDs
 */
typedef struct {
    /** The tile ID */
    uint8_t tile_id;
    /** The phy ID */
    uint8_t phy_id;
    /** The lane ID within the phy */
    uint8_t phy_lane_id;
    /** The RPCS ID */
    uint8_t rpcs_id;
} ALIGN_PACKED(4) kb900x_lane_mapping_t;

/**
 * \brief Struct used to map the sides and lanes to HW block IDs
 */
typedef struct {
    /** Side A RX | B TX */
    kb900x_lane_mapping_t a_rx[16];
    /** Side A TX | B RX */
    kb900x_lane_mapping_t b_rx[16];
} ALIGN_PACKED(4) kb9003_mapping_t;

/**
 * \brief Struct containing the phy info for a single phy
 */
typedef struct {
    /** RX_ATT */
    uint32_t rx_att;
    /** RX_CTLE */
    uint32_t rx_ctle;
    /** RX_VGA */
    uint32_t rx_vga;
    /** AFE_RTRIM */
    uint32_t afe_rtrim;
    /** RX_AFE_RATE */
    uint32_t rx_afe_rate;
    /** RX_AFE_CONFIG */
    uint32_t rx_afe_config;
    /** RX_DFE_TAP */
    uint32_t rx_dfe_taps[8];
    /** RX_DFE_FLOAT_TAP */
    uint32_t rx_dfe_float_taps[4];
    /** RX_DFE_FLOAT_TAP_POS */
    uint32_t rx_dfe_float_tap_pos;
    /** RX_DPLL_FREQ */
    uint32_t rx_dpll_freq;
    /** TX_ASIC_IN_1 */
    uint32_t tx_asic_in_1;
    /** TX_ASIC_IN_2 */
    uint32_t tx_asic_in_2;
    /** RX_STARTUP_FOM */
    uint32_t rx_startup_fom;
    /** RX_MM_FOM */
    uint32_t rx_mm_fom;
} ALIGN_PACKED(4) kb9003_single_phy_info_t;

/**
 * \brief Struct containing the phy info (all lanes)
 */
typedef struct {
    /** Side A RX | B TX */
    kb9003_single_phy_info_t a_rx[16];
    /** Side A TX | B RX */
    kb9003_single_phy_info_t b_rx[16];
} ALIGN_PACKED(4) kb9003_phy_info_t;
static_assert(sizeof(kb9003_phy_info_t) == 32 * sizeof(kb9003_single_phy_info_t),
              "kb9003_phy_info_t size mismatch!");

/**
 * \brief Struct containing a single SW RTSSM entry
 */
typedef struct {
    uint32_t curr_state : 6;
    uint32_t prev_state : 6;
    uint32_t prev_prev_state : 6;
    uint32_t speed : 3;
    uint32_t rpcs_id : 3;
    uint32_t is_valid : 1;
    uint32_t : 7;
    uint32_t timestamp;
} ALIGN_PACKED(4) kb900x_sw_rtssm_entry_t;
static_assert(sizeof(kb900x_sw_rtssm_entry_t) == sizeof(uint64_t),
              "kb900x_sw_rtssm_entry_t size mismatch!");

/**
 * \brief Struct containing the SW RTSSM log
 */
typedef struct {
    kb900x_sw_rtssm_entry_t entries[KB900X_SW_RTSSM_SIZE];
} ALIGN_PACKED(4) kb900x_sw_rtssm_logs_t;
static_assert(sizeof(kb900x_sw_rtssm_logs_t) == KB900X_SW_RTSSM_SIZE * sizeof(uint64_t),
              "kb900x_sw_rtssm_log_t size mismatch!");

/**
 * \brief Struct to hold an entry of the RPCS debug counter
 */
typedef struct {
    uint8_t tile_id;
    uint8_t rpcs_id;
    uint8_t event_code;
    uint8_t _;
    uint32_t value;
} ALIGN_PACKED(4) kb900x_rpcs_debug_counter_entry_t;
static_assert(sizeof(kb900x_rpcs_debug_counter_entry_t) == 2 * sizeof(uint32_t),
              "kb900x_rpcs_debug_counter_t size mismatch!");

/**
 * \brief Struct to hold the RPCS debug counter informations
 */
typedef struct {
    kb900x_rpcs_debug_counter_entry_t
        entries[KB900X_NUM_RPCS * KB9003_NUM_TILES * KB900X_NUM_RPCS_EVENTS];
} ALIGN_PACKED(4) kb900x_rpcs_debug_counter_t;
static_assert(sizeof(kb900x_rpcs_debug_counter_t) == KB900X_NUM_RPCS * KB9003_NUM_TILES *
                                                         KB900X_NUM_RPCS_EVENTS *
                                                         sizeof(kb900x_rpcs_debug_counter_entry_t),
              "kb900x_rpcs_debug_counter_t size mismatch!");

// Type and struct to inject different read/write methods (SMBus, raw I2C, mocked for unit tests)
typedef int (*KB900X_WRITE_OPERATION)(const kb900x_config_t *config, const uint32_t address,
                                      const uint8_t address_size, const uint32_t value);
typedef int (*KB900X_READ_OPERATION)(const kb900x_config_t *config, const uint32_t address,
                                     const uint8_t address_size, uint32_t *value);

typedef struct {
    KB900X_WRITE_OPERATION write;
    KB900X_READ_OPERATION read;
} KB900X_IO;

// Globals
extern KB900X_IO io;
extern const char *KB900X_STATE_STRING[];
extern const char *KB900X_GEN_STRING[];

// NOTE: these comments are used to denote what must be included in the headerfile for the CFFI
//! CFFI

/** \brief Open the I2C connection to the KB900x retimer.
 *
 * This function is used to open the I2C interface in order to communicate with KB900x.
 *
 * It uses `config->bus_id` and `config->retimer_addr` to open and configure the I2C interface,
 * and then sets `config->handle` to point to the opened I2C interface.
 *
 * After calling this function, the user needs to initialize or select the communication mode:
 *
 * - If you want to initialize the library communication mode to match KB900x's current
 * communication mode, use kb900x_detect_communication mode. WARNING:
 * kb900x_detect_communication_mode cannot detect if BIC mode is needed, because KB900x does not
 * know if it is connected to the library host over IPMB or just plain SMBUS. If you want to use BIC
 * mode, use kb900x_switch_communication_mode(..., KB900X_COMM_BIC) instead.
 * - If you want to set the library communication mode and configure KB900x accordingly,
 *   use kb900x_switch_communication_mode.
 *
 * \param[in, out] config the config context with the `bus_id` and the `retimer_addr` attributes
 * set, cannot be NULL, the function updates `handle` with the I2C handle
 *
 * \return 0 if no error, else the error code
 */
int kb900x_open(kb900x_config_t *config);

/** \brief Close the I2C connection to the KB900x retimer.
 *
 * This function closes the previously opened I2C interface.
 *
 * \param[in] config the config context, cannot be NULL
 */
void kb900x_close(kb900x_config_t *config);

/** \brief Set the communication mode used by the library.
 *
 * \warning If you switch to BIC / IPMB mode, you must configure `config->intf` and
 * `config->slot_id`.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[in] mode the communication mode
 *
 * \return 0 if no error, else the error code
 */
int kb900x_set_communication_mode(const kb900x_config_t *config, kb900x_communication_mode_t mode);

/** \brief Configure the library to use the same communication as KB900x currently uses, and write
 * the communication mode to `mode`.
 *
 * \warning kb900x_detect_communication_mode cannot detect if BIC mode is needed, because KB900x
 * does not know if it is connected to the library host over IPMB or just plain SMBUS. If connected
 * over BIC / IPMB, it will detect SMBUS mode. If you want to use BIC mode, call
 * kb900x_switch_communication_mode(..., KB900X_COMM_BIC) instead.
 *
 * \note This function first tries to communicate with KB900x over SMBUS. If it succeeds, the
 * detected communication is SMBUS. If it fails (PEC invalid), it then tries to communicate with
 * KB900x over TWI. If it succeeds, the detected communication mode is is TWI. If it fails, it
 * returns an error code, the library is not configured and `mode` is not set.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[out] mode the communication mode, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_detect_communication_mode(const kb900x_config_t *config,
                                     kb900x_communication_mode_t *mode);

/** \brief Configure KB900x and the library to use the desired communication mode.
 *
 * \note Switching from SMBUS / BIC to TWI mode is not yet implemented. This means that if KB900x
 * and the library are currently in SMBUS / BIC mode, and this function is called with mode = TWI,
 * this will fail to switch mode and return an error.
 *
 * \warning If you switch to BIC / IPMB mode, you must configure `config->intf` and
 * `config->slot_id`.
 *
 * \warning This function waits for 2s to give the
 * firmware enough time to boot.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[in] mode the desired communication mode
 *
 * \return 0 if no error, else the error code
 */
int kb900x_switch_communication_mode(const kb900x_config_t *config,
                                     kb900x_communication_mode_t mode);
// NOTE: these comments are used to denote what must be included in the headerfile for the CFFI
//! CFFI END

/** \brief Read the temperature of a retimer lane.
 *
 * The lane information is encoded in `side` and `lane`. The retimer has 2 sides (A and B)
 * and 16 lanes on each side.
 *
 * \warning Only in SMBUS / BIC mode.
 *
 * \note This function gets the temperature of the sensor close to the provided lane.
 * There are 16 sensors in total on the KB9003 (1 sensor for 2 lanes). Therefore
 * adjacent lanes may have the same temperature.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[in] side the side, 0 = side A (upstream), anything else = side B (downstream)
 * \param[in] lane the lane id, 0 to 15
 * \param[out] temperature pointer to the temperature[Celsius], cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_get_lane_temperature(const kb900x_config_t *config, int side, int lane,
                                float *temperature);

/** \brief Read the max retimer temperature.
 *
 * This function reads the temperature of all lanes of the retimer and returns the highest.
 *
 * \warning Only in SMBUS / BIC mode.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[out] temperature pointer to the temperature[Celsius], cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_get_temperature(const kb900x_config_t *config, float *temperature);

/** \brief Get the vendor ID.
 *
 * \warning Only in SMBUS / BIC mode.
 *
 * \note The Kandou AI vendor ID is 7791 (0x1E6F).
 *
 * \note The full list of vendor IDs can be found here:
 * https://pcisig.com/membership/member-companies
 *
 * \param[in] config the config context, cannot be NULL
 * \param[out] vendor_id pointer to the vendor ID, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_get_vendor_id(const kb900x_config_t *config, uint32_t *vendor_id);

/** \brief Read the retimer firmware version.
 *
 * The format is as follows: major.minor.patch.suffix. For more detail, see `kb900x_fw_version_t`.
 *
 * \warning Only in SMBUS / BIC mode.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[out] fw_version pointer to the firmware version, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_get_firmware_version(const kb900x_config_t *config, kb900x_fw_version_t *fw_version);

/** \brief Read the firmware health information.
 *
 * \warning Only in SMBUS / BIC mode.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[out] firmware_health pointer to the firmware health, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_get_firmware_health(const kb900x_config_t *config, kb900x_fw_health_t *firmware_health);

/** \brief Read the SW RTSSM log.
 *
 * This function writes the entries it reads into `logs.entries`.
 *
 * \warning Only in SMBUS / BIC mode.
 *
 * \warning This function may not initialize all entries in `logs.entries`. To avoid
 *       reading uninitialized memory, we recommend the caller initializes all
 *       entries in `logs.entries` such that `logs.entries[i].is_valid == 0`.
 *       This function then sets `logs.entries[i].is_valid = 1` for all the valid values
 *       that it found in the RTSSM log.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[out] logs a pointer to a kb900x_sw_rtssm_logs_t object, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_get_sw_rtssm_log(const kb900x_config_t *config, kb900x_sw_rtssm_logs_t *logs);

/** \brief Read the status of a given link.
 *
 * \warning Only in SMBUS / BIC mode.
 *
 * \note The link status should be considered invalid (no link with ID link_id) if
 * link_status.link_invalid == 1.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[in] link_id the link id (0-7)
 * \param[out] link_status pointer to the link status, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_get_link_status(const kb900x_config_t *config, int link_id,
                           kb900x_link_status_t *link_status);

// NOTE: these comments are used to denote what must be included in the headerfile for the CFFI
//! CFFI
/** \brief Write to a register with a 4-byte address.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[in] address the register address
 * \param[in] payload the payload to write in the register
 *
 * \return 0 if no error, else the error code
 */
int kb900x_write_register(const kb900x_config_t *config, const uint32_t address,
                          const uint32_t payload);

/** \brief Read from a register with a 4-byte address.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[in] address the register address
 * \param[out] result pointer where to store the value read, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_read_register(const kb900x_config_t *config, const uint32_t address, uint32_t *result);
// NOTE: these comments are used to denote what must be included in the headerfile for the CFFI
//! CFFI END

/** \brief Write a firmware image to the EEPROM.
 *
 * \note If the provided buffer is smaller than the EEPROM size (`config->eeprom_size`),
 * this function will fill the end of the EEPROM with 0xFF bytes.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[in] buffer a buffer containing a complete firmware image, cannot be NULL
 * \param[in] buffer_size the size of `buffer`, should not be larger than `config->eeprom_size`
 * \param[in] eeprom_config the EEPROM config structure, cannot be NULL, see
 * `kb900x_eeprom_config_t` for details
 *
 * \return 0 if no error, else the error code
 */
int kb900x_flash_firmware(const kb900x_config_t *config, const uint8_t *buffer,
                          const uint32_t buffer_size, const kb900x_eeprom_config_t *eeprom_config);

/** \brief Configure the firmware image in the EEPROM.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[in] eeprom_config the EEPROM config structure, cannot be NULL, see `eeprom_config_t` for
 * details
 * \param[in] config_payload the configuration payload to write to the EEPROM, cannot be NULL
 * \param[in] config_payload_size the size of `config_payload`
 *
 * \return 0 if no error, else the error code
 */
int kb900x_configure_firmware(const kb900x_config_t *config,
                              const kb900x_eeprom_config_t *eeprom_config,
                              const uint8_t *config_payload, const size_t config_payload_size);

/** \brief Compare the firmware image stored in the EEPROM, against a provided buffer.
 *
 * \note If the provided buffer is smaller than the EEPROM size (`config->eeprom_size`),
 * this function will compare the bytes of the EEPROM against 0xFF, if they are at addresses
 * >= `buffer_size`.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[in] buffer the buffer containing the expected firmware, cannot be NULL
 * \param[in] buffer_size the size of `buffer`
 * \param[in] eeprom_config the EEPROM config structure, cannot be NULL, see
 * `kb900x_eeprom_config_t` for details, cannot be NULL
 *
 * \return 0 if no error and the EEPROM contents match buffer, -EILSEQ if the bytes don't match,
 * otherwise some other error code
 */
int kb900x_check_firmware(const kb900x_config_t *config, const uint8_t *buffer,
                          const uint32_t buffer_size, const kb900x_eeprom_config_t *eeprom_config);

/** \brief Reset KB900x and reboot the firmware.
 *
 * \note We recommend waiting 2 seconds after this function returns before calling any other
 * function, to make sure the firmware boot sequence is complete.
 *
 * \param[in] config the config context, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_reset(const kb900x_config_t *config);

/**
 * \brief Read the boot status of KB900x.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[out] boot_status pointer to the boot status, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_get_boot_status(const kb900x_config_t *config, kb900x_boot_status_t *boot_status);

/**
 * \brief Read and return the rev ID of the KB900x chip.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[out] revid a pointer for writing the result, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_get_revid(const kb900x_config_t *config, uint32_t *revid);

/**
 * \brief Read the TX presets for all lanes.
 *
 * \warning Only in SMBUS / BIC mode.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[out] presets a pointer for writing the result, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_get_tx_presets(const kb900x_config_t *config, kb900x_all_presets_t *presets);

/**
 * \brief Get the HW RTSSM log.
 *
 * \warning Only in SMBUS / BIC mode.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[out] log a pointer for writing the result, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_get_hw_rtssm_log(const kb900x_config_t *config, kb900x_hw_rtssm_logs_t *log);

/**
 * \brief Convert HW RTSSM delta to us.
 *
 * \param[in] delta The HW RTSSM delta (see kb900x_get_hw_rtssm_log function)
 *
 * \return The corresponding time in us
 */
uint32_t kb900x_delta_to_us(uint8_t delta);

/**
 * \brief Get the FOM (figure of merit) on all lanes.
 *
 * This function fills a structure that contains the startup FOM
 * and mission mode FOM for all the lanes (both sides of the retimer).
 *
 * \warning Only in SMBUS / BIC mode.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[out] fom a pointer for writing the result, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_get_fom(const kb900x_config_t *config, kb900x_fom_t *fom);

/**
 * \brief Get the phy info on all lanes.
 *
 * This function fills a structure that contains all the phy info for all the lanes (both sides of
 * the retimer).
 *
 * \warning Only in SMBUS / BIC mode.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[out] phy_info a pointer for writing the result, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_get_phy_info(const kb900x_config_t *config, kb9003_phy_info_t *phy_info);

/**
 * \brief Get the firmware trace.
 *
 * This function reads the firmware trace from the KB900x retimer and fills the provided
 * `trace` structure with the trace data.
 *
 * \warning kb900x_register_dump_t::num_records has to be set to at least `KB900X_FW_TRACE_SIZE` and
 * kb900x_register_dump_t::records array has to be allocated accordingly.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[out] trace a pointer to a kb900x_register_dump_t with an array of size
 * `KB900X_FW_TRACE_SIZE` used to store the trace data, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_get_firmware_trace(const kb900x_config_t *config, kb900x_register_dump_t *trace);

/**
 * \brief Get the PHY and RPCS registers for all lanes.
 *
 * This function reads the PHY and RPCS registers for all lanes and fills the provided `rpcs_dump`
 * structure with the register data.
 *
 * \warning kb900x_register_dump_t::num_records has to be set to at least
 * `KB900X_PHY_RPCS_OUTPUT_SIZE` and kb900x_register_dump_t::records array has to be allocated
 * accordingly.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[out] rpcs_dump a pointer to a kb900x_register_dump_t with an array of size
 * `KB900X_PHY_RPCS_OUTPUT_SIZE` used to store the RPCS register data, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_get_phy_rpcs_registers(const kb900x_config_t *config, kb900x_register_dump_t *rpcs_dump);

/**
 * \brief Get the RPCS debug counter.
 *
 * This function reads the RPCS debug counter and fills the provided `kb900x_rpcs_debug_counter_t`
 * structure with the debug counter data.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[out] rpcs_dbg a pointer to a kb900x_rpcs_debug_counter_t, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_get_rpcs_dbg_counter(const kb900x_config_t *config,
                                kb900x_rpcs_debug_counter_t *rpcs_dbg);

//***** Log functions **********//
/**
 * \brief Write the firmware trace in a log file.
 *
 * \param[in] trace the firmware trace, cannot be NULL
 * \param[in] filename the name of the log file to generate, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_log_firmware_trace(kb900x_register_dump_t *trace, const char *filename);

/**
 * \brief Generate a log file from the HW RTSSM log.
 *
 * \note The log file is a JSON file that contains the HW RTSSM log.
 *
 * \param[in] data the HW RTSSM log, cannot be NULL
 * \param[in] filename the name of the JSON file to generate, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_log_hw_rtssm(const kb900x_hw_rtssm_logs_t *data, const char *filename);

/**
 * \brief Generate a log file from the TX presets.
 *
 * \note The log file is a JSON file that contains the TX presets.
 *
 * \param[in] data the TX presets, cannot be NULL
 * \param[in] filename the name of the JSON file to generate, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_log_tx_presets(const kb900x_all_presets_t *data, const char *filename);

/**
 * \brief Generate a log file from the dynamic and static FOM.
 *
 * \note The log file is a JSON file that contains the dynamic and static FOM.
 *
 * \param[in] data the FOM data, cannot be NULL
 * \param[in] filename the name of the JSON file to generate, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_log_fom(const kb900x_fom_t *data, const char *filename);

/**
 * \brief Generate a log file from the phy info.
 *
 * \note The log file is a JSON file that contains the phy info.
 *
 * \param[in] data the phy info, cannot be NULL
 * \param[in] filename the name of the JSON file to generate, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_log_phy_info(const kb9003_phy_info_t *data, const char *filename);

/**
 * \brief Generate a log file from the SW RTSSM logs.
 *
 * \note The log file is a JSON file that contains the SW RTSSM logs.
 *
 * \param[in] data the SW RTSSM logs, cannot be NULL
 * \param[in] filename the name of the JSON file to generate, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_log_sw_rtssm(const kb900x_sw_rtssm_logs_t *data, const char *filename);

/**
 * \brief Generate a log file from the registers dump.
 *
 * \param[in] record the registers dump, cannot be NULL
 * \param[in] filename the name of the log file to generate, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_log_registers_record(const kb900x_register_dump_t *record, const char *filename);

/**
 * \brief Generate a CSV file containing the RPCS and PHY registers.
 *
 * \param[in] data the data containing the RPCS and PHY registers, cannot be NULL
 * \param[in] filename the name of the CSV file to generate, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_log_phy_rpcs_registers(const kb900x_register_dump_t *data, const char *filename);

/**
 * \brief Generate a log file from the RPCS debug counter.
 *
 * \note The log file is a JSON file that contains the RPCS debug counter.
 *
 * \param[in] data the RPCS debug counter, cannot be NULL
 * \param[in] filename the name of the JSON file to generate, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_log_rpcs_dbg_counter(const kb900x_rpcs_debug_counter_t *data, const char *filename);

/**
 * \brief Read comprehensive debug information and dump it to a file.
 *
 * \warning Only in SMBUS / BIC mode.
 *
 * \note The log file is a JSON file that contains the TX Presets,
 * \note the dynamic and static FOM, the HW and SW RTSSM traces and
 * \note some usefull information about the PHYs.
 *
 * \param[in] config the config context, cannot be NULL
 * \param[in] filename the name of the JSON file to generate, cannot be NULL
 *
 * \return 0 if no error, else the error code
 */
int kb900x_error_dump(const kb900x_config_t *config, const char *filename);

//! CFFI END

#endif // _KB_SMBUS_H
