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

#ifndef _KB_COMMAND_H
#define _KB_COMMAND_H

// SMBus offsets
#define KB900X_ADDR_TEMP_1ST_LANE_B_RX (0x10)
#define KB900X_ADDR_TEMP_1ST_LANE_A_RX (0x30)

#define KB900X_ADDR_VID (0x0004)
#define KB900X_ADDR_FW_VERSION (0x0500)
#define KB900X_ADDR_TEMPERATURE (0x0510) // 0x10 = First lane A, step 0x04
#define KB900X_ADDR_LINK_STATUS_GATHER (0x0560)
#define KB900X_ADDR_LINK_STATUS_READY (0x0564)
#define KB900X_ADDR_LINK_STATUS (0x0568) // 0x68 = First link 0, step 0x04 (8 links total)
#define KB900X_ADDR_FIRMWARE_HEALTH (0x05B0)

#define KB900X_ADDR_PRESET_REQ (0x05D4)
#define KB900X_ADDR_PRESET_STATUS (0x05D8)
#define KB900X_ADDR_PRESET_START_ADDR (0x05DC)
#define KB900X_ADDR_PRESET_LENGTH (0x05E0)

#define KB900X_ADDR_RTSSM_REQ (0x05C4)
#define KB900X_ADDR_RTSSM_STATUS (0x05C8)
#define KB900X_ADDR_RTSSM_START_ADDR (0x05CC)
#define KB900X_ADDR_RTSSM_LENGTH (0x05D0)

// Register addresses
#define kb900x_cpu_reset_vector_addr 0xe0090010
#define kb900x_cpu_system 0xe0090008
#define kb900x_cpu_scratchpad 0xe009000c
#define kb900x_cpu_reset 0xe0090004
#define kb900x_cfg_top_revid 0xe0480004

// Constants
#define KB900X_APB_OFFSET (0xE0000000)
#define KB900X_TILE (0x01000000)
#define KB900X_PHY_CORE_OFFSET (0x00100000)
#define KB900X_PHY_CORE (0x00040000)
#define KB900X_PHY_LANE (0x00000800)

// RPCS Debug Counter
#define KB900X_RPCS_DBG_COUNTER_BASE_ADDR (0xE04C0660)
#define KB900X_RPCS_DBG_COUNTER_STEPS (448)
#define KB900X_RPCS_DBG_COUNTER_OFFSET_READ (4)

// Phy registers
#define KB900X_RX_STARTUP_FOM (0x8434)
#define KB900X_RX_MM_FOM (0x8430)
#define KB900X_RX_ATT (0xC054)
#define KB900X_RX_ATT_B1 (0xC080)
#define KB900X_RX_ATT_B2 (0xC2E8)
#define KB900X_RX_CTLE (0xC05C) // contains RX CTLE BOOST and RX CTLE POLE
#define KB900X_RX_CTLE_B1 (0xC088)
#define KB900X_RX_CTLE_B2 (0xC2F0)
#define KB900X_RX_VGA (0xC058)
#define KB900X_RX_VGA_B1 (0xC084)
#define KB900X_RX_VGA_B2 (0xC2EC)
#define KB900X_AFE_RTRIM (0xC4A8)
#define KB900X_RX_AFE_RATE (0xC4F8)
#define KB900X_RX_AFE_RATE_B1 (0xC504)
#define KB900X_RX_AFE_RATE_B2 (0xC510)
#define KB900X_RX_AFE_CONFIG (0xC4FC)
#define KB900X_RX_AFE_CONFIG_B1 (0xC508)
#define KB900X_RX_AFE_CONFIG_B2 (0xC514)
#define KB900X_RX_DFE_TAP1 (0xC060)
#define KB900X_RX_DFE_TAP1_B1 (0xC08C)
#define KB900X_RX_DFE_TAP1_B2 (0xC2F4)
#define KB900X_RX_DFE_TAP6 (0xC2A0)
#define KB900X_RX_DFE_TAP6_B1 (0xC2AC)
#define KB900X_RX_DFE_TAP6_B2 (0xC314)
#define KB900X_RX_DFE_FLOAT_TAP1 (0xC2BC)
#define KB900X_RX_DFE_FLOAT_TAP1_B1 (0xC2D4)
#define KB900X_RX_DFE_FLOAT_TAP1_B2 (0xC324)
#define KB900X_RX_DFE_FLOAT_TAP_POS (0xC2B8)
#define KB900X_RX_DFE_FLOAT_TAP_POS_B1 (0xC2D0)
#define KB900X_RX_DFE_FLOAT_TAP_POS_B2 (0xC320)
#define KB900X_RX_DPLL_FREQ (0x4274)
#define KB900X_TX_ASIC_IN_1 (0x4038)
#define KB900X_TX_ASIC_IN_2 (0x403C)
#define KB900X_RX_ASIC_IN_0 (0x40C4)
#define KB900X_RX_OVRD_IN_1 (0x4084)
#define KB900X_RX_PCS_IN_5 (0x8050)

// Macros to calculate the register addresses
#define KB900X_CALC_ADDR(tile_id, phy_id, phy_lane_id)                                             \
    (KB900X_APB_OFFSET + KB900X_PHY_CORE_OFFSET + tile_id * KB900X_TILE +                          \
     phy_id * KB900X_PHY_CORE + phy_lane_id * KB900X_PHY_LANE)

#define KB900X_RX_STARTUP_FOM_ADDR(tile_id, phy_id, phy_lane_id)                                   \
    (KB900X_CALC_ADDR(tile_id, phy_id, phy_lane_id) + KB900X_RX_STARTUP_FOM)

#define KB900X_RX_MM_FOM_ADDR(tile_id, phy_id, phy_lane_id)                                        \
    (KB900X_CALC_ADDR(tile_id, phy_id, phy_lane_id) + KB900X_RX_MM_FOM)

#define KB900X_RX_ATT_ADDR(tile_id, phy_id, phy_lane_id, bank)                                     \
    (KB900X_CALC_ADDR(tile_id, phy_id, phy_lane_id) +                                              \
     (bank == 2 ? KB900X_RX_ATT_B2 : (bank == 1 ? KB900X_RX_ATT_B1 : KB900X_RX_ATT)))

#define KB900X_RX_CTLE_ADDR(tile_id, phy_id, phy_lane_id, bank)                                    \
    (KB900X_CALC_ADDR(tile_id, phy_id, phy_lane_id) +                                              \
     (bank == 2 ? KB900X_RX_CTLE_B2 : (bank == 1 ? KB900X_RX_CTLE_B1 : KB900X_RX_CTLE)))

#define KB900X_RX_VGA_ADDR(tile_id, phy_id, phy_lane_id, bank)                                     \
    (KB900X_CALC_ADDR(tile_id, phy_id, phy_lane_id) +                                              \
     (bank == 2 ? KB900X_RX_VGA_B2 : (bank == 1 ? KB900X_RX_VGA_B1 : KB900X_RX_VGA)))

#define KB900X_AFE_RTRIM_ADDR(tile_id, phy_id, phy_lane_id)                                        \
    (KB900X_CALC_ADDR(tile_id, phy_id, phy_lane_id) + KB900X_AFE_RTRIM)

#define KB900X_RX_AFE_RATE_ADDR(tile_id, phy_id, phy_lane_id, bank)                                \
    (KB900X_CALC_ADDR(tile_id, phy_id, phy_lane_id) +                                              \
     (bank == 2 ? KB900X_RX_AFE_RATE_B2                                                            \
                : (bank == 1 ? KB900X_RX_AFE_RATE_B1 : KB900X_RX_AFE_RATE)))

#define KB900X_RX_AFE_CONFIG_ADDR(tile_id, phy_id, phy_lane_id, bank)                              \
    (KB900X_CALC_ADDR(tile_id, phy_id, phy_lane_id) +                                              \
     (bank == 2 ? KB900X_RX_AFE_CONFIG_B2                                                          \
                : (bank == 1 ? KB900X_RX_AFE_CONFIG_B1 : KB900X_RX_AFE_CONFIG)))

#define KB900X_RX_DFE_TAP_ADDR(tile_id, phy_id, phy_lane_id, bank, tap)                            \
    (KB900X_CALC_ADDR(tile_id, phy_id, phy_lane_id) +                                              \
     (tap < 5 ? (bank == 2 ? KB900X_RX_DFE_TAP1_B2                                                 \
                           : (bank == 1 ? KB900X_RX_DFE_TAP1_B1 : KB900X_RX_DFE_TAP1)) +           \
                    4 * tap                                                                        \
              : (bank == 2 ? KB900X_RX_DFE_TAP6_B2                                                 \
                           : (bank == 1 ? KB900X_RX_DFE_TAP6_B1 : KB900X_RX_DFE_TAP6)) +           \
                    4 * (tap - 5)))

#define KB900X_RX_DFE_FLOAT_TAP_ADDR(tile_id, phy_id, phy_lane_id, bank, tap)                      \
    (KB900X_CALC_ADDR(tile_id, phy_id, phy_lane_id) +                                              \
     (bank == 2 ? KB900X_RX_DFE_FLOAT_TAP1_B2                                                      \
                : (bank == 1 ? KB900X_RX_DFE_FLOAT_TAP1_B1 : KB900X_RX_DFE_FLOAT_TAP1)) +          \
     4 * tap)

#define KB900X_RX_DFE_FLOAT_TAP_POS_ADDR(tile_id, phy_id, phy_lane_id, bank)                       \
    (KB900X_CALC_ADDR(tile_id, phy_id, phy_lane_id) +                                              \
     (bank == 2 ? KB900X_RX_DFE_FLOAT_TAP_POS_B2                                                   \
                : (bank == 1 ? KB900X_RX_DFE_FLOAT_TAP_POS_B1 : KB900X_RX_DFE_FLOAT_TAP_POS)))

#define KB900X_RX_DPLL_FREQ_ADDR(tile_id, phy_id, phy_lane_id)                                     \
    (KB900X_CALC_ADDR(tile_id, phy_id, phy_lane_id) + KB900X_RX_DPLL_FREQ)

#define KB900X_TX_ASIC_IN_1_ADDR(tile_id, phy_id, phy_lane_id)                                     \
    (KB900X_CALC_ADDR(tile_id, phy_id, phy_lane_id) + KB900X_TX_ASIC_IN_1)

#define KB900X_TX_ASIC_IN_2_ADDR(tile_id, phy_id, phy_lane_id)                                     \
    (KB900X_CALC_ADDR(tile_id, phy_id, phy_lane_id) + KB900X_TX_ASIC_IN_2)

#define KB900X_RX_ASIC_IN_0_ADDR(tile_id, phy_id, phy_lane_id)                                     \
    (KB900X_CALC_ADDR(tile_id, phy_id, phy_lane_id) + KB900X_RX_ASIC_IN_0)

#define KB900X_RX_OVRD_IN_1_ADDR(tile_id, phy_id, phy_lane_id)                                     \
    (KB900X_CALC_ADDR(tile_id, phy_id, phy_lane_id) + KB900X_RX_OVRD_IN_1)

#define KB900X_RX_PCS_IN_5_ADDR(tile_id, phy_id, phy_lane_id)                                      \
    (KB900X_CALC_ADDR(tile_id, phy_id, phy_lane_id) + KB900X_RX_PCS_IN_5)

// Compute RPCS Debug Counter address
#define KB900X_RPCS_DBG_COUNTER_ADDR_CONF(tile_id, rpcs_id)                                        \
    ((((KB900X_RPCS_DBG_COUNTER_BASE_ADDR + (rpcs_id * KB900X_RPCS_DBG_COUNTER_STEPS)) << 8) >>    \
      8) |                                                                                         \
     (KB900X_APB_OFFSET + tile_id * KB900X_TILE))

#define KB900X_RPCS_DBG_COUNTER_ADDR_READ(rpcs_dbg_counter_addr_conf)                              \
    (rpcs_dbg_counter_addr_conf + KB900X_RPCS_DBG_COUNTER_OFFSET_READ)

#endif // _KB_COMMAND_H
