/**
 * @file phy_info.c
 * @author Kandou AI
 * @brief Example showing how to read the phy info
 *
 * This example opens a connection on /dev/i2c-1 (slave address 0x21) and
 * reads all the phy info on all lanes.
 * It then displays the data in 2 tables (one for each side).
 *
 * It is compatible with KB900x B0/B1.
 */

#include "kb900x.h"
#include <stdlib.h>
#include <time.h>

#define I2C_SLAVE_ADDR (0x21)
#define I2C_ID (1)

/**
 * \brief Print the header of the table.
 */
static void print_header()
{
    // Print header
    printf("| %-21s", "PHY INFO");
    for (uint8_t lane = 0; lane < 16; lane++) {
        printf("| LANE%02d%-5s", lane, "");
    }
    printf("|\n");

    printf("-----------------------------------------------------------------------------------"
           "-----------------------------------------------------------------------------------"
           "------------------------------------------------------------------\n");
}

/**
 * \brief Prints a row in the terminal.
 *
 * \param[in] name The name of the row (first column)
 * \param[in] ptr A pointer to an array of kb9003_single_phy_info_t
 * \param[in] offset The offset of the phy info to display in the kb9003_single_phy_info_t structure
 */
static void print_row(char *name, kb9003_single_phy_info_t *ptr, uint32_t offset)
{
    // Print row
    printf("| %-21s", name);
    for (uint8_t lane = 0; lane < KB9003_NUM_LANES; lane++) {
        printf("| 0x%08x ", *(uint32_t *)((uint8_t *)&ptr[lane] + offset));
    }
    printf("|\n");
}

int main(void)
{
    // Enable logging
    kandou_log_set_quiet(0);

    KANDOU_INFO("---- Reading from device: ----");
    KANDOU_INFO("I2C Device ID = %d", I2C_ID);
    KANDOU_INFO("I2C Slave Address = 0x%X", I2C_SLAVE_ADDR);

    // Create config struct
    kb900x_config_t config = {
        .bus_id = I2C_ID,
        .retimer_addr = I2C_SLAVE_ADDR,
    };

    // Open the connection with KB900x
    KANDOU_INFO("Opening I2C interface...");
    int ret = kb900x_open(&config);
    CHECK_SUCCESS_MSG(ret, "Failed to open I2C interface.");
    KANDOU_INFO("I2C handle = %d", config.handle);

    // Switch to SMBUS mode (required to various debug info)
    KANDOU_INFO("Switching to SMBUS mode...");
    ret = kb900x_switch_communication_mode(&config, KB900X_COMM_SMBUS);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to switch to SMBUS mode.");
        kb900x_close(&config);
        return -1;
    }

    KANDOU_INFO("Reading PHY info...");
    kb9003_phy_info_t phy_info;
    ret = kb900x_get_phy_info(&config, &phy_info);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to read PHY info: %s", kb900x_strerror(ret));
        kb900x_close(&config);
        return -1;
    }

    // Display each side
    for (uint8_t side = 0; side < 2; side++) {
        kb9003_single_phy_info_t *ptr;
        if (side == 0) {
            // Print title
            printf("Side: A RX | B TX\n");
            ptr = phy_info.a_rx;
        }
        else {
            // Print title
            printf("\n\nSide: A TX | B RX\n");
            ptr = phy_info.b_rx;
        }

        // Print table header
        print_header();

        // Print KB900X_RX_ATT
        print_row("KB900X_RX_ATT", ptr, offsetof(kb9003_single_phy_info_t, rx_att));

        // Print KB900X_RX_CTLE
        print_row("KB900X_RX_CTLE", ptr, offsetof(kb9003_single_phy_info_t, rx_ctle));

        // Print KB900X_RX_VGA
        print_row("KB900X_RX_VGA", ptr, offsetof(kb9003_single_phy_info_t, rx_vga));

        // Print KB900X_AFE_RTRIM
        print_row("KB900X_AFE_RTRIM", ptr, offsetof(kb9003_single_phy_info_t, afe_rtrim));

        // Print KB900X_RX_AFE_RATE
        print_row("KB900X_RX_AFE_RATE", ptr, offsetof(kb9003_single_phy_info_t, rx_afe_rate));

        // Print KB900X_RX_AFE_CONFIG
        print_row("KB900X_RX_AFE_CONFIG", ptr, offsetof(kb9003_single_phy_info_t, rx_afe_config));

        // Print RX_DFE_TAP
        for (uint8_t tap = 0; tap < 8; tap++) {
            char name[20];
            sprintf(name, "RX_DFE_TAP%d", tap + 1);
            print_row(name, ptr, offsetof(kb9003_single_phy_info_t, rx_dfe_taps[tap]));
        }

        // Print RX_DFE_FLOAT_TAP
        for (uint8_t tap = 0; tap < 4; tap++) {
            char name[20];
            sprintf(name, "RX_DFE_FLOAT_TAP%d", tap + 1);
            print_row(name, ptr, offsetof(kb9003_single_phy_info_t, rx_dfe_float_taps[tap]));
        }

        // Print KB900X_RX_DFE_FLOAT_TAP_POS
        print_row("KB900X_RX_DFE_FLOAT_TAP_POS", ptr,
                  offsetof(kb9003_single_phy_info_t, rx_dfe_float_tap_pos));

        // Print KB900X_RX_DPLL_FREQ
        print_row("KB900X_RX_DPLL_FREQ", ptr, offsetof(kb9003_single_phy_info_t, rx_dpll_freq));

        // Print KB900X_TX_ASIC_IN_1
        print_row("KB900X_TX_ASIC_IN_1", ptr, offsetof(kb9003_single_phy_info_t, tx_asic_in_1));

        // Print KB900X_TX_ASIC_IN_2
        print_row("KB900X_TX_ASIC_IN_2", ptr, offsetof(kb9003_single_phy_info_t, tx_asic_in_2));

        // Print KB900X_RX_STARTUP_FOM
        print_row("KB900X_RX_STARTUP_FOM", ptr, offsetof(kb9003_single_phy_info_t, rx_startup_fom));

        // Print KB900X_RX_MM_FOM
        print_row("KB900X_RX_MM_FOM", ptr, offsetof(kb9003_single_phy_info_t, rx_mm_fom));
    }

    kb900x_close(&config);

    // Generate log file
    const char filename[] = "phy_info_log.json";
    KANDOU_INFO("Logging PHY info to '%s'...", filename);
    ret = kb900x_log_phy_info(&phy_info, "phy_info_log.json");
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to log PHY info to file: %s", kb900x_strerror(ret));
    }

    return 0;
}
