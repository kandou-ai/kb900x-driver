/**
 * @file get_info.c
 * @author Kandou AI
 * @brief Example showing how to read different info from the retimer.
 *
 * This example opens a connection on /dev/i2c-1 (slave address 0x21) if no arg are provided.
 * The slave address and i2c_id can be provided as arguments with --slave-address and --i2c-id.
 *
 * It reads and displays link health status information.
 *
 * It is compatible with KB900x B0/B1.
 */

#include "kb900x.h"
#include "kb900x_i2c_comm.h"
#include "kb900x_i2c_master.h"
#include "kb900x_smbus_comm.h"
#include <stdlib.h>
#include <time.h>

#define I2C_SLAVE_ADDR (0x21)
#define I2C_ID (1)

int main(void)
{
    // Enable logging
    kandou_log_set_quiet(0);
    kandou_log_set_level(KB900X_LOG_DEBUG);

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

    // Switch to SMBUS mode (required for various debug info)
    KANDOU_INFO("Switching to SMBUS mode...");
    ret = kb900x_switch_communication_mode(&config, KB900X_COMM_SMBUS);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to switch to SMBUS mode.");
        kb900x_close(&config);
        return -1;
    }

    kb900x_eqz_sync_hdr_err_info_t info[KB900X_MAX_NUM_LINKS];

    uint32_t num_links = 0;
    ret = kb900x_get_link_health_status(&config, info, &num_links);

    if (ret != KB900X_E_OK) {
        printf("Failed to read EQZ info\n");
        return -1;
    }

    printf("Number of Links = %u\n\n", num_links);

    for (uint32_t i = 0; i < num_links; i++) {
        printf("================================\n");
        printf("Link %u\n", i);
        printf("================================\n");

        printf("PRE EQ\n");
        printf("UPP Error Counter : %u\n", info[i].pre_upp_error_counter);

        printf("UPP Total Counter : %u\n", info[i].pre_upp_total_counter);

        printf("DPP Error Counter : %u\n", info[i].pre_dpp_error_counter);

        printf("DPP Total Counter : %u\n", info[i].pre_dpp_total_counter);

        printf("\nPOST EQ\n");

        printf("UPP Error Counter : %u\n", info[i].post_upp_error_counter);

        printf("UPP Total Counter : %u\n", info[i].post_upp_total_counter);

        printf("DPP Error Counter : %u\n", info[i].post_dpp_error_counter);

        printf("DPP Total Counter : %u\n", info[i].post_dpp_total_counter);

        printf("\n");
    }

    kb900x_close(&config);
    return 0;
}
