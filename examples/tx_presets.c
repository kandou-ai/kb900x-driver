/**
 * @file tx_presets.c
 * @author Kandou AI
 * @brief Example showing how to read the TX presets
 *
 * This example opens a connection on /dev/i2c-1 (slave address 0x21) and
 * reads the TX presets.
 *
 * It is compatible with KB900x B0/B1.
 */

#include "kb900x.h"
#include <stdlib.h>
#include <time.h>

#define I2C_SLAVE_ADDR (0x21)
#define I2C_ID (1)

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

    // Switch to SMBUS mode (required to read the FOM)
    KANDOU_INFO("Switching to SMBUS mode...");
    ret = kb900x_switch_communication_mode(&config, KB900X_COMM_SMBUS);
    if (ret < 0) {
        KANDOU_ERR("Failed to switch to SMBUS mode.");
        kb900x_close(&config);
        return -1;
    }

    KANDOU_INFO("Reading TX presets...");
    kb900x_all_presets_t presets;
    ret = kb900x_get_tx_presets(&config, &presets);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to read TX presets: %s", kb900x_strerror(ret));
        kb900x_close(&config);
        return -1;
    }

    KANDOU_INFO("TX presets read successfully!");
    for (uint32_t i = 0; i < 16; i++) {
        printf("config.lane_id = %d\n", presets.lanes[i].config.lane_id);
        printf("config.data_rate = %d\n", presets.lanes[i].config.data_rate);
        printf("buffer.data.ep_rt = %d\n", presets.lanes[i].data.ep_rt);
        printf("buffer.data.rc_rt = %d\n", presets.lanes[i].data.rc_rt);
        printf("buffer.data.rt_ep = %d\n", presets.lanes[i].data.rt_ep);
        printf("buffer.data.rt_rc = %d\n", presets.lanes[i].data.rt_rc);
    }

    kb900x_close(&config);

    // Generate log file
    const char filename[] = "presets_log.json";
    KANDOU_INFO("Logging TX presets to '%s'...", filename);
    ret = kb900x_log_tx_presets(&presets, filename);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to log TX presets to file: %s", kb900x_strerror(ret));
    }

    return 0;
}
