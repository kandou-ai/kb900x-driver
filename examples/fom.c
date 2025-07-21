/**
 * @file fom.c
 * @author Kandou AI
 * @brief Example showing how to read the FOM
 *
 * This example opens a connection on /dev/i2c-1 (slave address 0x21) and
 * reads the FOM (mission mode and startup) on all lanes.
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

    KANDOU_INFO("Reading FoM...");
    kb900x_fom_t fom;
    ret = kb900x_get_fom(&config, &fom);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to read the FoM: %s", kb900x_strerror(ret));
        kb900x_close(&config);
        return -1;
    }

    KANDOU_INFO("FoM read successfully:");
    printf("Side: A RX | B TX\n");
    for (uint8_t lane = 0; lane < 16; lane++) {
        printf("Startup FOM = %d, MM FOM = %d\n", fom.startup_a_rx[lane], fom.mm_a_rx[lane]);
    }

    printf("Side: A TX | B RX\n");
    for (uint8_t lane = 0; lane < 16; lane++) {
        printf("Startup FOM = %d, MM FOM = %d\n", fom.startup_b_rx[lane], fom.mm_b_rx[lane]);
    }

    // Generate log file
    const char filename[] = "fom_log.json";
    KANDOU_INFO("Logging FoM to '%s'...", filename);
    ret = kb900x_log_fom(&fom, filename);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to log FoM to file: %s", kb900x_strerror(ret));
    }

    return 0;
}
