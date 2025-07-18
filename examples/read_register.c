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

    // Initialize library communication to match KB900x's mode
    KANDOU_INFO("Syncing with KB900x's communication mode...");
    kb900x_communication_mode_t mode;
    ret = kb900x_detect_communication_mode(&config, &mode);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to detect KB900x's communication mode: %s", kb900x_strerror(ret));
        kb900x_close(&config);
        return -1;
    }
    KANDOU_INFO("Communication mode: %d", mode);

    uint32_t result = 0;
    ret = kb900x_read_register(&config, 0xE008205C, &result);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to read register: %s", kb900x_strerror(ret));
    }
    else {
        KANDOU_INFO("SSI_VERSION_ID = 0x%08X", result);
    }

    // Close the connection with KB900x
    KANDOU_INFO("Closing device...");
    kb900x_close(&config);
    return 0;
}
