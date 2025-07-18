/**
 * @file vendor_id.c
 * @author Kandou AI
 * @brief Example showing how to read the vendor ID
 *
 * This example opens a connection on /dev/i2c-1 (slave address 0x21) and
 * reads the vendor ID.
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

    uint32_t vendor_id;
    ret = kb900x_get_vendor_id(&config, &vendor_id);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to read vendor ID: %s", kb900x_strerror(ret));
    }
    else {
        KANDOU_INFO("Vendor ID: 0x%04X", vendor_id);
    }

    kb900x_close(&config);
    return 0;
}
