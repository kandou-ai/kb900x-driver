/**
 * @file retimer_reset.c
 * @author Kandou AI
 * @brief Example showing how to reset the retimer
 *
 * This example opens a connection on /dev/i2c-1 (slave address 0x21) and
 * resets the KB900x retimer.
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

    // Send reset
    KANDOU_INFO("Resetting KB900x...");
    ret = kb900x_reset(&config);
    sleep(2);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to reset KB900x: %s", kb900x_strerror(ret));
    }
    else {
        KANDOU_INFO("Reset Done");
    }

    // Close the connection with KB900x
    KANDOU_INFO("Closing device...");
    kb900x_close(&config);
    return 0;
}
