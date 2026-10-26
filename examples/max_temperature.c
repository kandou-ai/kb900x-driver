/**
 * @file get_info.c
 * @author Kandou AI
 * @brief Example showing how to read different info from the retimer.
 *
 * This example opens a connection on /dev/i2c-1 (slave address 0x21) if no arg are provided.
 * The slave address and i2c_id can be provided as arguments with --slave-address and --i2c-id.
 *
 * It reads and displays the max temperature.
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

    float temperature;

    // Get the maximum temperature of the package
    ret = kb900x_get_max_temperature(&config, &temperature);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to read temperature: %s", kb900x_strerror(ret));
        kb900x_close(&config);
        return -1;
    }
    KANDOU_INFO("Maximum temperature = %.2f", temperature);

    // Close the connection with KB900x
    KANDOU_INFO("Closing device...");
    kb900x_close(&config);
    return 0;
}
