/**
 * @file sw_rtssm_log.c
 * @author Kandou AI
 * @brief Example showing how to dump the SW RTSSM log (RTSSM history gathered by FW).
 *
 * This example opens a connection on /dev/i2c-1 (slave address 0x21) and
 * dumps the RTSSM log into a file.
 *
 * It is compatible with KB900x B0/B1.
 */

#include "kb900x.h"
#include "kb900x_log.h"
#include "kb900x_utils.h"

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

    // Read the RTSSM log
    kb900x_sw_rtssm_logs_t rtssm_logs;
    for (size_t i = 0; i < KB900X_SW_RTSSM_SIZE; ++i) {
        rtssm_logs.entries[i].is_valid = 0;
    }

    KANDOU_INFO("Reading SW RTSSM logs...");
    ret = kb900x_get_sw_rtssm_log(&config, &rtssm_logs);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to read SW RTSSM logs: %s", kb900x_strerror(ret));
        kb900x_close(&config);
        return -1;
    }
    KANDOU_INFO("SW RTSSM logs read successfully!");

    // Generate log file
    const char filename[] = "sw_rtssm_log.json";
    KANDOU_INFO("Saving SW RTSSM logs to '%s'...", filename);
    ret = kb900x_log_sw_rtssm(&rtssm_logs, filename);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to save SW RTSSM logs to file: %s", kb900x_strerror(ret));
    }
    else {
        KANDOU_INFO("SW RTSSM logs saved to sw_rtssm_log.json");
    }

    kb900x_close(&config);
    return ret;
}
