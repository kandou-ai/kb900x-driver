/**
 * @file hw_rtssm_log.c
 * @author Kandou AI
 * @brief Example showing how to read the HW RTSSM log
 *
 * This example opens a connection on /dev/i2c-1 (slave address 0x21) and
 * reads the HW RTSSM log on all the loggers (8 loggers).
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

    // Switch to SMBUS mode (required to various debug info)
    KANDOU_INFO("Switching to SMBUS mode...");
    ret = kb900x_switch_communication_mode(&config, KB900X_COMM_SMBUS);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to switch to SMBUS mode.");
        kb900x_close(&config);
        return -1;
    }

    KANDOU_INFO("Reading HW RTSSM log...");
    kb900x_hw_rtssm_logs_t log;
    ret = kb900x_get_hw_rtssm_log(&config, &log);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to read HW RTSSM log: %s", kb900x_strerror(ret));
        kb900x_close(&config);
        return -1;
    }

    for (uint32_t i = 0; i < 8; i++) {
        printf("HW RTSSM LOGGER%d\n", i);

        printf("log map info: tileID=%d, RPCS ID=%d, CfgID=%d, curr pos=%d\n",
               log.logs[i].log_map_info.tile_id, log.logs[i].log_map_info.rpcs_id,
               log.logs[i].log_map_info.rtssm_cfg_id, log.logs[i].log_map_info.curr_pos);

        for (uint32_t j = 0; j < 32; j++) {
            printf("RTSSM logger%d entry%d: RTSSM=%s, data_rate=%s, delta=%uus\n", i, j,
                   KB900X_STATE_TO_STRING(log.logs[i].entries[j].rtssm),
                   KB900X_GEN_TO_STRING(log.logs[i].entries[j].data_rate),
                   kb900x_delta_to_us(log.logs[i].entries[j].delta));
        }
    }

    // Generate log file
    const char filename[] = "hw_rtssm_log.json";
    KANDOU_INFO("Logging HW RTSSM to '%s'...", filename);
    ret = kb900x_log_hw_rtssm(&log, "hw_rtssm_log.json");
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to log HW RTSSM to file: %s", kb900x_strerror(ret));
    }

    kb900x_close(&config);
    return 0;
}
