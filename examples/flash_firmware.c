/**
 * @file flash_firmware.c
 * @author Kandou AI
 * @brief Example showing how to upgrade the KB900x firmware
 *
 * This example opens a connection on /dev/i2c-1 (slave address 0x21) and
 * upgrades the KB900x firmware.
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

    // Detect communication mode
    kb900x_communication_mode_t mode;
    ret = kb900x_detect_communication_mode(&config, &mode);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to detect communication mode: %s", kb900x_strerror(ret));
        kb900x_close(&config);
        return -1;
    }
    KANDOU_INFO("Communication mode = %s", mode == KB900X_COMM_SMBUS ? "SMBus" : "I2C");

    // Flash firmware TWI
    KANDOU_INFO("Reading file...");
    const char *filename = "firmware.bin";
    uint8_t *buffer = NULL;
    size_t buffer_size;
    ret = read_file(filename, &buffer, &buffer_size);
    if (ret < 0) {
        KANDOU_FATAL("Failed to read file: %s", filename);
        kb900x_close(&config);
        return ret;
    }

    // Config of the EEPROM storing the firmware image
    const kb900x_eeprom_config_t eeprom_config = {.slave_addr = 0x50,
                                                  .eeprom_size = 1 << 18, // 2Mbits = 262144 bytes
                                                  .page_size = 256,
                                                  .write_cycle_time_ms = 10};

    // Buffer of size buffer_size populated with 0
    KANDOU_INFO("Flashing firmware %s...", filename);
    ret = kb900x_flash_firmware(&config, buffer, buffer_size, &eeprom_config);

    // Calculate the time in seconds
    if (ret < 0) {
        KANDOU_FATAL("Error while flashing firmware");
        free(buffer);
        kb900x_close(&config);
        return -1;
    }
    KANDOU_INFO("Flashing Done");
    free(buffer);
    kb900x_close(&config);
    return 0;
}
