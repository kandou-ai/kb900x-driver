/**
 * @file get_info.c
 * @author Kandou AI
 * @brief Example showing how to read different info from the retimer.
 *
 * This example opens a connection on /dev/i2c-1 (slave address 0x21) if no arg are provided.
 * The slave address and i2c_id can be provided as arguments with --slave-address and --i2c-id.
 *
 * It reads and displays different info such as FW version, vendor ID, temperatures etc...
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

    KANDOU_INFO("=== Reading Debug Info...");

    // Lane status
    kb900x_link_status_t status;
    const uint8_t link_id = 0;
    ret = kb900x_get_link_status(&config, link_id, &status);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to read link status: %s", kb900x_strerror(ret));
        kb900x_close(&config);
        return -1;
    }

    KANDOU_INFO("Link %d status : ", link_id);
    KANDOU_INFO("\tLink invalid: %u", status.link_invalid);
    KANDOU_INFO("\tL0 reached: %u", status.l0_reached);
    KANDOU_INFO("\tLink number: %u", status.link_number);
    KANDOU_INFO("\tRTSSM state: %u", status.rtssm_state);
    KANDOU_INFO("\tRTSSM speed: %u", status.rtssm_speed);
    KANDOU_INFO("\tLink width: %u", status.link_width);

    uint32_t vendor_id;
    float temperature;
    kb900x_fw_version_t version = {{0x00, 0x00, 0x00, 0x00}};

    // Get the vendor id
    ret = kb900x_get_vendor_id(&config, &vendor_id);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to read vendor IR: %s", kb900x_strerror(ret));
        kb900x_close(&config);
        return -1;
    }
    KANDOU_INFO("Vendor ID = 0x%X", vendor_id);

    // Get the firmware version
    ret = kb900x_get_firmware_version(&config, &version);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to read FW version: %s", kb900x_strerror(ret));
        kb900x_close(&config);
        return -1;
    }
    KANDOU_INFO("Firmware version = %d.%d.%d.%c", version.major, version.minor, version.patch,
                version.suffix);

    // Get the temperature for the Port A, Lane 0
    ret = kb900x_get_lane_temperature(&config, 0, 0, &temperature);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to read temperature: %s", kb900x_strerror(ret));
        kb900x_close(&config);
        return -1;
    }
    KANDOU_INFO("Temperature Port A, Lane 0 = %02f", temperature);

    // Get the temperature for the Port B, Lane 3
    ret = kb900x_get_lane_temperature(&config, 1, 3, &temperature);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to read temperature: %s", kb900x_strerror(ret));
        kb900x_close(&config);
        return -1;
    }
    KANDOU_INFO("Temperature Port B, Lane 3 = %02f", temperature);

    // Get the maximum temperature of the package
    ret = kb900x_get_temperature(&config, &temperature);
    if (ret != KB900X_E_OK) {
        KANDOU_ERR("Failed to read temperature: %s", kb900x_strerror(ret));
        kb900x_close(&config);
        return -1;
    }
    KANDOU_INFO("Maximum package temperature = %02f", temperature);

    // Close the connection with KB900x
    KANDOU_INFO("Closing device...");
    kb900x_close(&config);
    return 0;
}
