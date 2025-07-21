/*
 * Copyright (c) Kandou-AI.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <CLI11.hpp>
#include <cstdio>
#include <iostream>
#include <thread>

extern "C" {
#ifdef OPENBMC_BUILD
#include <openbmc/kb900x.h>
#include <openbmc/kb900x_log.h>
#else
#include <kb900x.h>
#include <kb900x_log.h>
#endif
}

/**
 * \brief Pretty-print a revid.
 *
 * \param[in] revid The revid to print.
 */
void _print_revid(uint32_t revid)
{
    switch (revid) {
        case 0x00:
            std::cout << "REVID: A0" << std::endl;
            break;
        case 0x10:
            std::cout << "REVID: B0" << std::endl;
            break;
        case 0x11:
            std::cout << "REVID: B1" << std::endl;
            break;
        default:
            std::cout << "UNKNOWN REVID: 0x" << std::hex << std::setw(8) << std::setfill('0')
                      << revid << std::endl;
    }
}

/**
 * \brief Pretty-print a firmware version.
 *
 * \param[in] v The firmware version to print.
 */
void _print_fw_version(const kb900x_fw_version_t &v)
{
    // GCC 12 does not support std::format :(
    printf("Firmware Version: %d.%02d.%02d.%d\n", v.major, v.minor, v.patch, v.suffix);
}

/**
 * \brief Pretty-print the firmware health.
 *
 * \param[in] h The firmware health to print.
 */
void _print_fw_health(const kb900x_fw_health_t &h)
{
    // GCC 12 does not support std::format :(
    printf("Firmware Health:\n- is_initialized: %d\n- liveliness: %d\n", h.fw_is_initialized,
           h.liveliness);
}

/**
 * \brief Pretty-print a link status.
 */
void _print_link_status(const kb900x_link_status_t &status, int link_id)
{
    printf("Link #%d Status:\n", link_id);
    printf("- link_invalid: %d\n", status.link_invalid);
    printf("- l0_reached: %d\n", status.l0_reached);
    printf("- link_number: %d\n", status.link_number);
    printf("- rtssm_state: %s\n", KB900X_STATE_TO_STRING(status.rtssm_state));
    printf("- rtssm_speed: %s\n", KB900X_GEN_TO_STRING(status.rtssm_speed));
    printf("- link_width: %d\n", status.link_width);
}

/**
 * \brief Open and configure SMBUS comms, then execute the given function with the given args, close
 * the comms and throw a CLI status for CLI11.
 *
 * \param[in, out] config A mutable reference to the config to use for communications, sets
 * `config.handle`.
 * \param[in] mode The communication mode to use.
 * \param[in] function The function to execute.
 * \param[in] args The arguments to pass to the function to execute.
 */
template <typename... Args>
void _with_comms(kb900x_config_t &config, kb900x_communication_mode_t mode,
                 std::function<bool(const kb900x_config_t &, Args...)> function, Args... args)
{
    if (mode == KB900X_COMM_BIC) {
        kb900x_set_communication_mode(&config, KB900X_COMM_BIC);
    }
    else {
        int ret = kb900x_open(&config);
        if (ret != KB900X_E_OK) {
            std::cout << "Failed to open KB900x comms: " << kb900x_strerror(ret) << std::endl;
            throw CLI::RuntimeError();
        }

        // Detect communication mode
        kb900x_communication_mode_t detected_mode;
        ret = kb900x_detect_communication_mode(&config, &detected_mode);
        // CHECK_SUCCESS_MSG(ret, "Failed to detect communication mode: %s", kb900x_strerror(ret));

        if (detected_mode != mode) {
            KANDOU_INFO("Switching mode to %d...", mode);
            ret = kb900x_switch_communication_mode(&config, mode);
            // CHECK_SUCCESS_MSG(ret, "Failed to switch communication mode: %s",
            // kb900x_strerror(ret));
            KANDOU_INFO("Mode switched!");
        }
    }

    bool success = function(config, args...);

    if (mode == KB900X_COMM_BIC) {
        kb900x_close(&config);
    }

    if (success) {
        throw CLI::Success();
    }
    else {
        throw CLI::RuntimeError();
    }
}

/**
 * \brief Enable logging and set level.
 *
 * \param[in] count The number of times the user has passed the -v flag.
 */
void set_logging_level(size_t count)
{
    // NOTE: this function is only called with count > 0
    kandou_log_set_quiet(0);

    int level = KB900X_LOG_LEVEL_LENGTH - count;
    kandou_log_set_level(level);
}

/**
 * \brief Read and print the REVID.
 *
 * \param[in] config The I2C comms config.
 *
 * \return true if success, false if failure
 */
bool revid(const kb900x_config_t &config)
{
    uint32_t revid;
    int ret = kb900x_get_revid(&config, &revid);
    if (ret != KB900X_E_OK) {
        std::cout << "Failed to get REVID: " << kb900x_strerror(ret) << std::endl;
        return false;
    }
    else {
        _print_revid(revid);
        return true;
    }
}

/**
 * \brief Read and print the firmware version.
 *
 * \param[in] config The I2C comms config.
 *
 * \return true if success, false if failure
 */
bool fw_version(const kb900x_config_t &config)
{
    kb900x_fw_version_t fw_version;
    int ret = kb900x_get_firmware_version(&config, &fw_version);
    if (ret != KB900X_E_OK) {
        std::cout << "Failed to read firmware version: " << kb900x_strerror(ret) << std::endl;
        return false;
    }
    else {
        _print_fw_version(fw_version);
        return true;
    }
}

/**
 * \brief Read and print the firmware health information.
 *
 * \param[in] config The I2C comms config.
 *
 * \return true if success, false if failure
 */
bool fw_health(const kb900x_config_t &config)
{
    kb900x_fw_health_t fw_health;
    int ret = kb900x_get_firmware_health(&config, &fw_health);
    if (ret != KB900X_E_OK) {
        std::cout << "Failed to read firmware health info: " << kb900x_strerror(ret) << std::endl;
        return false;
    }
    else {
        _print_fw_health(fw_health);
        return true;
    }
}

/**
 * \brief Read and print/save the FOM data.
 *
 * \param[in] config The I2C comms config.
 * \param[in] filename If not empty, the name of the file where to save the data.
 *
 * \return true if success, false if failure
 */
bool fom(const kb900x_config_t &config, const std::string filename)
{
    kb900x_fom_t fom;
    int ret = kb900x_get_fom(&config, &fom);
    if (ret != KB900X_E_OK) {
        std::cout << "Failed to read FOM data: " << kb900x_strerror(ret) << std::endl;
        return false;
    }
    else if (filename.length() == 0) {
        // TODO: print FOM
        std::cout << "FOM read successfully, TODO: print FOM data" << std::endl;
        return false;
    }
    else {
        ret = kb900x_log_fom(&fom, filename.c_str());
        if (ret != KB900X_E_OK) {
            std::cout << "Failed to save FOM data to file: " << kb900x_strerror(ret) << std::endl;
            return false;
        }
        else {
            printf("FOM data written to '%s'.\n", filename.c_str());
            return true;
        }
    }
}

/**
 * \brief Read and print/save the HW RTSSM data.
 *
 * \param[in] config The I2C comms config.
 * \param[in] filename If not empty, the name of the file where to save the data.
 *
 * \return true if success, false if failure
 */
bool hw_rtssm(const kb900x_config_t &config, const std::string filename)
{
    kb900x_hw_rtssm_logs_t log;
    int ret = kb900x_get_hw_rtssm_log(&config, &log);
    if (ret != KB900X_E_OK) {
        std::cout << "Failed to read HW RTSSM data: " << kb900x_strerror(ret) << std::endl;
        return false;
    }
    else if (filename.length() == 0) {
        // TODO: print FOM
        std::cout << "HW RTSSM read successfully, TODO: print HW RTSSM data" << std::endl;
        return false;
    }
    else {
        ret = kb900x_log_hw_rtssm(&log, filename.c_str());
        if (ret != KB900X_E_OK) {
            std::cout << "Failed to save HW RTSSM data to file: " << kb900x_strerror(ret)
                      << std::endl;
            return false;
        }
        else {
            printf("HW RTSSM data written to '%s'.\n", filename.c_str());
            return true;
        }
    }
}

/**
 * \brief Read and print/save the SW RTSSM data.
 *
 * \param[in] config The I2C comms config.
 * \param[in] filename If not empty, the name of the file where to save the data.
 *
 * \return true if success, false if failure
 */
bool sw_rtssm(const kb900x_config_t &config, const std::string filename)
{
    kb900x_sw_rtssm_logs_t log;
    int ret = kb900x_get_sw_rtssm_log(&config, &log);
    if (ret != KB900X_E_OK) {
        std::cout << "Failed to read SW RTSSM data: " << kb900x_strerror(ret) << std::endl;
        return false;
    }
    else if (filename.length() == 0) {
        // TODO: print FOM
        std::cout << "SW RTSSM read successfully, TODO: print SW RTSSM data" << std::endl;
        return false;
    }
    else {
        ret = kb900x_log_sw_rtssm(&log, filename.c_str());
        if (ret != KB900X_E_OK) {
            std::cout << "Failed to save SW RTSSM data to file: " << kb900x_strerror(ret)
                      << std::endl;
            return false;
        }
        else {
            printf("SW RTSSM data written to '%s'.\n", filename.c_str());
            return true;
        }
    }
}

/**
 * \brief Read and print/save the TX presets data.
 *
 * \param[in] config The I2C comms config.
 * \param[in] filename If not empty, the name of the file where to save the data.
 *
 * \return true if success, false if failure
 */
bool tx_presets(const kb900x_config_t &config, const std::string filename)
{
    kb900x_all_presets_t presets;
    int ret = kb900x_get_tx_presets(&config, &presets);
    if (ret != KB900X_E_OK) {
        std::cout << "Failed to read TX preset data: " << kb900x_strerror(ret) << std::endl;
        return false;
    }
    else if (filename.length() == 0) {
        // TODO: print FOM
        std::cout << "TX preset data read successfully, TODO: print TX preset data" << std::endl;
        return false;
    }
    else {
        ret = kb900x_log_tx_presets(&presets, filename.c_str());
        if (ret != KB900X_E_OK) {
            std::cout << "Failed to save TX preset data to file: " << kb900x_strerror(ret)
                      << std::endl;
            return false;
        }
        else {
            printf("TX preset data written to '%s'.\n", filename.c_str());
            return true;
        }
    }
}

/**
 * \brief Read and print/save the PHY info.
 *
 * \param[in] config The I2C comms config.
 * \param[in] filename If not empty, the name of the file where to save the data.
 *
 * \return true if success, false if failure
 */
bool phy_info(const kb900x_config_t &config, const std::string filename)
{
    kb9003_phy_info_t phy_info;
    int ret = kb900x_get_phy_info(&config, &phy_info);
    if (ret != KB900X_E_OK) {
        std::cout << "Failed to read PHY info: " << kb900x_strerror(ret) << std::endl;
        return false;
    }
    else if (filename.length() == 0) {
        // TODO: print FOM
        std::cout << "PHY info read successfully, TODO: print PHY info" << std::endl;
        return false;
    }
    else {
        ret = kb900x_log_phy_info(&phy_info, filename.c_str());
        if (ret != KB900X_E_OK) {
            std::cout << "Failed to save PHY info to file: " << kb900x_strerror(ret) << std::endl;
            return false;
        }
        else {
            printf("PHY info written to '%s'.\n", filename.c_str());
            return true;
        }
    }
}

/**
 * \brief Print/save a log file containing debug information.
 *
 * \param[in] config The I2C comms config.
 * \param[in] filename The name of the file where to save the data.
 *
 * \return true if success, false if failure
 */
bool error_dump(const kb900x_config_t &config, const std::string filename)
{
    int ret = kb900x_error_dump(&config, filename.c_str());
    if (ret != KB900X_E_OK) {
        std::cout << "Failed to save error dump to file: " << kb900x_strerror(ret) << std::endl;
        return false;
    }
    else {
        std::cout << "Error dump saved to " << filename << std::endl;
        return true;
    }
}

/**
 * \brief Print the global temperature.
 *
 * \param[in] config The I2C comms config.
 *
 * \return true if success, false if failure
 */
bool temperature(const kb900x_config_t &config)
{
    float temperature = 0.0;
    int ret = kb900x_get_temperature(&config, &temperature);
    if (ret != KB900X_E_OK) {
        std::cout << "Failed to read temperature: " << kb900x_strerror(ret) << std::endl;
        return false;
    }

    printf("KB900x temperature: %.2f C\n", temperature);
    return true;
}

/**
 * \brief Print the status of a given link.
 *
 * \param[in] config The I2C comms config.
 * \param[in] link_id The ID of the link whose status to read.
 *
 * \return true if success, false if failure
 */
bool link_status(const kb900x_config_t &config, int link_id)
{
    kb900x_link_status_t status;
    int ret = kb900x_get_link_status(&config, link_id, &status);
    if (ret != KB900X_E_OK) {
        std::cout << "Failed to read link status: " << kb900x_strerror(ret) << std::endl;
        return false;
    }

    _print_link_status(status, link_id);
    return true;
}

/**
 * \brief Write a given FW image to the EEPROM.
 *
 * \param[in] config The I2C comms config.
 * \param[in] eeprom_i2c_addr The I2C address of the EEPROM where to write the FW.
 * \param[in] filename The name of the FW image file.
 *
 * \return true if success, false if failure
 */
bool write_fw(const kb900x_config_t &config, uint8_t eeprom_i2c_addr, std::string filename)
{
    const kb900x_eeprom_config_t eeprom_config = {
        .slave_addr = eeprom_i2c_addr,
        .eeprom_size = 1 << 18,
        .page_size = 256,
        .write_cycle_time_ms = 10,
    };

    // Allocate buffer and fill it with file contents
    uint8_t *buffer = NULL;
    size_t buffer_size = 0;
    std::cout << "Reading file contents..." << std::endl;
    int ret = read_file(filename.c_str(), &buffer, &buffer_size);
    if (ret != KB900X_E_OK) {
        // NOTE: it seems that if read_file fails, we should not free buffer
        std::cout << "Failed to read file contents: " << kb900x_strerror(ret) << std::endl;
        return false;
    }

    // Write the firmware to the EEPROM
    std::cout << "Writing FW image to EEPROM..." << std::endl;
    ret = kb900x_flash_firmware(&config, buffer, buffer_size, &eeprom_config);
    if (ret != KB900X_E_OK) {
        std::cout << "Failed to write firmware to EEPROM: " << kb900x_strerror(ret) << std::endl;
        free(buffer);
        return false;
    }

    std::cout << "Firmware image successfully written to EEPROM." << std::endl;
    free(buffer);
    return true;
}

/**
 * \brief Read a KB900x register.ADJ_OFFSET_SINGLESHOT
 *
 * \param[in] config The I2C comms config.
 * \param[in] reg_addr The register address.
 *
 * \return true if success, false if failure
 */
bool rd_reg(const kb900x_config_t &config, uint32_t reg_addr)
{
    uint32_t result;
    int ret = kb900x_read_register(&config, reg_addr, &result);
    if (ret != KB900X_E_OK) {
        std::cout << "Failed to read register: " << kb900x_strerror(ret) << std::endl;
        return false;
    }

    printf("Value: 0x%08X\n", result);
    return true;
}

/**
 * \brief Write a KB900x register.ADJ_OFFSET_SINGLESHOT
 *
 * \param[in] config The I2C comms config.
 * \param[in] reg_addr The register address.
 * \param[in] reg_val The value to write.
 *
 * \return true if success, false if failure
 */
bool wr_reg(const kb900x_config_t &config, uint32_t reg_addr, uint32_t reg_val)
{
    int ret = kb900x_write_register(&config, reg_addr, reg_val);
    if (ret != KB900X_E_OK) {
        std::cout << "Failed to write register: " << kb900x_strerror(ret) << std::endl;
        return false;
    }

    printf("Register write done.\n");
    return true;
}

int main(int argc, char **argv)
{
    CLI::App app{"Kandou AI OpenBMC Internal CLI"};
    app.require_subcommand(1);

    // Default values are here
    // TODO: match with Wiwynn's setup
    kb900x_config_t config = {
        .intf = 0x05,
        .slot_id = 0x01,
        .bus_id = 3,
        .retimer_addr = 0x23,
        .handle = 0,
    };

    // ### Common arguments for all subcommands ###
    app.add_option("-b", config.bus_id, "The ID of the I2C bus that is connected to the KB900x.")
        ->capture_default_str(); // Show default value in help message
    app.add_option("-r", config.retimer_addr, "The I2C address of the KB900x.")
        ->capture_default_str(); // Show default value in help message
    // Enable verbose logging
    app.add_flag("-v", set_logging_level,
                 "Enable verbose logging. Add more v's to increase the log level.");

    // Default mode is SMBUS, if BIC support is compiled, the user may change this to BIC with the
    // --bic flag
    kb900x_communication_mode_t mode = KB900X_COMM_SMBUS;

    // ### BIC-mode-specific arguments ###
    CLI::Option *bic_flag =
        app.add_flag("--bic", [&](uint64_t) { mode = KB900X_COMM_BIC; }, "Use BIC/IPMB mode.");
    app.add_option("-i", config.intf, "Set config.intf.")
        ->check(CLI::Range(0, UINT8_MAX))
        ->default_str(std::to_string(config.intf)) // capture_default_str does not work on uint8_t
        ->needs(bic_flag);
    app.add_option("-s", config.slot_id, "Set config.slot_id.")
        ->check(CLI::Range(0, UINT8_MAX))
        ->default_str(
            std::to_string(config.slot_id)) // capture_default_str does not work on uint8_t
        ->needs(bic_flag);

    // ### Subcommands ###
    // NOTE: subcommand callbacks all capture option variables by reference, to ensure the values of
    // these variables are read *after* CLI11 has parsed the program arguments and set the option
    // variables. We then call the appropriate function via a wrapper that sets up the SMBUS
    // communications first.

    CLI::App *revid_app = app.add_subcommand("revid", "Show the KB900x revision ID.");
    revid_app->callback([&]() { _with_comms(config, mode, std::function(revid)); });

    CLI::App *fw_version_app =
        app.add_subcommand("fw-version", "Show the KB900x firmware version.");
    fw_version_app->callback([&]() { _with_comms(config, mode, std::function(fw_version)); });

    CLI::App *fw_health_app =
        app.add_subcommand("fw-health", "Show the KB900x firmware health information.");
    fw_health_app->callback([&]() { _with_comms(config, mode, std::function(fw_health)); });

    // NOTE: single filename variable for all subcommands. OK because we require exactly one
    // subcommand, making all subcommands mutually exclusive
    std::string filename = "";

    CLI::App *fom_app =
        app.add_subcommand("fom", "Read and print/save the figure-of-merit (FOM) data.");
    fom_app->add_option("-f", filename,
                        "If provided, the data will be saved to this file instead of printing "
                        "it to the console.");
    fom_app->callback([&]() { _with_comms(config, mode, std::function(fom), filename); });

    CLI::App *hw_rtssm_app =
        app.add_subcommand("hw-rtssm", "Read and print/save the HW RTSSM log.");
    hw_rtssm_app->add_option(
        "-f", filename,
        "If provided, the data will be saved to this file instead of printing it to the console.");
    hw_rtssm_app->callback([&]() { _with_comms(config, mode, std::function(hw_rtssm), filename); });

    CLI::App *sw_rtssm_app =
        app.add_subcommand("sw-rtssm", "Read and print/save the SW RTSSM log.");
    sw_rtssm_app->add_option(
        "-f", filename,
        "If provided, the data will be saved to this file instead of printing it to the console.");
    sw_rtssm_app->callback([&]() { _with_comms(config, mode, std::function(sw_rtssm), filename); });

    CLI::App *tx_presets_app =
        app.add_subcommand("tx-presets", "Read and print/save the TX presets.");
    tx_presets_app->add_option(
        "-f", filename,
        "If provided, the data will be saved to this file instead of printing it to the console.");
    tx_presets_app->callback(
        [&]() { _with_comms(config, mode, std::function(tx_presets), filename); });

    CLI::App *phy_info_app = app.add_subcommand("phy-info", "Read and print/save the PHY info.");
    phy_info_app->add_option(
        "-f", filename,
        "If provided, the data will be saved to this file instead of printing it to the console.");
    phy_info_app->callback([&]() { _with_comms(config, mode, std::function(phy_info), filename); });

    CLI::App *error_dump_app =
        app.add_subcommand("error-dump", "Generate a log file containing debug info.");
    error_dump_app->add_option("-f", filename, "The path to a file to store the debug info.")
        ->required();
    error_dump_app->callback(
        [&]() { _with_comms(config, mode, std::function(error_dump), filename); });

    CLI::App *temperature_app = app.add_subcommand("temperature", "Get the KB900x temperature.");
    temperature_app->callback([&]() { _with_comms(config, mode, std::function(temperature)); });

    CLI::App *link_status_app = app.add_subcommand("link-status", "Read the status of a link.");
    int link_id;
    link_status_app->add_option("-l", link_id, "The ID of the link whose status to read.")
        ->required();
    link_status_app->callback(
        [&]() { _with_comms(config, mode, std::function(link_status), link_id); });

    CLI::App *write_fw_app =
        app.add_subcommand("write-fw", "Write a firmware image to the EEPROM.");
    uint8_t eeprom_i2c_addr = 0x50;
    write_fw_app->add_option("-f", filename, "The FW image file to write.")
        ->check(CLI::ExistingFile)
        ->required();
    write_fw_app->add_option("-a", eeprom_i2c_addr, "The I2C slave address of the EEPROM.")
        ->capture_default_str();
    write_fw_app->callback(
        [&]() { _with_comms(config, mode, std::function(write_fw), eeprom_i2c_addr, filename); });

    CLI::App *rd_reg_app = app.add_subcommand("rdr", "Read a KB900x register value.");
    uint32_t reg_addr = 0;
    rd_reg_app->add_option("reg_addr", reg_addr, "The address of the register")->required();
    rd_reg_app->callback([&]() { _with_comms(config, mode, std::function(rd_reg), reg_addr); });

    CLI::App *wr_reg_app = app.add_subcommand("wrr", "Write a KB900x register value.");
    uint32_t reg_val = 0;
    wr_reg_app->add_option("reg_addr", reg_addr, "The address of the register")->required();
    wr_reg_app->add_option("reg_val", reg_val, "The value to write")->required();
    wr_reg_app->callback(
        [&]() { _with_comms(config, mode, std::function(wr_reg), reg_addr, reg_val); });

    // CLI11 parses the subcommand and options from the program args, then calls the callback that
    // is associated with the selected subcommand
    CLI11_PARSE(app, argc, argv);

    return 0;
}
