KB900x SMBus Library Changelog
==============================

v0.5.1
------

**Fixes**

- Fix EEPROM read/write functions to use uint32_t for address
- Fix logging format when parsing region table
- Remove duplicate read of the region table of the EEPROM
- Fix firmware trace size
- Correct interpretation of kb900x_get_vendor_id return code

v0.5.0
------

**New Features / Improvements**

- Public Release of the OpenBMC driver on GitHub

v0.4.3
------

**New Features / Improvements**

- Implement kb900x_get_rpcs_dbg_counter function to retrieve RPCS debug data
- Implement kb900x_log_rpcs_dbg_counter function to log RPCS debug data to a JSON file
- Implement kb900x_get_phy_rpcs_registers function to retrieve PHY RPCS registers
- Implement kb900x_log_phy_rpcs_registers function to log PHY RPCS registers to a CSV file
- Implement kb900x_get_firmware_trace function to retrieve firmware trace
- Implement kb900x_log_firmware_trace function to log firmware trace to a file
- Implement kb900x_log_registers_record function to log register records to a JSON file
- Extend kb900x_error_dump function to include firmware trace under the form of a register dump
- Implement PEC to trigger retry in BIC communication mode
- Implement returned address check to trigger retry in BIC and SMBus communication modes

v0.4.2
------

**Fixes**

- Fix potential segmentation fault in kb900x_error_dump and log functions
- Fix better messages in kb900x_error_dump

v0.4.1
------

**Fixes**

- BIC 3OU is now represented by 0x25 instead of 0x30
- Select BIC communication mode by default if build with -Dbic_communication=true
- Always print a status message in kb900x_error_dump
- Log enabled by default with KB900X_LOG_ERR level

v0.4.0
------

**New Features / improvements**

- kb900x_configure_firmware function to change firmware configuration.

**Fixes**

- Install the headers in the correct location (openbmc/).
- Prefix i2c_open and LOG_LEVEL symbols with kb900x_ to avoid conflicts with other libraries.
- kb900x_get_vendor_id function to return the vendor ID as an uint32_t instead of an int.
- kb900x_bic_write function to write the correct number of bytes.

v0.3.0
------

**New Features / improvements**

- SMBus/TWI or BIC communication mode selection added (build option).
- BIC communication using libbic library.
- Get TX presets.
- Get FOM (figure of merit)
- Get phy info.
- Get HW RTSSM.
- Dump phy and RPCS registers.

v0.2.0
------

**New Features / improvements**

- KB900x B1 support.
- Recovery mode.
- TWI/SMBus communication switch.
- FW upgrade.
- Link status SMBus command.
- RTSSM history dump.

v0.1.0
------

- Initial release for KB900x B0.
