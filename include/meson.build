inc_sources = [
    'kb900x_smbus_comm.h',
    'kb900x_i2c_comm.h',
    'kb900x.h',
    'kb900x_i2c_master.h',
    'kb900x_eeprom.h',
    'kb900x_utils.h',
    'kb900x_log.h'
]

bic_communication = get_option('bic_communication')
if bic_communication
    inc_sources += 'kb900x_bic_comm.h'
endif

# Make this library usable from the system's
# package manager.
install_headers(
  inc_sources,
  preserve_path: true,
)
