# Define the library
lib_name = 'kb900x_smbus'
lib_sources = [
    'kb900x_smbus_comm.c',
    'kb900x_i2c_comm.c',
    'kb900x.c',
    'kb900x_i2c_master.c',
    'kb900x_eeprom.c',
    'kb900x_utils.c',
    'kb900x_log.c'
]

if bic_communication
    lib_sources += 'kb900x_bic_comm.c'
endif

inc = include_directories('../include')

# Create the library
smbus_sh_lib = library(
    lib_name,
    sources: lib_sources,
    include_directories: inc,
    install: true,
    dependencies: [bic_dep],
    # c_args: '',
    soversion: '0',
    # gnu_symbol_visibility: 'hidden'
)

# Make this library usable as a Meson subproject.
regli_smbus_dep = declare_dependency(
   include_directories: [inc, include_directories('.')],
   link_with : smbus_sh_lib)

if get_option('enable_test')
    # Build test executable
    test_exe = executable('test_regli_smbus',
        sources: ['../tests/test_main.c', lib_sources],
        include_directories: inc,
        dependencies: [bic_dep]
    )
endif
