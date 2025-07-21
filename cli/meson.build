add_languages('cpp', native: false)

# Build CLI executable
executable('kandou-cli',
    sources: ['cli.cpp'],
    include_directories: [inc, './CLI11/'],
    link_with: smbus_sh_lib,
    dependencies: [regli_smbus_dep, bic_dep])
