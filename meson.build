project('kb900x_smbus_library', 'c',
    version : run_command('cat', meson.project_source_root() / 'VERSION').stdout().strip(),
    default_options : [
        'c_std=c17',
        'cpp_std=c++20',
        'b_ndebug=if-release',
        'werror=true',
        'warning_level=3'
    ])

# Access the current build type
buildtype = get_option('buildtype')

# Conditionally add arguments for debug builds
if buildtype == 'debug'
    add_project_arguments('-DDEBUG', language: 'c')
endif

enable_test = get_option('enable_test')
if enable_test
    # Set compiler flags for C++ files (test files)
    # This avoid to flag C++ ISO standard
    # issues in C code. (Like anonymous struct not allowed)
    c_flags = ['-Wno-error=pedantic', '-std=c++20']
    add_project_arguments(c_flags, language: 'cpp')
endif

enable_cli = get_option('enable_cli')
if enable_cli
    # Set compiler flags for C++ files (test files)
    # This avoid to flag C++ ISO standard
    # issues in C code. (Like anonymous struct not allowed)
    c_flags = ['-Wno-error=pedantic', '-std=c++20']
    add_project_arguments(c_flags, language: 'cpp')
endif

logging_colors = get_option('logging_colors')
if logging_colors
    add_project_arguments('-DLOGGING_COLORS', language: 'c')
endif

bic_dep = dependency('', required: false)  # fallback if not used
bic_communication = get_option('bic_communication')
if bic_communication
    external_lib_dir = meson.project_source_root() / 'external_lib'
    bic_dep = declare_dependency(
        link_args: ['-L' + external_lib_dir, '-lbic'],
        compile_args: ['-DBIC_COMMUNICATION']
    )
    add_project_link_arguments('-Wl,--allow-shlib-undefined,-dynamic-linker=/lib/ld-linux-armhf.so.3', language: 'c')
endif

# Add the subdirectory for the headers
subdir('include')

# Add the subdirectory for the source code
subdir('src')

# Add the examples subdirectory
subdir('examples')

# Add the CLI subdirectory
if enable_cli
    subdir('cli')
endif

# Add the test subdirectory
if enable_test
    subdir('tests')
endif

# Set up clang-tidy checks
clang_tidy = find_program('clang-tidy', required: false)

if clang_tidy.found()
    # Add 'src/' prefix to each file in lib_sources
    lib_sources_with_path = []
    foreach file: lib_sources
        lib_sources_with_path += 'src/' + file
    endforeach

    # Add 'include/' prefix to each file in inc_sources
    inc_sources_with_path = []
    foreach file: inc_sources
        inc_sources_with_path += 'include/' + file
    endforeach

    # Add the clang-tidy target for your library
    clang_tidy_target = custom_target(
        'clang-tidy',
        input: [lib_sources_with_path, inc_sources_with_path],
        output: 'clang-tidy-output.txt',  # You can change the output file name
        command: [clang_tidy, '@INPUT@', '--fix-errors', '--extra-arg=-std=c17'],
        build_by_default: false,
    )
endif

pkg_mod = import('pkgconfig')
pkg_mod.generate(
    name : 'kb900x_smbus_library',
    filebase : 'kb900x_smbus_library',
    description : 'KB900x SMBus Library.',
    # subdirs : 'src',
    libraries : smbus_sh_lib,
    version : run_command('cat', meson.project_source_root() / 'VERSION').stdout().strip(),
)