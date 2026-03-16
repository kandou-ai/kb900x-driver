![](kandou-logo.svg)

# KB900x Driver

An I2C/SMBus driver to communicate with the Regli Retimer family.

## OpenBMC Build

Copy the file `OpenBMC/libregli-retimer_X.X.X.bb` in your yocto project (`meta-<project>/recipes-<project>/regli-retimer/`) and build using the following command:

```bash
    bitbake libregli-retimer
```

## Linux Build

### Compile

This project uses Meson as a (python based) build system. To compile it, install the following dependencies :

- Meson : https://mesonbuild.com/Getting-meson.html
- Ninja : https://ninja-build.org/
- Python >= 3.10 : https://www.python.org/downloads/
- Any C/C++ compiler : gcc, clang, aso...

Once Meson and Ninja have been installed, run the following commands

```bash
    # Configure
    meson setup . build
    # Compile
    meson compile -C build
    # Run Main
    ./build/src/example_regli_smbus --i2c-id 1 --slave-address 0x21
    # Install headers
    meson install -C build
```

#### Compilation options

Compilation options can be specified when configuring the project with Meson.

```bash
meson setup . build -D$option=$value
```

Here are the available options :
- `enable_test` : Enable the tests (default : false)
- `logging_colors` : Enable the colors for the logging output (default : false)

### Use

You can now include the Regli SMBUS library with :

```c
#include <openbmc/kb900x.h>
```

### Run the tests

> [!WARNING]
> The test suite is not publicly available. If you wish to run the tests, please contact the project maintainer to request access.

The tests are written in C++ and use the Google Test suite, please install :

- Google Test : https://google.github.io/googletest/

Once GTest have been installed, run the following commands

```bash
    # Configure
    meson setup . build -Denable_test=true
    # Run the hardware tests
    meson test hardware -C build --verbose --test-args='--i2c-id $device_id --slave-address $i2c_slave_addr --comm-mode $comm_mode'
    # Run the mocked tests
    meson test mock -C build --verbose
```

For tests that need more than 30 seconds to run, you can use the following command :

```bash
    # Run the hardware tests
    meson test hardware --timeout-multiplier 0 -C build --verbose --test-args='--i2c-id $device_id --slave-address $i2c_slave_addr --comm-mode $comm_mode'
```

## Run the linter (Clang-Tidy)

We use the clang-tidy linter, please install :

- clang-tidy : https://clang.llvm.org/extra/clang-tidy/

Once Clang-Tidy have been installed, run the following commands

```bash
    # Configure
    meson setup . build
    # Run the linter/formater
    meson compile clang-tidy -C build
```
