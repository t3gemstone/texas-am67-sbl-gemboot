<p align="center">
    <picture>
        <source media="(prefers-color-scheme: dark)" srcset=".meta/logo-dark.png" width="40%" />
        <source media="(prefers-color-scheme: light)" srcset=".meta/logo-light.png" width="40%" />
        <img alt="T3 Foundation" src=".meta/logo-light.png" width="40%" />
    </picture>
</p>

# Texas AM67 SBL Gemboot

 [![T3 Foundation](./.meta/t3-foundation.svg)](https://www.t3vakfi.org/en) [![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## What is it?

This project is prepared for flashing images to the emmc of [t3-gem-o1](https://docs.t3gemstone.org/tr/boards/o1/introduction) boards using the [gem-imager](https://github.com/t3gemstone/gem-imager)

# Prerequisites

The project utilizes Taskfile as its build system. Please install it from [here](https://taskfile.dev/installation/).

# Usage

To compile the project, TI MCU+SDK and compilers are required. They can be installed with the following command:

```bash
$ task install-sdk
```

To configure pinmux settings with sysconfig tool:

```bash
$ task syscfg-gui
```

To build the project:

```bash
task build
```

The compiled binary is located under the build folder as `tiboot3.bin`