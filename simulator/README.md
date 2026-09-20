# Simulator

This directory contains a debug configuration for the [NuttX simulator
target][sim]. This allows debugging of logic on a desktop instead of working
with the embedded target directly.

## Usage

In order to use this, apply the `defconfig` to the build system using NuttX's
`./tools/configure.sh` tool. Then, copy the `init.rc` file to the simulator's
`etc/init.d` directory, where it will overwrite the existing one. You can
restore it after you're done testing. The full path is
`boards/sim/sim/sim/src/etc/init.d/init.rc`.

The NuttX build directory will be mounted as `/data`. It will be where log files
are written. You should also put your altimeter configuration file here. A
sample one is provided under `config.conf`.

[sim]: https://nuttx.apache.org/docs/latest/platforms/sim/sim/boards/sim/index.html
