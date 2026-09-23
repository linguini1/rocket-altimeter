# NuttX Rocket Altimeter Tool Suite

This repository hosts a collection of [NuttX][nuttx] applications that together
compose application-level firmware high-powered rocket altimeters. The intention
is for the application-level firmware to be modular and extensible such that it
can work for altimeters of differing feature-sets:

* Timer-based deployment altimeters
* Barometer-based altimeters
* Bluetooth-capable altimeters
* Different numbers of pyro channels
* Continuity monitoring altimeters
* Different storage capabilities
* Different battery power supplies

... and more!

This was written with the primary considered use case of running on the
[Peanut][peanut] altimeter, but is written to work on other NuttX-based systems
with a subset of all of the supported features. Core logic can be (and has been)
tested via emulators or the [NuttX simulator][sim] by enabling the mocking
capabilities of the tool suite. Most testing has taken place on [Peanut][peanut]
itself.

**Learn more:**

This tool suite, along with the [Peanut][peanut] altimeter, is the subject of
one of my [2026 Apache Community over Code conference
presentations][coc-alt-pres]. I discuss the process of designing an altimeter
with NuttX in mind, as well as what considerations go into robust firmware
design and extensible feature support.

## Installation & Build

In order to use this tool suite, just clone this repository in your
[`nuttx-apps`][nuttx-apps] directory. It will be detected by Kconfig and can be
selected/configured as normal through the NuttX build system.

```console
$ cd nuttx-apps
$ git clone https://github.com/linguini1/rocket-altimeter.git
```

In `make menuconfig`, a menu called "Rocket Altimeter" will appear under
"Application Configuration". All applications and tools can be selected within
this menu. Note that some applications have other dependencies which might need
to be enabled first (i.e. `CONFIG_UORB`, `CONFIG_ADC`, etc.). Depending on your
`kconfig` frontend, you can press `z` to allow display of hidden items. This
will show which items do not have their dependencies yet, and checking the help
information on each item will display the dependencies you must first enable.

## Intended Use

The intended use of the programs included in this tool suite is to be used in
conjunction with the [NuttX `nxinit` system][nxinit]. This system provides a
robust way of describing a system initialization process through an `init.rc`
file, with hooks for different stages of NuttX's initialization (i.e. boot,
after network stack is initialized, etc). This eliminates the need for `nsh`, or
custom application-level logic for performing board initialization. `nxinit`
also monitors any programs/services it starts, so it is capable of restarting
failed programs, running programs periodically, rebooting the device on critical
failures, etc.

It is recommended that a NuttX device leveraging the rocket altimeter tool suite
would initialize its system through `nxinit` as the entry point. This allows an
`init.rc` file to specify which of the altimeter programs are to be started, and
how many instances. These programs are all primarily configured through command
line arguments to allow different instances to have different behaviours (as
opposed to Kconfig options, which are compile-time and thus apply to all
instances of the program).

A sample `init.rc` file and configuration for the [NuttX simulator][simulator]
are given in [`simulator/`](./simulator).

## Applications

Each application included in this tool suite has an accompanying README
explaining its functionality, configuration options and how it might be used as
part of application-level altimeter firmware.

Any program may be swapped out or modified to fit the needs of a different
altimeter device. The architecture is designed for this to be as frictionless as
possible, since programs primarily exchange data with one another through the
[uORB][uorb] framework. This also gives us easy mocking for free.

A list of the applications and their short summary is as follows:

* `btdaemon/`: A BLE daemon, responsible for publishing device information,
  battery level, etc. over BLE
* `config/`: A command line tool for reading and modifying altimeter
  configuration files. Also provides a library interface for other programs to
  extract configuration values at runtime.
* `deployment/`: Controls the actuation of GPIO-based deployment/pyro channels
  using altitude information or timers (or both).
* `fake_baro/`: Program that can emulate a barometric pressure sensor for
  mocking purposes. Can replay data from a file (previous flight), a
  mathematical curve (good for on-device with low storage) or from data received
  over BLE.
* `logger/`: Extensible program for logging altimeter data to log files
  (altitude, deployment events, etc.).
* `processing/`: Collection of data processing programs
  * `adc_uorb/`: Turns voltage measurements from an ADC device into uORB voltage
    topics so that measurements can be accessed by channel
  * `altitude/`: Computes altitude from barometric pressure measurements
  * `batmon/`: Monitors a battery's voltage and charge level (supports different
    battery types)
  * `continuity/`: Monitors the continuity status of deployment channels
  * `events/`: Computes flight events (ascent, apogee, descent, etc.) using
    altitude, velocity, etc., and a state machine
  * `velocity`: Discretely computes velocity from altitude
* `sensor/`: Definitions for custom [uORB][uorb] topics used by the tool suite

[nuttx]: https://github.com/apache/nuttx
[nuttx-apps]: https://github.com/apache/nuttx-apps
[peanut]: https://github.com/linguini1/peanut
[sim]: https://nuttx.apache.org/docs/latest/platforms/sim/sim/boards/sim/index.html
[nxinit]: https://nuttx.apache.org/docs/latest/applications/system/nxinit/index.html
[uorb]: https://nuttx.apache.org/docs/latest/applications/system/uorb/index.html
[coc-alt-pres]: https://web.cvent.com/event/ac71ce47-2b5f-424c-abfe-5b48255315fb/summary?session=b94e377f-e18d-4d64-92bf-246464bb62e4&shareLink=true
