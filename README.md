# Rocket Altimeter Application

This repository hosts a [NuttX][nuttx] application that performs altimeter
calculations for rocket recovery system deployment, altitude tracking and
logging. It also contains tools for testing the altimeter logic without access
to the direct sensor hardware.

This is intended to run on the [peanut][peanut] altimeter, but is written to
at least partially work on other NuttX-based altimeter systems.

In order to use this application, just clone this repository in your
[`nuttx-apps`][nuttx-apps] directory. It will be detected by Kconfig and can be
selected/configured as normal through the NuttX build system.

```console
$ cd nuttx-apps
$ git clone https://github.com/linguini1/rocket-altimeter.git
```

## Mocking

To mock-test the altimeter logic, you should enable both the `processing`
application(s) and the `fake_baro` application. Once you've configured both
programs to your liking, run them in the following order:

```console
nsh> fake_baro; altitude_fusion &; velocity_fusion & events_topic &;
nsh> deployment &
```

You can then look at the published data from the topics using `uorb_listener`,
which is one of the stock NuttX apps.

It is likely preferred to do this mocking on the `sim` architecture. In this
case, you can copy one of the `data.csv` files from the `fake_baro` test data
collection to your main NuttX directory. When you run the simulator, the CSV
file will then be accessible from `data/data.csv`.

[nuttx]: https://github.com/apache/nuttx
[nuttx-apps]: https://github.com/apache/nuttx-apps
[peanut]: https://github.com/linguini1/peanut
