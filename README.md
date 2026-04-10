# NuttX Rocket Altimeter Toolsuite

This repository hosts a collection of [NuttX][nuttx] applications that enables
altimeter functionality for rocket recovery systems on a NuttX device. This
includes logic for deployment, altitude & velocity computations, data logging
and ground testing.

This is intended to run on the [peanut][peanut] altimeter, but is written to
work on other NuttX-based systems with a subset of all of the supported features
(at minimum, a barometric pressure sensor should be present). Core logic can be
tested via emulators or the NuttX simulator.

In order to use this toolsuite, just clone this repository in your
[`nuttx-apps`][nuttx-apps] directory. It will be detected by Kconfig and can be
selected/configured as normal through the NuttX build system.

```console
$ cd nuttx-apps
$ git clone https://github.com/linguini1/rocket-altimeter.git
```

## Applications

The tool suite is comprised of multiple different applications which work
together/run concurrently to perform altimeter functions.

### Processing

The processing library contains a few different applications for data
processing/publishing via uORB.

* Altitude: convert barometer data into an altitude measurement
* Velocity: convert altitude data into an averaged velocity measurement
* Events: convert altitude and velocity data into flight events (lift-off,
apogee, landing, etc.)
* Battery: read ADC data and publish it as voltage data over uORB. This
application is optional depending if you have another source of battery
information

### Deployment

This application consumes altitude and event data over uORB and uses it to
decide when to trigger deployment of parachutes. Deployment options are
configurable (i.e. single/dual deploy, deployment altitude, etc.)

This application logs the deployment events it performs to a uORB topic.

### Logging

This application consumes barometer and deployment event uORB data and logs it
to a user-specified file as it comes in. No other data is logged by this
application due to space constraints on most altimeter devices, but you can
create your own logging application to record the other uORB data that is
generated. Everything is derived from barometer data so it is the most relevant
to log.

### Config

This application allows tweaking of the system-wide configuration file, which
records user settings like:

* Deployment options
* Battery warning settings

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
