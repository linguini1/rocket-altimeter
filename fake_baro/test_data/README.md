# Fake Barometer Test Data

This directory contains barometer data in CSV format, intended for use with the
`fake_baro` NuttX application for testing. The barometer data is taken from
published rocket flights and includes a short manifest which describes the key
characteristics of the flight.

Data has been formatted to adhere to the uORB `fakesensor` requirements:
```csv
interval:10
pressure,temperature
1013.25,15
...
```
