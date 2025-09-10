# Sample: SEN0466 CO Sensor

This sample application demonstrates how to interface with the DFRobot SEN0466 CO sensor, which is a factory-calibrated carbon monoxide sensor that communicates via I2C. The sensor can measure CO concentrations from 0-1000 ppm and includes temperature compensation.

## Requirements

- DFRobot SEN0466 CO sensor
- Development board with I2C support (tested on nRF52840dk)
- I2C connection to the sensor

## Hardware Setup

Connect the SEN0466 sensor to your development board:

- VCC: 3.3V or 5V (sensor supports both)
- GND: Ground
- SDA: I2C data line (configure in device tree)
- SCL: I2C clock line (configure in device tree)

The sensor's I2C address is 0x74 (7-bit addressing).

## Building and Running

```bash
west build -p always -b nrf52840dk/nrf52840 samples/sensor/sen0466
west flash
```

## Expected Output

```
*** Booting nRF Connect SDK v3.1.0-6c6e5b32496e ***
*** Using Zephyr OS v4.1.99-1612683d4010 ***
[00:00:00.254,425] <inf> main: Starting SEN0466 CO sensor sample application
[00:00:00.254,455] <inf> main: SEN0466 device SEN0466 is ready
[00:00:00.257,202] <inf> main: CO[ppm]: 0
[00:00:00.257,202] <inf> main: TEMP[C]: 24.897305
[00:00:02.260,009] <inf> main: CO[ppm]: 0
[00:00:02.260,040] <inf> main: TEMP[C]: 24.897305
[00:00:04.262,847] <inf> main: CO[ppm]: 0
[00:00:04.262,878] <inf> main: TEMP[C]: 24.897305
```
