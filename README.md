# Embedded Gesture Recognition

An ESP32 prototype that reads an MPU6050 accelerometer/gyroscope, buffers six-axis sensor samples, runs an exported Edge Impulse classifier and indicates **Circle** or **Static** states with LEDs.

**Status: hardware-dependent prototype.** Classification thresholds in source are not measured accuracy. No fresh board build, flash or evaluation was performed for this documentation update.

## Start here

[Employer guide](docs/EMPLOYER_GUIDE.md) · [Arduino sketch](sketch_dec19a.ino) · [Original wiring notes](docs/README_LEGACY.md)

## Data flow

```text
MPU6050 over I2C
  -> six-axis feature buffer
  -> Edge Impulse inference
  -> confidence + motion/state checks
  -> serial diagnostics and LED feedback
```

## Hardware and wiring

| Device | Connection |
|---|---|
| MPU6050 VCC / GND | ESP32 3.3 V / GND |
| MPU6050 SCL / SDA | GPIO 22 / GPIO 21 |
| Red LED, Circle | GPIO 13 through a suitable current-limiting resistor |
| Blue LED, Static | GPIO 12 through a suitable current-limiting resistor |

The historical prototype used 220-ohm LED resistors. Verify your board's pin/voltage constraints, including any boot-strapping implications, before wiring.

## Build workflow

1. Clone this repository and install Arduino IDE with support for your ESP32 board.
2. Import the existing `Gesture_Recognition_inferencing.zip` through the IDE's ZIP-library workflow. It provides the referenced inference header; it is not a new training dataset.
3. Open `sketch_dec19a.ino`, select the correct board and serial port, and review the sensor/model configuration.
4. Verify/compile before uploading. Uploading changes device firmware.
5. Inspect serial diagnostics at **115200 baud** and check behavior against known gestures.

These steps describe the intended workflow; toolchain/library compatibility has not been freshly verified. The source declares a nominal 60 Hz setting but uses integer millisecond scheduling; do not assume an exactly 60 Hz measured sampling rate.

## Engineering highlights

Sensor acquisition, fixed-window feature buffering, generated-model integration and a stateful decision layer rather than directly mapping the highest class score to an LED.

## Limits and attribution

The repository contains inference firmware and an exported library, not a complete reproducible training/evaluation record. Training-data provenance, class balance, held-out accuracy, latency, energy use and robustness across users need separate evidence. ESP32, MPU6050 and Edge Impulse components retain their upstream rights; do not assume the generated library has the same license as any new documentation.



