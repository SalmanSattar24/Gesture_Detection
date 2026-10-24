# Historical README — not current project status

Preserved before the October 23, 2026 documentation refresh. Commands, links, performance language and implementation claims below may be stale or unverified. This archive is not an endorsement of those claims. Read the [current overview](../README.md) and [employer guide](EMPLOYER_GUIDE.md) first. Original relative links may refer to the former root location.

---

The Adruino sketch is the main file that needs to be flashed to the ESP32

The .zip file needs to be imported as a library to Arduino IDE before the sketech is run.


Wiring diagram:

MPU6050 Sensor:
- VCC → 3.3V (ESP32)
- GND → GND (ESP32)
- SCL → GPIO 22 (ESP32)
- SDA → GPIO 21 (ESP32)

LEDs:
Red LED (Circle):
- Anode (+) → GPIO 13 (ESP32) through 220Ω resistor
- Cathode (-) → GND (ESP32)

Blue LED (Static):
- Anode (+) → GPIO 12 (ESP32) through 220Ω resistor
- Cathode (-) → GND (ESP32)



