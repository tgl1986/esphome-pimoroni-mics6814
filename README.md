# ESPHome External Component: Pimoroni MiCS6814 (IO Expander / MS51)

This repository provides an **ESPHome external component** for the **Pimoroni MiCS6814 Breakout** (IO Expander / MS51).
Communication happens over **I²C** and is intended for **ESPHome + ESP-IDF** (no Arduino framework required).

## Features

- Read the three MiCS6814 channels:
  - **oxidising**
  - **reducing**
  - **NH3**
- Publish:
  - raw ADC readings
  - computed sensor resistances **Rs (Ohm)**
  - **R0 baseline** per channel (stored in ESPHome preferences / NVS)
  - **Rs/R0 ratio** per channel
- Control:
  - heater enable/disable (active-low on the breakout)
  - calibration button that captures R0 as the mean Rs over a configurable time window
  - optional manual R0 set via `number:` entities

## Hardware

- Pimoroni MiCS6814 Breakout (IO Expander based)
- ESP32 (e.g. ESP32 DevKit / WROOM-32)
- I²C wiring: SDA/SCL + 3V3 + GND

Typical IO Expander I²C address is **0x18 or 0x19** depending on configuration/firmware.
Enable `i2c.scan: true` in ESPHome to confirm your device address.

## Installation (local)

1. Copy the `components/` folder into the same directory as your ESPHome YAML.
2. Reference it in your YAML:

```yaml
external_components:
  - source:
      type: local
      path: ./components
    components: [ pimoroni_mics6814 ]
```

3. Use one of the configs in `examples/` (see `examples/full.yaml`).

## Calibration concept (R0)

MiCS6814 sensors are commonly used by comparing **Rs/R0** ratios.
This component stores **R0 per channel** in ESPHome preferences (NVS) and publishes ratios.

The calibration button captures R0 as the mean Rs over a time window
(default: 30 seconds). Calibrate in stable conditions (typically "clean air") and allow
the sensor to warm up first.

> This does not claim lab-grade ppm accuracy. ppm approximation typically requires
> a fitted model from the sensor datasheet curves and careful environmental control.

## References

- Protocol / compatibility notes (Pimoroni forum):
  https://forums.pimoroni.com/t/mics-6814/20777
- Ohms-to-ppm discussion (Enviro+ approach):
  https://forums.pimoroni.com/t/enviro-ohms-to-ppm/12207

## License

No license has been chosen yet. Add one before publishing if you intend others to use/redistribute.
