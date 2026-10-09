# esp32-smart-curtain

Zigbee-controlled motorized curtain firmware for the Seeed XIAO ESP32, exposed as a standard Zigbee window-covering device so it joins and is controlled like any off-the-shelf curtain motor (e.g. via Home Assistant / Zigbee2MQTT).

## Design

The firmware is split into small layers behind interfaces rather than one big loop, so hardware-facing parts can be swapped without touching the control logic (the position sensor is already simulated this way):

- **`app/CurtainController`** — the state machine (open/close/stop/move-to-position), driven purely by a motor interface and a position-sensor interface. It has no idea whether the motor or sensor are real or simulated.
- **`motor/IMotorDriver`** / **`motor/GpioMotorDriver`** — drives a DRV8833 H-bridge over GPIO (XIAO D1/D2 for direction, D3 for sleep/standby).
- **`position/IPositionSensor`** / **`position/SimulatedPositionSensor`** — position feedback is currently simulated (modeled at a fixed %/s) while the real sensor integration is pending; swapping in a real sensor only means implementing `IPositionSensor`.
- **`zigbee/ZigbeeController`** — wraps Espressif's `esp-zigbee-lib`, exposing the curtain through standard Zigbee window-covering clusters. Zigbee stack callbacks run on their own statically-allocated FreeRTOS task and talk to the controller through a command queue, so stack callbacks never block (or get blocked by) the control loop.
- **`core/Result`** — a small `[[nodiscard]]` result type used instead of exceptions (not available in this build) for fallible operations (`motor.initialize()`, `zigbee.start()`, ...).

`app_main` wires these together and runs a simple fixed-period loop: pull any pending Zigbee commands, update the controller, and publish state changes back over Zigbee.

## Hardware

- Seeed Studio XIAO ESP32 (ESP32-C/S series)
- DRV8833 dual H-bridge motor driver
- ESP-IDF >= 5.5, `espressif/esp-zigbee-lib` ^2.0.0 (see `main/idf_component.yml`)

## Building

Standard ESP-IDF project:

```
idf.py set-target esp32s3   # or whichever XIAO ESP32 variant you're on
idf.py build
idf.py -p <port> flash monitor
```

## Status

Core state machine, motor driver, and Zigbee integration are implemented and wired up end-to-end. Position sensing is simulated — real sensor hardware (e.g. a limit switch or rotary encoder) hasn't been integrated yet.
