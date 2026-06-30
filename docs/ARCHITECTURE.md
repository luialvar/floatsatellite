# Embedded Architecture

This document describes the cleaned STM32 embedded architecture used in this repository.

## System Boundary

The full FloatSat project had several subsystems:

- Mechanical structure and flywheel design.
- Electrical power, wiring, and motor-driver integration.
- Embedded firmware on STM32.
- ESP32 communication bridge.
- Ground-station and MQTT server.

This repository contains the STM32 embedded firmware and tools only. The ESP32, MQTT server, and GUI are treated as external systems connected through the documented UART protocol.

## Data Flow

```text
UART commands
    |
    v
uart_link.cpp
    |
    v
main.cpp target mailbox
    |
    +--> yaw_control.cpp
    +--> roll/pitch PI controllers in main.cpp
    |
    v
reaction_wheels.cpp
    |
    v
SimpleFOC drivers and reaction wheels

LSM9DS1 IMU
    |
    v
attitude.cpp
    |
    v
Madgwick attitude estimate
    |
    v
control and telemetry
```

## Scheduling Model

The firmware uses `TIM2` as a 1 kHz base timer. The interrupt service routine only sets volatile flags. All work runs in `loop()`.

This keeps the firmware simple and avoids doing I2C, UART parsing, motor code, or serial printing inside an interrupt.

| Rate | Module | Work |
| ---: | --- | --- |
| Continuous | `uart_link` | Drain UART bytes, resync on start bytes, verify CRC |
| Continuous | `reaction_wheels` | Call `loopFOC()` and `move()` |
| 200 Hz | `attitude` | Read IMU and update Madgwick |
| 50 Hz | `main` and `yaw_control` | Compute controller outputs |
| 50 Hz | `reaction_wheels` | Apply wheel slew limiting |
| 10 Hz | `uart_link` | Send telemetry |

## Module Responsibilities

### `main.cpp`

- Owns scheduler flags.
- Owns target state and roll/pitch controller state.
- Connects attitude estimation, controller output, reaction wheels, and telemetry.

### `attitude.cpp`

- Initializes LSM9DS1 over I2C.
- Runs a startup magnetometer calibration.
- Updates the Madgwick filter.
- Applies a configurable IMU mount-yaw correction.
- Exposes roll, pitch, yaw, and angular rates in radians.

### `yaw_control.cpp`

- Provides yaw attitude and yaw-rate modes.
- Wraps yaw attitude error to `[-pi, pi]`.
- Applies output saturation and basic anti-windup.

### `reaction_wheels.cpp`

- Configures three SimpleFOC motor/driver pairs.
- Maps body-rate commands into wheel speed commands.
- Clamps maximum wheel speed.
- Slew-limits wheel speed changes.
- Adapts voltage limit based on requested wheel speed.

### `uart_link.cpp`

- Defines the STM32 side of the UART protocol.
- Parses fixed-size command frames from ESP32.
- Sends fixed-size telemetry frames back to ESP32.
- Uses CRC16-CCITT for frame validation.

## Control Loop

The controller is intentionally modest:

```text
target attitude/rate
        |
        v
attitude error
        |
        v
PI attitude-to-rate command
        |
        v
reaction-wheel speed target
        |
        v
slew-limited motor command
```

This was a practical choice for a university prototype: the control law is easy to inspect, tune, and debug during limited lab time.

## Safety Notes

- Power motor drivers only when the satellite is mechanically secured.
- Do not stress-test wheels continuously at high speeds.
- Keep the control loop non-blocking after startup.
- Check wiring before changing pin mappings.
- Re-tune controller gains after mechanical inertia changes.
