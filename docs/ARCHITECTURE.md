# Embedded Architecture

This document describes the cleaned STM32 embedded architecture in this repository and places it in the context of the wider FloatSat demonstrator described in [main.pdf](main.pdf).

## Project Level Context

FloatSat was developed as a master's-level university group project at the University of Wuerzburg. The complete system was not only firmware: it combined a mechanical SABU platform, electrical integration, STM32 control firmware, an ESP32 wireless bridge, an MQTT/backend layer, a web ground station, and control-system development.

This repository contains the STM32 embedded slice. It is intentionally documented with the external interfaces visible, because the firmware only makes sense as one subsystem in the full demonstrator.

## System Boundary

The full FloatSat project had several subsystems:

| Subsystem | Responsibility | Repository status |
| --- | --- | --- |
| Mechanical platform | Disc structure, flywheel placement, SABU mounting, mass distribution | External, summarized from report |
| Electrical system | STM32, ESP32, IMU, motor drivers, EPS, batteries, wiring | External, firmware pin map included |
| STM32 firmware | Sensor acquisition, estimation, control, reaction wheels, UART protocol | Included |
| ESP32 bridge | UART binary frames to MQTT JSON and back | External |
| Ground station | MQTT broker/client, WebSocket backend, web dashboard, 3D visualization | External |

The design report describes the full chain. The code here stops at the STM32 side of the UART link.

## Physical And Electrical Assumptions

Firmware behavior depends on several physical design choices from the report:

- The final platform is a compact disc-based structure rather than the initial tower concept.
- Reaction-wheel axes are arranged orthogonally to provide three-axis torque authority.
- The IMU is mounted close to the geometric center / center of mass.
- Heavy components are kept low so the center of mass sits close to and slightly below the center of rotation.
- STM32 and ESP32 are separate boards with separate responsibilities.
- Motors are open-loop BLDC outputs driven through SimpleFOC Mini style driver hardware.

Those assumptions explain why the firmware is organized around a central attitude estimate, three wheel channels, and a narrow UART command/telemetry interface.

## Data Flow

```text
Ground station / backend / MQTT
        |
        v
ESP32 wireless bridge
        |
        | UART binary protocol
        v
uart_link.cpp
        |
        v
main.cpp target mailbox
        |
        +--> roll/pitch PI controllers
        +--> yaw_control.cpp
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

The code separates control logic from communication logic. UART parsing updates a small command mailbox; the control loop reads the latest values at its own rate.

## Scheduling Model

The firmware uses `TIM2` as a 1 kHz base timer. The interrupt service routine only sets volatile flags. All real work runs in `loop()`.

This is a deliberate embedded-systems decision:

- IMU reads can involve I2C waits and should not run inside an interrupt.
- UART parsing should keep draining the hardware buffer without blocking control.
- Serial debug printing should never happen from the timer ISR.
- Motor commands and SimpleFOC ticks need to run frequently but outside the ISR.
- `TIM1` is left available for motor/PWM-related work rather than used as a general scheduler.

| Rate | Module | Work |
| ---: | --- | --- |
| Continuous | `uart_link` | Drain UART bytes, resync on start bytes, verify CRC |
| Continuous | `reaction_wheels` | Call `loopFOC()` and `move()` |
| 200 Hz | `attitude` | Read IMU and update Madgwick |
| 50 Hz | `main` and `yaw_control` | Compute attitude/rate control outputs |
| 50 Hz | `reaction_wheels` | Apply wheel slew limiting |
| 10 Hz | `uart_link` | Send telemetry from this cleaned firmware checkout |

The full report describes the ground-station side as designed for a 100 Hz visualization pipeline. This checkout currently uses a conservative 10 Hz telemetry setting in `floatsat_config.h`.

## Module Responsibilities

### `main.cpp`

- Owns the scheduler flags.
- Owns the target mailbox copy used by the controller.
- Owns the roll/pitch PI controller state.
- Connects attitude estimation, yaw control, reaction-wheel output, and telemetry.
- Keeps interrupt work minimal by executing tasks from the main loop.

### `attitude.cpp`

- Initializes the SparkFun LSM9DS1 IMU over I2C.
- Runs startup magnetometer calibration.
- Feeds the Madgwick filter.
- Applies the IMU mount-yaw correction.
- Converts the attitude estimate into roll, pitch, yaw, and angular rates in radians.

### `yaw_control.cpp`

- Provides yaw attitude and yaw-rate modes.
- Wraps yaw attitude error to `[-pi, pi]`.
- Applies saturation and basic anti-windup.
- Exposes a compact step function used by `main.cpp`.

### `reaction_wheels.cpp`

- Configures three SimpleFOC motor/driver pairs.
- Maps body-rate commands into wheel speed commands.
- Clamps maximum wheel speed.
- Slew-limits speed changes to reduce harsh acceleration.
- Adapts voltage limit based on requested wheel speed.

### `uart_link.cpp`

- Defines the STM32 side of the UART protocol.
- Parses fixed-size command frames from the ESP32.
- Sends fixed-size telemetry frames to the ESP32.
- Uses CRC16-CCITT for frame validation.
- Stores valid commands in a non-blocking mailbox.

## Control Loop

The firmware implements a conservative practical controller:

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

The report discusses a cascaded PID architecture and loop-rate separation as the general control direction. In this cleaned firmware, roll and pitch use simple PI attitude loops, while yaw supports both attitude and angular-rate commands. The design is intentionally easy to inspect and tune during lab work.

## Communication Architecture

The embedded communication boundary is binary UART:

```text
STM32 <-> ESP32
```

The wider project then continues as:

```text
ESP32 <-> MQTT broker/backend <-> WebSocket/frontend
```

The ESP32 bridge and ground station are not included in this repository, but the protocol is documented so the boundary is clear. See [UART_PROTOCOL.md](UART_PROTOCOL.md).

## Safety And Integration Notes

- Power motor drivers only when the satellite is mechanically secured.
- Do not stress-test wheels continuously at high speed.
- Keep the main control loop non-blocking after startup.
- Re-check wiring before changing pin mappings.
- Re-tune gains after mechanical inertia, flywheel, or mass-distribution changes.
- Treat the report's mechanical and electrical assumptions as part of the firmware contract.
