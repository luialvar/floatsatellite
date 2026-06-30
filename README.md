# FloatSat Embedded Control Firmware

Clean embedded-firmware repository for a university group project in the FloatSat course at the University of Wuerzburg.

The complete project was a 3-DoF satellite prototype for a spherical air bearing test bench. The full system included mechanical design, electronics, attitude determination and control, a ground-station user interface, and an MQTT-based communication server. This repository intentionally contains only the embedded side: STM32 firmware, reaction-wheel control, IMU attitude estimation, and the UART protocol boundary used by the communication layer.

## Project Context

FloatSat is a hands-on satellite-systems project. Student teams design, build, wire, program, test, and document a small prototype satellite that can stabilize itself on a spherical air bearing unit and perform basic attitude manoeuvres.

Main objective:

- Stabilize a satellite prototype on a low-friction spherical air bearing.
- Estimate attitude and angular velocity from onboard sensors.
- Drive reaction wheels to perform basic 3-axis attitude control.
- Exchange commands and telemetry with a ground-station stack.

This repository is a cleaned portfolio-ready version of the embedded software produced during that group project. The messy lab branches, generated PlatformIO caches, and one-off hardware tests were reviewed and consolidated into a single maintainable firmware tree.

## What Is Included

- STM32F401 BlackPill firmware built with PlatformIO and Arduino STM32.
- SparkFun LSM9DS1 IMU acquisition.
- Madgwick attitude estimation.
- Three SimpleFOC reaction-wheel motor channels.
- Fixed-rate cooperative scheduling using TIM2.
- UART telemetry and command protocol with CRC16.
- Yaw attitude/rate controller with anti-windup.
- Roll and pitch PI rate-command loops.
- Python tools for serial attitude visualization and gain estimation.
- Documentation for architecture, UART framing, and the cleanup decisions.

## What Is Not Included

- The MQTT server and ground-station application are not part of this repository.
- CAD, PCB, and full system report files are not included here.
- Generated PlatformIO outputs such as `.pio/`, `.vscode/`, and `compile_commands.json` are intentionally ignored.
- Old experiment folders are documented but not carried into the final tree.

## Hardware Target

Primary embedded board:

- STM32F401 BlackPill.
- Arduino STM32 framework.
- Upload over DFU.

Sensors and actuators:

- SparkFun LSM9DS1 IMU over I2C.
- 3x T-Motor GB2208 brushless motors with flywheels.
- 3x SimpleFOC Mini style 3-PWM motor drivers.
- ESP32 communication bridge on the other side of the UART link.

Important pins used by the firmware:

| Function | Pins |
| --- | --- |
| IMU I2C | `PB10` SCL, `PB3` SDA |
| UART link to ESP32 | `PA2` TX, `PA3` RX |
| Reaction wheel X PWM | `PA8`, `PA9`, `PA10`, enable `PB13` |
| Reaction wheel Y PWM | `PB0`, `PA5`, `PB4`, enable `PA15` |
| Reaction wheel Z PWM | `PB6`, `PB7`, `PB8`, enable `PB12` |

## Firmware Architecture

```text
Ground station / MQTT server
          |
          | MQTT, UI commands, telemetry display
          v
ESP32 bridge
          |
          | UART framed protocol, CRC16
          v
STM32F401 firmware in this repository
          |
          +-- uart_link: command parsing and telemetry framing
          +-- attitude: LSM9DS1 + Madgwick state estimation
          +-- yaw_control: yaw attitude/rate controller
          +-- main: scheduling and control integration
          +-- reaction_wheels: SimpleFOC motor output and slew limiting
          v
Reaction wheels and FloatSat body
```

The embedded code is intentionally split between control logic and communication logic. UART parsing never blocks the control loop, and wheel commands are rate-limited before reaching the motor drivers.

## Runtime Tasks

The firmware uses `TIM2` as a 1 kHz base timer. The interrupt only sets flags; the main loop performs the actual work.

| Task | Rate | Responsibility |
| --- | ---: | --- |
| Attitude update | 200 Hz | Read IMU, update Madgwick filter, refresh angular-rate estimates |
| Control update | 50 Hz | Convert attitude/rate targets into body-rate commands |
| Wheel ramp update | 50 Hz | Slew-limit wheel speed targets and adapt voltage limits |
| Telemetry update | 10 Hz | Send compact telemetry over UART and print debug CSV |
| UART parser | Continuous | Drain bytes and update latest command mailbox |

## Control Strategy

The current firmware is deliberately conservative and testable:

- Roll and pitch use simple PI attitude-to-rate loops.
- Yaw supports two modes:
  - attitude mode: target yaw angle in radians.
  - rate mode: target yaw rate in rad/s.
- Integrators include a simple anti-windup rule.
- The controller outputs desired body rates.
- Reaction-wheel speeds are derived from body-rate commands using a configurable scale factor.
- Motor targets are clamped and slew-limited before being sent to SimpleFOC.

The course allowed more complex approaches such as SMC, MPC, or learning-based methods. For this prototype, a PID/PI-style controller was chosen because it is understandable, tuneable in the lab, and robust enough for early hardware integration.

## Repository Layout

```text
.
├── include/
│   ├── attitude.h
│   ├── floatsat_config.h
│   ├── reaction_wheels.h
│   ├── uart_link.h
│   └── yaw_control.h
├── src/
│   ├── attitude.cpp
│   ├── main.cpp
│   ├── reaction_wheels.cpp
│   ├── uart_link.cpp
│   └── yaw_control.cpp
├── tools/
│   ├── attitude_viewer.py
│   └── pid_gain_calculator.py
├── docs/
│   ├── ARCHITECTURE.md
│   ├── CODE_CURATION.md
│   └── UART_PROTOCOL.md
├── platformio.ini
└── README.md
```

## Build And Upload

Install PlatformIO, then run:

```bash
make build
make upload
make monitor
```

The `Makefile` is a thin wrapper around PlatformIO. If you prefer calling PlatformIO directly, use:

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

The configured upload protocol is DFU:

```ini
upload_protocol = dfu
```

If your shell exposes PlatformIO as `platformio` instead of `pio`, the equivalent build command is:

```bash
platformio run
```

## First Boot Procedure

1. Connect the STM32F401 board.
2. Power the motor drivers only when the setup is mechanically safe.
3. Flash the firmware.
4. Open the serial monitor at `115200`.
5. During startup, rotate the satellite through multiple orientations for magnetometer calibration.
6. Wait for `FloatSat firmware ready.`
7. Start sending UART command frames from the ESP32 bridge or test sender.

## UART Boundary

The STM32 communicates with the ESP32 bridge over UART at 115200 baud. Frames use:

- Start bytes: `0xAA 0x55`.
- Fixed payload length.
- CRC16-CCITT over payload only.
- Little-endian `uint16` fixed-point fields.

Command frame from ESP32 to STM32:

```text
AA 55 | type value_lo value_hi | crc_lo crc_hi
```

Telemetry frame from STM32 to ESP32:

```text
AA 55 | roll pitch yaw roll_rate pitch_rate yaw_rate | crc_lo crc_hi
```

More detail is in [docs/UART_PROTOCOL.md](docs/UART_PROTOCOL.md).

## Development Tools

Live attitude viewer:

```bash
python3 tools/attitude_viewer.py --port /dev/ttyACM0 --baud 115200
```

PID gain estimate:

```bash
python3 tools/pid_gain_calculator.py
```

The attitude viewer reads the debug CSV printed by the firmware:

```text
yaw_rad,pitch_rad,roll_rad
```

## Cleanup Notes

The original workspace contained multiple test projects and a messy upstream repository with several branches. Those sources were reviewed before creating this repository. Useful logic was consolidated; generated files and outdated experiments were left out.

Short version:

- `finn_github`: reviewed for latest branch work, IMU correction, motor ramping, and PID experiments.
- `floatsat-embedded-master`: used as the main source for attitude, UART, motor, and yaw-control modules.
- `floatsat-embedded-open_loop_integration`: used for earlier open-loop UART and reaction-wheel integration ideas.
- `floatsat_comm_final`, `floatsat_test`, `esp32_final`, and `tests/`: treated as bring-up and communication experiments.
- `.pio`, `.vscode`, generated build output, and `compile_commands.json`: discarded.

Full cleanup rationale is in [docs/CODE_CURATION.md](docs/CODE_CURATION.md).

## Known Limitations

- Control gains are conservative starting values and must be tuned on the real air-bearing setup.
- Magnetometer calibration is blocking at startup.
- The code assumes the wiring and pin mapping listed above.
- Telemetry rate fields use the signed 180-degree fixed-point mapping documented in the UART protocol.
- The MQTT server and UI are expected to live in a separate repository or service.

## Why This Repository Exists

The lab workspace was useful for fast iteration, but not ideal for review by another engineer. This repository is the cleaned embedded deliverable: a readable project structure, clear module boundaries, documented communication contracts, and no generated clutter.
