# FloatSat Embedded Control Firmware

Clean STM32 embedded-firmware repository for FloatSat, a master's-level university group project at the University of Wuerzburg.

The wider FloatSat project was a three-axis attitude-control demonstrator for a Spherical Air Bearing Unit (SABU). It combined mechanical design, electronics, embedded control, wireless communication, a ground-station dashboard, and control-system development. I had the opportunity to participate in this larger master's-level team project and contributed on the software architecture and embedded-integration side. This repository is the cleaned, reviewable version of the STM32 firmware contribution.

The complete group report is preserved in [docs/main.pdf](docs/main.pdf). The Markdown documentation in this repository summarizes the embedded part and links it back to the full system design without implying that the external ground-station, ESP32, CAD, or electrical-design repositories live here.

## Project Context

FloatSat is a practical satellite-systems project. The goal is to build a compact satellite-like platform that can rotate on a nearly frictionless air bearing, estimate its attitude, and use reaction wheels to stabilize or manoeuvre itself.

The full system was designed by a student team with separate responsibilities:

| Area | Full-system responsibility | Status in this repo |
| --- | --- | --- |
| Mechanical design | Disc-based structure, reaction-wheel mounting, mass distribution, SABU interface | Documented through the report only |
| Electrical design | STM32, ESP32, IMU, motor drivers, EPS, battery wiring | Documented through the report and firmware pin map |
| Embedded firmware | Scheduling, IMU acquisition, attitude estimation, control, UART framing, motor output | Included |
| Communication bridge | ESP32 UART-to-MQTT bridge | External, protocol boundary documented |
| Ground station | MQTT backend, WebSocket bridge, web dashboard, 3D visualization | External, interface context documented |
| Control development | PID/cascaded-control design, loop-rate reasoning, test evolution | Partly included as firmware logic and documentation |

The report describes the final demonstrator as a reaction-wheel-based experimental platform for showing basic spacecraft attitude-control concepts. This repository focuses on the embedded software slice of that system.

## System Highlights

- STM32F401 BlackPill firmware built with PlatformIO and Arduino STM32.
- SparkFun LSM9DS1 IMU acquisition over I2C.
- Madgwick attitude estimation with a configurable IMU mount-yaw correction.
- Three SimpleFOC reaction-wheel motor channels.
- Non-blocking scheduler based on a 1 kHz TIM2 tick.
- Roll, pitch, and yaw command handling.
- Yaw attitude and yaw-rate modes.
- Fixed-length UART command and telemetry frames with CRC16-CCITT.
- Python utilities for serial attitude visualization and gain estimation.
- Documentation for the architecture, UART boundary, cleanup decisions, and full-system report.

## What Is Included

- The cleaned STM32 firmware under `src/` and `include/`.
- A PlatformIO project targeting `blackpill_f401cc`.
- A small `Makefile` wrapper for common build, upload, monitor, and check commands.
- Python helper tools under `tools/`.
- Embedded-focused documentation under `docs/`.
- The full group report at [docs/main.pdf](docs/main.pdf).

## What Is Not Included

- ESP32 bridge source code.
- MQTT server, EMQX configuration, NestJS backend, or Next.js ground-station dashboard.
- CAD models, 3D-print files, PCB design files, or full wiring source files.
- Generated PlatformIO build output such as `.pio/`.
- Old lab branches, throwaway bring-up folders, and duplicate experimental code.

The repository boundary is intentional: this is the embedded STM32 deliverable, with the communication contract documented so that the external systems can integrate with it.

## Full System Architecture

```text
Ground-station dashboard
        |
        | WebSocket / REST
        v
NestJS backend
        |
        | MQTT topics over EMQX
        v
ESP32 wireless bridge
        |
        | Binary UART frames, CRC16
        v
STM32F401 firmware in this repository
        |
        +-- uart_link: command parsing and telemetry framing
        +-- attitude: LSM9DS1 acquisition and Madgwick filter
        +-- yaw_control: yaw attitude/rate control
        +-- main: scheduling, roll/pitch loops, integration
        +-- reaction_wheels: SimpleFOC motor output and slew limiting
        v
Reaction wheels and FloatSat body on SABU
```

The full report documents a ground-station pipeline designed for smooth 100 Hz visualization. This cleaned STM32 firmware currently sets `kTelemetryUpdateHz = 10` in [include/floatsat_config.h](include/floatsat_config.h), which keeps the serial/debug path conservative while preserving the same binary frame format.

## Hardware Target

Primary embedded board:

- STM32F401 BlackPill.
- Arduino STM32 framework.
- DFU upload.

Sensors and actuators:

- SparkFun LSM9DS1 9-DoF IMU.
- 3x T-Motor GB2208 brushless motors with flywheels.
- 3x SimpleFOC Mini style 3-PWM motor drivers.
- ESP32 communication bridge connected over UART.

Important firmware pins:

| Function | Pins |
| --- | --- |
| IMU I2C | `PB10` SCL, `PB3` SDA |
| UART link to ESP32 | `PA2` TX, `PA3` RX |
| Reaction wheel X PWM | `PA8`, `PA9`, `PA10`, enable `PB13` |
| Reaction wheel Y PWM | `PB0`, `PA5`, `PB4`, enable `PA15` |
| Reaction wheel Z PWM | `PB6`, `PB7`, `PB8`, enable `PB12` |

Mechanical and electrical details from the report that matter for firmware:

- The platform is a compact disc-based structure rather than the earlier tower concept.
- The assembled mass is documented as 1684.50 g, below the 3 kg course constraint.
- The center of mass was designed to sit close to and slightly below the SABU center of rotation.
- The reaction-wheel axes are mutually orthogonal.
- The IMU is placed close to the geometric center / center of mass to reduce lever-arm effects.
- STM32 and ESP32 responsibilities are deliberately separated: control stays on the STM32, wireless/network logic stays outside it.

## Runtime Scheduling

The firmware uses `TIM2` as a 1 kHz base timer. The interrupt only sets flags; all substantial work runs in the main `loop()`. This avoids I2C reads, UART parsing, motor updates, or serial output inside the interrupt context.

| Task | Rate in this checkout | Responsibility |
| --- | ---: | --- |
| UART parser | Continuous | Drain bytes, resync on start bytes, validate CRC, update command mailbox |
| SimpleFOC tick | Continuous | Keep motor drivers serviced |
| Attitude update | 200 Hz | Read IMU, update Madgwick, refresh angular-rate estimates |
| Control update | 50 Hz | Convert attitude/rate targets into body-rate commands |
| Wheel ramp update | 50 Hz | Slew-limit wheel speed targets and adapt voltage limits |
| Telemetry update | 10 Hz | Send compact UART telemetry and print debug CSV |

`TIM2` is used for scheduling so that motor/PWM-related timers remain available for actuation.

## Control Strategy

The current firmware keeps the controller understandable and testable:

- Roll and pitch use PI attitude-to-rate loops.
- Yaw supports attitude mode and rate mode.
- Controller outputs are saturated to safe body-rate commands.
- Integrators include a simple anti-windup rule.
- Body-rate commands are mapped to reaction-wheel speed targets.
- Wheel targets are clamped and slew-limited before reaching SimpleFOC.

The full project considered and documented a more general cascaded-control approach. For the cleaned firmware, the implementation is intentionally conservative so it can be inspected, flashed, and tuned on real hardware without hiding behavior behind a complex controller.

## UART Boundary

The STM32 communicates with the ESP32 bridge over UART at `115200` baud. The binary protocol uses:

- Start bytes: `0xAA 0x55`.
- Fixed payload lengths.
- CRC16-CCITT over the payload.
- Little-endian `uint16` fields.
- Centi-degree fixed-point encoding.

Command frame from ESP32 to STM32:

```text
AA 55 | type value_lo value_hi | crc_lo crc_hi
```

Telemetry frame from STM32 to ESP32:

```text
AA 55 | roll pitch yaw roll_rate pitch_rate yaw_rate | crc_lo crc_hi
```

The ESP32 side, described in the report but not included here, translates between this binary UART protocol and MQTT JSON messages used by the ground station. More detail is in [docs/UART_PROTOCOL.md](docs/UART_PROTOCOL.md).

## Repository Layout

```text
.
|-- include/
|   |-- attitude.h
|   |-- floatsat_config.h
|   |-- reaction_wheels.h
|   |-- uart_link.h
|   `-- yaw_control.h
|-- src/
|   |-- attitude.cpp
|   |-- main.cpp
|   |-- reaction_wheels.cpp
|   |-- uart_link.cpp
|   `-- yaw_control.cpp
|-- tools/
|   |-- attitude_viewer.py
|   `-- pid_gain_calculator.py
|-- docs/
|   |-- README.md
|   |-- ARCHITECTURE.md
|   |-- CODE_CURATION.md
|   |-- UART_PROTOCOL.md
|   `-- main.pdf
|-- Makefile
|-- platformio.ini
`-- README.md
```

## Build And Upload

Install PlatformIO, then use the wrapper targets:

```bash
make build
make upload
make monitor
```

The wrapper auto-detects `pio`, `platformio`, or `~/.platformio/penv/bin/pio`.

Raw PlatformIO equivalents:

```bash
pio run -e blackpill_f401cc
pio run -e blackpill_f401cc -t upload
pio device monitor -b 115200
```

The configured upload protocol is DFU:

```ini
upload_protocol = dfu
```

Useful maintenance commands:

```bash
make check-python
make size
make clean
make cleanall
```

## First Boot Procedure

1. Connect the STM32F401 board.
2. Keep the motor drivers unpowered until the setup is mechanically safe.
3. Flash the firmware.
4. Open the serial monitor at `115200`.
5. During startup, rotate the satellite through multiple orientations for magnetometer calibration.
6. Wait for `FloatSat firmware ready.`
7. Start sending UART command frames from the ESP32 bridge or a test sender.

## Development Tools

Live attitude viewer:

```bash
python3 tools/attitude_viewer.py --port /dev/ttyACM0 --baud 115200
```

PID gain estimate helper:

```bash
python3 tools/pid_gain_calculator.py
```

The attitude viewer reads the debug CSV printed by the firmware:

```text
yaw_rad,pitch_rad,roll_rad
```

## Documentation

- [docs/README.md](docs/README.md): documentation index.
- [docs/main.pdf](docs/main.pdf): complete group design report.
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md): embedded architecture and system boundary.
- [docs/UART_PROTOCOL.md](docs/UART_PROTOCOL.md): STM32-to-ESP32 binary protocol and external communication context.
- [docs/CODE_CURATION.md](docs/CODE_CURATION.md): why this repository is a cleaned deliverable rather than a dump of lab folders.

## Known Limitations

- Control gains are conservative starting values and must be tuned on the real SABU setup.
- Magnetometer calibration is blocking at startup.
- The code assumes the wiring and pin mapping listed above.
- The cleaned firmware publishes telemetry at 10 Hz, while the wider ground-station architecture was designed for higher-rate visualization.
- ESP32, MQTT, backend, and dashboard code are external to this repository.
- Mechanical CAD, PCB, and full electrical source files are represented through the report, not as editable project files here.

## Why This Repository Exists

The original lab workspace was useful for fast experimentation, but hard to review. This repository turns the embedded contribution into a clean deliverable: readable modules, explicit system boundaries, documented protocols, build commands that work from the repo root, and enough project context to understand how the STM32 firmware fits into the full FloatSat demonstrator.
