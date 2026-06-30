# FloatSat Documentation Index

This folder contains the documentation for the cleaned FloatSat STM32 firmware repository.

The most important file is [main.pdf](main.pdf), the complete group design report for the master's-level FloatSat project. The Markdown files summarize the embedded part of that report and explain how the firmware in this repository fits into the wider team system.

## Documents

| File | Purpose |
| --- | --- |
| [main.pdf](main.pdf) | Complete group report covering mechanical design, electrical design, attitude estimation, control, ground station, inter-subsystem communication, results, and requirements |
| [ARCHITECTURE.md](ARCHITECTURE.md) | Embedded architecture, real-time scheduling model, module responsibilities, and system boundary |
| [UART_PROTOCOL.md](UART_PROTOCOL.md) | Binary STM32-to-ESP32 UART protocol, fixed-point encoding, CRC behavior, and MQTT context |
| [CODE_CURATION.md](CODE_CURATION.md) | Explanation of how the messy development workspace was turned into this clean deliverable |

## How To Read This Repo

Start with the root [README.md](../README.md) for a portfolio-level overview. Then read [ARCHITECTURE.md](ARCHITECTURE.md) to understand the firmware modules and [UART_PROTOCOL.md](UART_PROTOCOL.md) to understand the ESP32 boundary.

Use [main.pdf](main.pdf) when you need the full project context: team responsibilities, mechanical constraints, electrical layout, ground-station architecture, and the original system requirements.

## Repository Boundary

This repository contains the STM32 embedded firmware. It does not contain the ESP32 bridge, MQTT backend, Next.js ground station, CAD files, or PCB sources. Those parts are described because they define the interfaces and engineering context around the firmware.
