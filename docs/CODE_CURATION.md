# Code Curation Notes

This repository is a cleaned embedded deliverable, not a raw dump of the original FloatSat lab workspace.

The wider project was a master's-level university group project with mechanical, electrical, embedded, communication, ground-station, and control-system work. The complete report is preserved as [main.pdf](main.pdf). This repository focuses on the STM32 firmware contribution and keeps the rest of the system visible through documentation and protocol boundaries.

## Curation Goal

The goal was to turn a useful but messy development workspace into something another engineer can review:

- One clear PlatformIO project.
- One STM32 firmware tree.
- No generated build output.
- No duplicate old firmware versions.
- No half-integrated experiments in the main path.
- Explicit documentation for what belongs inside and outside this repo.

## Reviewed Sources

### `finn_github`

Reviewed as the main upstream repository with messy branch history.

Useful material:

- Three-axis motor pin mapping.
- IMU mount-yaw correction from the newer branch line.
- UART framing and telemetry format.
- Experiments around yaw/rate control.
- Branch `rebuild` ramping and voltage-limit ideas.

Not carried directly:

- Large commented-out `main.cpp` blocks.
- Debug-only prints such as `DEBUG 1`.
- Partially applied PID experiments that compute outputs but do not consistently drive the motors.
- Generated or editor-specific files.

### `floatsat-embedded-master`

Reviewed as the most complete local STM32 code snapshot.

Useful material:

- LSM9DS1 and Madgwick attitude estimator.
- UART command and telemetry parser.
- Simple yaw-control helper.
- Three reaction-wheel SimpleFOC integration.
- Serial attitude viewer idea.

Not carried directly:

- Active open-loop test block in `main.cpp`.
- Duplicate commented firmware versions.
- Old generic PlatformIO README files.

### `floatsat-embedded-open_loop_integration`

Reviewed as an earlier open-loop integration snapshot.

Useful material:

- Reaction-wheel bring-up structure.
- PID gain calculation script.
- Communication module split into `comm.cpp` / `comm.hpp`.

Not carried directly:

- `compile_commands.json`.
- `.pio` libraries and build output.
- Early test-only motor loop.

### `floatsat_comm_final`, `floatsat_prev_final`, `floatsat_test`, `esp32_final`, `tests/`

Reviewed as local experiments.

Useful material:

- UART frame format confirmation.
- STM32 communication timing tests.
- ESP32 bridge boundary.
- Early sensor and motor bring-up.

Not carried directly:

- Random telemetry generators.
- Incomplete ESP32 bridge header/API mismatch.
- One-off test folders.
- Duplicated PlatformIO template files.

## Report-Driven Documentation Additions

After adding [main.pdf](main.pdf), the README and docs were expanded to capture the full-system context:

- The project is presented as a master's-level university group project.
- The repository is described as the embedded STM32 slice of a larger team demonstrator.
- Mechanical design constraints are summarized because they affect firmware assumptions.
- Electrical architecture is summarized because it explains the STM32/ESP32 split.
- The UART protocol is documented as the boundary to the external ESP32/MQTT/ground-station stack.
- The telemetry-rate distinction is made explicit: the report discusses a 100 Hz ground-station pipeline, while this firmware checkout currently transmits at 10 Hz.

## Final Selection

The final repository keeps:

- Clean STM32 firmware.
- Hardware configuration and control constants.
- Build configuration for PlatformIO.
- A small Makefile wrapper.
- Python tools that help demonstrate or tune the embedded system.
- Documentation explaining the project, architecture, UART boundary, and cleanup decisions.
- The final group report PDF.

The final repository drops:

- Generated build products.
- Vendor caches.
- Dead code.
- Local editor state.
- Broken or superseded experiments.
- External ground-station and ESP32 code that should live in their own projects.

## Why Not Keep All Branches?

For a portfolio and review repository, a clean main branch is easier to understand than a branch maze. The useful branch-level ideas were consolidated into readable modules, and the old history remains available in the original development checkouts if needed.

This cleaned tree should answer three questions quickly:

1. What did the embedded STM32 firmware do?
2. How did it connect to the rest of the FloatSat system?
3. Why were these files selected for the final deliverable?
