# Code Curation Notes

The original workspace contained one Git repository with several experimental branches plus multiple local PlatformIO folders used for hardware bring-up. This final repository is intentionally not a dump of all of that material.

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
- Communication module split into `comm.cpp`/`comm.hpp`.

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

## Final Selection

The final repository keeps:

- The cleaned STM32 firmware.
- Tooling that helps demonstrate or tune the embedded system.
- Documentation explaining the project, architecture, and UART boundary.

The final repository drops:

- Generated build products.
- Vendor caches.
- Dead code.
- Local editor state.
- Broken or superseded experiments.

## Why Not Keep Branches?

For a portfolio repository, a clean main branch is easier to review than a branch maze. The useful branch-level ideas were consolidated into readable modules, and the old branch history remains available in the original `finn_github` checkout if needed.
