# UART Protocol

This document describes the binary UART protocol between the STM32 firmware in this repository and the external ESP32 communication bridge.

The full FloatSat system uses a longer communication chain:

```text
STM32 firmware <-> ESP32 bridge <-> MQTT broker/backend <-> web ground station
```

Only the STM32 side is implemented in this repository. The report in [main.pdf](main.pdf) documents the broader ESP32, MQTT, backend, and dashboard architecture.

## Design Goals

- Keep the STM32 control loop independent from Wi-Fi, MQTT, JSON, and UI code.
- Use a compact binary format that is cheap to parse on a microcontroller.
- Avoid raw floating-point transfer between boards.
- Detect corrupted frames before they can update control targets.
- Allow the ESP32 to translate telemetry and commands into the JSON schema expected by the ground station.

## Serial Settings

- Baud rate: `115200`.
- STM32 UART pins: `PA2` TX, `PA3` RX.
- Start bytes: `0xAA 0x55`.
- CRC: CRC16-CCITT, polynomial `0x1021`, initial value `0xFFFF`.
- Byte order: little endian for all `uint16` fields.

The CRC is computed over the payload only, not over the start bytes.

## Rate Note

The full group report describes a ground-station visualization pipeline designed around 100 Hz telemetry. This cleaned STM32 firmware currently defines:

```cpp
constexpr uint32_t kTelemetryUpdateHz = 10;
```

The frame structure remains compatible with the wider architecture, but the transmit rate in this checkout is conservative.

## Fixed-Point Encoding

Unsigned angles use centi-degrees:

```text
raw = round(degrees * 100)
degrees = raw / 100
```

Valid unsigned angle range:

```text
0.00 deg -> raw 0
360.00 deg -> raw 36000
```

Signed angular rates use the compatibility mapping used by the lab bridge:

```text
0.00..180.00 deg/s     -> raw 0..18000
-0.01..-180.00 deg/s   -> raw 18001..36000
```

To decode a signed field:

```text
if raw <= 18000:
    signed_centi = raw
else:
    signed_centi = 18000 - raw

value_deg = signed_centi / 100
```

The STM32 converts attitude targets to radians internally after decoding.

## Command Frame

Direction: ESP32 to STM32.

Frame length: 7 bytes.

```text
AA 55 | type value_lo value_hi | crc_lo crc_hi
```

Payload:

| Byte | Field | Meaning |
| ---: | --- | --- |
| 0 | `type` | Command selector |
| 1 | `value_lo` | Low byte of encoded value |
| 2 | `value_hi` | High byte of encoded value |

Command types:

| Type | Meaning | Units on wire | STM32 interpretation |
| ---: | --- | --- | --- |
| 0 | Roll attitude target | unsigned centi-degrees | radians |
| 1 | Pitch attitude target | unsigned centi-degrees | radians |
| 2 | Yaw attitude target | unsigned centi-degrees | radians |
| 3 | Yaw-rate target | signed centi-deg/s mapping | rad/s |

The STM32 stores valid commands in a mailbox. A new command updates only the selected axis or mode.

## Telemetry Frame

Direction: STM32 to ESP32.

Frame length: 16 bytes.

```text
AA 55 | roll pitch yaw roll_rate pitch_rate yaw_rate | crc_lo crc_hi
```

Each payload field is a `uint16`, little endian.

| Field | Encoding |
| --- | --- |
| `roll` | unsigned centi-degrees, wrapped to `0..360` |
| `pitch` | unsigned centi-degrees, wrapped to `0..360` |
| `yaw` | unsigned centi-degrees, wrapped to `0..360` |
| `roll_rate` | signed centi-deg/s mapping |
| `pitch_rate` | signed centi-deg/s mapping |
| `yaw_rate` | signed centi-deg/s mapping |

The ESP32 bridge can translate this payload into a telemetry JSON object for MQTT and the web dashboard.

## Parser Behavior

The parser is a streaming state machine:

1. Wait for `0xAA`.
2. Wait for `0x55`.
3. Read the fixed payload length.
4. Read CRC low byte.
5. Read CRC high byte.
6. Validate CRC.
7. Update the command mailbox only if the frame is valid.

Malformed frames are dropped and the parser resynchronizes on the next start bytes. The parser does not block the main control loop.

## External MQTT Context

The full system report describes the ESP32 as the bridge between this binary UART protocol and an MQTT-based ground station.

Typical external topics from the report:

| Topic | Direction | Purpose |
| --- | --- | --- |
| `floatsat/telemetry` | ESP32 to backend | Publish decoded telemetry JSON |
| `floatsat/commands` | Backend to ESP32 | Send operator commands |

Telemetry uses fire-and-forget behavior in the external pipeline, while commands are treated as more reliability-sensitive. This distinction belongs outside the STM32 firmware; the STM32 only receives validated UART command frames.

## Example Command Payloads

Set yaw attitude target to 45 degrees:

```text
type = 2
value = 4500
```

Set yaw-rate target to -10 deg/s:

```text
type = 3
value = 19000
```

The full byte frame must include the start bytes and CRC. Use `uart_link.cpp` as the reference implementation for CRC calculation and fixed-point conversion.
