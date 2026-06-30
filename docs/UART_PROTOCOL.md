# UART Protocol

The UART link connects the STM32 firmware in this repository to an ESP32 bridge. The ESP32 side can then forward data to an MQTT server or ground-station application.

## Serial Settings

- Baud rate: `115200`.
- STM32 pins: `PA2` TX, `PA3` RX.
- Start bytes: `0xAA 0x55`.
- CRC: CRC16-CCITT, polynomial `0x1021`, initial value `0xFFFF`.
- Byte order: little endian for all `uint16` fields.

The CRC is computed over the payload only, not over the start bytes.

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

Signed angular rates use the same compatibility mapping used by the lab bridge:

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

The STM32 keeps the latest command values in a mailbox. A new command updates only the selected axis/mode.

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

## Parser Behavior

The parser is a streaming state machine:

1. Wait for `0xAA`.
2. Wait for `0x55`.
3. Read the fixed payload length.
4. Read CRC low byte.
5. Read CRC high byte.
6. Validate CRC.
7. Update the command mailbox only if the frame is valid.

Malformed frames are dropped and the parser resynchronizes on the next start bytes.
