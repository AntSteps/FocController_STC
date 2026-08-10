# Fast Control Protocol

## 1. Purpose

The fast protocol is a compact RS485 control path for an MCU master. It coexists
with the full host-PC protocol and does not replace parameter, calibration, or
debug frames.

- UART: 1 Mbps, 8N1
- Multi-byte integers: big-endian
- CRC: CRC-8, polynomial `0x07`, initial value `0x00`, no reflection, no final XOR
- Device address: the same `DEVICE_ID` used by the full protocol
- Fast commands do not use sequence numbers or ACK frames

## 2. Fast Ctrl Request

Header: `0x1545`

Fixed length: 7 bytes

| Offset | Field | Type | Size | Description |
| ---: | --- | --- | ---: | --- |
| 0 | head | uint16 | 2 | `0x1545` |
| 2 | motor_id | uint8 | 1 | Target device ID |
| 3 | control | uint8 | 1 | Command and reply-request bit |
| 4 | value | int16/uint16 | 2 | Meaning depends on command |
| 6 | crc8 | uint8 | 1 | CRC over bytes 0 through 5 |

### 2.1 Control byte

| Bits | Name | Description |
| --- | --- | --- |
| 1:0 | command | Fast command type |
| 6:2 | reserved | Must be zero |
| 7 | request_status | Return one Fast Status after applying the command |

Commands:

| Value | Command | Value encoding | Reply behavior |
| ---: | --- | --- | --- |
| 0 | STOP | Must be zero | Optional Fast Status |
| 1 | SET_SPEED | Signed speed Q6 | Optional Fast Status |
| 2 | START_CALIBRATION | Must be zero | Never replies |
| 3 | SET_OUTPUT_LIMIT | Unsigned milliamps | Optional Fast Status |

For `START_CALIBRATION`, `request_status` is ignored. Fast status polling during
calibration is not supported.

### 2.2 Speed encoding

`SET_SPEED` uses a signed 16-bit value:

```text
speed_rad_s = raw / 64
raw = round(speed_rad_s * 64)
```

The accepted protocol range is `-32000` through `32000`, corresponding to
`-500` through `500 rad/s`. The resolution is `0.015625 rad/s`.

The configured `velocity_limit` is applied after decoding. A valid value beyond
that configured limit is clamped and reported through the Fast Status flag when
a status reply is requested.

### 2.3 Runtime output limit

`SET_OUTPUT_LIMIT` interprets `value` as unsigned milliamps:

```text
runtime_limit_A = value / 1000
```

The command applies a symmetric runtime model-current limit:

```text
runtime_out_max = +runtime_limit_A
runtime_out_min = -runtime_limit_A
```

The runtime limit:

- does not modify EEPROM;
- does not reset the speed PI;
- cannot exceed the configured `out_max/out_min` safety envelope;
- returns to the configured envelope after device reset or controller reinitialization.

This is a q-axis model-current limit, not a DC bus-current limit.

## 3. Fast Status Reply

Header: `0x0C21`

Fixed length: 13 bytes

| Offset | Field | Type | Size | Encoding |
| ---: | --- | --- | ---: | --- |
| 0 | head | uint16 | 2 | `0x0C21` |
| 2 | motor_id | uint8 | 1 | Replying device ID |
| 3 | status | uint8 | 1 | Compact status flags |
| 4 | velocity | int16 | 2 | Signed Q6, `raw / 64 rad/s` |
| 6 | iq_estimate | int16 | 2 | Signed milliamps |
| 8 | position | uint16 | 2 | Corrected mechanical angle per turn |
| 10 | bus_voltage | uint16 | 2 | Millivolts |
| 12 | crc8 | uint8 | 1 | CRC over bytes 0 through 11 |

### 3.1 Status flags

| Bit | Name | Description |
| ---: | --- | --- |
| 0 | MOTOR_ENABLED | Speed-control mode is enabled |
| 1 | COMMAND_CLAMPED | The command value was clamped |
| 2 | OUTPUT_SATURATED | Current or voltage output reached a limit |
| 3 | ENCODER_FAULT | Encoder data or magnetic field is invalid |
| 4 | DRIVER_FAULT | Driver nFAULT is active |
| 5 | BUS_UNDERVOLTAGE | Bus voltage is below the control threshold |
| 6 | MOTOR_FAULT | Reserved motor fault summary |
| 7 | reserved | Always zero |

`COMMAND_CLAMPED` describes the command that requested this reply.

`velocity` is valid while speed control is enabled. Entering `STOP` resets the
local speed observer, so a status reply requested by `STOP` reports zero speed
even if the shaft is still coasting mechanically. Corrected position remains
available in `STOP`.

### 3.2 Estimated current

The reported current is inferred from the final applied q-axis voltage:

```text
iq_estimate = (uq_applied - Ke * measured_speed) / R
```

It is a model estimate, not a current-sensor measurement.

### 3.3 Corrected mechanical position

Position uses the same corrected mechanical angle consumed by the speed
observer. It includes direction, electrical offset, and first/second harmonic
correction, normalized to `[0, 2*pi)`:

```text
position_raw = corrected_angle_rad * 65536 / (2*pi)
corrected_angle_rad = position_raw * (2*pi) / 65536
```

## 4. Command behavior

- A valid `SET_SPEED` applies the speed target and refreshes the control watchdog.
- `STOP` immediately sets target speed to zero and disables motor output.
- `SET_OUTPUT_LIMIT` changes only the runtime output envelope.
- `START_CALIBRATION` starts the existing automatic calibration service and never replies.
- Invalid headers, CRCs, IDs, reserved bits, or command values are ignored.
- Full-protocol requests and replies retain their existing ACK and sequence behavior.

The current firmware watchdog timeout is 1000 ms. A real-time deployment should
select its command period with adequate margin and may shorten this timeout in a
separate safety configuration change.

## 5. Request examples

The following complete frames include CRC:

```text
SET_SPEED 100 rad/s, no reply:
15 45 03 01 19 00 8D

SET_SPEED 100 rad/s, request Fast Status:
15 45 03 81 19 00 86

SET_OUTPUT_LIMIT 1.500 A, request Fast Status:
15 45 03 83 05 DC E1

START_CALIBRATION:
15 45 03 02 00 00 DA

STOP:
15 45 03 00 00 00 0C
```

For a one-shot runtime-limit update, the master should set `request_status` and
wait for Fast Status before resuming periodic speed commands. This confirms that
the distinct limit command has been consumed without introducing an ACK type.
