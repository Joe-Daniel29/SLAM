# Serial Protocol — Pi ↔ Arduino Communication

## Overview

The Raspberry Pi and Arduino communicate over UART serial (`/dev/ttyAMA0` on the Pi, pins 0/1 on the Arduino) at 115200 baud. Two packet types flow between them.

## Pi → Arduino: Motor command packet (6 bytes)

```
Byte:   [0]    [1]    [2]       [3]        [4]       [5]
Data:   0xAA   0x55   left_i8   right_i8   checksum  0x0D
        ────   ────   ───────   ────────   ────────  ────
        header header  signed    signed     sum of   tail
                       -127→127  -127→127   [0..3]
```

- **Header** (0xAA 0x55): Magic bytes to identify packet start. The receiver scans the byte stream for this pattern.
- **left_i8 / right_i8**: Signed 8-bit motor speed. Positive = forward, negative = backward. Range: -127 to +127.
- **Checksum**: Sum of bytes [0] through [3], masked to 8 bits (`& 0xFF`). Simple error detection.
- **Tail** (0x0D): End marker.

On the Arduino, the signed byte is scaled to PWM:
```
PWM = map(cmd_byte, -127, 127, -255, 255)
```
Values below ±15 PWM are treated as deadband (set to 0) to avoid motor whine.

## Arduino → Pi: IMU packet (16 bytes)

```
Byte:   [0]    [1]    [2]    [3]    [4]    [5]    [6]    [7]
Data:   0xBB   0x66   ax_h   ax_l   ay_h   ay_l   az_h   az_l
        ────   ────   ──── accel ────────────────────────────
        header header         X          Y          Z

Byte:   [8]    [9]    [10]   [11]   [12]   [13]   [14]      [15]
Data:   gx_h   gx_l   gy_h   gy_l   gz_h   gz_l   checksum  0x0D
        ──── gyro ──────────────────────────────    ────────  ────
              X          Y          Z               sum[0..13] tail
```

- Each sensor value is a 16-bit signed integer sent big-endian (high byte first)
- **Accelerometer** (ax, ay, az): Raw counts. At ±2g: divide by 16384 for g's, multiply by 9.80665 for m/s²
- **Gyroscope** (gx, gy, gz): Raw counts. At ±250°/s: divide by 131 for °/s
- **Checksum**: Sum of bytes [0] through [13], masked to 8 bits
- Sent at ~50 Hz (every 20 ms)

### Reassembling int16 from two bytes

```cpp
int16_t value = (high_byte << 8) | low_byte;
```

The cast to `int16_t` is important — it interprets the bit pattern as signed (two's complement), so values above 0x7FFF become negative.

## Future: Encoder packet (10 bytes) — not yet implemented

```
Byte:   [0]    [1]    [2..5]          [6..9]           [10]      [11]
Data:   0xCC   0x77   left_ticks_i32  right_ticks_i32  checksum  0x0D
```

Reserved for when quadrature encoders are added.

## Synchronization and framing

Serial is a continuous byte stream with no inherent packet boundaries. Both sides use a sliding-window approach:

**Arduino (receiving motor commands):**
```
Wait for 0xAA → expect 0x55 next → buffer 4 more bytes → validate checksum
```
If any step fails, reset and wait for the next 0xAA.

**Pi (receiving IMU packets):**
```
Maintain a rolling buffer of recent bytes
When buffer has ≥16 bytes, check if the last 16 match: [0xBB][0x66]...[0x0D]
Validate checksum → parse if valid, otherwise keep scanning
```

## Why this protocol?

- **Simplicity**: Fixed-size packets with magic headers are easy to parse on an Arduino with limited RAM
- **Robustness**: Checksums catch corruption from electrical noise on UART lines
- **Deterministic**: No variable-length encoding or escape characters to worry about
- **Low latency**: At 115200 baud, a 16-byte packet takes ~1.4 ms to transmit

## Common gotcha

When uploading new sketches to the Arduino, **disconnect the Pi's TX line from Arduino pin 0** — otherwise the Pi's serial traffic interferes with the upload process.
