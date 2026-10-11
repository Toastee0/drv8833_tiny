# drv8833_tiny

Drive a micro leadscrew stepper (the tiny 2-phase kind with a ~5 mm screw and a
plastic nut) from a Seeed XIAO ESP32-S3 and a DRV8833, and jog it from a
Windows GUI.

- `main/` – ESP-IDF firmware. Half-steps the motor and takes G-code-style
  commands over the XIAO's native USB port.
- `gui/` – **Drv8833tinyNC**, a Rust/egui control panel: DRO, jogging, homing,
  soft limits, mm units and an A↔B bounce test.

## Wiring

| XIAO pin | GPIO | DRV8833 | Motor   |
|----------|------|---------|---------|
| D0       | 1    | AIN1    | coil A  |
| D1       | 2    | AIN2    | coil A  |
| D2       | 3    | BIN1    | coil B  |
| D3       | 4    | BIN2    | coil B  |

Tie DRV8833 `nSLEEP` high. These motors have low coil resistance and the
DRV8833 has no current limit without sense resistors, so keep `VM` around
3–5 V.

## Firmware

Requires ESP-IDF (built with v6.1).

```
idf.py set-target esp32s3
idf.py -p COMx flash
```

The first flash over a board running other firmware may need download mode:
hold **B**, plug in, release. If the board stays silent after flashing, reset
it with:

```
python -m esptool --chip esp32s3 -p COMx --before no-reset --after watchdog-reset chip-id
```

### Protocol

115200 8N1 over USB (baud is ignored by the USB-Serial-JTAG port). Units are
half-steps; every line is answered with `ok` or `error: ...`.

| Command            | Meaning                                         |
|--------------------|-------------------------------------------------|
| `G90` / `G91`      | absolute / relative moves                       |
| `G0 X<n> [F<n>]`   | move; `F` is feed in steps/s (1–1000). `G1` too |
| `G92 [X<n>]`       | set current position (default 0)                |
| `M17` / `M18`      | hold coils when idle / release coils            |
| `$`                | list commands                                   |
| `?`                | status, no newline needed: `<Idle\|MPos:0\|F:200\|Hold:0\|Abs:1>` |
| `!`                | stop immediately, no newline needed             |

Words on one line are applied left to right, e.g. `G91 G0 X10 F300`.

## GUI

```
cd gui
cargo run --release
```

Pick the port and connect, then **Home to motor**: it creeps into the motor
face and sets X0 there. Soft limits, the A↔B test and the **X0**/**Far end**
buttons need a homed axis. Travel, limits and direction live under
**Axis setup**; the defaults (700 half-steps ≈ 5 mm, limits 0–680) match the
motor this was built for, so measure yours and adjust.

The MDI box sends raw firmware commands, which bypass soft limits and the
direction flip.
