# Hardware test log — ESP32 UART protocol v2

## Test environment

- Firmware: `robot_uart_bench_v2` (exact commit and build options not recorded)
- Host: Linux, Python 3, `tools/uart_bench/` scripts
- UART port: `/dev/ttyUSB0`
- Hardware: ESP32 + robot controller; motor power status not recorded
- Date: not recorded (results reported in chat)

## UART-ODOM-001 — PASS (with timing observation)

Command:

```bash
python3 tools/uart_bench/check_odometry.py --port /dev/ttyUSB0 --duration 10
```

Reported output:

```text
Odometry frames=501, mean rate=50.07 Hz, max gap=50.7 ms, decode errors=0, overflow=0
Latest [x,y,heading,v,w,left,right]: (-0.01118, 0.00108, -0.19231, 0.0, 0.0, -0.0, 0.0)
```

Result: odometry stream decodes successfully; mean rate approximately matches 50 Hz target. User manually rotated right wheel during capture, so nonzero position and heading are expected. Maximum receive gap of 50.7 ms merits investigation; host receive timestamps alone cannot identify ESP32 scheduling jitter.

Attachment: `odometry.csv` was shared separately; not bundled in this project archive.

## UART-CONFIG-001 — PASS (read-only)

Command:

```bash
python3 tools/uart_bench/check_config.py --port /dev/ttyUSB0
```

Reported output:

```text
PID: KpL,KiL,KdL,KpR,KiR,KdR = (0.951, 5.475, 0.0, 1.426, 10.009, 0.0)
```

Result: PID read request received a syntactically valid response and host decoder extracted six coefficients. This does not yet verify that coefficients match NVS or effective controller state, nor does it test writing settings.

## Pending tests

- Malformed frames, argument bounds, and error responses (motors unpowered)
- ESP32 timestamp / frame sequence for distinguishing device timing from host USB/UART batching
- Explicit zero-PWM stop, watchdog, motor movement (only with wheels safely suspended)
- NVS write/read persistence and power-cycle validation (later stage)

## General safety note

No dedicated hardware motor-disable line is available. A software stop or zero setpoint does not guarantee motor power isolation after controller failure. Do not test active wheel motion on the ground.

## UART-PARSER-001 — PENDING

Test script: `tools/uart_bench/check_malformed.py` (non-motion, no write commands).

```bash
python3 tools/uart_bench/check_malformed.py --port /dev/ttyUSB0
```

Acceptance: baseline odometry + PID response present; after malformed read-only/unknown frames odometry continues, read-only PID still responds with identical values, and host reports zero decode errors. Note: absence of ACK/NACK means this is a recovery smoke test, not exhaustive proof that each bad frame was rejected internally.

## UART-PARSER-001 — PASS (reported by operator)

`check_malformed.py` output: baseline odometry 115 frames; recovery 137 frames;
zero host decode errors both phases. PID before and after identical:
`(0.951, 5.475, 0.0, 1.426, 10.009, 0.0)`.
This verifies recovery, not internal per-command rejection.

## MOTOR-SAFETY-001 — PENDING (new firmware)

1. Disconnect **motor power** first. Build and flash modified ESP32 firmware.
2. Run `python3 tools/uart_bench/check_stop.py --port /dev/ttyUSB0`.
3. Only after passing and confirming correct hardware wiring: securely suspend
   wheels, prepare a physical power cutoff and perform a manual low-speed STOP
   test under supervision. This project does not ship an automated motor spin test.

**PASS of `check_stop.py` proves only protocol continuity**, not actual PCA9685
register state or torque removal. Do not mark physical STOP verified without direct
observation or hardware measurement.

## MOTOR-SAFETY-001 — UART PASS (reported by operator)

Firmware safety patch compiled and flashed. `check_stop.py` reported 100
odometry frames before STOP, 101 after STOP, unchanged six PID coefficients,
zero host decode errors, and PASS. This does **not** prove zero PWM or physical
motor stop.

## Batch suite — 11 runs on hardware (2026-11-02), firmware build unidentified

`tools/uart_bench/hardware_suite.py` runs the non-motion checks in one serial
session and writes timestamped JSON, CSV and Markdown reports. The archived
reports (`archive/reports/`) show that the suite **was executed on hardware**:
11 runs on 2026-11-02, port `/dev/ttyUSB0`, 115200 baud, 2.5 s per phase.

| Runs | Checks | Result |
|---|---|---|
| 10 (223041Z … 232318Z) | 7 | 7/7 PASS in every run |
| 1 (233445Z) | 8 | 8/8 PASS (adds `UART-I2C-DIAG-001`) |

No check failed in any of the runs. The PID coefficients read back were
identical in all of them: `[0.951, 5.475, 0.0, 1.426, 10.009, 0.0]`.

**What these runs do not prove.** The firmware build is not identified: the
commit and the build options were not recorded, and every run reports the
protocol version as `2;0` (`$501`), whereas the current firmware emits
`$501;2;3;#`. The runs therefore validate an **older** build and must not be
cited as validation of the current firmware. The suite itself states its
limits in every report (verbatim from the JSON): no physical PWM measurement,
host receive timestamps do not measure ESP32 scheduler jitter, no NVS writes
and no motor movement.

To close this gap, record before each run in this log: the firmware commit,
the build options and the `$501` response of the flashed image.

### v2.2.1 dispatch-only refactor — pending hardware validation

- Reference baseline: API v2.2, UART suite 7/7 PASS (user report).
- Changed: moved command IDs 210/211/402/501 from `.ino` into `command_dispatcher.cpp`.
- Protocol and control math unchanged. New source-level dispatcher coverage test added.
- Target firmware build: **NOT YET VERIFIED**.
- Post-refactor hardware suite: **NOT YET RUN**.
