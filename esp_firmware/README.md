# ESP32 UART bench — protocol v2

This package focuses on **ESP32 + Python UART bench tests**. ROS 2 is **not needed**; `serial_bridge_cpp_reference/` is retained only as source reference, not built or launched. `jetbot_stats.py` is unchanged and uses **Serial1**, a separate channel.

## Quick start (motors unpowered)

1. Back up the working firmware and calibration. Compile `esp32_firmware/esp32_firmware.ino` with the project’s Arduino ESP32 libraries (including matching `uPID`). Full target build has **not** been verified here.
2. Install Python dependencies: `python3 -m pip install -r tools/uart_bench/requirements.txt`.
3. Inspect UART device: `ls -l /dev/serial/by-id/`. The firmware’s main serial may appear as `/dev/ttyUSB0` or `/dev/ttyACM0`. Stop Serial Monitor/ROS bridge before opening the same port.
4. Run offline tests: `cd tools/uart_bench && python3 -m unittest -v test_protocol`.
5. Passive monitor: `python3 tools/uart_bench/monitor.py --port /dev/ttyUSB0 --duration 10 --csv odometry.csv`.
6. Odometry: `python3 tools/uart_bench/check_odometry.py --port /dev/ttyUSB0 --duration 10`.
7. Read PID (RAM only): `python3 tools/uart_bench/check_config.py --port /dev/ttyUSB0`.
8. Optional stop: `python3 tools/uart_bench/stop.py --port /dev/ttyUSB0`.

**Only after verifying above with wheels securely suspended**, use `motor_probe.py --port /dev/ttyUSB0 --confirm-wheels-lifted` with someone at the motor power switch. Motor power must NOT be enabled for initial steps. The probe sends low speed for up to one second by default and sends STOP in `finally`, but a program crash, USB fault, ESP32 freeze or PCA9685 fault can defeat software stop.

The UART scripts never write NVS and never start motors except for explicit `motor_probe.py`. A serial write succeeding does not imply the ESP32 accepted a command; protocol v2 does not yet provide ACK/NACK.

**Known limitations:** See `TODO.md` and `PROTOCOL.md`; no hardware kill/enable, no end-to-end checksum or transactional ACK, watchdog semantics, I2C timing, and full target build not tested.

## Safety patch: explicit software STOP (MOTOR-SAFETY-001)

`MotorController::emergencyStop()` clears speed targets, PI integrals,
LADRC observer state, reference filter, open-loop duty, and explicitly writes
zero to all four PWM direction pins. On timeout or hardware-health fault,
the control loop enters a stopped state until a **new valid motion command**;
settings requests do not restart the motors. `103` uses the same method.

This is a *software* stop only. It does not isolate motor power or guarantee
that I2C writes succeed. Never run active tests without a physical means of
cutting power and securely suspended wheels. The added `check_stop.py` sends
only STOP/read-only commands and does not prove that a motor is physically off.

Run on a newly flashed board, with motor power disconnected:

```bash
python3 tools/uart_bench/check_stop.py --port /dev/ttyUSB0
```

## Batched safe hardware test (no ROS 2)

The single-port `hardware_suite.py` performs baseline odometry reception, a read-only
PID request, optional malformed-frame recovery, and STOP UART continuity. It **never**
sends nonzero motion commands or writes NVS. Both ESP32 and motors share power;
keep the wheels suspended and the shared power cutoff accessible even for these
non-motion tests. Stop other clients of the same serial port first.

```bash
python3 tools/uart_bench/hardware_suite.py --port /dev/ttyUSB0 --duration 2.5 --output reports/
```

Reports are timestamped JSON, CSV, and Markdown in `reports/` (add them to git
only when desired). A FAIL exits 1; device/port errors exit 2. To avoid
resending deliberately malformed frames, add `--no-malformed`.

This suite **does not** measure PCA9685 PWM pins, motor torque, ESP32 scheduler
jitter, hardware watchdog behavior, or NVS persistence. No motor motion test
is included. Full ESP32 build and hardware execution of this suite remain to
be performed by the operator.

### v2.1 hardware regression

After flashing the updated firmware, run the same `hardware_suite.py` command. It now also queries `501` and `402`. This extension does not exercise motion or NVS writes. Firmware build and hardware behavior are unverified in this release; validate on your setup.

## v2.2 safe bench suite

After flashing v2.2 and updating tools, run:

```bash
python3 tools/uart_bench/hardware_suite.py --port /dev/ttyUSB0 --duration 2.5 --output reports/
```

The new `UART-TXCONFIG-001` sends a sequenced *read* of PID and one deliberately invalid PID-RAM update (`KpL=-1`) to verify `503 NACK`. No valid gain modification or NVS write occurs. **No motor commands are sent** except STOP. Expected version is `2.2`. Prior hardware validation (6/6) was on v2.1, not v2.2.

## v2.2.1 — single command dispatcher

All validated main-UART commands, including IDs 210, 211, 402 and 501, now flow
through `dispatchCommand()` in `command_dispatcher.cpp`. The `.ino` main loop
retains framing, shape validation, motion-watchdog gating, and diagnostic
counters; no longer contains a second command router. `CommandContext` holds
non-owning references to the diagnostic counters/device flags used by those
handlers. The on-wire protocol and controller parameters are unchanged.

Offline regression: `python3 -m unittest discover -s tools/uart_bench -p 'test_*.py' -v`.
The new source-level dispatch coverage test checks enum/switch exhaustiveness,
but is **not** a substitute for compiling for ESP32. Run the established safe
`hardware_suite.py` after flashing, and confirm the previous 7/7 PASS baseline.

## v2.2.2 — configuration and diagnostics services

`CommandDispatcher` now routes configuration commands to `config_service.cpp` and
version/health queries to `diagnostics_service.cpp`. The service bodies have been
moved without changing their command logic, controller math, UART formats or
storage semantics. Existing `CommandContext` is intentionally retained as the
non-owning dependency bundle for this incremental refactor.

No automatic ESP32 compile/build check or CI build job has been added, per the
project workflow. Compile manually in Arduino IDE and run the established
7-check safe UART suite after flashing. The offline Python unit tests are
still available for protocol/source regressions.

Known caveat: legacy configuration commands with optional `save` flags still
write to NVS when explicitly requested; the **safe bench suite never sends
such commands**. Do not change parameters during this regression check.

## v2.2.3 — service context boundaries

`CommandContext` is now a lightweight aggregate of `MotionContext`,
`ConfigContext`, and `DiagnosticsContext`. Each service receives only its
relevant non-owning references. UART protocol 2.2, control algorithms, NVS
behavior, and the safe UART test suite are unchanged.

This release intentionally **does not** add automatic ESP32 compilation.
Run the normal manual Arduino build and then `tools/uart_bench/hardware_suite.py`
on real hardware. Python unit tests do not prove the ESP32 build succeeds.

## v2.2.4 — UART service and scheduling boundaries

`serial_service.h/.cpp` now owns main-UART frame dispatch and the separate
Serial1 Wi-Fi-display frame handoff. It keeps the original order: poll both
receivers, evaluate safety, run control and odometry, then process up to four
main frames and one secondary frame. No command IDs or response formats changed.

`control_scheduler.h` contains the existing due-time decisions for the 100 Hz
control loop, 50 Hz odometry, and I2C window. It does **not** introduce new
threads, change periods, or catch up missed control cycles. The actual control
and odometry work remain in `esp32_firmware.ino` for this increment.

Manual Arduino compile and flashing are required; no automatic firmware build
was added. Re-run `hardware_suite.py` on the actual ESP32 (7/7 expected).
Treat the firmware as unvalidated until that test is confirmed.

## v2.2.4 linker build-fix

`serial_service.cpp` no longer includes `config.h`, which contains global variable definitions. The minimum battery voltage is passed as a value through `SerialServiceContext`. This avoids duplicate definitions at link time without changing protocol or control logic. Target-board compilation and hardware validation remain to be completed.

## v2.2.5 — single ownership of mutable configuration

`config.h` now declares 18 mutable runtime parameters via `extern`; their
existing exact default values are defined once in `esp32_firmware/config.cpp`.
Compile and link `config.cpp` alongside the sketch (Arduino IDE automatically
compiles `.cpp` files next to the `.ino`). Physical constants and timers stay
in `config.h`. NVS loading and runtime writes still target the same variables.
No UART API, controller, scheduler, or storage logic changes.

No automatic ESP32 build was added. Compile/flash manually and run the existing
7-case safe UART suite. The included Python test checks source declarations,
not the target toolchain/linker.

## v2.2.6 — control and odometry service

`control_loop_service.h/.cpp` now owns the original 100 Hz speed-estimation,
controller update and telemetry, plus the 50 Hz encoder-distance odometry.
`loop()` continues to schedule both with `ControlScheduler` and preserves
UART polling, watchdog, command processing, and I2C ordering. Mutable
configuration remains in `config.cpp`; no firmware auto-build was introduced.
Manual board compilation and the same seven safe UART tests are required
before tagging this version.

### v2.2.7 HardwareMonitor

Extracted the existing INA219 sample and I2C device monitoring into `hardware_monitor.h/.cpp`. The main loop still gates these operations on the same scheduler window and interval and preserves the order relative to the display and safety handling. Original reinitialization logic and error thresholds remain unchanged. Manual ESP32 compile and safe UART bench are required before tagging. No automatic build validation is added.


### v2.2.8: SafetyManager extraction

The unchanged watchdog/STOP decision block is now in `safety_manager.h/.cpp`, with non-owning references to the existing motion-command timer, latch, counter and telemetry flag. `SerialService` continues to refresh the same timer and clear the same latch. `loop()` invokes `safetyManager.check()` at exactly the previous location, immediately after UART polling. UART protocol, controller, I2C, PWM and control periods remain unchanged. Manual ESP32 build and safe hardware UART suite are required before tagging.
