# Outstanding items (not silently fixed)

## Priority integration/safety
- No hardware enable/kill switch; zero setpoint is not assured safe shutdown under CPU/driver failure. Consider physical power isolation.
- Settings confirmation is partial: `$210` has a request ID and ACK/NACK (`$502`/`$503`) since v2.2, but the legacy setters `$201`/`$204` still answer nothing and no persistence confirmation exists. The `5xx` range stays reserved.
- ~~Add full input range checking before refreshing movement watchdog~~ — closed by owner decision (R-07, 2026-11-11): non-finite motion values are rejected and do not refresh the watchdog, but finite out-of-range values are still accepted and do refresh it. The contract is documented in `esp_firmware/PROTOCOL.md` ("Motion watchdog").
- Host-side coverage of the v2 mappings exists (`esp_firmware/tests/`, `esp_firmware/tools/host_test/`) and CI runs it (`.github/workflows/ci.yml`: host-tests, python-tests, protocol-guard, firmware-syntax). Still open: ROS 2 colcon integration and an ESP32 compilation gate — the latter is blocked by the missing `uPID` library (R-12, see `docs/ai/project-state.md`).
- The ROS node currently emits generic raw frames on `serial/cmd` for configuration; typed configuration services are a proposed next step.

## Deferred
- Nonblocking I2C timing and recovery and missing-device policy.
- Configuration validity and NVS defaults consistency.
- Profile control-loop jitter on hardware and test motor PWM fail-safe after failed PCA9685 transactions.
- `jetbot_stats.py` obtains **and transmits Wi-Fi passwords in plaintext** over Serial1 and writes them to stdout; assess whether OLED needs password display at all. Keep the second channel isolated until explicitly redesigned.
- ~~Tooling under the original `tools/` directory requires v2 command-ID migration~~ — done: `esp_firmware/tools/*.py` use the v2 IDs (`$102`–`$104`, `$202`–`$209`, `$301`, `$401`, `$403`, `$404`) and the ROS 2 bridge `serial_bridge_cpp` is on v2 as well.

- UART bench testing is the active workflow; ROS 2 integration remains unverified and deferred.
- Software STOP ID 103 only sets zero speed target; direct zero PWM has not been independently verified.

- Transactional ACK/NACK and request IDs exist since v2.2, but only for `$210`. Still open: acknowledging *applied* settings for the legacy setters (`$201`/`$204`, where an out-of-range PID is silently dropped and still counted as accepted) and NVS failure handling.
- `402` diagnostics count structural rejects, not malformed frame overflows and not semantic invalidity inside dispatch handlers: a `$201`/`$204` frame with an out-of-range PID is counted as accepted and then silently dropped by the handler. Add separate parser/validation metrics, or confirm the applied values by reading `$202` back on the host side.
- Safety: watchdog timing and physical zero-PWM still need hardware verification; no separate motor-power cutoff exists.

- v2.2: NACK for malformed frames with unusable `request_id` is impossible; add envelope-level framing, checksum and request IDs when redesigning wire transport.
- v2.2: transactional NVS operations not implemented; existing legacy configuration setters still lack acknowledgements.
- v2.2: ack for 210 reports software apply, not motor or Flash verification; validate on hardware.

## Architectural follow-ups

- ~~`CommandContext` still contains many references~~ — done: the context is split into service-specific ones (`SerialServiceContext`, `SafetyManagerContext`, `StartupServiceContext`, `ConfigContext`, …) and `command_context.h` no longer exists.
- Existing config setters are not all transactional; align ACK/NACK and validation in a later API revision.
- Full target compilation and device behavior of the current branch are not verified: the host suite runs on the PC, but the ESP32 build and the hardware suite need a board. Compile and flash manually, then run `hardware_suite.py` before marking anything as validated.
- No automated ESP32 compilation gate: the decision to add one was never revisited, and it is blocked anyway — 11 of 18 firmware targets are skipped by `syntax_check.sh` because `uPID` is unavailable (R-12).

### Hardware verification checklist
- [ ] Manually compile and flash ESP32 firmware.
- [ ] Run hardware_suite.py; expect 7/7 PASS and unchanged PID. The suite checks the protocol version `$501` = `2;3`.
- [ ] Physical PWM zero, timing jitter and active movement remain unverified.

## Reference documents

- Current architecture audit: `docs/audits/ARCHITECTURE_AUDIT_v2.4.0.md` — findings A-01…A-16 with file:line evidence.
- Project state, risks and accepted decisions: `../docs/ai/project-state.md`, `../docs/ai/decisions.md`.
- `../tools/ci/guard.py` — repository guard (protocol IDs, parser comments, `$501` version copies, hygiene); it runs in the `protocol-guard` CI job.
