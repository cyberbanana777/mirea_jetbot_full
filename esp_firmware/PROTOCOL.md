# Main UART protocol v2 (BREAKING CHANGE)

This is the sole protocol supported on the main ESP32 Serial <-> Jetson ROS 2 `serial_bridge_cpp` channel. **No negotiation, no v1 aliases.** Serial1 + `jetbot_stats.py` is a separate display/network-info path and is not modified.

## Framing

ASCII: `$<id>;<arg1>;...;#` (optional CR/LF after `#`). `id` is parsed with `strtol` and must be a plain decimal integer; fields are parsed with `strtof` and must be finite decimal numbers (`.` is the decimal point). Max input fields on ESP32: 7 — a frame carrying an 8th argument is rejected as malformed. No checksum.

A fully received frame that is malformed (bad syntax, non-numeric or non-finite field, wrong number of arguments) or carries an unknown ID is **not applied** and has no side effect. It is counted in the `rejected_commands` diagnostic counter, as is a motion frame that passes the structural check but is rejected by the safety or value check (see «Safety»). Note that only completely received frames are counted: UART receiver overflow and unfinished frames are not included in that counter. `accepted_commands` counts frames that passed the structural check and were dispatched; it does **not** mean that the settings were actually applied.

ACK/NACK is supported only for transactional 210; legacy-style setting commands still have no confirmation.

### Frame length limits

The ESP32 receive buffer is 128 bytes (`SerialReceiver::BUFFER_SIZE`) and a frame
is accepted only if it fits completely, `$` and `#` included: **the maximum
accepted frame length is 127 bytes**. A longer frame overflows the buffer and is
dropped silently — no `rejected_commands` increment and no response. The receiver
resynchronizes on the next `$`.

The longest command in the «Jetson -> ESP32» table is `$210` (7 fields). With the
widest numeric representation a sender can produce (`format(v, '.9g')` for a
float64 — 16 characters) it occupies **125 bytes**, so the margin is 2 bytes.
With the `.5f` format used by `web_regulators_configurator` (|v| <= 1000) the same
frame is 90 bytes. Every other command is shorter.

The opposite direction has a different limit: `$405` is produced by the firmware
and travels to the Jetson, where the receiver accepts up to 192 bytes
(`uart_bench.Framer`) and the bridge parser up to 256 bytes (`FrameParser`).
With saturated 32-bit counters `$405` is 140 bytes including CRLF, so it fits.

Both limits are checked automatically: `esp_firmware/tests/test_frame_limits.cpp`
compares the maximum length of every command with the buffer limit and with this
table, and `esp_firmware/tools/uart_bench` refuses to encode a command longer
than 127 bytes. A change of the numeric format or of the number of fields in a
command must come with an update of that test.

## Jetson -> ESP32

| ID | Argument list | Meaning |
|---|---|---|
|101|v_m_s, omega_rad_s|robot velocity /cmd_vel|
|102|left_m_s, right_m_s|wheel targets|
|103|none|software stop (zero wheel speeds)|
|104|left_duty, right_duty|open-loop calibration; range checking remains a follow-up|
|201|kpL,kiL,kdL,kpR,kiR,kdR|PID RAM only|
|202|none|request PID values|
|203|wc,wo,b0L,b0R[,ff[,save]]|set LADRC parameters|
|204|kpL,kiL,kdL,kpR,kiR,kdR|PID + NVS save|
|205|gainL,deadzoneL,gainR,deadzoneR[,save]|feedforward|
|206|none|request FF/filter/ADRC state|
|207|tau_s[,save]|reference filter|
|208|mode[,save]|0 PI / 1 LADRC|
|209|none|request LADRC state|
|210|request_id,kpL,kiL,kdL,kpR,kiR,kdR|transactional PID set (RAM only), ACK/NACK|
|211|request_id|request PID values with a request id (response `$211`)|
|402|none|request diagnostic snapshot (response `$402`)|
|403|enabled|toggle 100 Hz diagnostic stream (odometry paused while enabled)|
|404|[min_edges[,aligned]]|speed estimator window request/change|
|405|none|request read-only I2C diagnostic snapshot (response `$405`)|
|501|none|request protocol version (response `$501`)|

## ESP32 -> Jetson

| ID | Values | Meaning |
|---|---|---|
|301|x_m,y_m,heading_rad,v_m_s,w_rad_s,left_m_s,right_m_s|odometry, 50 Hz when diagnostics off|
|202|kpL,kiL,kdL,kpR,kiR,kdR|PID response|
|205|gainL,deadzoneL,gainR,deadzoneR|FF response|
|207|tau_s|filter response|
|203|mode,wc,wo,b0L,b0R,ff|ADRC response|
|401|time_us,refL,velL,dutyL,refR,velR,dutyR|diagnostics stream|
|404|min_edges,aligned|speed estimator response|
|211|request_id,kpL,kiL,kdL,kpR,kiR,kdR|transactional PID read response|
|402|uptime_ms,accepted_commands,rejected_commands,watchdog_stops,motor_stopped,ina_ok,pwm_ok|diagnostic snapshot|
|405|ina_state,ina_failures,ina_attempts,ina_recoveries,pwm_state,pwm_failures,pwm_attempts,pwm_recoveries,oled_state,oled_failures,oled_attempts,oled_recoveries|read-only I2C snapshot|
|501|major,minor|protocol version (not a firmware version)|
|502|request_id,command|ACK for a transactional command|
|503|request_id,command,error|NACK for a transactional command (error `1` = invalid PID)|

### Design decision

The `2xx` range is the **configuration family in both directions** (SET and GET response have the same ID in selected cases), rather than attempting to allocate strictly by UART direction. `3xx` is state telemetry, `4xx` is diagnostics, `1xx` motion. `5xx` is **already allocated**: `501` is protocol metadata (version), `502`/`503` are ACK/NACK for transactional commands. This is a transitional protocol contract, not yet a finalized transactional API.

### ROS 2

**Current state (as of the `fix/esp32_firmware` branch): the ROS-side migration to v2 is COMPLETE.** The `serial_bridge_cpp` package at the repository root is on **v2**:

- it emits `$101;<linear>;<angular>;#` for `/cmd_vel`;
- it accepts feedback frame type `301` (odometry, 8 fields) and type `202` (PID, 7 fields).

The v2 implementation was developed in a separate copy under `esp_firmware/serial_bridge_cpp_reference/`, then merged into the working package and the copy was removed (it declared the same package name `serial_bridge_cpp`, which would have collided with the working package). The merged code adds: `std::isfinite` checks in the frame parser (`nan` / `inf` are rejected), validation of the message ID (integer in `0..999`), an **exact** (not "at least") field count per frame type, and a `/cmd_vel` idle timer that sends `$103;#` (see below).

The protocol constants live in `serial_bridge_cpp/include/serial_bridge_cpp/protocol.hpp` and are shared with the host test that checks the contract with the firmware (`esp_firmware/tests/test_end_to_end_v2.cpp`).

`serial/cmd` remains a low-level raw-frame topic, so external publishers must use v2 IDs. `serial/feedback` keeps raw telemetry. Frames that the ESP32 sends but the bridge does not map to a ROS topic yet (`$203`, `$205`, `$207`, `$211`, `$401`, `$402`, `$404`, `$405`, `$501`, `$502`, `$503`) are logged at DEBUG level instead of being reported as errors; structured ROS publishers for diagnostics are a follow-up.

**The `/cmd_vel` idle timer.** The bridge stops the robot if no twist arrives on `/cmd_vel` for `cmd_vel_timeout` seconds (0.5 s by default, `0` disables it) and sends `$103;#`. The timer is edge-triggered: it fires once per silence period and is re-armed by the next `/cmd_vel` message. Raw motion frames from other publishers of `serial/cmd` (`$101`, `$102`, `$104`) cancel the timer, because those publishers are responsible for stopping the robot themselves. The ESP32 watchdog (1 s) remains the backstop: `$103` does not refresh it.

The user-facing tool `web_regulators_configurator` is on v2 as well. The former `simple_PID_configurator` (v1-only, GUI + ROS node) was removed, because `web_regulators_configurator` covers its functionality and works with a v2 firmware.

## Safety

There is NO hardware motor power disconnect/enable. Software stop sets a zero target but cannot guarantee physical braking or de-energization in the presence of processor/I2C/PCA9685 failure. Do not run unattended; bench test with wheels lifted first. A hardware enable / independent motor power cut is recommended for physical safety.

Motion watchdog updates only on structurally valid motion commands (IDs 101, 102, 104), not on queries. The order of checks in the receive path is: shape validation → safety/finite-value validation → **only then** the watchdog timestamp is refreshed. A command that is rejected by either of the first two checks therefore does **not** refresh the watchdog, and the motion timeout still applies.

**The watchdog also switches telemetry off.** When the host is silent for longer than the watchdog timeout (1000 ms, `emergency_timer_timeout` in `config.h`), the firmware performs an emergency stop and clears the telemetry flag: `safety_manager.cpp` writes `ctx_.telemetryEnabled`, which is a reference to the global `telemetry_on` — the same flag the control loop reads. `$401` frames therefore stop and the odometry frame `$301` takes their place. Telemetry is switched on again only by an explicit `$403;1;#`. A host that merely reads parameters (`$202`, `$206`, `$402`, …) does not refresh the watchdog, so after every silence period it has to send `$403;1;#` again. The bench tools in `esp_firmware/tools/` keep the watchdog alive with a zero motion command while waiting for telemetry.

**Current limitation (residual defect, not fixed):** the motion-value validation checks only the argument count and finiteness — it does **not** check the value range. A command with a finite but out-of-range value (for example `104;5.0;5.0;#`) is accepted, **does** refresh the watchdog, and is silently clipped later in the motor controller (`duty` to [-1, 1], wheel speeds to ±0.8·max wheel speed). Range checking remains a priority follow-up.

## v2.1 read-only API extension

- `$501;#` -> `$501;2;1;#` (protocol major/minor, **not** a firmware version).
- `$402;#` -> `$402;uptime_ms;accepted_commands;rejected_commands;watchdog_stops;motor_stopped;ina_ok;pwm_ok;#`.
- Counters reset at boot, may wrap after 2^32 increments. `accepted_commands` counts frames passing structural validation, **not successful application of settings**. `rejected_commands` counts fully received but structurally invalid/unknown frames; UART receiver overflow/unfinished frames are not included. `watchdog_stops` counts transitions into communication timeout, even if motors were already stopped.
- Replies are best-effort: responses may be omitted when the TX buffer lacks room. For this reason retry read-only queries with an application timeout.
- `ACK/NACK` for setting commands is **not yet implemented**. Do not mistake successful parsing for successful NVS writes.

Existing 101/103/202/301 IDs and payloads remain unchanged for hardware regression testing.

## v2.2 transactional PID (RAM only)

- `$501;#` → `$501;2;2;#` (protocol version).
- `$211;request_id;#` → `$211;request_id;kpL;kiL;kdL;kpR;kiR;kdR;#`.
- `$210;request_id;kpL;kiL;kdL;kpR;kiR;kdR;#` → `$502;request_id;210;#` after applying PID in RAM; or `$503;request_id;210;1;#` for values outside allowed bounds.
- `request_id` is an exact integer from **1 to 65535** (ESP32 stores it as uint16). Error `1` means invalid PID coefficient: non-finite, negative, or >1000.
- No Flash/NVS write in 210. Old 201/204 exist for now, but are **not transactional**. Do not use 204 in the safe suite.
- Syntactically malformed frames or invalid request IDs cannot be reliably correlated, so they are silently rejected; this is a known limitation. The read-only 211 reply is correlated by request ID, but has no separate ACK. Do not retry mutation 210 automatically after a lost ACK until idempotency is defined; setting the same six gains is mathematically idempotent but could reset controller state.
- Frames can be lost. A received ACK means PID setters ran, **not** that motors physically responded. A NACK does not modify gains. Neither constitutes a persistent-storage acknowledgement.

## v2.3.3 — read-only I2C diagnostic snapshot

Request `$405;#` produces `$405;ina_state;ina_failures;ina_attempts;ina_recoveries;pwm_state;pwm_failures;pwm_attempts;pwm_recoveries;oled_state;oled_failures;oled_attempts;oled_recoveries;#`.

All twelve fields are unsigned decimal integers. Devices are ordered **INA219, PCA9685, OLED**. States: `0` not detected at startup, `1` healthy, `2` probe/recovery fault, `3` recovered. Counters accumulate since boot and saturate at `4294967295`. The response is best-effort and can be dropped if UART TX space is insufficient. A read adds no I2C operations and does not change NVS, PWM, watchdog or settings. I2C ACK alone cannot prove that output pins or motor power are safe.

## Versioning

The project currently carries **three independent version axes**. They are not interchangeable, and mixing them is the source of most version confusion in this repository.

| Axis | Where it is defined | Current value | Meaning |
|---|---|---|---|
| Protocol version | Response to `$501;#` (`protocol_wrapper.cpp`) | `2;2` (major 2, minor 2) | Version of the **wire contract** described in this document. It was bumped to 2.1 with the read-only API and to 2.2 with transactional PID. |
| Firmware build version | `VERSION` constant in `esp32_firmware.ino` | `2.4.0` | Version of the **firmware build**. It is shown on the OLED / startup output, but it is **not** transmitted over UART — `$501` reports the protocol version, not the build. |
| Documentation revision | Section headings in `esp_firmware/README.md`, entries in `esp_firmware/CHANGELOG.rst` | README up to v2.3.3; CHANGELOG last numbered entry 2.2.0 (2026-10-25) plus an «Unreleased» section covering the v2 line | History of the incremental firmware/protocol work. |

Known inconsistencies, recorded rather than silently resolved:

1. **The source comment contradicts the emitted value.** The firmware emits `$501;2;2;#`, but the comment on the same source line says `// major=2, minor=1`. The emitted value is authoritative; the comment is stale.
2. **The firmware build version does not reflect the v2 work in this branch.** `VERSION = "2.4.0"` was inherited from an earlier commit on `main` and was not bumped while the protocol v2 line was developed. Consequently "2.4.0", "2.2" and "v2.3.3" refer to three different axes and must not be compared directly.
3. **The protocol version was not bumped for v2.3.x features.** The read-only I2C snapshot (`405`) was added in the v2.3.3 documentation revision, while `$501` still reports `2;2`.

Practical consequence for integrators: to identify what is running on a device, query `$501;#` — this yields the protocol contract version only. There is currently **no way to read the firmware build version over UART**, so an on-device build cannot be confirmed from the Jetson side. Aligning the three axes (or documenting them explicitly as independent) is an open decision recorded in `docs/ai/decisions.md` (ADR-015).
