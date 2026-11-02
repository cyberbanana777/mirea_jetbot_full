# Outstanding items (not silently fixed)

## Priority integration/safety
- No hardware enable/kill switch; zero setpoint is not assured safe shutdown under CPU/driver failure. Consider physical power isolation.
- Add ACK/NACK, request IDs, and persistence confirmation for settings. `5xx` range reserved.
- Add full input range checking before refreshing movement watchdog (not only valid message shape).
- Add unit tests covering all v2 mappings on both sides; integrate ROS 2 colcon and Arduino/PlatformIO CI.
- The ROS node currently emits generic raw frames on `serial/cmd` for configuration; typed configuration services are a proposed next step.

## Deferred
- Nonblocking I2C timing and recovery and missing-device policy.
- Configuration validity and NVS defaults consistency.
- Profile control-loop jitter on hardware and test motor PWM fail-safe after failed PCA9685 transactions.
- `jetbot_stats.py` obtains **and transmits Wi-Fi passwords in plaintext** over Serial1 and writes them to stdout; assess whether OLED needs password display at all. Keep the second channel isolated until explicitly redesigned.
- Documentation and tooling under the original `tools/` directory require v2 command-ID migration; they are not bundled in this integration snapshot.

- UART bench testing is the active workflow; ROS 2 integration remains unverified and deferred.
- Software STOP ID 103 only sets zero speed target; direct zero PWM has not been independently verified.

- v2.1 intentionally has no transactional ACK/NACK. Design request sequence IDs and acknowledge *applied* settings (including NVS failure handling), then add host tests.
- `402` diagnostics count structural rejects, not malformed frame overflows and not semantic invalidity inside dispatch handlers. Add separate parser/validation metrics.
- Safety: watchdog timing and physical zero-PWM still need hardware verification; no separate motor-power cutoff exists.
