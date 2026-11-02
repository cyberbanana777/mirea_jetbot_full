# Main UART protocol v2 (BREAKING CHANGE)

This is the sole protocol supported on the main ESP32 Serial <-> Jetson ROS 2 `serial_bridge_cpp` channel. **No negotiation, no v1 aliases.** Serial1 + `jetbot_stats.py` is a separate display/network-info path and is not modified.

## Framing

ASCII: `$<id>;<arg1>;...;#` (optional CR/LF after `#`). `id` is an integer; fields are finite decimal numbers; `.` is the decimal point. Max input fields on ESP32: 6. Any malformed/unknown command is ignored. No ACK/NACK or checksum in this stage; do not assume a setting was applied because the serial write succeeded.

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
|403|enabled|toggle 100 Hz diagnostic stream (odometry paused while enabled)|
|404|[min_edges[,aligned]]|speed estimator window request/change|

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

### Design decision

The `2xx` range is the **configuration family in both directions** (SET and GET response have the same ID in selected cases), rather than attempting to allocate strictly by UART direction. `3xx` is state telemetry, `4xx` is diagnostics, `1xx` motion. `5xx` reserved for future request sequencing / ACK / NACK and protocol metadata. This is a transitional protocol contract, not yet a finalized transactional API.

### ROS 2

`/cmd_vel` now emits `$101;v;w;#`. The bridge accepts `$301;...;#` for `/odom`, `/sensors/wheel/*/speed`, TF, and `$202;...;#` for `/PIDcoeffs`. `serial/cmd` remains a low-level raw-frame topic, so existing external publishers must update their IDs. `serial/feedback` keeps raw telemetry. Other incoming frames currently appear as unexpected messages; structured ROS publishers for diagnostics are a follow-up.

## Safety

There is NO hardware motor power disconnect/enable. Software stop sets a zero target but cannot guarantee physical braking or de-energization in the presence of processor/I2C/PCA9685 failure. Do not run unattended; bench test with wheels lifted first. A hardware enable / independent motor power cut is recommended for physical safety. Motion watchdog updates only on structurally valid motion commands, not queries. **Current limitation:** a syntactically valid but semantically rejected command may still refresh the watchdog; track as a priority follow-up.
