# serial_bridge_cpp

C++-замена трёх Python-узлов из `serial_bridge_package`
(`twist_to_command`, `serial_bridge_node`, `feedback_processor`) одним узлом.

## Что делает

```
/cmd_vel (Twist) ──────────┐
/low_level/serial/cmd ─────┤──> UART (/dev/esp32) ──> ESP32
           (сырые "$...#")  │
                            │   ESP32 ──> UART ──> поток чтения (poll)
                            │                        │
                            │                        ├─> /odom (Odometry)
                            │                        ├─> TF odom -> base_link
                            │                        ├─> /sensors/wheel/{left,right}/speed
                            │                        ├─> /PIDcoeffs
                            │                        └─> /low_level/serial/feedback (сырые кадры)
```

Имена топиков и формат такие же, как у Python-версии, поэтому Nav2, SLAM и
`simple_PID_configurator` работают без изменений.

## Чем лучше Python-версии

- поток чтения спит в `poll()` до прихода байтов (Python-версия опрашивала
  порт в цикле без ожидания и держала ядро на 100%);
- нет промежуточных строковых топиков и двух лишних процессов;
- при пропадании порта (переподключение USB) узел сам открывает его снова;
- узел зарегистрирован как компонент и может работать в component container.

## Запуск

```bash
ros2 launch serial_bridge_cpp serial_bringup.launch.py
# или в составе всего робота (по умолчанию use_cpp:=True):
ros2 launch completed_scripts_jetbot jetbot_sensors_and_motors.launch.py
```

## Параметры

| Параметр | По умолчанию | Описание |
|---|---|---|
| `serial_port` | `/dev/esp32` | порт ESP32 |
| `baudrate` | `115200` | скорость |
| `frame_id` | `odom` | кадр одометрии |
| `child_frame_id` | `base_link` | кадр робота |
| `publish_tf` | `true` | публиковать TF odom -> base_link |
| `publish_raw_feedback` | `true` | публиковать сырые кадры в `serial/feedback` (нужно для PID-конфигуратора) |
| `reconnect_period` | `1.0` | период попыток переоткрыть порт, с |

## Тесты

```bash
colcon build --packages-select serial_bridge_cpp
colcon test --packages-select serial_bridge_cpp && colcon test-result --verbose
```
