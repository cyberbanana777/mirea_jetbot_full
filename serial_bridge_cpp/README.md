# serial_bridge_cpp

Связь ROS 2 с ESP32 робота jetbot_mirea через последовательный порт.
Один C++-узел `serial_bridge_node`: принимает команды движения, передаёт их
в ESP32 и публикует одометрию, TF, скорости колёс и коэффициенты PID.

## Как устроено

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
                            │                        ├─> /low_level/serial/feedback (сырые кадры)
                            │                        └─> /low_level/serial/text (текст ESP32 вне кадров)
```

## Возможности

- **Одна нода вместо трёх.** Заменяет Python-пакет `serial_bridge_package`
  (`twist_to_command`, `serial_bridge_node`, `feedback_processor`).
  Имена топиков и форматы сообщений прежние, поэтому Nav2, SLAM и
  `simple_PID_configurator` работают без изменений.
- **Почти не нагружает процессор.** Поток чтения спит в `poll()` до прихода
  байтов. Прежний Python-мост опрашивал порт в цикле без ожидания и держал
  одно ядро на 100%. Замер на эмуляторе ESP32 (одометрия 10 Гц):
  93% одного ядра у Python-версии, 0.4% у C++.
- **Разбор кадров как в прошивке ESP32.** Новый `$` начинает кадр заново,
  кадр длиннее 256 символов отбрасывается.
- **Сообщения ESP32 видны в ROS.** Текст вне кадров (ошибки инициализации,
  Wi-Fi, OTA, загрузчик после перезагрузки) публикуется построчно в
  `/low_level/serial/text` и пишется в лог узла как `ESP32: ...` (строки с
  error / fail / warn — уровнем WARN). Непечатаемые байты заменяются на `?`.
- **Переподключение порта.** Если ESP32 пропал (переподключили USB), узел
  пишет об этом в лог и сам переоткрывает порт, как только устройство
  появится снова.
- **Компонент.** Узел можно загрузить в component container
  (`serial_bridge_cpp::SerialBridgeNode`).
- **Без внешних зависимостей** для работы с портом (POSIX termios).

## Запуск

```bash
# отдельно
ros2 launch serial_bridge_cpp serial_bringup.launch.py

# в составе всего робота
ros2 launch completed_scripts_jetbot jetbot_sensors_and_motors.launch.py
```

Аргументы `serial_bringup.launch.py`: `serial_port` (по умолчанию
`/dev/esp32`), `baudrate` (`115200`), `publish_raw_feedback` (`true`).

Конфиг RViz с одометрией и целями навигации:

```bash
rviz2 -d $(ros2 pkg prefix serial_bridge_cpp)/share/serial_bridge_cpp/rviz/show_all_stats.rviz
```

## Параметры узла

| Параметр | По умолчанию | Описание |
|---|---|---|
| `serial_port` | `/dev/esp32` | порт ESP32 |
| `baudrate` | `115200` | скорость |
| `frame_id` | `odom` | кадр одометрии |
| `child_frame_id` | `base_link` | кадр робота |
| `publish_tf` | `true` | публиковать TF odom -> base_link |
| `publish_raw_feedback` | `true` | публиковать сырые кадры в `serial/feedback` (нужно для PID-конфигуратора) |
| `log_text_messages` | `true` | писать текст ESP32 вне кадров в лог узла (в `serial/text` он публикуется всегда) |
| `reconnect_period` | `1.0` | период попыток переоткрыть порт, с |
| `pose_covariance_diagonal` | `[1e-3, 1e-3, 1e6, 1e6, 1e6, 1e-2]` | диагональ ковариации позы в `/odom` (x, y, z, крен, тангаж, рыскание) |
| `twist_covariance_diagonal` | `[1e-3, 1e6, 1e6, 1e6, 1e6, 1e-2]` | диагональ ковариации скорости в `/odom` (vx, vy, vz, wx, wy, wz) |

**Ковариации — заглушки.** Реальные дисперсии одометрии ещё не измерены
(см. `IMPROVEMENTS.md` в корне репозитория). `1e6` означает «ось не измеряется»
(дифференциальный привод не наблюдает z, крен, тангаж и боковые скорости),
небольшие значения для x, y, рыскания и vx, wz условные. Если подключать
`robot_localization`, их нужно сначала измерить.

## Протокол ESP32

| Направление | Кадр | Значение |
|---|---|---|
| ROS -> ESP32 | `$1;v;w;#` | линейная (м/с) и угловая (рад/с) скорость из `/cmd_vel` |
| ESP32 -> ROS | `$1;x;y;theta;v;w;v_left;v_right;#` | одометрия и скорости колёс, 50 Гц (прошивка ≥ 2.1.0; раньше 10 Гц) |
| ESP32 -> ROS | `$2;KpL;KiL;KdL;KpR;KiR;KdR;#` | коэффициенты PID |

Остальные команды прошивки можно отправить строкой в `/low_level/serial/cmd`,
узел передаёт их в порт как есть: `$2;v_left;v_right;#` — скорости колёс,
`$3;...#` — PID только в ОЗУ, `$4;...#` — PID с записью во flash,
`$5;#` — запрос текущих коэффициентов PID.

Всё, что ESP32 печатает вне кадров (`Serial.println` в прошивке), приходит
в `/low_level/serial/text`:

```bash
ros2 topic echo /low_level/serial/text
```

Так видно, почему мигает светодиод ошибки, не подключился Wi-Fi или не
стартовал OTA, и перезагружается ли ESP32 (после сброса загрузчик печатает
строку вида `ets Jun  8 2016 ...`, часть символов будет `?`).

## Тесты

```bash
colcon build --packages-select serial_bridge_cpp
colcon test --packages-select serial_bridge_cpp && colcon test-result --verbose
```
