# Архитектура jetbot_mirea (архитектурная память)

| Поле | Значение |
|---|---|
| Репозиторий | `https://github.com/cyberbanana777/mirea_jetbot_full.git` |
| Ветка базовой линии | `fix/esp32_firmware` |
| `base_commit` (код) | `b4c4e0a519792e7f848c5f7c6d35f1a4ea2868d7` |
| Дата анализа | 2026-11-11 |
| Коммит с этой документацией | `docs(ai): initialize architecture memory and align docs with code` (документация не меняет код, поэтому `base_commit` кода остаётся действительным) |
| Назначение документа | Устойчивая архитектурная память проекта: границы, слои, потоки и инварианты, которые нельзя нарушать при рефакторинге |

**Как читать документ.** Каждое утверждение помечено:

- **[Код]** — подтверждено чтением исходников или воспроизводимым запуском тестов;
- **[Док]** — заявлено в документации репозитория, но не подтверждено кодом или тестом;
- **[Гипотеза]** — правдоподобное предположение, требующее проверки.

Ссылки даны в формате `путь/файл:строка` или `путь/файл:функция`. Все номера строк
относятся к `base_commit`.

---

## 1. Системный контекст

Робот `jetbot_mirea` состоит из двух вычислительных узлов и набора периферийных
устройств:

| Узел | Роль | ПО | **[Код]** |
|---|---|---|---|
| Jetson Nano | Бортовой компьютер: ROS 2 Humble (в Docker), Nav2/SLAM, драйверы сенсоров, веб-конфигуратор | ROS 2 | `README.md:13`, `completed_scripts_jetbot/package.xml` |
| ESP32 | Нижний уровень: энкодеры, регуляторы колёс, PCA9685 (PWM), INA219 (ток/напряжение), OLED, светодиод ошибки | Arduino sketch | `esp_firmware/esp32_firmware/esp32_firmware.ino` |

Связь между ними — **один последовательный канал** (`Serial` на ESP32 ↔ USB-TTY на
Jetson), плюс отдельный канал `Serial1` для служебных сообщений. **[Код]**

- Основной канал: `/dev/esp32`, 115200 бод — `serial_bridge_cpp/launch/serial_bringup.launch.py`;
  симлинк создаёт `init_scripts/esp32/add_symlink_for_esp32.bash` (udev-правило
  `ttyUSB*` → `esp32`). **[Код]**
- Служебный канал (`Serial1`): Jetson шлёт `$6;ssid;password;ip;#` для вывода сети и
  IP на OLED. Протокол v2 его **не затрагивает** — `esp_firmware/PROTOCOL.md:3`,
  `esp_firmware/esp32_firmware/esp32_firmware.ino:parseJetsonMessage`,
  `init_scripts/jetbot_stats/jetbot_stats.py:167`. **[Код]**

Физический силовой контур: PCA9685 по I²C управляет H-мостами моторов; отдельной
аппаратной линии отключения питания моторов **нет**. **[Код]**
`esp_firmware/esp32_firmware/motor_controller.cpp:emergencyStop` (сброс целей и запись
нулей в 4 PWM-канала), `esp_firmware/docs/testing/HARDWARE_TEST_LOG.md` («General
safety note»). **[Док]** `esp_firmware/TODO.md` прямо фиксирует, что программный
STOP не подтверждён независимой проверкой нулевого PWM.

---

## 2. Два независимых контура сборки

Репозиторий — монорепозиторий с **двумя несовместимыми системами сборки**,
разделёнными маркером `COLCON_IGNORE`. **[Код]**

| Контур | Корневой каталог | Сборка | `COLCON_IGNORE` |
|---|---|---|---|
| ROS 2 (Humble) | корень репозитория | `colcon build` | — |
| ESP32 (Arduino) | `esp_firmware/` | Arduino IDE / `arduino-cli` | `esp_firmware/COLCON_IGNORE` (0 байт, tracked, mode 100755) |
| Вспомогательные скрипты | `init_scripts/` | вручную | `COLCON_IGNORE` |

Пакеты ROS 2 верхнего уровня: `serial_bridge_cpp`, `lidar_filter_cpp`, `area_patrol`,
`completed_scripts_jetbot`, `trajectory_maker`, `web_regulators_configurator`,
`jetbot_mirea_description`, `sllidar_ros2`, `realsense_ros_gazebo`,
`realsense_d435i_configs`. **[Код]** (по `package.xml` и `setup.py`).

`init_scripts/` не является ROS-пакетом: в нём нет `package.xml`, он запускается как
обычные Python-скрипты. **[Код]** `simple_PID_configurator/` удалён в Этапе 2
(ADR-017); `web_regulators_configurator` — единственный настройщик ПИД.

### 2.1. Отдельный контур ESP32

`esp_firmware/esp32_firmware/` — Arduino sketch: 17 `.cpp`, 23 `.h`, 1 `.ino`
(41 файл). **[Код]**

Внутри `esp_firmware/` дополнительно лежат: `tools/host_test/` (хостовые тесты
регулятора и эквивалентности энкодеров), `tools/uart_bench/` (эталонный
Python-кодек протокола и аппаратные скрипты), `tools/{adrc,console,mtcal,steptest}.py`
(**переведены на v2 в Этапе 2**), `docs/audits/`, `docs/testing/`.

Исторические материалы вынесены в `esp_firmware/archive/` (Этап 5) и в сборке не
участвуют: `esp32_firmware_old/` (монолитная прошивка v1), `Speed_for_PID_test_speed*/`
(скетчи Arduino), `reports/` (11 прогонов стенда 2026-11-02). **[Код]**

### 2.2. Слой автоматических проверок

Над обоими контурами добавлен слой проверок, который **не требует ROS 2 и не
требует платы** (Этап 6): **[Код]**

| Слой | Состав | Что проверяет |
|---|---|---|
| Хостовые тесты | `esp_firmware/tools/host_test/` (Makefile), `esp_firmware/tests/` | контракт протокола v2, границы длины кадра, согласованность ID и комментариев |
| Python-тесты | `esp_firmware/tools/uart_bench/` (37 тестов), `web_regulators_configurator/test/`, `area_patrol/test/` | эталонный кодек, стендовые скрипты, чистые Python-модули пакетов |
| Guard-скрипт | `tools/ci/guard.py` | ID протокола ↔ `PROTOCOL.md`, комментарии `command_parser.h` ↔ `decodeCommand`, гигиена индекса, отсутствие висячих путей |
| Синтаксическая проверка | `esp_firmware/tools/host_test/syntax_check_without_upid.sh` | файлы прошивки, не требующие `uPID` (остальные — SKIP) |
| CI | `.github/workflows/ci.yml` | запуск всех четырёх слоёв на `push`/`pull_request` |

Вне слоя остаются: сборка ROS 2-пакетов (`colcon`/`ament`), файлы прошивки с
зависимостью `uPID` (R-12) и аппаратные прогоны (R-14).

---

## 3. Архитектура контура ESP32

### 3.1. Слои

Прошивка разделена на пять слоёв. Разделение — результат рефакторинга ветки
`fix/esp32_firmware` (13 коммитов начиная с `555a561`); до него всё было в `.ino`.
**[Код]**

```
┌───────────────────────────────────────────────────────────────────────┐
│ 5. Композиционный корень         esp32_firmware.ino (287 стр.)        │
│    глобальные объекты (43-56), setup() (228), loop() (234)           │
├───────────────────────────────────────────────────────────────────────┤
│ 4. Сервисы (не владеют состоянием, получают контекст по ссылке)      │
│    SerialService · SafetyManager · ControlLoopService ·               │
│    HardwareMonitor · StartupService · ConfigService ·                │
│    DiagnosticsService · CommandDispatcher                             │
├───────────────────────────────────────────────────────────────────────┤
│ 3. Политики (чистые функции, без оборудования)                       │
│    command_policy.h · safety_policy.h                                │
├───────────────────────────────────────────────────────────────────────┤
│ 2. Драйверы и периферия                                             │
│    MotorController · Encoders · FlashStorageManager · DisplayManager  │
│    ErrorManager · SerialReceiver · SpeedEstimator · Odometry · Filter │
├───────────────────────────────────────────────────────────────────────┤
│ 1. Протокол                                                          │
│    command_parser.cpp (приём) · protocol_wrapper.cpp (передача)        │
└───────────────────────────────────────────────────────────────────────┘
```

**Слой 3 (политики) — ключевое архитектурное решение.** `command_policy.h` и
`safety_policy.h` не содержат обращений к оборудованию и глобальному состоянию,
поэтому проверяются на хосте без ESP32. **[Код]** Проверка чистоты:
`command_policy.h:7-47`, `safety_policy.h:6-10`.

### 3.2. Композиционный корень

`esp32_firmware.ino` — единственное место, где создаются глобальные объекты:
`encoders`, `MotorController controller` (стр. 45), `INA219 INA(0x41)` (49),
`FlashStorageManager flashStorage("Memory2")` (53), `DisplayManager display`
(54), `SerialReceiver mainReceiver(Serial)` (55), `SerialReceiver
jetsonReceiver(Serial1)` (56). **[Код]**

Сервисы получают **невладеющие** контексты — структуры с ссылками на разделяемые
объекты. Это устраняет скрытое глобальное состояние, но оставляет риск
«не владеющий, но влияющий»: любой сервис может изменить состояние другого.
**[Код]** `esp32_firmware/esp32_firmware/command_dispatcher.h` (62 стр. — определения
`MotionContext`, `ConfigContext`, `DiagnosticsContext` и т. д.).

### 3.3. Порядок работы главного цикла

`loop()` (`esp32_firmware.ino:234`) выполняет строго фиксированную последовательность:
**[Код]**

| № | Действие | Строка | Период / условие |
|---|---|---|---|
| 1 | `serialService.poll()` — приём байтов в оба `SerialReceiver` | 238 | каждый проход |
| 2 | `safetyManager.check()` — watchdog и fail-closed остановка | 240 | каждый проход |
| 3 | Контур скорости | 248 | 100 Гц (`CONTROL_PERIOD_US = 10000`) |
| 4 | Одометрия | 255 | 50 Гц (`odom_timer_timeout = 20000`) |
| 5 | `serialService.processCommands()` — разбор и диспетчеризация команд | 259 | до 4 кадров за вызов |
| 6 | `serialService.processSecondary()` — `$6;...` из Serial1 | 260 | при наличии кадра |
| 7 | Выборка INA219 | ~266 | `update_voltage_timer_timeout = 200` мс |
| 8 | `hardwareMonitor.checkI2CDevices()` | 272 | `I2C_CHECK_INTERVAL = 500` мс |
| 9 | `display.update(...)` | ~278 | интервал дисплея |
| 10 | `error_manager.setFault(!inaOk \|\| !pwmOk \|\| !oledOk \|\| lowBattery)` | 285 | каждый проход |

Порядок важен: проверка безопасности (2) идёт **до** разбора команд (5), поэтому
команда, пришедшая в «небезопасном» состоянии, не может продлить окно
движения. **[Код]**

### 3.4. Подсистема управления движением

- **Оценка скорости:** `Encoders` → `SpeedEstimator::updateAligned` (по фронтам
  энкодеров, с выравниванием по времени) → линейные скорости колёс.
  `esp_firmware/esp32_firmware/control_loop_service.cpp:controlStep`. **[Код]**
- **Регулятор:** `MotorController::update` — feedforward + ПИ или LADRC
  (режим `CTRL_MODE`). `esp_firmware/esp32_firmware/motor_controller.cpp`. **[Код]**
- **Одометрия:** `DiffDriveOdometry` (`odometry.h`) по тикам энкодеров;
  `convertTicksToDistance` → `odometryStep`. **[Код]**
- **Ограничения уставок:** `setTargetVelocity`/`setWheelSpeeds` обрезают скорость по
  `limit = _maxWheelSpeed * 0.8f` (`MAX_CONSTRUCTIVE_VELOCITY = 0.5` м/с →
  ±0.4 м/с на колесо); `setOpenLoop` обрезает duty в [-1, 1].
  `motor_controller.cpp:setTargetVelocity`, `config.h:33`. **[Код]**

> **Важное наблюдение о безопасности:** обрезка значений выполняется **после**
> валидации политикой, то есть команда с заведомо недопустимым значением
> (например, `duty = 5.0`) принимается, **обновляет watchdog** и лишь затем
> молча обрезается. Политика проверяет конечность и число аргументов
> (`command_policy.h:validMotionValues`), но не диапазон.
> **[Код]** `esp_firmware/esp32_firmware/serial_service.cpp:19-29`,
> `motor_controller.cpp:setOpenLoop`. Это и есть точный смысл фразы
> «range checking remains a follow-up» в `esp_firmware/PROTOCOL.md`.

### 3.5. Подсистема безопасности

Единая политика отказа — **fail-closed**: **[Код]**

```cpp
// esp_firmware/esp32_firmware/safety_policy.h:6-10
static bool mustStop(bool commandTimedOut, bool inaOk, bool pwmOk,
                     float voltage, float minVoltage) {
  return commandTimedOut || !inaOk || !pwmOk ||
         !isfinite(voltage) || voltage < minVoltage;
}
```

Пороги напряжения: `min_voltage = 3.3*3 = 9.9` В, `max_voltage = 4.2*3 = 12.6` В
(`config.h:9-10`). **[Код]**

Применение:
- `SafetyManager::check()` (`safety_manager.cpp:4-18`) — при `mustStop` и не
  остановленном контроллере вызывается `emergencyStop()`; фронт «хост замолчал»
  увеличивает `watchdogStops` (с защёлкой `watchdogLatched`, чтобы не считать один
  и тот же факт каждый проход); при молчании хоста телеметрия отключается.
- `SerialService::processCommands()` (`serial_service.cpp:20-22`) — при
  `mustStop` **движущиеся** команды отбрасываются и **не** обновляют watchdog.

Порядок обновления watchdog (`:14-28`): сначала `validShape` → затем (для motion-команд)
проверка `mustStop`/`validMotionValues` → и только потом `lastMotionMs = millis()`.
Следовательно, отклонённая политикой команда watchdog **не** обновляет; обновляет
команда с недопустимым по диапазону, но конечным значением (см. §3.4). **[Код]**

### 3.6. Подсистема конфигурации

- `ConfigService` (`config_service.cpp`) — применение коэффициентов и параметров.
  Разделены режимы RAM (201) и NVS/flash (204), транзакционный режим (210) с
  ACK/NACK. **[Код]**
- `FlashStorageManager` (`flash_storage.cpp`) — доступ к NVS (`"Memory2"`). **[Код]**
- Валидация ПИ-коэффициентов: `validPid` — диапазон [0, 1000], все значения
  конечные (`command_policy.h:6-12`). **[Код]**
- **Непоследовательность (подтверждённый дефект):** при невалидном ПИ для 201/204
  выполняется тихий no-op — без NACK и без изменения счётчиков; для 210 тот же случай
  корректно возвращает NACK с ошибкой 1 и корректирует счётчики
  (`config_service.cpp:124-138`). **[Код]**

### 3.7. Диагностика и наблюдаемость

| Канал | Назначение | Периодичность |
|---|---|---|
| `$301` | одометрия | 50 Гц, **только** при выключенной телеметрии |
| `$401` | поток телеметрии (уставка/скорость/duty) | 100 Гц при включённой телеметрии |
| `$202` | коэффициенты ПИ | по запросу |
| `$402` | снимок состояния (uptime, accepted, rejected, watchdog_stops, flags) | по запросу |
| `$405` | I²C-диагностика INA219/PCA9685/OLED (12 полей) | по запросу |
| `$501` | версия протокола | по запросу |
| `$502`/`$503` | ACK/NACK транзакционного 210 | по событию |

Телеметрия и одометрия **взаимоисключающие**: `odometryStep` публикует `$301` только
при `!telemetryOn` (`control_loop_service.cpp:odometryStep`). **[Код]**

Ответы прошивки — **best-effort**: если в буфере UART нет места, кадр молча
пропускается (`protocol_wrapper.cpp`, проверки `Serial.availableForWrite()`).
**[Код]** Это же отражено в `esp_firmware/PROTOCOL.md` (раздел «v2.1»). **[Док]**

---

## 4. Архитектура контура ROS 2

### 4.1. Топология

```
Nav2 / teleop / area_patrol
        │ /cmd_vel (geometry_msgs/Twist)
        ▼
serial_bridge_cpp::SerialBridgeNode ──UART──► ESP32
        │                                         │
        │ /odom, /PIDcoeffs,                      │
        │ /sensors/wheel/{left,right}/speed,      │
        │ TF odom→base_link                       │
        ▼                                         ▼
   Nav2 / RViz / web_regulators_configurator ◄── raw frames (serial/feedback)
```

**[Код]** Топики моста: `serial_bridge_cpp/src/serial_bridge_node.cpp:91-110`
(публикаторы `/odom`, `/PIDcoeffs`, `/sensors/wheel/left/speed`,
`/sensors/wheel/right/speed`, `serial/text`, `serial/feedback`; подписки `/cmd_vel`
и `serial/cmd`).

### 4.2. Мост `serial_bridge_cpp`

Один C++-узел, заменявший прежнюю Python-триаду (`twist_to_command`,
`serial_bridge_node`, `feedback_processor`). **[Док]** `serial_bridge_cpp/README.md`.

| Аспект | Реализация | **[Код]** |
|---|---|---|
| Поток чтения | отдельный `std::thread`, `port.read()` с таймаутом 100 мс, переподключение раз в `reconnect_period` (1 с) | `serial_bridge_node.cpp:74-75,118,204-230` |
| Кадроразбор | `FrameParser` (`frame_parser.hpp`) — вырезает `$...#`, остальное — текстовые строки | `frame_parser.hpp:32-116` |
| Разбор чисел | `strtod` + проверка «поле целиком число» + `errno == ERANGE` + `isfinite` | `frame_parser.hpp:136-141` |
| Преобразование в числа | `static_cast<int>(fields_[0])` — идентификатор кадра | `serial_bridge_node.cpp:296` |
| `/cmd_vel` → UART | `format_twist()` → `snprintf("$101;%.3f;%.3f;#", linear, angular)` | `protocol.hpp:88-97`, `serial_bridge_node.cpp:135-148` |
| Таймер простоя `/cmd_vel` | **реализован**: параметр `cmd_vel_timeout` (0.5 с), таймер 100 мс, при простое шлёт `$103` | `serial_bridge_node.cpp:76-79,112-115,167-192` |

Таймер простоя `/cmd_vel` **реализован**: при молчании дольше `cmd_vel_timeout`
(по умолчанию 0.5 с) мост один раз шлёт `$103` — программную остановку, которую
не продлевает watchdog прошивки. Запись в корневом `IMPROVEMENTS.md`
(«Таймаут `/cmd_vel` в `serial_bridge_cpp`») помечена как выполненная. **[Код]+[Док]**

### 4.3. Конфигураторы и вспомогательные инструменты

- `web_regulators_configurator` — веб-UI на `http://<IP>:8080`, работает на роботе.
  Публикует в `/low_level/serial/cmd`, читает `/low_level/serial/feedback`.
  `web_regulators_configurator/web_regulators_configurator/node.py:64-65,85,118-152`.
  **[Код]**
- `simple_PID_configurator` — набор GUI (Tkinter / PySide2) и ROS-узел; публиковал в
  `/low_level/serial/cmd`. **Удалён в Этапе 2** (ADR-017): работал по протоколу v1,
  дублировал функциональность `web_regulators_configurator`. **[Код]**

Изначально оба инструмента были привязаны к **протоколу v1** (`$1`, `$2`, `$3`,
`$4`, `$5`) и потому неработоспособны с прошивкой v2. **[Код]**

**Статус после Этапа 2.** `web_regulators_configurator` переведён на протокол v2:
команды `$101`/`$102`/`$103`/`$201`/`$202`/`$204`, разбор ответов `$301` (одометрия),
`$202` (ПИД) и `$401` (телеметрия контура) — `web_regulators_configurator/web_regulators_configurator/protocol.py`,
`node.py:118-152`. **[Код]**

### 4.4. Навигация и восприятие

`completed_scripts_jetbot` — пакет-агрегатор launch-файлов и конфигураций
(AMCL, SLAM, Nav2) для реального робота и симуляции; `jetbot_mirea_description` —
описание URDF/ Gazebo; `lidar_filter_cpp` — секторный фильтр лидара;
`area_patrol` — патрулирование зоны на `nav2_simple_commander`;
`trajectory_maker` — генерация траекторий. **[Код]** (`package.xml`, `setup.py`).

---

## 5. Сквозные потоки данных

### 5.1. Команда движения (проектное намерение v2)

```
/cmd_vel (Twist)
  → serial_bridge_node::on_cmd_vel            serial_bridge_node.cpp:135-148
  → "$101;v;w;#"                              [v2-формат]
  → UART
  → SerialReceiver::update                    serial_receiver.cpp:15
  → SerialService::processCommands            serial_service.cpp:10
      → CommandParser::parse                  command_parser.cpp:35
      → CommandPolicy::validShape             command_policy.h:31
      → mustStop / validMotionValues          serial_service.cpp:20-22
      → lastMotionMs = millis()               serial_service.cpp:27
  → CommandDispatcher → MotorController::setTargetVelocity
  → PCA9685 → H-мосты → моторы
```

### 5.2. Одометрия

```
энкодеры → Encoders → SpeedEstimator → ControlLoopService::controlStep (100 Гц)
  → Odometry (DiffDriveOdometry) → odometryStep (50 Гц)
  → odomPublish → "$301;x;y;h;v;w;vl;vr;#"   protocol_wrapper.cpp:4
  → UART → FrameParser → handle_frame        serial_bridge_node.cpp:217,274
  → /odom + TF + /sensors/wheel/*/speed       serial_bridge_node.cpp:336-357
```

### 5.3. Конфигурация регуляторов

```
веб/CLI → /low_level/serial/cmd (String) → мост → UART
  → SerialService → CommandDispatcher → ConfigService
  → RAM (201/210) или NVS (204) → FlashStorageManager
  → ответ $202 / $211 / $502 / $503
```

---

## 6. Контракт протокола v2

Формат: ASCII `$<id>;<arg1>;...;#`, допускается CR/LF после `#`. Аргументов не
более 7 (`command_parser.cpp:52`). Контрольной суммы нет. **[Код]**

Диапазоны ID (`command_parser.cpp:decodeCommand`, `command_parser.h`): **[Код]**

| Диапазон | Назначение | ID |
|---|---|---|
| `1xx` | движение | 101 SET_TWIST, 102 SET_WHEEL_SPEEDS, 103 STOP, 104 SET_DUTY |
| `2xx` | конфигурация (в обе стороны) | 201 SET_PID_TEMP, 202 REQUEST_PID, 203 SET_ADRC, 204 SET_PID_FLASH, 205 SET_FF, 206 REQUEST_FF, 207 SET_REF_FILTER, 208 SET_CTRL_MODE, 209 REQUEST_ADRC, 210 SET_PID_TX, 211 REQUEST_PID_TX |
| `3xx` | телеметрия состояния | 301 одометрия (ответ) |
| `4xx` | диагностика | 402 REQUEST_DIAGNOSTICS, 403 SET_TELEMETRY, 404 SET_SPEED_WINDOW, 405 REQUEST_I2C_DIAGNOSTICS |
| `5xx` | метаданные протокола, ACK/NACK | 501 REQUEST_INFO, 502 ACK, 503 NACK |

Прочие ID → `UNKNOWN` → кадр отбрасывается и учитывается в `rejected`.
**[Код]** `command_parser.cpp:30,45-46`, `serial_service.cpp:14-18`.

Согласованный эталон кодеков — `esp_firmware/tools/uart_bench/protocol.py`
(`IDS`, `EXPECTED_RX_FIELDS`, `MAX_FRAME_BYTES = 192`). **[Код]**

---

## 7. Инварианты, которые нельзя нарушать

Ниже — перечень извлечённых из кода правил, нарушение которых ломает
безопасность или совместимость.

1. **Fail-closed.** Любая потеря наблюдаемости (INA219, PCA9685, питание ниже
   минимума, нечисловое напряжение) немедленно ведёт к нулевому PWM.
   `safety_policy.h:6-10`, `safety_manager.cpp:6-10`. **[Код]**
2. **Проверка безопасности до обработки команды.** `safetyManager.check()` идёт
   раньше `serialService.processCommands()` в цикле. `esp32_firmware.ino:240,259`.
   **[Код]**
3. **Watchdog продлевается только успешно валидированными motion-командами.**
   `serial_service.cpp:14-29`. **[Код]**
4. **Запись нулей во все 4 PWM-канала при остановке** — независимо от кэша
   направления. `motor_controller.cpp:emergencyStop`. **[Код]**
5. **`reinitializePWM()` обязан проверять `begin()`** — иначе «восстановление»
   сообщает успех при отсутствующем PCA9685. `motor_controller.cpp:62-75`. **[Код]**
6. **Флаги здоровья = `detected && initialized`.** `startup_service.cpp:29,33,37`.
   **[Код]**
7. **Политики не зависят от оборудования** — иначе теряется возможность
   хостового тестирования. `command_policy.h`, `safety_policy.h`. **[Код]**
8. **Формат `$`…`#` и лимит 7 аргументов** — общий контракт Jetson↔ESP32.
   `command_parser.cpp:37-60`. **[Код]**
9. **`Serial1` (`$6;...`) не входит в протокол v2** и обрабатывается отдельным
   путём. `esp32_firmware.ino:parseJetsonMessage`. **[Код]**
10. **Ответы best-effort** — вызывающий обязан повторять read-only запросы с
    таймаутом. `protocol_wrapper.cpp` (проверки `availableForWrite`),
    `esp_firmware/PROTOCOL.md` (v2.1). **[Код]+[Док]**
11. **Кадр host → ESP32 не длиннее 127 байт** (включая `$` и `#`) — иначе
    `SerialReceiver` молча сбросит приём, без счётчика и без ответа. Худшая
    команда `$210` занимает 125 байт, запас — 2 байта; проверяется тестом
    `esp_firmware/tests/test_frame_limits.cpp`. **[Код]**

---

## 8. Известные архитектурные разрывы (по состоянию ветки `fix/esp32_firmware`)

Полный реестр с приоритетами и планом — в `docs/ai/project-state.md`. Здесь —
краткий перечень архитектурно значимого.

1. **Разрыв интеграции протокола.** *Было:* прошивка реализует только v2, а рабочий
   пакет `serial_bridge_cpp` — только v1: шлёт `$1;v;w;#` (прошивка отбрасывает как
   `UNKNOWN`) и не понимает `$301` (ждёт тип 1). Ни движение, ни одометрия не
   работают в связке. Миграция v2 существовала только в неиспользуемой копии
   `esp_firmware/serial_bridge_cpp_reference/`, которая не собиралась colcon
   (`esp_firmware/COLCON_IGNORE`) и имела то же имя пакета `serial_bridge_cpp`.
   **Закрыто в Этапе 2:** копия удалена, рабочий мост переведён на v2, добавлен
   сквозной host-тест «мост ↔ прошивка». **[Код]**
2. **Отсутствие CI** — **закрыто**: добавлен `.github/workflows/ci.yml` с четырьмя
   job (`host-tests`, `python-tests`, `protocol-guard`, `firmware-syntax`).
   ESP32-сборка в CI по-прежнему отсутствует: блокер — библиотека `uPID` (R-12). **[Код]**
3. **Осиротевшие тесты.** *Было:* `esp_firmware/tests/test_protocol_v2.cpp` и
   `test_bridge_v2.cpp` не были подключены ни к Makefile, ни к CMake, ни к скриптам.
   **Частично закрыто в Этапе 2:** оба подключены к `esp_firmware/tools/host_test/Makefile`
   (цель `test_protocol`), добавлен `test_end_to_end_v2.cpp`; CI (`.github/workflows/ci.yml`)
   запускает их в job `host-tests`.
   **[Код]**
4. **Разделение владения состоянием на уровне сервисов.** Контексты не владеют
   объектами; инварианты поддерживаются соглашениями, а не типами. **[Код]**
5. **Отсутствие аппаратного отключения питания моторов.** **[Код]+[Док]**

---

## 9. Связанные документы

| Документ | Назначение |
|---|---|
| `docs/ai/components.md` | пообъектный каталог компонентов и их ответственности |
| `docs/ai/decisions.md` | принятые архитектурные решения с обоснованием и статусом |
| `docs/ai/project-state.md` | состояние проекта, риски, готовность к рефакторингу, план |
| `esp_firmware/PROTOCOL.md` | контракт UART-протокола v2 |
| `esp_firmware/docs/audits/ARCHITECTURE_AUDIT_v2.4.0.md` | текущий аудит (A-01…A-16, доказательства file:line) |
| `esp_firmware/docs/audits/ARCHITECTURE_AUDIT_v2.3.0.md` | предыдущий аудит (база v2.2.9, частично устарел) |
| `esp_firmware/docs/testing/HARDWARE_TEST_LOG.md` | журнал аппаратных проверок |
| `esp_firmware/TODO.md`, `IMPROVEMENTS.md`, `TODO.md` | бэклог задач (описание состояния v1) |
