# Компоненты jetbot_mirea (архитектурная память)

| Поле | Значение |
|---|---|
| Ветка базовой линии | `fix/esp32_firmware` |
| `base_commit` (код) | `b4c4e0a519792e7f848c5f7c6d35f1a4ea2868d7` |
| Дата анализа | 2026-11-11 |
| Коммит с этой документацией | `docs(ai): initialize architecture memory and align docs with code` |

Метки: **[Код]** — подтверждено исходниками; **[Док]** — заявлено документацией;
**[Гипотеза]** — требует проверки. Номера строк — для `base_commit`.

---

## 1. Контур ESP32 (`esp_firmware/esp32_firmware/`)

Arduino sketch: 17 `.cpp`, 23 `.h`, 1 `.ino`. Собирается Arduino IDE / `arduino-cli`,
в colcon не участвует (`esp_firmware/COLCON_IGNORE`). **[Код]**

### 1.1. Композиционный корень

| Компонент | Файл | Ответственность |
|---|---|---|
| `esp32_firmware.ino` | `esp_firmware/esp32_firmware/esp32_firmware.ino` | Создание всех глобальных объектов (41-52), `setup()` (226), `loop()` (232), разбор служебного `$6;...` из Serial1 (`parseJetsonMessage`, 155), `VERSION = "2.4.0"` (10) |

### 1.2. Сервисный слой

Все сервисы получают **невладеющие** контексты (ссылки на разделяемые объекты) и
не хранят собственного состояния между вызовами, кроме явно описанных полей
контекста. **[Код]**

| Компонент | Файлы | Ответственность | Ключевые точки |
|---|---|---|---|
| `SerialService` | `serial_service.{h,cpp}` | Приём кадров из двух `SerialReceiver`, разбор, валидация политикой, обновление watchdog, диспетчеризация; максимум 4 кадра за вызов | `processCommands` (`serial_service.cpp:10`), `processSecondary` (36) |
| `SafetyManager` | `safety_manager.{h,cpp}` | Fail-closed остановка, счётчик watchdog-срабатываний с защёлкой, отключение телеметрии при молчании хоста | `check` (`safety_manager.cpp:4`) |
| `ControlLoopService` | `control_loop_service.{h,cpp}` | Контур скорости 100 Гц, оценка скоростей по энкодерам, одометрия 50 Гц, публикация `$301`/`$401` | `controlStep`, `odometryStep` (`control_loop_service.cpp:44,77`) |
| `ControlScheduler` | `control_scheduler.h` | Чистое планирование моментов запуска контуров (без `delay`) | `controlDue`, `odometryDue` |
| `HardwareMonitor` | `hardware_monitor.{h,cpp}` | Периодический опрос I²C-устройств, переинициализация PCA9685, учёт probe/recovery-счётчиков, выборка питания | `checkI2CDevices`, `samplePower` |
| `StartupService` | `startup_service.{h,cpp}` | Инициализация периферии, флаги `detected && initialized`, финальный `emergencyStop()` | `begin` (`startup_service.cpp:29,33,37`) |
| `ConfigService` | `config_service.{h,cpp}` | Применение ПИ/ADRC/FF/режима/окна скорости; RAM vs NVS; транзакционный 210 с ACK/NACK | `handleConfigCommand` (`config_service.cpp:124-138`) |
| `DiagnosticsService` | `diagnostics_service.{h,cpp}` | Read-only снимки: `$402` (состояние), `$405` (I²C), `$501` (версия), `$202` (ПИ) | `diagnostics_service.cpp` |
| `CommandDispatcher` | `command_dispatcher.{h,cpp}` | Единая точка маршрутизации команды → motion/config/diagnostics; определения контекстов | `command_dispatcher.h` (62 стр.) |

### 1.3. Слой политик (чистые функции, без оборудования)

| Компонент | Файл | Правила |
|---|---|---|
| `CommandPolicy` | `command_policy.h` | `validPid` (0..1000, конечные), `validRequestId` (1..65535, целое), `validTransactionalPid` (7 аргументов), `refreshesMotionWatchdog` (101/102/104), `validMotionValues` (2 аргумента, конечные), `validShape` (число аргументов по типу команды) |
| `SafetyPolicy` | `safety_policy.h` | `mustStop` — fail-closed: таймаут, !INA, !PWM, нечисловое напряжение, напряжение ниже минимума |

### 1.4. Драйверы и периферия

| Компонент | Файл | Ответственность | Существенные детали |
|---|---|---|---|
| `MotorController` | `motor_controller.{h,cpp}` | ПИ/LADRC + feedforward, вывод на PCA9685, аварийная остановка, переинициализация ШИМ | `emergencyStop` (76-96) пишет нули в 4 канала; `reinitializePWM` (62-75) проверяет `begin()`; обрезка уставок `limit = _maxWheelSpeed*0.8f` (98-122); `setOpenLoop` обрезает duty в [-1,1] |
| `Encoders` | `encoders.{h,cpp}` | Подсчёт импульсов, снимок фаз и времени фронта, атомарное чтение и сброс тиков | `snapshotRightPhases`/`snapshotLeftPhases`, `getRightAndReset`/`getLeftAndReset` |
| `SpeedEstimator` | `speed_estimator.h` | Оценка скорости по фронтам с выравниванием по времени | `updateAligned` |
| `Odometry` (`DiffDriveOdometry`) | `odometry.h` | Поза по пройденному пути (средняя точка) | `x()`, `y()`, `heading()` |
| `Filter` | `filter.h` | Сглаживание (скользящее среднее) | — |
| `FlashStorageManager` | `flash_storage.{h,cpp}` | Доступ к NVS (`"Memory2"`) | — |
| `DisplayManager` | `display_manager.{h,cpp}` | OLED: напряжение, ток, заряд, IP, ошибки | — |
| `ErrorManager` | `errors.{h,cpp}` | Светодиод ошибки без `delay()` | `setFault` |
| `SerialReceiver` | `serial_receiver.{h,cpp}` | Приём `$...#`; новый `$` сбрасывает незавершённый кадр; переполнение буфера молча сбрасывает приём | `BUFFER_SIZE = 128` (`serial_receiver.h:12`) |

### 1.5. Протокольный слой

| Компонент | Файл | Ответственность |
|---|---|---|
| `CommandParser` | `command_parser.{h,cpp}` | `decodeCommand` (ID → тип), строгий разбор `;#`, `strtol`/`strtof`, ≤7 аргументов, отбраковка нечисловых ID и нефинитных значений |
| `ProtocolWrapper` | `protocol_wrapper.{h,cpp}` | Формирование исходящих кадров: `$301`, `$202`, `$401`, `$205`, `$207`, `$203`, `$404`, `$501`, `$402`, `$502`, `$503`, `$211`, `$405`; best-effort при заполненном TX-буфере |
| Эталонный кодек | `esp_firmware/tools/uart_bench/protocol.py` | Python-реализация формата: `IDS`, `EXPECTED_RX_FIELDS`, `MAX_FRAME_BYTES = 192`, `encode`/`decode`/`Framer` (счётчик `overflow`) |

---

## 2. Контур ROS 2

### 2.1. Пакеты верхнего уровня

| Пакет | Роль | Ключевые файлы | Зависимости (`package.xml`) |
|---|---|---|---|
| `serial_bridge_cpp` | Мост Jetson↔ESP32 по UART: команды, одометрия, TF, скорости колёс, PID, сырые кадры | `src/serial_bridge_node.cpp`, `src/serial_port.cpp`, `include/serial_bridge_cpp/{frame_parser,covariance,serial_port}.hpp`, `launch/serial_bringup.launch.py` | rclcpp, rclcpp_components, std_msgs, geometry_msgs, nav_msgs, tf2_ros |
| `lidar_filter_cpp` | Секторный фильтр лидара | `src/lidar_filter_node.cpp`, `include/lidar_filter_cpp/sector.hpp` | rclcpp, rclcpp_components, sensor_msgs |
| `area_patrol` | Патрулирование зоны через `nav2_simple_commander` | `area_patrol/area_patrol_node.py`, `area_patrol/geometry.py`, `config/area_patrol.yaml` | rclpy, geometry_msgs, nav_msgs, std_srvs, visualization_msgs, nav2_simple_commander, launch_ros, python3-yaml |
| `web_regulators_configurator` | Веб-UI настроек ПИ/режимов, ступенчатый тест, «мёртвая рука» | `web_regulators_configurator/{node,protocol,telemetry,web_server}.py` | rclpy, std_msgs, nav_msgs, ament_index_python |
| `trajectory_maker` | Генерация траекторий, запуск/стоп перемещения | `trajectory_maker/{start_move_action,start_move_topic}.py` | rclpy, nav2, geometry_msgs |
| `completed_scripts_jetbot` | Агрегатор launch-файлов и конфигураций реальной работы и симуляции | `launch/*.launch.py`, `config/*_params_{real,sim}.yaml`, `maps/` | ros2launch, realsense2-camera, slam-toolbox, tf2-ros, nav2-*, sllidar_ros2, serial_bridge_cpp, jetbot_mirea_description, lidar_filter_cpp, realsense_d435i_configs, web_regulators_configurator |
| `jetbot_mirea_description` | Описание робота и сценарии Gazebo/реального запуска | `launch/{gazebo,real_robot}.launch.py` | — |
| `sllidar_ros2` | Драйвер лидара RPLIDAR | пакет драйвера | — |
| `realsense_ros_gazebo` | Сценарии камеры в Gazebo | пакет драйвера | — |
| `realsense_d435i_configs` | Профили запуска RealSense D435i | `config/{rgb_depth,rgb_depth_pointcloud,rgb_usb2,rgb_usb3}.yaml`, `launch/d435i.launch.py` | — |

### 2.2. Не-ROS-каталоги

| Каталог | Роль | Примечание |
|---|---|---|
| `simple_PID_configurator/` | GUI-настройка ПИ (Tkinter / PySide2) + ROS-узел; публикует в `/low_level/serial/cmd` | `COLCON_IGNORE`, без `package.xml`, README отсутствует |
| `init_scripts/` | udev-симлинки, `jetbot_stats` (отдельный Serial1 `$6;...`), служебные скрипты | `COLCON_IGNORE`, без `package.xml` |
| `esp_firmware/tools/` | Хостовые тесты, UART-бенчмарки, калибровочные скрипты | `adrc/console/mtcal/steptest.py` — **на ID v1**, несовместимы с v2 |
| `esp_firmware/serial_bridge_cpp_reference/` | Копия моста с миграцией на v2 | Под `esp_firmware/COLCON_IGNORE` → colcon не собирает; `package.xml` с тем же именем `serial_bridge_cpp` → конфликт при переносе в корень workspace |
| `docs/` | Даташиты, инструкции (PDF/DOCX), схемы, фото (~52 МБ) | `docs/ai/` — архитектурная память (создана этим анализом) |

### 2.3. Инфраструктура

| Аспект | Состояние | **[Код]** |
|---|---|---|
| CI (`.github/`) | **отсутствует** | — |
| Версии зависимостей | ROS 2 Humble; Python 3; g++/Arduino | `README.md:13` |
| Сборочная система ROS | `colcon` (ament) | `*/package.xml`, `*/CMakeLists.txt` |
| Отсутствующие в окружении инструменты | gtest, colcon, ROS 2, библиотека `uPID` | проверки §4 |

---

## 3. Тестовые активы

### 3.1. Хостовые тесты ESP32

| Тест | Расположение | Состояние | **[Код]** |
|---|---|---|---|
| `test_estimator` | `esp_firmware/tools/host_test/` | проходит | «ВСЕ ПРОВЕРКИ ПРОШЛИ (0 не прошло)» |
| `test_estimator_aligned` | там же | проходит | то же |
| `test_encoders_equiv` | там же | проходит | «PASS» |
| `test_protocol_v2` | `esp_firmware/tests/test_protocol_v2.cpp` | проходит при ручной компиляции | «ESP32 v2 parser tests PASS» |
| `test_bridge_v2` | `esp_firmware/tests/test_bridge_v2.cpp` | **падает** против рабочего `include/serial_bridge_cpp/frame_parser.hpp` (exit 134, core dumped); проходит против reference-копии | assertion на `!parse_fields("$301;nan;#", values)` |
| `syntax_check.sh` | `esp_firmware/tools/host_test/` | не запускается: отсутствует `uPID.h` | `motor_controller.h:7` |

`test_protocol_v2.cpp` и `test_bridge_v2.cpp` **не подключены** ни к Makefile, ни к
CMake, ни к скриптам — grep по репозиторию пуст. **[Код]**

### 3.2. Тесты ROS-пакетов

| Тест | Расположение | Состояние |
|---|---|---|
| `test_covariance`, `test_frame_parser` | `serial_bridge_cpp/test/` | gtest через `ament_add_gtest`; **не запускались** — gtest/colcon/ROS 2 отсутствуют |
| `test_sector` | `lidar_filter_cpp/test/` | то же |
| `test_frame_parser`, `test_covariance` | `esp_firmware/serial_bridge_cpp_reference/test/` | копия рабочих тестов; набор для v2 не мигрирован |
| `test_geometry.py` | `area_patrol/test/` | pytest-стиль: 7 passed через корректный harness |
| `test_protocol.py`, `test_telemetry.py`, `test_step_test.py` | `web_regulators_configurator/test/` | pytest-стиль: 7 + 4 + 5 = 16 passed через harness |
| `uart_bench` (31 тест) | `esp_firmware/tools/uart_bench/` | `python3 -m unittest discover` → **Ran 31 tests, OK** |

Примечание: pytest-файлы при прямом `python3 file.py` дают пустой вывод и exit 0 —
это **не** признак успеха; корректный прогон требует harness с `sys.path` и
`importlib`. **[Код]**

---

## 4. Проверки, выполненные в рамках анализа

| Проверка | Результат |
|---|---|
| `python3 -m unittest discover -s tools/uart_bench -p 'test_*.py'` (из `esp_firmware/`) | ✅ Ran 31 tests, OK |
| Хостовые тесты регулятора и энкодеров (сборка в `/tmp/host_test_build`) | ✅ пройдены |
| `esp_firmware/tests/test_protocol_v2.cpp` (сборка в `/tmp`) | ✅ PASS |
| `esp_firmware/tests/test_bridge_v2.cpp` vs **рабочий** include моста | ❌ assertion failed, exit 134, core dumped (рабочий парсер принимает `nan`) |
| `esp_firmware/tests/test_bridge_v2.cpp` vs **reference** include | ✅ PASS |
| pytest-наборы `area_patrol` + `web_regulators_configurator` через harness | ✅ 23 passed, 0 failed |
| `-fsyntax-only` по `esp32_firmware/*.cpp` на заглушках | ✅ 7 файлов OK; ❌ 11 файлов — единая причина: нет `uPID.h` |
| `./tools/host_test/syntax_check.sh` | ❌ нет `uPID.h` |
| gtest / colcon / сборка ROS-пакетов | ❌ не выполнены — инструменты и ROS 2 отсутствуют |
| Аппаратные bench-скрипты, запись NVS, управление моторами | ⛔ не запускались (нет оборудования и `pyserial`) |

Артефакты сборки находятся **вне репозитория**: `/tmp/host_test_build`,
`/tmp/bridge_v2_check`. **[Код]**
