# TODO: документация после обновления низкоуровневого ПО

Python-пакеты `serial_bridge_package` и `lidar_filter` удалены, их заменили
C++-пакеты `serial_bridge_cpp` и `lidar_filter_cpp`. Пакет `realsense_pkg`
удалён, камеру теперь запускает `realsense_d435i_configs`. Методички
(`docs/Короткие_инструкции`) и справочники (`docs/Справочная_информация`) пока
описывают старое состояние. Ниже перечислено, что и где нужно поправить.

Актуальное описание новых пакетов лежит в `serial_bridge_cpp/README.md`,
`lidar_filter_cpp/README.md` и `realsense_d435i_configs/README.md`, история
изменений — в `CHANGELOG.rst` пакетов.

> У каждого `.docx` есть PDF-двойник с тем же именем (`docx/` ↔ `pdf/`).
> После правки `.docx` нужно заново сохранить из него PDF.

## Кратко: что изменилось для пользователя

| Было | Стало |
|---|---|
| `serial_bridge_package`: 3 ноды (`twist_to_command`, `serial_bridge_node`, `feedback_processor`) | `serial_bridge_cpp`: 1 нода `serial_bridge_node` |
| `ros2 launch serial_bridge_package serial_bringup.launch.py` | `ros2 launch serial_bridge_cpp serial_bringup.launch.py` |
| `lidar_filter` (Python, `lidar_filter.py`) | `lidar_filter_cpp` (C++), имя исполняемого файла то же: `lidar_filter` |
| `ros2 launch lidar_filter filter.launch.py` | `ros2 launch lidar_filter_cpp filter.launch.py` |
| `ros2 run lidar_filter lidar_filter` | `ros2 run lidar_filter_cpp lidar_filter` |
| `use_sim_time` в `filter.launch.py` по умолчанию `True` | по умолчанию `False`; фильтру он больше не нужен: время скана копируется из исходного сообщения |
| камера: `realsense2_camera/rs_launch.py` + `completed_scripts_jetbot/config/realsense_config.yaml` (конфиг фактически не применялся) | `realsense_d435i_configs/launch/d435i.launch.py`, пресет выбирается аргументом `camera_mode` (по умолчанию `rgb_depth_pointcloud`) |
| статический TF камеры `realsense_link -> camera_link` (`realsense_link` нет в URDF) | `d435_link -> camera_link` |

Не изменилось: топики `/cmd_vel`, `/odom`, TF `odom -> base_link`,
`/sensors/wheel/{left,right}/speed`, `/PIDcoeffs`, `/low_level/serial/cmd`,
`/low_level/serial/feedback`, `/scan`, `/scan_filtered`; протокол ESP32
(приложение Б справочника 2 и схема `docs/Схемы/Протокол связи c ESP32`);
команда `ros2 launch completed_scripts_jetbot jetbot_sensors_and_motors.launch.py`
и её прежние аргументы (добавился только `camera_mode`).

---

## Методички (`docs/Короткие_инструкции`)

### [ ] S2 — `S2_Как_запустить_режим_картографирования_для_симуляции`

**Важно: сейчас команда не работает, пакета `lidar_filter` больше нет.**

- Шаг «Фильтр ограничивает угол сканирования лидара…»:
  `ros2 launch lidar_filter filter.launch.py` →
  `ros2 launch lidar_filter_cpp filter.launch.py`.
  Аргумент `use_sim_time:=True` добавлять не нужно: в симуляции фильтр работает
  и без него.

### [ ] S3 — `S3_Как_запустить_режим_автономной_навигации_по_карте_для_симуляции`

**Важно: сейчас команда не работает.**

- Шаг «Запустите фильтр лидара»: та же замена, что в S2:
  `ros2 launch lidar_filter_cpp filter.launch.py`.

---

## Справочники (`docs/Справочная_информация`)

### [ ] `2_jetbot_drivers` — глава «ROS2-пакет для взаимодействия с ESP32 через SerialPort – «serial_bridge_package»» (стр. 8–14 по оглавлению)

Главу нужно переписать под `serial_bridge_cpp`. Что устарело:

- [ ] **Название главы и пункт оглавления:** `serial_bridge_package` → `serial_bridge_cpp`.
- [ ] **«Описание»:** «В пакете содержатся 3 ноды» → одна нода `serial_bridge_node`.
  Она сама переводит `/cmd_vel` в протокол ESP32, общается с портом и разбирает
  ответы в ROS-сообщения.
- [ ] **Пункты 2.1–2.3** (ноды `twist_to_command`, `serial_bridge_node`,
  `feedback_processor`) объединить в один пункт про `serial_bridge_node`.
- [ ] **Таблицы 1, 2, 4 (интерфейсы ROS2)** заменить одной таблицей:
  - подписки: `/cmd_vel` (`geometry_msgs/msg/Twist`), `/low_level/serial/cmd`
    (`std_msgs/msg/String`, сырые команды `$...#` в порт);
  - публикации: `/odom` (`nav_msgs/msg/Odometry`), TF `odom -> base_link`,
    `/sensors/wheel/left/speed` и `/sensors/wheel/right/speed`
    (`std_msgs/msg/Float32`), `/PIDcoeffs` (`std_msgs/msg/String`),
    `/low_level/serial/feedback` (`std_msgs/msg/String`, сырые кадры от ESP32),
    `/low_level/serial/text` (`std_msgs/msg/String`, текст ESP32 вне кадров:
    ошибки, Wi-Fi, OTA; с `serial_bridge_cpp` 1.2.0).
- [ ] **Топики, которых больше нет:** `/esp32_input`, `/esp32_feedback`,
  `serial/from`, `serial/to`. Они упоминаются в тексте нод и в таблицах.
- [ ] **Таблицы 3 и 5 (параметры)** заменить одной таблицей параметров:
  `serial_port` (`/dev/esp32`), `baudrate` (`115200`), `frame_id` (`odom`),
  `child_frame_id` (`base_link`), `publish_tf` (`true`),
  `publish_raw_feedback` (`true`), `log_text_messages` (`true`),
  `reconnect_period` (`1.0` с).
  Параметров `ros_in_topic` и `ros_out_topic` больше нет. Ещё в таблице 3
  значение по умолчанию записано как `dev/esp32` без начального `/`.
- [ ] **`child_frame_id`:** в таблицах 5 и 6 указан `base_footprint`. В
  launch-файле и раньше был `base_link`, сейчас по умолчанию тоже `base_link`.
- [ ] **Команды запуска нод** (`ros2 run serial_bridge_package twist_to_command` /
  `serial_bridge_node` / `feedback_processor`) заменить одной:
  `ros2 run serial_bridge_cpp serial_bridge_node --ros-args -r __ns:=/low_level`.
  Пространство имён нужно, чтобы топики были `/low_level/serial/...`: их ждёт
  PID-конфигуратор. Проще запускать через launch-файл.
- [ ] **Фраза «Ожидает строго форматированные сообщения … без обработки ошибок
  связи»** больше не верна. Теперь новый `$` начинает кадр заново, текст вне
  кадров и слишком длинные кадры отбрасываются, а при потере порта нода сама
  переоткрывает его раз в `reconnect_period`.
- [ ] **Пункт 2.4 «Launch-файл», таблица 6:** одна нода `serial_bridge_node` в
  пространстве имён `low_level`; аргументы `serial_port`, `baudrate`,
  `publish_raw_feedback`. Перепривязок топиков больше нет. Команда:
  `ros2 launch serial_bridge_cpp serial_bringup.launch.py`.
- [ ] **Фраза «Все ноды имеют имена такие же, как и точки входа из файла
  установки (setup.py)»:** пакет теперь на CMake, `setup.py` нет.
- [ ] **Рисунок 1 (RQT-граф)** переснять: на нём три ноды, теперь одна.
- [ ] **Добавить:** нода — компонент `serial_bridge_cpp::SerialBridgeNode`
  (можно грузить в component container). В пакете есть конфиг RViz
  `rviz/show_all_stats.rviz`, перенесённый из старого пакета. Нагрузка на CPU:
  93% одного ядра у Python-версии против 0.4% у C++ (замер на эмуляторе ESP32).
- [ ] **«Особенности»** (про симлинк `/dev/esp32`) актуальны. Можно дописать
  про автоматическое переподключение порта.
- [ ] **Добавить:** в `/odom` заполняются ковариации (параметры
  `pose_covariance_diagonal`, `twist_covariance_diagonal`); значения по
  умолчанию — заглушки, пока дисперсии не измерены (см. `IMPROVEMENTS.md`).
  Одометрия приходит с частотой 50 Гц.

### [ ] `2_jetbot_drivers` — глава «Реализация приводного уровня в ПО (прошивке) ESP32 – «esp_firmware»» (стр. 6–8)

Прошивка 2.1.0 (см. `esp_firmware/CHANGELOG.rst`).

- [ ] **«Аварийная остановка при зависании Jetson Nano»:** «Если с Jetson Nano
  сообщения не приходят в течение 5 секунд…» → 1 секунды.
- [ ] **«Подсчёт одометрии робота»:** дописать, что одометрия отправляется с
  частотой 50 Гц (каждые 20 мс), поза считается по пройденному колёсами пути,
  а ПИД-регуляторы работают раз в 100 мс. Скорости в сообщении одометрии
  обновляются раз в 100 мс.
- [ ] **«Использование»** («В нормальном режиме ESP32 отсылает на Jetson Nano
  сообщения-ответы типа 1»): указать частоту — 50 Гц.

### [ ] `2_jetbot_drivers` — глава «ROS2-пакет для настройки ПИД-регуляторов … «simple_regulators_configurator_pkg»» (стр. 14–16)

- [ ] **«Проводная настройка»** и **«Беспроводная настройка → Jetson Nano»**:
  `ros2 launch serial_bridge_package serial_bringup.launch.py` →
  `ros2 launch serial_bridge_cpp serial_bringup.launch.py` (два места).
  Сам конфигуратор менять не нужно: топики прежние.

### [ ] `3_Доп_ПО` — обзор архитектуры (уровни системы, таблица 1)

- [ ] **Уровень 2, «Моторы»:** `serial_bridge_package` → `serial_bridge_cpp`.
- [ ] **Уровень 3:** «Пакет lidar_filter (ros2-пакет ограничивающий угол обзора
  лидара - lidar_filter)» → `lidar_filter_cpp`.
- [ ] **Таблица 1 «Сценарии и launch-файлы»,** колонка «Задействованные пакеты»:
  «lidar_filter (уровень 3)» → `lidar_filter_cpp` (строки «Запуск сенсоров и
  моторов» в обеих копиях таблицы).

### [ ] `3_Доп_ПО` — глава «ROS2-пакет, ограничивающий угол обзора лидара – «lidar_filter»» (стр. 14 по оглавлению)

- [ ] **Название главы и пункт оглавления:** `lidar_filter` → `lidar_filter_cpp`.
- [ ] **«В пакете содержится 1 нода (lidar_filter.py)»** → C++-нода
  (`src/lidar_filter_node.cpp`), исполняемый файл `lidar_filter`; плюс
  launch-файл `filter.launch.py`.
- [ ] **Команды:** `ros2 run lidar_filter lidar_filter` →
  `ros2 run lidar_filter_cpp lidar_filter`;
  `ros2 launch lidar_filter filter.launch.py` →
  `ros2 launch lidar_filter_cpp filter.launch.py`.
- [ ] **Абзац про `use_sim_time`** («По умолчанию его значение в этом файле
  задаётся как «True»…») переписать. Теперь по умолчанию `False`, и на работу
  фильтра значение не влияет: время выходного скана берётся из исходного
  сообщения (раньше подставлялось текущее время, поэтому параметр был важен).
- [ ] **Таблица 4 (параметры), `output_frame_id`:** в колонке «Значение по
  умолчанию» стоит описание («идентификатор_фрейма для темы вывода»).
  Правильно: значение по умолчанию `""` (пусто — frame_id как у входного скана),
  а описание перенести в «Комментарий».
- [ ] **Добавить в описание:**
  - время выходного скана совпадает с исходным (важно для TF в AMCL, SLAM и
    костмапах);
  - при разрывном секторе (`invert_sector:=True` или сектор через ±180°)
    выкинутые лучи внутри диапазона заменяются на `NaN`, а углы остальных лучей
    не сдвигаются (старая версия склеивала куски скана, и углы съезжали);
  - маска сектора считается один раз, а не на каждом скане;
  - нода — компонент `lidar_filter_cpp::LaserSectorFilter`.

### [ ] `3_Доп_ПО` — раздел «Launch-файл jetbot_sensors_and_motors.launch.py»

- [ ] Список подсистем: «Лидар (пакет sllidar_ros2) и его фильтр (пакет
  lidar_filter)» → `lidar_filter_cpp`; «Моторы через последовательный мост
  (пакет serial_bridge_package)» → `serial_bridge_cpp`.
- [ ] Таблица аргументов (таблица 7): добавить `camera_mode` (string,
  `rgb_depth_pointcloud`) — пресет камеры: `rgb_usb3`, `rgb_usb2`, `rgb_depth`,
  `rgb_depth_pointcloud`; действует при `enable_front_camera:=True`.
  Остальные аргументы не изменились.

### [ ] Камера RealSense: `realsense_pkg` → `realsense_d435i_configs`

Затрагивает `3_Доп_ПО` и `2_jetbot_drivers`.

- [ ] **`3_Доп_ПО`, уровень 2:** «Камера Intel RealSense D435i: пакет
  realsense2_camera (реальный робот)…» → драйвер `realsense2_camera`, пресеты и
  launch-файл в пакете `realsense_d435i_configs`.
- [ ] **`3_Доп_ПО`, раздел «Launch-файл rviz_and_robot_description.launch.py»:**
  «Статический трансформ между фреймами realsense_link и camera_link» →
  `d435_link` и `camera_link`. `d435_link` — кадр камеры в URDF, `camera_link` —
  корень дерева TF, которое публикует драйвер камеры.
- [ ] **`3_Доп_ПО`, раздел «Launch-файл jetbot_sensors_and_motors.launch.py»:**
  «Фронтальная камера Intel RealSense D435i (пакет realsense2_camera) с
  пользовательской конфигурацией» → камера запускается через
  `realsense_d435i_configs/launch/d435i.launch.py`, конфигурация выбирается
  аргументом `camera_mode` (таблица пресетов — в README пакета). Аргумент
  `camera_mode` добавить в таблицу 7 (см. выше).
- [ ] **`3_Доп_ПО`, «Примечания и структура пакета» → «Конфигурационные
  файлы»:** строку «realsense_config.yaml – пользовательские настройки камеры
  RealSense» убрать: файл удалён, пресеты лежат в
  `realsense_d435i_configs/config/`.
- [ ] **`2_jetbot_drivers`, глава «ROS2-пакет для взаимодействия с камерой
  глубины»:** сейчас там написано «Взаимодействие с данным модулем ещё не было
  реализовано». Описать `realsense_d435i_configs`:
  - пресеты (`rgb_usb3`, `rgb_usb2`, `rgb_depth`, `rgb_depth_pointcloud`) и
    команду `ros2 launch realsense_d435i_configs d435i.launch.py mode:=...`;
  - запуск в составе робота:
    `ros2 launch completed_scripts_jetbot jetbot_sensors_and_motors.launch.py enable_front_camera:=True`;
  - топики `/camera/camera/color/image_raw`, `/camera/camera/depth/image_rect_raw`,
    `/camera/camera/depth/color/points`;
  - TF `d435_link -> camera_link`;
  - нужен порт USB 3.x (кроме пресета `rgb_usb2`).
- [ ] **Учесть разницу с симуляцией:** в Gazebo плагин камеры публикует топики
  без namespace (`/color/image_raw`, `/depth/image_raw`, `/depth/color/points`)
  в кадрах `d435_*_optical_frame`, а реальная камера — `/camera/camera/...` в
  кадрах `camera_*_optical_frame`. Если в методичках появятся команды для
  камеры, их нужно давать отдельно для робота и для симуляции.

---

### [ ] Веб-настройщик ПИД `web_regulators_configurator`

Затрагивает методичку R3 и `2_jetbot_drivers`.

- [ ] **R3 «Как настроить ПИД-регуляторы»:** добавить способ через веб-страницу:
  `ros2 launch completed_scripts_jetbot jetbot_sensors_and_motors.launch.py enable_web_configurator:=True`,
  затем открыть `http://<IP робота>:8080` (IP — на OLED-дисплее). Описать
  ступенчатый тест как основной способ подбора коэффициентов.
- [ ] **`2_jetbot_drivers`:** добавить главу про пакет (по образцу глав про
  `simple_*`): возможности, запуск, параметры, безопасность («мёртвая рука»,
  вкладка должна быть активной), Docker (`--network host` или `-p 8080:8080`).
  Содержание — в `web_regulators_configurator/README.md`.
- [ ] **`3_Доп_ПО`, таблица 7:** аргументы `enable_web_configurator` и
  `web_configurator_port` у `jetbot_sensors_and_motors.launch.py`.

---

## Замечено попутно (к переходу на C++ не относится)

- [ ] `2_jetbot_drivers`, приложение Г, и методичка S1 ссылаются на
  `init_scripts/install_deps.bash`. Такого файла нет: в репозитории есть
  `install_dependensies_jetbot.bash` и `install_dependensies_host.bash`
  (методичка A1 называет их правильно).
- [ ] `2_jetbot_drivers`, приложение Г: после `mkdir -p jetbot_ws/src` и
  `cd jetbot_ws/src` идёт `cd ../..`. Это выход на уровень выше `jetbot_ws`, и
  `colcon build` запустится не в рабочем пространстве. Нужно `cd ..`.
- [ ] `2_jetbot_drivers`, глава `sllidar_ros2`, «Ссылки»: текст ссылки
  `https://github.com/Slamtec/sllidar_ros2`, а сама ссылка ведёт на
  `https://github.com/waveshare/jetbot/tree/master`.
- [ ] `3_Доп_ПО`, таблица 1: опечатки в именах launch-файлов —
  `jetbot_sensors_and_motos.launch.py` (надо `motors`) и
  `rviz_and_robot_descriptin.launch.py` (надо `description`). С такими
  именами команды не запустятся.
- [ ] Методичка R3 и `2_jetbot_drivers` (глава «simple_PID_configurator»):
  `pip install -r pip_requirements.txt` и `python3 fast_gui.py`. В папке
  `simple_PID_configurator` таких файлов нет, есть `pip_requirements_tk.txt`
  / `pip_requirements_pyside.txt` и `fast_gui_tk.py` / `fast_gui_pyside.py`.
  Ещё в `config.py` по умолчанию стоит `SERIAL_PORT = '/dev/ttyUSB0'`, а в R3
  показан `'/dev/esp32'`.
- [ ] `3_Доп_ПО`, «Launch-файл rviz_and_robot_description.launch.py»:
  написано, что он запускает `jetbot_mirea_rviz.launch.py`. Такого файла нет,
  на самом деле подключается `jetbot_mirea_description/launch/real_robot.launch.py`.
- [ ] `3_Доп_ПО`, «Конфигурационные файлы»: `amcl_config_real.yaml`,
  `amcl_config_sim.yaml` → на самом деле `amcl_params_real.yaml`,
  `amcl_params_sim.yaml`.
- [ ] `3_Доп_ПО`: нумерация таблиц в тексте не совпадает с подписями
  («В таблице 2…» над «Таблица 3», «В таблице 9…» над «Таблица 7»).

---

## Проверено, правок не требуют

Методички A0, A1, A1.5, A2.1, A2.2, A3, N1, R1, R2, R3, R4, R5, S1;
справочник `1_Техничка`; схемы в `docs/Схемы` (монтажная, структурная,
протокол ESP32); `docs/Полезные ссылки.xlsx`.

- R1: ожидаемые топики (`/scan`, `/odom`, `/cmd_vel`) остались прежними.
- R3: `simple_PID_configurator` работает с портом напрямую, без ROS, поэтому
  замена пакетов его не касается. Как и раньше, его нельзя запускать
  одновременно с `jetbot_sensors_and_motors.launch.py`: порт `/dev/esp32` будет
  занят.
