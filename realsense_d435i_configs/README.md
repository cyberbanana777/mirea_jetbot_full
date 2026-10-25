# realsense_d435i_configs

Пресеты и launch-файл для камеры Intel RealSense D435i робота jetbot_mirea
(драйвер `realsense2_camera`). IMU во всех пресетах выключен.

| mode                   | RGB            | Depth        | PointCloud   | USB  |
|------------------------|----------------|--------------|--------------|------|
| `rgb_usb3`             | 1920x1080 @ 30 | -            | -            | 3.x  |
| `rgb_usb2`             | 640x480 @ 30   | -            | -            | 2.0  |
| `rgb_depth`            | 1280x720 @ 30  | 848x480 @ 30 | -            | 3.x  |
| `rgb_depth_pointcloud` | 848x480 @ 30   | 848x480 @ 30 | да (цветное) | 3.x  |

По умолчанию — `rgb_depth_pointcloud`.

## Запуск

```bash
# в составе всего робота (камера по умолчанию выключена)
ros2 launch completed_scripts_jetbot jetbot_sensors_and_motors.launch.py \
    enable_front_camera:=True camera_mode:=rgb_depth_pointcloud

# отдельно
ros2 launch realsense_d435i_configs d435i.launch.py mode:=rgb_usb3
ros2 launch realsense_d435i_configs d435i.launch.py mode:=rgb_usb2
ros2 launch realsense_d435i_configs d435i.launch.py mode:=rgb_depth
ros2 launch realsense_d435i_configs d435i.launch.py mode:=rgb_depth_pointcloud serial_no:=123456789
```

Аргументы `d435i.launch.py`:

| Аргумент | По умолчанию | Описание |
|---|---|---|
| `mode` | `rgb_depth_pointcloud` | пресет из таблицы выше |
| `camera_name` | `camera` | имя ноды |
| `camera_namespace` | `camera` | namespace ноды |
| `serial_no` | `""` | серийный номер камеры, если их несколько |
| `config_file` | `""` | свой yaml вместо пресета |

## Топики и TF

При настройках по умолчанию нода называется `/camera/camera`, топики:

- `/camera/camera/color/image_raw` — цвет;
- `/camera/camera/depth/image_rect_raw` — глубина;
- `/camera/camera/depth/color/points` — цветное облако точек (`rgb_depth_pointcloud`);
- `/camera/camera/aligned_depth_to_color/image_raw` — глубина в кадре цвета (`rgb_depth`).

Драйвер сам публикует TF своих кадров от `camera_link` (`camera_depth_optical_frame`,
`camera_color_optical_frame` и т.д.) с заводской калибровкой.
`jetbot_sensors_and_motors.launch.py` и `rviz_and_robot_description.launch.py`
привязывают это дерево к модели робота статическим TF `d435_link -> camera_link`
(единичное преобразование). `d435_link` — кадр камеры в URDF, его начало совпадает
с началом модуля глубины, как и у `camera_link`.

## Совместимость

Пресеты проверены по `ros2 param dump` для обёртки 4.58.x (`rgb_camera.color_profile`,
`depth_module.depth_profile`). В пресетах `rgb_usb3`, `rgb_usb2` и `rgb_depth`
дополнительно указаны старые имена (`rgb_camera.profile`, `depth_module.profile`)
для версий ≤ 4.51; лишние параметры нода игнорирует.

Во всех пресетах включён `initial_reset: true`: при старте камера аппаратно
сбрасывается. Это добавляет несколько секунд к запуску, зато лечит «залипшую»
после прошлого запуска камеру.
