# lidar_filter_cpp

Фильтр `sensor_msgs/LaserScan` по угловому сектору. Обрезает круговой скан
лидара (по умолчанию до −135°…135°, то есть до 270°), убирая «мёртвую зону»,
занятую корпусом робота. Заменяет Python-пакет `lidar_filter`: параметры,
топики, имя исполняемого файла (`lidar_filter`) и launch-файл
(`filter.launch.py`) те же, изменилось только имя пакета.

## Возможности

- **Быстрая обработка.** Маска сектора считается один раз и пересчитывается
  только при изменении геометрии скана (число лучей, `angle_min`,
  `angle_increment`). Обработка скана сводится к копированию куска массива.
- **Правильное время скана.** `header.stamp` берётся из исходного скана, поэтому
  AMCL, костмапы и SLAM ищут TF на момент измерения. Прежняя Python-версия
  подставляла текущее время.
- **Не зависит от `use_sim_time`.** Фильтр не берёт время из часов, поэтому
  одинаково работает на роботе и в симуляции. В `filter.launch.py` по умолчанию
  `use_sim_time:=False`.
- **Корректная геометрия разрывного сектора.** При `invert_sector:=True` или
  секторе через ±180° выкинутые лучи внутри диапазона заменяются на `NaN`, а углы
  остальных лучей остаются верными. Прежняя версия склеивала оставшиеся куски
  подряд и сдвигала углы. Для сплошного сектора скан просто обрезается.
- **Компонент.** Узел можно загрузить в component container
  (`lidar_filter_cpp::LaserSectorFilter`).

## Запуск

```bash
# отдельно (на роботе или в симуляции)
ros2 launch lidar_filter_cpp filter.launch.py

# свой сектор
ros2 launch lidar_filter_cpp filter.launch.py start_angle_deg:=-90.0 end_angle_deg:=90.0

# в составе всего робота
ros2 launch completed_scripts_jetbot jetbot_sensors_and_motors.launch.py
```

## Параметры

| Параметр | По умолчанию | Описание |
|---|---|---|
| `start_angle_deg` | `-135.0` | начало сектора, град |
| `end_angle_deg` | `135.0` | конец сектора, град |
| `invert_sector` | `false` | оставить всё, кроме сектора |
| `input_topic` | `/scan` | входной топик (QoS BEST_EFFORT) |
| `output_topic` | `/scan_filtered` | выходной топик |
| `output_frame_id` | `""` | frame_id выхода (пусто — как у входа) |

## Тесты

```bash
colcon build --packages-select lidar_filter_cpp
colcon test --packages-select lidar_filter_cpp && colcon test-result --verbose
```
