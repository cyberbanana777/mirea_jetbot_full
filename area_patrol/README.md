# area_patrol

Бесконечно выдаёт цели Nav2 внутри области, которую обводишь кликами в RViz. ROS 2 Humble.

Режимы: `random` — случайные точки; `circle` — по кругу; `square` — по квадрату.
Круг и квадрат вписываются в область автоматически (или задай `shape_radius`).

## Установка

Положить папку `area_patrol` в `src` рабочего пространства на ноутбуке (где работает Nav2):

```bash
sudo apt install ros-humble-nav2-simple-commander   # если ещё не стоит
colcon build --packages-select area_patrol
source install/setup.bash
```

## Запуск

1. Навигация с картой (AMCL + Nav2) уже работает, в RViz **Fixed Frame = map**.
2. Запустить узел:
   ```bash
   ros2 launch area_patrol area_patrol.launch.py                        # random
   ros2 launch area_patrol area_patrol.launch.py mode:=circle           # или square
   ros2 launch area_patrol area_patrol.launch.py use_sim_time:=true     # в симуляции
   ```
3. В RViz инструмент **Publish Point** (панель сверху): кликать вершины области по очереди.
   Замкнуть — кликнуть рядом с первой точкой (ближе 0.2 м). Робот поедет.
4. Показать область и цели: в RViz `Add -> By topic -> /area_patrol/markers` (MarkerArray).
   Жёлтым — область, пока рисуется; зелёным — готовая; красным — текущие цели.

Область сохраняется в `~/.ros/area_patrol/area.yaml` и подхватывается при следующем запуске.
Новая область:

```bash
ros2 service call /area_patrol/reset_area std_srvs/srv/Trigger
```

Остановить — `Ctrl+C`: текущая цель Nav2 отменяется, робот встаёт.

## Как выбираются цели

- Точка внутри области не ближе `edge_margin` к её границе.
- Клетка в `/global_costmap/costmap` не дороже `max_cost` (0..100): так цель не попадает
  в препятствие и в зону раздувания вокруг него. Неизвестные клетки отбрасываются.
- `random`: не ближе `min_goal_distance` к роботу, и планировщик Nav2 находит путь.
  Курс в цели — по направлению движения (меньше доворота).
- `circle` / `square`: точки фигуры в занятых клетках пропускаются; круг начинается
  с точки, ближайшей к роботу, и едется без остановок (`NavigateThroughPoses`).
- Цель не достигнута за `goal_timeout` или Nav2 не справился — берётся следующая.

Все параметры — в `config/area_patrol.yaml`.

## Важно

- Область привязана к карте. Новая карта (другой `origin`) — сбросить область.
- Узел ждёт только `bt_navigator` и не трогает начальную позу AMCL.
