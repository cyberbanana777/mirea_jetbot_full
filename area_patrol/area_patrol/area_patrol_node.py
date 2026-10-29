"""area_patrol — бесконечно выдаёт цели Nav2 внутри области, обведённой кликами в RViz.

Как пользоваться:
  1. Запустить навигацию с картой (AMCL + Nav2) и RViz с Fixed Frame = map.
  2. ros2 launch area_patrol area_patrol.launch.py          (mode:=random | circle | square)
  3. В RViz инструментом «Publish Point» обвести область по вершинам.
     Замкнуть — кликнуть рядом с первой точкой (ближе close_distance).
  4. Робот поехал. Область сохраняется в area_file и подхватывается при следующем запуске.
  Сбросить область и нарисовать заново:
     ros2 service call /area_patrol/reset_area std_srvs/srv/Trigger
  Остановить — Ctrl+C (текущая цель Nav2 отменяется, робот встаёт).

Режимы:
  random — случайные точки внутри области: свободные по global costmap
           и достижимые (проверка через планировщик).
  circle — круг, вписанный в область, по кругу без остановок (NavigateThroughPoses).
  square — квадрат, вписанный в тот же круг.
"""
import math
import os
import random
import time

import rclpy
import yaml
from geometry_msgs.msg import Point, PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from area_patrol import geometry as geo

MODES = ('random', 'circle', 'square')

# Как у Nav2 для карт и amcl_pose: последнее сообщение получает и опоздавший подписчик
LATCHED_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class AreaPatrol(BasicNavigator):

    def __init__(self):
        try:
            super().__init__(node_name='area_patrol')
        except TypeError:  # на случай версии BasicNavigator без node_name
            super().__init__()

        self.mode = self.declare_parameter('mode', 'random').value
        self.area_file = os.path.expanduser(
            self.declare_parameter('area_file', '~/.ros/area_patrol/area.yaml').value)
        self.load_saved_area = self.declare_parameter('load_saved_area', True).value
        self.close_distance = self.declare_parameter('close_distance', 0.2).value
        self.edge_margin = self.declare_parameter('edge_margin', 0.1).value
        self.min_goal_distance = self.declare_parameter('min_goal_distance', 0.5).value
        self.max_cost = self.declare_parameter('max_cost', 50).value
        self.max_path_checks = self.declare_parameter('max_path_checks', 10).value
        self.goal_timeout = self.declare_parameter('goal_timeout', 180.0).value
        self.pause_between_goals = self.declare_parameter('pause_between_goals', 1.0).value
        self.shape_radius = self.declare_parameter('shape_radius', 0.0).value
        self.circle_points = self.declare_parameter('circle_points', 12).value
        self.square_points_per_side = self.declare_parameter('square_points_per_side', 2).value
        costmap_topic = self.declare_parameter(
            'costmap_topic', '/global_costmap/costmap').value

        if self.mode not in MODES:
            raise ValueError(f'mode должен быть одним из {MODES}, а не {self.mode!r}')

        self.rng = random.Random()
        self.points = []            # вершины, пока область рисуется
        self.polygon = None         # замкнутая область [(x, y), ...]
        self.costmap = None         # nav_msgs/OccupancyGrid, значения 0..100, -1 — неизвестно
        self.robot_xy = None        # (x, y) по amcl_pose
        self.reset_requested = False
        self.goal_markers = []      # точки текущей цели/круга — для RViz

        self.create_subscription(PointStamped, '/clicked_point', self._on_click, 10)
        self.create_subscription(OccupancyGrid, costmap_topic, self._on_costmap, LATCHED_QOS)
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl_pose, LATCHED_QOS)
        self.create_service(Trigger, '~/reset_area', self._on_reset)
        self.marker_pub = self.create_publisher(MarkerArray, '~/markers', LATCHED_QOS)

        if self.load_saved_area:
            self._load_area()
        self._publish_markers()

    # ------------------------------------------------------------------ подписки

    def _on_costmap(self, msg):
        self.costmap = msg

    def _on_amcl_pose(self, msg):
        p = msg.pose.pose.position
        self.robot_xy = (p.x, p.y)

    def _on_click(self, msg):
        if self.polygon is not None:
            self.get_logger().info(
                'Область уже задана. Чтобы нарисовать новую: '
                'ros2 service call /area_patrol/reset_area std_srvs/srv/Trigger')
            return
        if msg.header.frame_id.lstrip('/') != 'map':
            self.get_logger().warn(
                f'Точка пришла во фрейме {msg.header.frame_id!r}, нужен map. '
                'Поставь в RViz Fixed Frame = map.')
            return

        x, y = msg.point.x, msg.point.y
        if len(self.points) >= 3:
            x0, y0 = self.points[0]
            if math.hypot(x - x0, y - y0) <= self.close_distance:
                self.polygon = list(self.points)
                self.points = []
                self._save_area()
                self.get_logger().info(
                    f'Область замкнута: {len(self.polygon)} вершин, '
                    f'площадь {abs(geo.polygon_area(self.polygon)):.2f} м². Поехали.')
                self._publish_markers()
                return

        self.points.append((x, y))
        hint = ' Замкнуть — кликнуть рядом с первой точкой.' if len(self.points) >= 3 else ''
        self.get_logger().info(f'Вершина {len(self.points)}: ({x:.2f}; {y:.2f}).{hint}')
        self._publish_markers()

    def _on_reset(self, request, response):
        self.polygon = None
        self.points = []
        self.goal_markers = []
        self.reset_requested = True     # главный цикл отменит текущую цель
        self._publish_markers()
        self.get_logger().info('Область сброшена. Обведи новую инструментом Publish Point.')
        response.success = True
        response.message = 'area reset'
        return response

    # ------------------------------------------------------------------ файл области

    def _save_area(self):
        try:
            os.makedirs(os.path.dirname(self.area_file), exist_ok=True)
            with open(self.area_file, 'w', encoding='utf-8') as f:
                yaml.safe_dump(
                    {'frame_id': 'map', 'points': [[float(x), float(y)] for x, y in self.polygon]},
                    f, allow_unicode=True)
            self.get_logger().info(f'Область сохранена в {self.area_file}')
        except OSError as e:
            self.get_logger().warn(f'Не удалось сохранить область: {e}')

    def _load_area(self):
        if not os.path.isfile(self.area_file):
            self.get_logger().info(
                'Сохранённой области нет. Обведи область в RViz инструментом Publish Point.')
            return
        try:
            with open(self.area_file, encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            pts = [(float(p[0]), float(p[1])) for p in data.get('points', [])]
        except (OSError, yaml.YAMLError, TypeError, ValueError, IndexError) as e:
            self.get_logger().warn(f'Не удалось прочитать {self.area_file}: {e}')
            return
        if len(pts) < 3:
            self.get_logger().warn(f'В {self.area_file} меньше трёх точек — игнорирую.')
            return
        self.polygon = pts
        self.get_logger().info(
            f'Загружена область из {self.area_file}: {len(pts)} вершин. '
            'Новая — через сервис reset_area.')

    # ------------------------------------------------------------------ проверки точек

    def _cost_at(self, x, y):
        """Стоимость клетки global costmap (0..100) или None — вне карты / нет карты."""
        m = self.costmap
        if m is None:
            return None
        info = m.info
        mx = int(math.floor((x - info.origin.position.x) / info.resolution))
        my = int(math.floor((y - info.origin.position.y) / info.resolution))
        if not (0 <= mx < info.width and 0 <= my < info.height):
            return None
        return m.data[my * info.width + mx]

    def _is_free(self, x, y):
        """Свободно и вне зоны раздувания: -1 (неизвестно) и высокая стоимость — нет."""
        c = self._cost_at(x, y)
        return c is not None and 0 <= c <= self.max_cost

    def _make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _is_reachable(self, goal):
        """Спросить планировщик, есть ли путь от текущей позы робота до цели."""
        path = self.getPath(PoseStamped(), goal)     # use_start=False -> от позы робота
        return path is not None and len(path.poses) > 0

    # ------------------------------------------------------------------ выбор целей

    def _pick_random_goal(self):
        path_checks = 0
        for _ in range(500):
            # getPath внутри крутит узел: за это время могли вызвать reset_area
            if self.polygon is None or self.reset_requested:
                return None
            p = geo.random_point_in_polygon(self.polygon, self.rng, self.edge_margin)
            if p is None:
                self.get_logger().error(
                    'В области нет места с отступом edge_margin от края — уменьши edge_margin.')
                return None
            x, y = p
            if not self._is_free(x, y):
                continue
            if self.robot_xy is not None:
                rx, ry = self.robot_xy
                if math.hypot(x - rx, y - ry) < self.min_goal_distance:
                    continue
                yaw = math.atan2(y - ry, x - rx)    # смотрим по ходу движения — меньше доворота
            else:
                yaw = self.rng.uniform(-math.pi, math.pi)
            goal = self._make_pose(x, y, yaw)
            reachable = self._is_reachable(goal)
            if self.polygon is None or self.reset_requested:
                return None     # область сбросили, пока планировщик считал путь
            if reachable:
                return goal
            path_checks += 1
            if path_checks >= self.max_path_checks:
                break
        return None

    def _make_lap(self):
        res = geo.inner_center_and_radius(self.polygon, self.edge_margin)
        if res is None:
            self.get_logger().error('Область слишком узкая для круга/квадрата. Уменьши edge_margin.')
            return None
        (cx, cy), r_max = res
        r = r_max
        if self.shape_radius > 0.0:
            if self.shape_radius > r_max:
                self.get_logger().warn(
                    f'shape_radius {self.shape_radius:.2f} м не помещается в область, '
                    f'беру максимум {r_max:.2f} м.')
            r = min(self.shape_radius, r_max)

        if self.mode == 'circle':
            pts = geo.circle_waypoints(cx, cy, r, max(3, self.circle_points))
        else:
            pts = geo.square_waypoints(cx, cy, r / math.sqrt(2.0),
                                       max(1, self.square_points_per_side))

        free = [p for p in pts if self._is_free(p[0], p[1])]
        if len(free) < len(pts):
            self.get_logger().warn(
                f'{len(pts) - len(free)} из {len(pts)} точек фигуры заняты препятствиями — пропускаю их.')
        if len(free) < 2:
            self.get_logger().error('Свободных точек фигуры меньше двух. Нарисуй область побольше.')
            return None

        # начинаем с точки, ближайшей к роботу, — не ехать через весь круг к «нулевой»
        if self.robot_xy is not None:
            rx, ry = self.robot_xy
            k = min(range(len(free)), key=lambda i: math.hypot(free[i][0] - rx, free[i][1] - ry))
            free = free[k:] + free[:k]
        free.append(free[0])  # замкнуть круг: последняя цель — снова первая точка
        self.get_logger().info(
            f'{self.mode}: центр ({cx:.2f}; {cy:.2f}), радиус {r:.2f} м, {len(free) - 1} точек.')
        return [self._make_pose(x, y, yaw) for x, y, yaw in free]

    # ------------------------------------------------------------------ движение

    def _drive(self, goal=None, lap=None):
        """Отправить цель (или круг) и ждать результата. Отменяет по таймауту и reset_area."""
        if lap is not None:
            self.goal_markers = [(p.pose.position.x, p.pose.position.y) for p in lap]
            for p in lap:
                p.header.stamp = self.get_clock().now().to_msg()
            accepted = self.goThroughPoses(lap)
        else:
            self.goal_markers = [(goal.pose.position.x, goal.pose.position.y)]
            accepted = self.goToPose(goal)
        self._publish_markers()
        if not accepted:
            self.get_logger().warn('Nav2 отклонил цель.')
            return

        timeout = self.goal_timeout     # на одну цель или на один круг
        start = time.monotonic()
        while not self.isTaskComplete():          # внутри крутится spin_once(0.1 с)
            if self.reset_requested:
                self.cancelTask()
                return
            if time.monotonic() - start > timeout:
                self.get_logger().warn(f'Цель не достигнута за {timeout:.0f} с — отменяю.')
                self.cancelTask()
                return

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Доехал.')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Цель отменена.')
        else:
            self.get_logger().warn('Nav2 не смог доехать — беру следующую цель.')

    def _wait(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self):
        self.get_logger().info(f'Режим {self.mode}. Жду, пока Nav2 станет активным...')
        # localizer НЕ 'amcl': иначе BasicNavigator сам опубликует initialpose в (0; 0)
        # и собьёт AMCL. Ждём только bt_navigator.
        self.waitUntilNav2Active(navigator='bt_navigator', localizer='bt_navigator')
        self.get_logger().info('Nav2 активен.')

        waiting_logged = False
        while rclpy.ok():
            if self.reset_requested:
                self.reset_requested = False
            if self.polygon is None or self.costmap is None:
                if not waiting_logged:
                    what = 'область (Publish Point в RViz)' if self.polygon is None \
                        else 'global costmap'
                    self.get_logger().info(f'Жду: {what}.')
                    waiting_logged = True
                rclpy.spin_once(self, timeout_sec=0.2)
                continue
            waiting_logged = False

            if self.mode == 'random':
                goal = self._pick_random_goal()
                if goal is None and (self.polygon is None or self.reset_requested):
                    continue        # область сбросили, пока выбирали цель
                if goal is None:
                    self.get_logger().warn(
                        'Не нашёл свободную достижимую точку в области — пробую снова через 2 с.')
                    self._wait(2.0)
                    continue
                self.get_logger().info(
                    f'Новая цель: ({goal.pose.position.x:.2f}; {goal.pose.position.y:.2f}).')
                self._drive(goal=goal)
            else:
                lap = self._make_lap()
                if lap is None:
                    self._wait(2.0)
                    continue
                self._drive(lap=lap)

            self._wait(self.pause_between_goals)

    # ------------------------------------------------------------------ RViz

    def _publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        verts = self.polygon if self.polygon is not None else self.points
        closed = self.polygon is not None
        if verts:
            line = self._marker(0, Marker.LINE_STRIP, stamp, 0.02,
                                (0.1, 0.8, 0.2) if closed else (1.0, 0.8, 0.0))
            pts = list(verts) + ([verts[0]] if closed else [])
            line.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in pts]
            arr.markers.append(line)

            dots = self._marker(1, Marker.SPHERE_LIST, stamp, 0.06,
                                (0.1, 0.8, 0.2) if closed else (1.0, 0.8, 0.0))
            dots.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in verts]
            arr.markers.append(dots)

        if self.goal_markers and closed:
            goals = self._marker(2, Marker.SPHERE_LIST, stamp, 0.08, (0.9, 0.1, 0.1))
            goals.points = [Point(x=float(x), y=float(y), z=0.05) for x, y in self.goal_markers]
            arr.markers.append(goals)

        self.marker_pub.publish(arr)

    @staticmethod
    def _marker(mid, mtype, stamp, scale, rgb):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = stamp
        m.ns = 'area_patrol'
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b = rgb
        m.color.a = 1.0
        return m


def main(args=None):
    # Ctrl+C не должен сразу гасить контекст rclpy: сначала отменим цель, иначе робот доедет её.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)

    node = AreaPatrol()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Остановка: отменяю текущую цель.')
        if rclpy.ok() and node.result_future is not None:
            node.cancelTask()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
