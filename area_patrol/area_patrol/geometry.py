"""Геометрия области патрулирования: многоугольник на плоскости карты (frame map).

Чистый Python без ROS — модуль можно проверять отдельно (см. test/test_geometry.py).
Многоугольник — список вершин [(x, y), ...] без повтора первой точки в конце.
"""
import math
import random


def point_in_polygon(x, y, poly):
    """Луч из точки вправо: нечётное число пересечений с рёбрами -> точка внутри."""
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        # ребро пересекает горизонталь y, и точка пересечения правее x
        if (yi > y) != (yj > y):
            x_cross = xi + (y - yi) * (xj - xi) / (yj - yi)
            if x < x_cross:
                inside = not inside
        j = i
    return inside


def polygon_area(poly):
    """Площадь со знаком (формула шнурков): > 0, если вершины идут против часовой."""
    s = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return s / 2.0


def polygon_centroid(poly):
    """Центр масс многоугольника. Для вырожденного (площадь ~0) — среднее вершин."""
    a = polygon_area(poly)
    if abs(a) < 1e-9:
        return (sum(p[0] for p in poly) / len(poly), sum(p[1] for p in poly) / len(poly))
    cx = cy = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        cross = x1 * y2 - x2 * y1
        cx += (x1 + x2) * cross
        cy += (y1 + y2) * cross
    return (cx / (6.0 * a), cy / (6.0 * a))


def distance_to_segment(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    if length2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def distance_to_boundary(x, y, poly):
    """Расстояние от точки до ближайшего ребра многоугольника."""
    n = len(poly)
    return min(
        distance_to_segment(x, y, *poly[i], *poly[(i + 1) % n]) for i in range(n)
    )


def bounding_box(poly):
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    return min(xs), min(ys), max(xs), max(ys)


def random_point_in_polygon(poly, rng=random, edge_margin=0.0, max_tries=1000):
    """Случайная точка внутри многоугольника, не ближе edge_margin к его границе.

    Метод отбора: берём точку в описанном прямоугольнике, пока не попадём внутрь.
    Возвращает (x, y) или None, если за max_tries не нашлось.
    """
    x0, y0, x1, y1 = bounding_box(poly)
    for _ in range(max_tries):
        x = rng.uniform(x0, x1)
        y = rng.uniform(y0, y1)
        if point_in_polygon(x, y, poly) and (
            edge_margin <= 0.0 or distance_to_boundary(x, y, poly) >= edge_margin
        ):
            return (x, y)
    return None


def inner_center_and_radius(poly, edge_margin=0.0, grid=40):
    """Центр и радиус наибольшего круга, вписанного в область (для режимов circle/square).

    Ищем точку, самую удалённую от границы (для выпуклой фигуры это почти центр,
    для вогнутой — середина самой «толстой» части). Сетка grid x grid по описанному
    прямоугольнику, затем два уточнения сеткой помельче вокруг лучшей точки.
    Возвращает ((cx, cy), radius - edge_margin) или None, если круг не помещается.
    """
    x0, y0, x1, y1 = bounding_box(poly)
    best, best_d = None, -1.0
    span_x, span_y = x1 - x0, y1 - y0
    cx0, cy0 = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    for _ in range(3):
        for i in range(grid + 1):
            for j in range(grid + 1):
                x = cx0 - span_x / 2.0 + span_x * i / grid
                y = cy0 - span_y / 2.0 + span_y * j / grid
                if point_in_polygon(x, y, poly):
                    d = distance_to_boundary(x, y, poly)
                    if d > best_d:
                        best, best_d = (x, y), d
        if best is None:
            return None
        # уточняем: окно в 4 шага сетки вокруг лучшей точки
        cx0, cy0 = best
        span_x, span_y = 4.0 * span_x / grid, 4.0 * span_y / grid
    r = best_d - edge_margin
    return (best, r) if r > 0.0 else None


def circle_waypoints(cx, cy, radius, n_points):
    """Точки по окружности против часовой; курс — по касательной (направление движения)."""
    pts = []
    for k in range(n_points):
        a = 2.0 * math.pi * k / n_points
        pts.append((cx + radius * math.cos(a), cy + radius * math.sin(a), a + math.pi / 2.0))
    return pts


def square_waypoints(cx, cy, half_side, points_per_side=2):
    """Точки по квадрату против часовой; курс — вдоль стороны.

    points_per_side: сколько точек ставить на каждую сторону, считая угол (>= 1).
    Промежуточные точки нужны, чтобы планировщик не срезал квадрат по диагонали.
    """
    corners = [
        (cx + half_side, cy - half_side),
        (cx + half_side, cy + half_side),
        (cx - half_side, cy + half_side),
        (cx - half_side, cy - half_side),
    ]
    pts = []
    for i in range(4):
        (x1, y1), (x2, y2) = corners[i], corners[(i + 1) % 4]
        yaw = math.atan2(y2 - y1, x2 - x1)
        for k in range(points_per_side):
            t = k / points_per_side
            pts.append((x1 + t * (x2 - x1), y1 + t * (y2 - y1), yaw))
    return pts
