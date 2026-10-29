import math
import random

from area_patrol import geometry as g

SQUARE = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
# буква «L»: и центр масс (1.1; 1.1), и центр прямоугольника (1.5; 1.5) лежат в вырезе
L_SHAPE = [(0, 0), (3, 0), (3, 1), (1, 1), (1, 3), (0, 3)]


def test_point_in_polygon():
    assert g.point_in_polygon(1.0, 1.0, SQUARE)
    assert not g.point_in_polygon(3.0, 1.0, SQUARE)
    assert g.point_in_polygon(0.5, 2.5, L_SHAPE)
    assert not g.point_in_polygon(2.0, 2.0, L_SHAPE)   # в вырезе «L»


def test_area_and_centroid():
    assert math.isclose(abs(g.polygon_area(SQUARE)), 4.0)
    cx, cy = g.polygon_centroid(SQUARE)
    assert math.isclose(cx, 1.0) and math.isclose(cy, 1.0)
    # вершины по часовой — центр тот же
    cx, cy = g.polygon_centroid(list(reversed(SQUARE)))
    assert math.isclose(cx, 1.0) and math.isclose(cy, 1.0)


def test_random_points_inside_with_margin():
    rng = random.Random(1)
    for _ in range(500):
        x, y = g.random_point_in_polygon(L_SHAPE, rng, edge_margin=0.2)
        assert g.point_in_polygon(x, y, L_SHAPE)
        assert g.distance_to_boundary(x, y, L_SHAPE) >= 0.2 - 1e-9


def test_random_point_impossible_margin():
    assert g.random_point_in_polygon(SQUARE, random.Random(0), edge_margin=5.0, max_tries=200) is None


def test_inner_center_and_radius():
    (cx, cy), r = g.inner_center_and_radius(SQUARE, edge_margin=0.1)
    assert math.isclose(cx, 1.0, abs_tol=0.01) and math.isclose(r, 0.9, abs_tol=0.01)
    # «L» с плечами шириной 1 м: наибольший круг стоит в углу, на диагонали y = x,
    # и касается двух внешних стенок и внутреннего угла (1; 1): r = 2 - sqrt(2) ~ 0.586
    (cx, cy), r = g.inner_center_and_radius(L_SHAPE)
    assert g.point_in_polygon(cx, cy, L_SHAPE)
    assert math.isclose(r, 2.0 - math.sqrt(2.0), abs_tol=0.01)
    assert g.inner_center_and_radius(SQUARE, edge_margin=2.0) is None


def test_circle_and_square_inside():
    (cx, cy), r = g.inner_center_and_radius(SQUARE, edge_margin=0.1)
    for x, y, _ in g.circle_waypoints(cx, cy, r, 12):
        assert g.point_in_polygon(x, y, SQUARE)
    half = r / math.sqrt(2.0)
    pts = g.square_waypoints(cx, cy, half, points_per_side=2)
    assert len(pts) == 8
    for x, y, _ in pts:
        assert g.point_in_polygon(x, y, SQUARE)
    # курс первой стороны квадрата — «вверх» (+y)
    assert math.isclose(pts[0][2], math.pi / 2.0)


def test_circle_inside_concave():
    (cx, cy), r = g.inner_center_and_radius(L_SHAPE, edge_margin=0.1)
    for x, y, _ in g.circle_waypoints(cx, cy, r, 16):
        assert g.point_in_polygon(x, y, L_SHAPE)
