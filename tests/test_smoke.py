"""Minimal smoke tests proving the test harness runs.

Deliberately dependency-light: these exercise only pure, stdlib-only
functions from the legacy codebase so `pytest` collects and passes without
GTK/PyGObject installed. Do not add `gi` or other heavy imports here.
"""

from math import atan2, isclose, pi

from utils.linalg2_util import distance, mag, rotate_vector, unit
from utils.math_util import cartesian_to_polar, normalize_angle


def test_normalize_angle_maps_to_minus_pi_to_pi():
    assert isclose(normalize_angle(0.5), 0.5, abs_tol=1e-12)
    assert isclose(normalize_angle(3.0 * pi), pi, abs_tol=1e-12)
    # atan2(sin, cos) returns -pi for negative multiples of pi; both -pi and
    # pi represent the same heading and both lie inside [-pi, pi].
    assert isclose(abs(normalize_angle(-3.0 * pi)), pi, abs_tol=1e-12)
    assert isclose(normalize_angle(-0.5), -0.5, abs_tol=1e-12)


def test_cartesian_to_polar():
    r, theta = cartesian_to_polar([3.0, 4.0])
    assert isclose(r, 5.0)
    assert isclose(theta, atan2(4.0, 3.0), abs_tol=1e-12)


def test_cartesian_to_polar_angle():
    r, theta = cartesian_to_polar([0.0, 2.0])
    assert isclose(r, 2.0)
    assert isclose(theta, pi / 2, abs_tol=1e-12)


def test_linalg_mag_and_distance():
    assert mag([3.0, 4.0]) == 5.0
    assert distance([0.0, 0.0], [3.0, 4.0]) == 5.0


def test_linalg_unit():
    assert unit([2.0, 0.0]) == [1.0, 0.0]
    assert unit([0.0, 0.0]) == [0.0, 0.0]  # degenerate input guarded in code


def test_rotate_vector_quarter_turn():
    result = rotate_vector([1.0, 0.0], pi / 2)
    assert isclose(result[0], 0.0, abs_tol=1e-12)
    assert isclose(result[1], 1.0, abs_tol=1e-12)
