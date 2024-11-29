from math import sin, cos, atan2, pi, tan
import random


# map the given angle to the equivalent angle in [ -pi, pi ]
def normalize_angle(theta):
    return atan2(sin(theta), cos(theta))


def frange(x, y, jump=1.0):
    r = []
    while x < y:
        r.append(x)
        x += jump

    return r


def generate_random_robot_poses(num_points):
    """Generates a list of random points with x, y coordinates and radian angles.

    Args:
    num_points: The number of points to generate.

    Returns:
    A list of lists, where each inner list represents a point with x, y, and radian_angle coordinates.
    """

    points = []
    for _ in range(num_points):
        x = random.uniform(-1, 1)
        y = random.uniform(-1, 1)
        radian_angle = random.uniform(0, 2 * pi)  # Random angle between 0 and 2*pi
        points.append([x, y, radian_angle])
    return points

def cartesian_to_polar(vect):
    r = ((vect[0] ** 2) + (vect[1] ** 2)) ** 0.5
    theta = atan2(vect[1] , vect[0])
    return r, theta