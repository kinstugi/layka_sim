from math import sin, cos, atan2, pi, tan, sqrt
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


def generate_random_robot_poses(num_points = 10, min_distance = 0.25):
    """Generates a list of random points with x, y coordinates and radian angles,
    ensuring that all points are at least `min_distance` apart.

    Args:
    num_points: The number of points to generate.
    min_distance: The minimum allowable distance between any two points.

    Returns:
    A list of lists, where each inner list represents a point with x, y, and radian_angle coordinates.
    """
    def is_far_enough(new_point, points, min_distance):
        for point in points:
            distance = sqrt((new_point[0] - point[0])**2 + (new_point[1] - point[1])**2)
            if distance < min_distance:
                return False
        return True

    points = []
    while len(points) < num_points:
        x = random.uniform(-1, 1)
        y = random.uniform(-1, 1)
        radian_angle = random.uniform(0, 2 * pi)  # Random angle between 0 and 2*pi
        new_point = [x, y, radian_angle]
        if is_far_enough(new_point, points, min_distance):
            points.append(new_point)

    return points

def cartesian_to_polar(vect):
    r = ((vect[0] ** 2) + (vect[1] ** 2)) ** 0.5
    theta = atan2(vect[1] , vect[0])
    return r, theta