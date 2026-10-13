import unittest, math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


def normalize_angle_positive(angle: float) -> float:
    """
    Ensure that the angle in radians lies between 0 and 2 * math.pi 
    """
    return angle % (2.0 * math.pi)


def normalize_angle_symmetric(angle: float) -> float:
    """
    Ensure that the angle in radians lies between -math.pi and math.pi
    """
    angle = normalize_angle_positive(angle)
    if angle > math.pi:
        angle -= 2.0 * math.pi
    return angle


def shortest_angle_diff(angle1: float, angle2: float) -> float:
    """
    Find the shortest distance between two angles.
    Parameters will be in radians.
    Result will be in the range [0, math.pi]
    """
    option1 = normalize_angle_positive(angle1 - angle2)
    option2 = normalize_angle_positive(angle2 - angle1)
    if option1 > option2:
        return option2
    else:
        return option1


def turn_from_to(starting_angle: float, goal_angle: float) -> float:
    """
    Return an angle that, if you were to add it to the starting angle,
    would produce the goal angle.
    The returned angle should be the smallest possible angle to 
    return in terms of absolute value.
    Parameters will be in radians.
    """
    diff = shortest_angle_diff(starting_angle, goal_angle)
    if normalize_angle_positive(starting_angle + diff - goal_angle) <= normalize_angle_positive(starting_angle - diff - goal_angle):
        return diff
    else:
        return -diff


class RobotPose:
    """
    Represents a robot pose (x, y, theta)
    theta should always be normalized
    """
    def __init__(self, x: float, y: float, theta: float):
        self.x = x
        self.y = y
        self.theta = normalize_angle_symmetric(theta)

    def __repr__(self):
        return f"RobotPose({self.x},{self.y},{self.theta})"

    def turn(self, angle: float):
        self.theta = normalize_angle_symmetric(self.theta + angle)

    def move(self, distance: float):
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)

    def distance_to(self, goal_x: float, goal_y: float) -> float:
        return math.sqrt((self.x - goal_x)**2 + (self.y - goal_y)**2)

    def turn_to(self, goal_x: float, goal_y: float) -> float:
        heading_offset = math.atan2(goal_y - self.y, goal_x - self.x)
        return turn_from_to(self.theta, heading_offset)

    def point(self) -> Point:
        return Point(x=self.x, y=self.y)


def odom2pose(odom: Odometry) -> RobotPose:
    p = odom.pose.pose.position
    q = odom.pose.pose.orientation
    qy = q.w * q.z + q.x * q.y
    qx = q.w**2 + q.x**2 - q.y**2 - q.z**2
    return RobotPose(p.x, p.y, math.atan2(qy, qx))


class AngleMathTest(unittest.TestCase):
    def test_normalize_angle_positive(self):
        for theta, normed in [
            (-math.pi, math.pi),
            (-math.pi/2, 3/2 * math.pi),
            (-3/4 * math.pi, 5/4 * math.pi),
            (-7/8 * math.pi, 9/8 * math.pi),
            (2 * math.pi, 0.0), 
            (3/2 * math.pi, 3/2 * math.pi), 
            (9 * math.pi, math.pi),
            (-3 * math.pi, math.pi),
            (5/2 * math.pi, math.pi/2),
        ]:
            self.assertEqual(normed, normalize_angle_positive(theta))

    def test_normalize_angle_symmetric(self):
        for theta, normed in [
            (-math.pi, math.pi),
            (2 * math.pi, 0.0), 
            (3/2 * math.pi, -math.pi/2), 
            (9 * math.pi, math.pi),
            (-3 * math.pi, math.pi),
            (5/2 * math.pi, math.pi/2),
        ]:
            self.assertEqual(normed, normalize_angle_symmetric(theta))

    def test_angle_diff(self):
        for theta1, theta2, diff in [
            ( 3/4 * math.pi,  1/4 * math.pi, 1/2 * math.pi), 
            ( 1/4 * math.pi,  3/4 * math.pi, 1/2 * math.pi), 
            (-1/3 * math.pi,  1/3 * math.pi, 2/3 * math.pi), 
            (15/8 * math.pi,  1/8 * math.pi, 1/4 * math.pi), 
            ( 1/8 * math.pi, 15/8 * math.pi, 1/4 * math.pi), 
            ( 9/2 * math.pi, -1/2 * math.pi,       math.pi), 
            (       math.pi, -2/3 * math.pi, 1/3 * math.pi),
            (      -math.pi, -2/3 * math.pi, 1/3 * math.pi), 
            (       math.pi,  2/3 * math.pi, 1/3 * math.pi),
            (      -math.pi,  2/3 * math.pi, 1/3 * math.pi),
        ]:
            self.assertAlmostEqual(diff, shortest_angle_diff(theta1, theta2), places=5)

    def test_turn_from_to(self):
        for theta1, theta2, diff in [
            ( 3/4 * math.pi,  1/4 * math.pi, -1/2 * math.pi), 
            ( 1/4 * math.pi,  3/4 * math.pi,  1/2 * math.pi), 
            (-1/3 * math.pi,  1/3 * math.pi,  2/3 * math.pi), 
            (-1/8 * math.pi,  1/8 * math.pi,  1/4 * math.pi),
            (15/8 * math.pi,  1/8 * math.pi,  1/4 * math.pi), 
            ( 1/8 * math.pi, 15/8 * math.pi, -1/4 * math.pi), 
            ( 9/2 * math.pi, -1/2 * math.pi,        math.pi), 
            (       math.pi, -2/3 * math.pi,  1/3 * math.pi),
            (      -math.pi, -2/3 * math.pi,  1/3 * math.pi), 
            (       math.pi,  2/3 * math.pi, -1/3 * math.pi),
            (      -math.pi,  2/3 * math.pi, -1/3 * math.pi),
        ]:
            self.assertAlmostEqual(diff, turn_from_to(theta1, theta2), places=5)

    def test_move(self):
        pose = RobotPose(0, 0, 0)
        for (turn, distance, x, y, theta) in [
            (math.pi/4, 1.4142, 1.0, 1.0, math.pi/4),
            (math.pi/4, 1.0, 1.0, 2.0, math.pi/2),
            (-math.pi/3, 1.0, 1.866, 2.500, math.pi/6),
            (math.pi/2, 1.0, 1.366, 3.366, 2.0*math.pi/3),
            (math.pi/2, 1.5, 0.067, 2.616, -5.0*math.pi/6),
        ]:
            pose.turn(turn)
            pose.move(distance)
            self.assertAlmostEqual(pose.x, x, places=3)
            self.assertAlmostEqual(pose.y, y, places=3)
            self.assertAlmostEqual(pose.theta, theta, places=3)

    def test_distance_to(self):
        pose = RobotPose(1.0, -1.0, 0.0)
        for (gx, gy, d) in [(1.0, -1.0, 0.0), (4.0, 3.0, 5.0), (-4.0, -13.0, 13.0)]:
            self.assertAlmostEqual(pose.distance_to(gx, gy), d, places=3)

    def test_turn_to(self):
        pose = RobotPose(1.0, -1.0, math.pi / 2)
        for (gx, gy, d) in [
            (0.0,  0.0, math.pi / 4),
            (0.0, -1.0, math.pi / 2),
            (1.0,  0.0, 0.0),
            (2.0,  0.0, -math.pi / 4),
            (0.0, -2.0, 3/4 * math.pi),
            (1.923879532, -0.617316567, -3/8 * math.pi),
            (2.0, -1.0, -math.pi/2),
            (1.0, -2.0, math.pi),
            (1.38268343236509, -1.9238795325112865, -7/8 * math.pi),
            (0.6173165676349097, -1.9238795325112865, 7/8 * math.pi)            
        ]:
            self.assertAlmostEqual(pose.turn_to(gx, gy), d, places=3)

    def test_to_point(self):
        pose = RobotPose(1.0, -1.0, math.pi / 2)
        self.assertEqual(pose.point(), Point(x=1.0, y=-1.0))


if __name__ == '__main__':
    unittest.main()
