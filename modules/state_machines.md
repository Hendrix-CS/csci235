---
layout: work
type: Module
num: 3
worktitle: State Machines
---

## New Topic: Odometry
<!-- Exploration: odom from the command line -->
Type the command below into the command line:
```
ros2 topic echo /[your robot name]/odom
```

<!-- Application: ROS motors from command line -->
Open a second shell. In the second shell, write a ROS2 command to drive the robot forwards.
Put it at a low speed, and make sure it has some room. While it is running, observe the 
first shell window, and answer the following questions:
* What information is published by the `odom` topic?
* How does that information change as the robot drives forward?
* Stop the robot. Now write a ROS2 command for it to spin in place. How does the `odom`
  information change as the robot spins?
<!-- Concept invention: Odometry -->
* What fields from the `odom` message are most relevant to determining the robot's
  position and orientation?
  
## Position and Orientation Calculations
The next part depends on an external library we will need to install. Run the 
following command, entering your password when requested:
```
sudo apt install ros-iron-tf-transformations
```

Create a new `module3` folder. Then:
```
cd module3
micro odometry_math.py
```

Then copy and paste the following program:
```
import unittest

import math
from geometry_msgs.msg import Point, Quaternion
from typing import Tuple
from tf_transformations import euler_from_quaternion


def find_goal_heading(p1: Point, p2: Point) -> float:
    """
    Given two points, find the heading necessary to aim at the second
    point from the first point.
    """
    return math.atan2(p2.y - p1.y, p2.x - p1.x)


def find_euclidean_distance(p1: Point, p2: Point) -> float:
    """
    Given two points, find the distance between them.
    """
    return ((p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z))**(1/2)


def find_roll_pitch_yaw(orientation: Quaternion) -> Tuple[float, float, float]:
    """
    Given a Quaternion, this returns the following in order:
    - roll: Rotation around the robot's x axis
      - Since the x axis is the direction of forward motion,
        this is the amount of left/right tilt
    - pitch: Rotation around the robot's y axis
      - Since the y axis is perpendicular to forward motion,
        this is the amount of forward/backward tilt
    - yaw: Rotation around the robot's z axis
      - Since the z axis is perpendicular to the ground,
        this is the robot's orientation on the ground.
    """
    return euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])


def find_angle_diff(angle1: float, angle2: float) -> float:
    """
    Find the shortest difference between two angles.
    Parameters should be in radians.
    """
    return find_normalized_angle(angle1 - angle2)


def find_normalized_angle(angle: float) -> float:
    """
    Ensure that the angle in radians lies between -math.pi and math.pi
    """
    angle = angle % (2 * math.pi)
    if angle > math.pi:
        angle -= 2 * math.pi
    return angle


class OdometryMathTest(unittest.TestCase):
    def test_distance(self):
        for a, b, c in [(3, 4, 5), (5, 12, 13), (7, 24, 25), (8, 15, 17), (9, 40, 41)]:
            p1 = Point()
            p2 = Point()
            p1.x = a
            p1.y = b
            self.assertEqual(c, find_euclidean_distance(p1, p2))

    def test_normalized_angle(self):
        for theta, normed in [(2 * math.pi, 0.0), (3/2 * math.pi, -math.pi/2), (9 * math.pi, math.pi)]:
            self.assertEqual(normed, find_normalized_angle(theta))

    def test_angle_diff(self):
        for theta1, theta2, diff in [
            ( 3/4 * math.pi,  1/4 * math.pi,  1/2 * math.pi), #1
            ( 1/4 * math.pi,  3/4 * math.pi, -1/2 * math.pi), #2
            (-1/3 * math.pi,  1/3 * math.pi, -2/3 * math.pi), #3
            (15/8 * math.pi,  1/8 * math.pi, -1/4 * math.pi), #4         
            ( 1/8 * math.pi, 15/8 * math.pi,  1/4 * math.pi), #5
            ( 9/2 * math.pi, -1/2 * math.pi,        math.pi), #6
        ]:
            self.assertAlmostEqual(diff, find_angle_diff(theta1, theta2), places=5)


if __name__ == '__main__':
    unittest.main()
```

<!-- Exploration: Odometry math -->
Read over the code. Then answer the following questions:
* Why is the arctangent used in `find_goal_heading()`?
* Read the [documentation for `atan` and `atan2`](https://docs.python.org/3/library/math.html#math.atan).
  Why is it important to use `atan2` rather than `atan` for the purpose of robot navigation? 
* For each of roll, pitch, and yaw, what is a scenario in which a nonzero value tells you
  something interesting about the robot's position?
* Examine the test cases for the `angle_diff()` function. Note that each 
  test case is numbered to facilitate references.
<!-- Concept invention: Angular difference -->
<!-- Application: Robot turns -->
  * For each test case, imagine that the robot's current orientation is the 
    first value and its goal orientation is the second value. Remembering how 
    you programmed the robots to turn left and right, which direction would 
    the robot turn to reach its goal orientation?
  * For each test case, does the target difference conflict with your
    intuition about how subtraction works? 
  * For each test case that conflicts with your intuition, state why the 
    conflict exists, and also state why the given difference is correct
    in the context of angles and headings. Feel free to examine the 
    implementation of the `angle_diff()` and `normalize_angle()` functions
    as part of your answer.

## Odometry in Python
<!-- Application: Odometry Topic, ROS2 subscriptions -->
```
cp ../module2/sensor_messenger.py .
cp ../module2/key_motor_demo.py .
micro sensor_messenger.py
```

Within `sensor_messenger.py`, add this import:
```
from nav_msgs.msg import Odometry
```

Then perform the following additional modifications to `sensor_messenger.py`:
* Add a subscription to the `Odometry` topic. 
* Define the callback function that you name in the subscription.
* Add attributes to the `SensorNode` class to store odometry information and
  frequency. Don't store the entire message; only store information from the 
  fields you determined to be most pertinent earlier. 
* Add the odometry information you consider useful to the `String` object 
  that the `SensorNode` publishes. 
* Drive the robot around for a while, observing how its location is reported.
  What are some applications of odometry that would be useful?

## States

<!--
Patrol task
* Two states: Drive and turn
* Two input types: distance from start, alignment to start
  * close/far, aligned/unaligned
-->

<!-- outline 
* Introduce hazard sensors
* Introduce being in different states based on last obstacle encountered
* Odometry
* Patrol task
-->

## New Topic: Hazards

<!-- Exploration: hazard_detection from the command line -->
Type the command below into the command line:
```
ros2 topic echo /[your robot name]/hazard_detection
```

* Press the bumper directly at the front of the robot. What does it display?
* Press the bumper in different places, throughout its coverage of the front half of the
  robot. What does it display when it is touched in different places?
<!-- Concept invention: which bump sensors does it have? -->
* How many distinct bump sensors does the iRobot Create3 have?
  * What are their names?
<!-- Exploration: Other hazards -->
* Pick up the robot. What messages does it display?
<!-- Concept invention: pick-me-up-sensors -->
* How many distinct cliff-detection sensors does the iRobot Create3 have?
  * What are their names?
* What else can the iRobot Create3 sense to determine that it is not entirely 
  on the ground?
  * What are their names?

