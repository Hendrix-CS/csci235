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
    return ((p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2)**(1/2)


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

## Encoding numerical inputs as symbols

<!-- Exploration: Encoding inputs symbolically -->
Create a new file called `odometry_patrol.py`, and copy and paste the following code into it:

```
import sys, math
from typing import List, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from odometry_math import find_euclidean_distance, find_roll_pitch_yaw, find_angle_diff, find_goal_heading


class OdometryNode(Node):
    def __init__(self, robot_name: str, distance_threshold: float, heading_threshold: float):
        super().__init__(f'OdometryNode_{robot_name}')
        self.start = None
        self.distance_threshold = distance_threshold
        self.heading_threshold = heading_threshold
        self.create_subscription(Odometry, f"{robot_name}/odom", self.odom_callback, qos_profile_sensor_data)
        self.topic_name = f'{robot_name}_odometry_topic'
        self.output = self.create_publisher(String, self.topic_name, qos_profile_sensor_data)

    def publish(self, data: Any):
        output = String()
        output.data = f"{data}"
        self.output.publish(output)

    def odom_callback(self, msg: Odometry):
        if self.start is None:
            self.start = msg.pose.pose.position
        else:
            d = find_euclidean_distance(self.start, msg.pose.pose.position)
            if d < self.distance_threshold:
                in_out = "in_bounds"
            else:
                in_out = "out_of_bounds"

            heading_target = find_goal_heading(msg.pose.pose.position, self.start)
            r, p, y = find_roll_pitch_yaw(msg.pose.pose.orientation)
            heading_difference = find_angle_diff(y, heading_target)
            if abs(heading_difference) < self.heading_threshold:
                alignment = 'aligned'
            elif heading_difference < 0:
                alignment = 'neg_heading'
            else:
                alignment = 'pos_heading'

            msg = {'input': (in_out, alignment), 
                   'debug': f"""
start: {self.start}
distance: {d:.2f}{' ' * 10}
heading_target: {heading_target:.2f}{' ' * 10}
yaw: {y:.2f}{' ' * 10}
"""
            }

            self.publish(msg)
```


Examine this program. Then answer the following questions:
* Imagine the `distance_threshold` is 0.5 and the `heading_threshold` is `math.pi / 8`. The 
robot drives 0.4 meters. What message will the `OdometryNode` publish?
* Now the robot has driven 0.6 meters. What message will the `OdometryNode` publish?
* The robot was facing away from the starting point. It has turned `math.pi/2` radians.
  What message will the `OdometryNode` publish?
<!-- Concept invention: symbolic sensor values -->
* There are six possible combinations of publishable messages from an `OdometryNode`. List
  those combinations. For each combination, under what circumstances will that message be
  published?

## States

Create a new file called `state_machine.py`, and copy and paste the following code into it:

```
from typing import Callable, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data


class StateNode(Node):
    def __init__(self, robot_name: str, input_topic: str, start_state: str, transition_table: Dict[str,Callable[[str],str]]):
        super().__init__(f"StateNode_{robot_name}")
        self.state = start_state
        self.last_input = None
        self.last_debug = None
        self.transition_table = transition_table
        self.topic_name = f"{robot_name}_state"
        self.debug_topic = f"{robot_name}_state_debug"
        self.create_subscription(String, input_topic, self.input_callback, qos_profile_sensor_data) 
        self.create_timer(0.25, self.timer_callback)
        self.output = self.create_publisher(String, self.topic_name, qos_profile_sensor_data)
        self.debug = self.create_publisher(String, self.debug_topic, qos_profile_sensor_data)

    def input_callback(self, msg: String):
        msg = eval(msg.data)
        self.last_input = msg['input']
        self.last_debug = msg['debug']

    def timer_callback(self):
        if self.last_input is not None:
            update = self.transition_table[self.state](self.last_input)
            if update is not None:
                self.state = update
            output = String()
            output.data = self.state
            self.output.publish(output)
            output.data = f"""
Input: {self.last_input}{' ' * 10}
Debug: {self.last_debug}
State: {self.state}{' ' * 10}
"""
            self.debug.publish(output)
            self.last_input = None
```

Examine the program. Imagine that `start_state` has the value `"forward"`. Imagine that
`transition_table` has the following value:
```
{'forward': forward_transition,
 'left':    turn_transition,
 'right':   turn_transition
}
```

Also imagine the following function definitions:
```
def forward_transition(input: str) -> str:
    if input[0] == 'out_of_bounds':
        if input[1] == 'neg_heading':
            return 'left'
        elif input[1] == 'pos_heading':
            return 'right'


def turn_transition(input: str) -> str:
    if input[1] == 'aligned':
        return 'forward'
```

<!-- Exploration: How state machines work -->
<!-- Application: Symbolic inputs -->
Imagine as well that it subscribes to the topic published by an `OdometryNode`. 
Answer the following questions:
* What would be an input that would transition the `StateMachine` object to the `left` state?
* What would be an input that would transition the `StateMachine` object in the `left` state
  back to the `forward` state?
* What would be a sequence of inputs that would transition the `StateMachine` object through
  the following state sequence, starting from `forward`: 
  * `right`, `forward`, `left`, `forward`
<!-- Concept invention: Emergent behavior of a state machine -->
* Describe in an abstract way the behavior you would expect of a robot controlled by 
  this `StateMachine` object.

Now create a file called `simple_patrol.py`, and copy and paste the code below:
```
import sys, curses, math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

from odometry_patrol import OdometryNode
from state_concept import StateNode
from curses_runner import CursesNode, run_curses_nodes


def forward(speed: float) -> TwistStamped:
    t = TwistStamped()
    t.header.frame_id = "base_link"
    t.twist.linear.x = speed
    return t


def turn(speed: float) -> TwistStamped:
    t = TwistStamped()
    t.header.frame_id = "base_link"
    t.twist.angular.z = speed
    return t


class DriveNode(Node):
    def __init__(self, robot_name: str, state_topic: str):
        super().__init__(f"DriveNode_{robot_name}")
        self.motors = self.create_publisher(TwistStamped, f"{robot_name}/cmd_vel_stamped", qos_profile_sensor_data)
        self.create_subscription(String, state_topic, self.state_callback, qos_profile_sensor_data)

    def state_callback(self, msg: String):
        if msg.data == 'forward':
            self.publish_twist(forward(0.5))
        elif msg.data == 'left':
            self.publish_twist(turn(1.0))
        elif msg.data == 'right':
            self.publish_twist(turn(-1.0))

    def publish_twist(self, t: TwistStamped):
        t.header.stamp = self.get_clock().now().to_msg()
        self.motors.publish(t)


def forward_transition(input: str) -> str:
    if input[0] == 'out_of_bounds':
        if input[1] == 'neg_heading':
            return 'left'
        elif input[1] == 'pos_heading':
            return 'right'


def turn_transition(input: str) -> str:
    if input[1] == 'aligned':
        return 'forward'


def main(stdscr):
    rclpy.init()
    sensor_node = OdometryNode(sys.argv[1], 0.5, math.pi / 32)
    state_node = StateNode(sys.argv[1], sensor_node.topic_name, 'forward', {
        'forward': forward_transition,
        'left': turn_transition,
        'right': turn_transition
    })
    curses_node = CursesNode(state_node.debug_topic, 2, stdscr)
    drive_node = DriveNode(sys.argv[1], state_node.topic_name)
    run_curses_nodes(stdscr, [drive_node, state_node, curses_node, sensor_node])
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 simple_patrol.py robot_name")
    else:
        curses.wrapper(main)
```

<!-- Concept invention: State machine behavior -->
Examine the program, and answer the following questions:
* Based both on the program text above as well as your analysis of `odometry_patrol.py` and
  `state_machine.py`, what do you expect the robot will do when you run `simple_patrol.py`?
* Run the program. Did the robot behave as you expected? Were there any surprises?

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

## Hazard-avoiding state machine
<!-- Application: Symbolic inputs and State machines -->

Consider the following desired behavior:
* If nothing exceptional occurs, the robot drives forward.
* If the robot encounters a hazard, the robot turns 90 degrees.
  * If the hazard is to its left, it turns right.
  * If the hazard is to its right, it turns left.
  * If the hazard is in front of it, it turns in the same direction it most recently turned.
* Once the robot completes its 90 degree turn, it drives forward again.

Answer the following questions:
* What would be a set of states that would comprehensively represent the above behavior?
  * **Hint**: Do not use more than four states.
* What motor commands would you associate with each state?
* What symbolic inputs would you use to represent the situations that cause these states 
  to change? 
  * **Hint**: Do not use more than five input symbols.
* Create a table with a column for each state and a row for each input symbol. In each table
  entry, write the state that the robot will enter when the input for its row is received 
  while in the state of its column. 
  * **Note**: With no more than four states and five input symbols, your table should have 
    no more than 20 entries.
* Create two files: `avoid_hazard_input.py` and `hazard_avoider.py`. Model 
  `avoid_hazard_input.py` on `odometry_patrol.py` and model `hazard_avoider.py` on 
  `simple_patrol.py`. Encode your inputs, state transitions, and motor outputs in
  those two files following the specification you just created.
* Test your program. Continue to modify it until it matches your specification. Overall,
  how did the program following your specification perform?
* What would you say are the benefits and challenges of the state machine approach to 
  programming a robot?
