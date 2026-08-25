---
layout: work
type: Module
num: 7
worktitle: Reinforcement Learning
---

<!-- Concept 

Q-learning states are squares based on odometry.
Positive rewards for low IR values.
Small positive reward for maintaining the current direction of movement.
Negative rewards for high IR values.
Negative rewards for hitting things.

One could then path-plan by using BFS or A* on the Q-states.

-->

## Reinforcement Learning

**Reinforcement learning** is a technique for teaching a robot how to act
based on its experiences. The robot receives **rewards** for having 
certain experiences. Over time, it learns to select actions that will 
maximize its rewards.

The specific reinforcement learning algorithm we will employ is 
**Q-learning**. In Q-learning, the robot has a table of states and actions.
For every pair of state and action, the table stores the **expected reward**
of performing that action in that state. The goal of the Q-learning 
algorithm is to employ experience to calculate the expected reward.

For the first set of exercises, consider the following states and actions:

### States

|       | Column 0 | Column 1 | Column 2 |
| ----  | -------- | -------- | -------- |
| Row 0 | 0 (0, 0) | 1 (1, 0) | 2 (2, 0) |
| Row 1 | 3 (0, 1) | 4 (1, 1) | 5 (2, 1) |
| Row 2 | 6 (0, 2) | 7 (1, 2) | 8 (2, 2) |

### Actions

* North
  * From `3`, `4`, or `5`, go to `0`, `1`, or `2`
  * From `6`, `7`, or `8`, go to `3`, `4`, or `5`
* South
  * From `0`, `1`, or `2`, go to `3`, `4`, or `5`
  * From `3`, `4`, or `5`, go to `6`, `7`, or `8`
* East
  * From `0`, `3`, or `6`, go to `1`, `4`, or `7`
  * From `1`, `4`, or `7`, go to `2`, `5`, or `8`
* West
  * From `1`, `4`, or `7`, go to `0`, `3`, or `6`
  * From `2`, `5`, or `8`, go to `1`, `4`, or `7`

### Rewards
* In states `6`, `7`, and `1`, the robot receives a reward of 1.
* In states `0`, `5`, and `8`, the robot receives a reward of -1.
* In states `2`, `3`, and `4`, the robot receives a reward of zero.

### Example
* Imagine the robot starts in state `4` and takes the following actions:
  * North, West, South, South, East, East, North, North, South, West, West, South, East, West
* At each step, record the state and the **current** and **total** rewards the robot has
earned.
* For each state:
  * How many times did the robot visit the state?
  * What is the **best** action the robot could take from that state, in 
    terms of maximizing rewards?
  * What is the **worst** action the robot could take from that state?
* When visiting a state, there may be a trade-off between immediate rewards
and future rewards. Among these nine states, which ones have the most striking
trade-off between immediate and future rewards? Why?
* Should the expected reward for taking an action in a state depend more
on the immediate or future reward? Why or why not? 
* What might be a way of balancing immediate and future rewards?

## Q-Learning

In Q-Learning, we calculate an update of expected rewards using the following formula:
* `update = delta * q_value[new_state][best_action in new_state] + reward`
* `delta` is a number between 0.0 and 1.0, also known as the **discount**
  * The goal of the discount is to balance future rewards with 
    current rewards.
  * What would be the effect of a discount of 0.0?
  * How about a discount of 1.0?
* This update is applied to the **previous state**, according to the following formula:
  * `q_value[previous_state] = (1.0 - alpha) * q_value[previous_state] + alpha * update`
  * `alpha` is a number between 0.0 and 1.0, also known as the **learning rate**.
  * The goal of the learning rate is to balance new information against previously
    acquired information.
  * What would be the effect of a learning rate of 0.0?
  * How about a learning rate of 1.0?
  * Do you think the learning rate should be constant, or should it vary 
    over the course of the run? Why or why not?
* One approach to selecting an action from a given state
  is to pick the action with the highest q-value, that is, the highest
  expected reward.
  * What advantages would this approach to action selection have?
  * What disadvantages would it have?  
* Another approach is to pick the action that has been tried least often
  from that state.
  * What advantages would this approach to action selection have?
  * What disadvantages would it have?  
* In what ways could combining these two approaches be beneficial?

## Encoding States

The Python program below will be important for converting ROS2 odometry values
into Q-learning states. Copy and paste it into `grid.py`:

```
from geometry_msgs.msg import Point
from typing import Tuple
import unittest

class GridConverter:
    def __init__(self, width: float, height: float, cell_size: float):
        self.cols = int(width / cell_size)
        self.rows = int(height / cell_size)
        self.cell_size = cell_size
        self.max_x = width / 2.0
        self.max_y = height / 2.0

    def square_within_grid(self, grid_square: Tuple[int, int]) -> bool:
        return 0 <= grid_square[0] < self.cols and 0 <= grid_square[1] < self.rows

    def odom_to_grid(self, position: Point) -> Tuple[int, int]:
        col = int((position.x + self.max_x) / self.cell_size)
        row = int((position.y + self.max_y) / self.cell_size)
        return col, row
    
    def grid_to_odom(self, grid_square: Tuple[int, int]) -> Point:
        col, row = grid_square
        p = Point()
        p.x = col * self.cell_size - self.max_x + self.cell_size / 2.0
        p.y = row * self.cell_size - self.max_y + self.cell_size / 2.0
        return p
    
    def num_states(self) -> int:
        return self.cols * self.rows
    
    def grid_to_state(self, grid_square: Tuple[int, int]) -> int:
        col, row = grid_square
        return row * self.cols + col
    
    def state_to_grid(self, state_num: int) -> Tuple[int, int]:
        return state_num % self.cols, state_num // self.cols
    

def add_squares(sq1: Tuple[int, int], sq2: Tuple[int, int]) -> Tuple[int, int]:
    return sq1[0] + sq2[0], sq1[1] + sq2[1]


class GridConverterTest(unittest.TestCase):
    def test_state_to_square(self):
        conv = GridConverter(1.5, 1.5, 0.5)
        self.assertEqual(conv.cols, 3)
        self.assertEqual(conv.rows, 3)
        expected = [(x, y) for y in range(3) for x in range(3)]
        for state in range(9):
            grid_square = conv.state_to_grid(state)
            self.assertEqual(expected[state], grid_square)
            self.assertEqual(conv.grid_to_state(grid_square), state)

    def test_from_odom(self):
        conv = GridConverter(10.0, 10.0, 2.0)
        for (odom_x, odom_y, expected_col, expected_row) in [
            (4.5, 2.0, 4, 3), 
            (-3.1, -0.5, 0, 2), 
            (0.0, 0.0, 2, 2), 
            (0.8, -0.8, 2, 2), 
            (1.0, 1.0, 3, 3), 
            (-1.0, -1.0, 2, 2)]:
            p = Point()
            p.x = odom_x
            p.y = odom_y
            col, row = conv.odom_to_grid(p)
            self.assertEqual(expected_col, col)
            self.assertEqual(expected_row, row)

    def test_from_grid(self):
        conv = GridConverter(10.0, 10.0, 2.0)
        for (col, row, expected_odom_x, expected_odom_y) in [
            (4, 3, 4.0, 2.0),
            (0, 2, -4.0, 0.0),
            (2, 2, 0.0, 0.0),
            (3, 3, 2.0, 2.0)
        ]:
            odom = conv.grid_to_odom((col, row))
            self.assertAlmostEqual(odom.x, expected_odom_x, 5)
            self.assertAlmostEqual(odom.y, expected_odom_y, 5)


if __name__ == '__main__':
    unittest.main()
```

Examine the code and the unit tests. Then answer the following questions:
* Why do we need to add `self.max_x` and `self.max_y` when converting from
odometry to grid squares?
* How do we ensure that every grid square corresponds to a unique state
  number?
* Why might it be helpful to add `self.cell_size / 2.0` when converting grid squares
to odometry? 

## A Q-Learning implementation

Examine the Python implementation of Q-Learning below. Copy and paste it
into `qlearning.py`:
```
import random
from typing import Sequence, Tuple


class QParameters:
    def __init__(self, target_visits: int=2, epsilon: float=0.0, discount: float=0.5, rate_constant: int=10, num_states: int=2, num_actions: int=2):
        self.target_visits = target_visits
        self.epsilon = epsilon
        self.discount = discount
        self.rate_constant = rate_constant
        self.set_num_states_actions(num_states, num_actions)

    def set_num_states_actions(self, num_states: int, num_actions: int):
        self.num_states = num_states
        self.num_actions = num_actions
        self.state_actions = {state: [act for act in range(num_actions)] for state in range(num_states)}

    def forbid_state_actions(self, forbidden_state_actions: Sequence[Tuple[int,int]]):
        for (state, action) in forbidden_state_actions:
            if state in self.state_actions and action in self.state_actions[state]:
                self.state_actions[state].remove(action)


class QTable:
    def __init__(self, params: QParameters):
        self.q = [[0.0] * params.num_actions for i in range(params.num_states)]
        self.visits = [[0] * params.num_actions for i in range(params.num_states)]
        self.target_visits = params.target_visits
        self.epsilon = params.epsilon
        self.discount = params.discount
        self.rate_constant = params.rate_constant
        self.last_state = 0
        self.last_action = 0
        self.state_actions = params.state_actions
        self.total_updates = 0

    def num_states(self) -> int:
        return len(self.q)
    
    def num_actions(self) -> int:
        return len(self.q[0])

    def sense_act_learn(self, new_state: int, reward: float) -> int:
        assert 0 <= new_state < self.num_states()
        alpha = self.learning_rate(self.last_state, self.last_action)
        update = alpha * (self.discount * self.q[new_state][self.best_action(new_state)] + reward)
        self.q[self.last_state][self.last_action] *= 1.0 - alpha
        self.q[self.last_state][self.last_action] += update

        self.visits[self.last_state][self.last_action] += 1
        if self.is_exploring(new_state):
            new_action = self.least_visited_action(new_state)
        else:
            new_action = self.best_action(new_state)

        self.last_state = new_state
        self.last_action = new_action
        self.total_updates += 1
        return new_action

    def learning_rate(self, state: int, action: int) -> float:
        return self.rate_constant / (self.rate_constant + self.visits[state][action])

    def best_action(self, state: int) -> int:
        best = self.state_actions[state][0]
        for action in self.state_actions[state]:
            if self.q[state][best] < self.q[state][action]:
                best = action
        assert 0 <= best < len(self.q[state])
        return best

    def is_exploring(self, state: int) -> bool:
        below_target = False
        for action in self.state_actions[state]:
            if self.visits[state][action] < self.target_visits:
                below_target = True
        return below_target or random.random() < self.epsilon

    def least_visited_action(self, state: int) -> int:
        least_visited = self.state_actions[state][0]
        for action in self.state_actions[state]:
            if self.visits[state][least_visited] > self.visits[state][action]:
                least_visited = action
        return least_visited
```

Answer the following questions:
* What data structure represents the table of Q values?
* How is the learning rate determined from the number of times an
  action is attempted from a given state?
* From how the code is written, what do you think it means to **explore**?
* Why do you think we might want to forbid some actions in certain states?

## A Q-Learning Node

### Preliminary code
Our implementation relies on our previous implementations of fuzzy logic. 
Copy over `fuzzy.py` and `curses_runner.py` from previous modules. Use
the simplified `odometry_math.py`, `goal_fuzzy_input.py` and `goal_fuzzy_navigator.py` provided below:

`odometry_math.py`:

```
import unittest

import math
from geometry_msgs.msg import Point, Quaternion
from typing import Tuple


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
    q1, q2, q3, q0 = orientation.x, orientation.y, orientation.z, orientation.w
    
    roll = math.atan2(2 * (q0 * q1 + q2 * q3), q0**2 - q1**2 - q2**2 + q3**2)
    pitch = math.asin(2 * (q0 * q2 - q1 * q3))
    yaw = math.atan2(2 * (q0 * q3 + q1 * q2), q0**2 + q1**2 - q2**2 - q3**2)

    return roll, pitch, yaw


def find_yaw(orientation: Quaternion) -> float:
    """
    Given a Quaternion, this returns the yaw, that is, rotation
    around the robot's z axis. Since the z axis is perpendicular 
    to the ground, this is the robot's orientation on the ground.
    """
    return find_roll_pitch_yaw(orientation)[-1]


def find_angle_diff(target_angle: float, current_angle: float) -> float:
    """
    Find the shortest difference between two angles.
    Parameters should be in radians.
    If the target turn direction is left, result will be positive.
    If the target turn direction is right, result will be negative.
    """
    return find_normalized_angle(target_angle - current_angle)


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

    def test_roll_pitch_yaw(self):
        for x, y, z, w, rpw in [
            (-0.0031924284994602203, 0.005276054609566927, -0.8277794122695923, 0.561019778251648, 
             (-0.012317164052438935, 0.0006346888584906615, -1.9503363787472408)),
            (-0.002063487656414509,  0.0034074627328664064, -0.9837535619735718, 0.17948013544082642,
             (-0.007445015587338619, -0.002836786557391314, -2.7806632489220133))
        ]:
            q = Quaternion()
            q.x, q.y, q.z, q.w = x, y, z, w
            roll, pitch, yaw = find_roll_pitch_yaw(q)
            print(roll, pitch, yaw)
            self.assertAlmostEqual(rpw[0], roll,  places=5)
            self.assertAlmostEqual(rpw[1], pitch, places=5)
            self.assertAlmostEqual(rpw[2], yaw,   places=5)


if __name__ == '__main__':
    unittest.main()
```

`goal_fuzzy_input.py`:

```
from typing import Any, Dict

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Point

from odometry_math import find_euclidean_distance, find_yaw, find_normalized_angle, find_goal_heading
import fuzzy

def compute_fuzzy_errors(goal: Point, position: Point, yaw: float, angle_limit: float, distance_limit: float) -> Dict[str, float]:
    errors = {'left': 0.0, 'right': 0.0, 'distance': 0.0}
    distance_diff = find_euclidean_distance(goal, position)
    goal_direction = find_goal_heading(position, goal)
    angle_diff = find_normalized_angle(goal_direction - yaw)

    if angle_diff > 0:
        errors['left'] = fuzzy.fuzzify(angle_diff, 0.0, angle_limit)
    else:
        errors['right'] = fuzzy.fuzzify(-angle_diff, 0.0, angle_limit)

    either_turn = fuzzy.f_or(errors['left'], errors['right'])
    dist = fuzzy.fuzzify(distance_diff, 0.0, distance_limit / 2.0)
    errors['distance'] = fuzzy.f_and(dist, fuzzy.f_not(either_turn))

    return errors

class FuzzyGoalNode(Node):
    def __init__(self, robot_name: str, goal_topic: str, angle_limit: float=0.2, distance_limit: float=0.25):
        super().__init__(f'FuzzyGoalNode_{robot_name}')
        self.create_subscription(Odometry, f"{robot_name}/odom", self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(String, goal_topic, self.goal_callback, qos_profile_sensor_data)
        self.output_topic = f'{robot_name}_goal_error'
        self.output = self.create_publisher(String, self.output_topic, qos_profile_sensor_data)
        self.goal = None
        self.angle_limit = angle_limit
        self.distance_limit = distance_limit

    def publish(self, publisher: Publisher, data: Any):
        output = String()
        output.data = f"{data}"
        publisher.publish(output)

    def goal_callback(self, msg: String):
        value = eval(msg.data)
        if value is None:
            self.goal = None
        else:
            x, y = value
            self.goal = Point()
            self.goal.x = x
            self.goal.y = y

    def odom_callback(self, msg: Odometry):     
        if self.goal is None: 
            self.publish(self.output, None)
        else:
            yaw = find_yaw(msg.pose.pose.orientation)
            errors = compute_fuzzy_errors(self.goal, msg.pose.pose.position, yaw, self.angle_limit, self.distance_limit)
            self.publish(self.output, errors)
```

`goal_fuzzy_navigator.py`:

```
from typing import Dict, Tuple

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

import fuzzy


class FuzzyDriveNode(Node):
    def __init__(self, robot_name: str, fuzzy_topic: str, x_limit: float=0.5, z_limit: float=1.0):
        super().__init__(f"FuzzyDriveNode_{robot_name}")
        self.x_limit = x_limit
        self.z_limit = z_limit
        self.motors = self.create_publisher(TwistStamped, f"{robot_name}/cmd_vel_stamped", qos_profile_sensor_data)
        self.create_subscription(String, fuzzy_topic, self.fuzzy_callback, qos_profile_sensor_data)

    def fuzzy_callback(self, msg: String):
        fuzzy_values = eval(msg.data)
        t = self.make_twist()
        if fuzzy_values is not None:
            t.twist.linear.x, t.twist.angular.z = defuzzify_x_z(fuzzy_values, self.x_limit, self.z_limit)
            self.motors.publish(t)
            
    def make_twist(self) -> TwistStamped:
        t = TwistStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = self.get_clock().now().to_msg()
        return t


def defuzzify_x_z(fuzzy_values: Dict[str, float], x_limit: float, z_limit: float) -> Tuple[float, float]:
    x = fuzzy.defuzzify(fuzzy_values["distance"], 0, x_limit)
    turn_limit = z_limit * (1.0 if fuzzy_values["left"] > fuzzy_values["right"] else -1.0)
    z = fuzzy.defuzzify(fuzzy.f_or(fuzzy_values["left"], fuzzy_values["right"]), 0, turn_limit)
    return x, z
```

### Q-Learning Node Code

Examine the Python implementation of a Q-Learning ROS2 node. Copy and paste it
into `qrunner.py`:

```
from grid import GridConverter, add_squares
from qlearning import QTable, QParameters
from goal_fuzzy_input import FuzzyGoalNode
from goal_fuzzy_navigator import FuzzyDriveNode
from curses_runner import CursesNode, run_curses_nodes

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Point
from irobot_create_msgs.msg import InterfaceButtons

from typing import Tuple
import random
import sys
import curses

ACTIONS = [(0, -1), (0, 1), (1, 0), (-1, 0)]
BUTTON_1_REWARD = 1.0
BUTTON_2_REWARD = -1.0


class QRunningNode(Node):
    def __init__(self, robot_name: str, params: QParameters, conv: GridConverter):
        super().__init__(f"{robot_name}_q_runner")
        self.conv = conv
        params.set_num_states_actions(conv.num_states(), len(ACTIONS))
        forbidden = []
        for state in range(params.num_states):
            col, row = conv.state_to_grid(state)
            for action in range(params.num_actions):
                act_col = col + ACTIONS[action][0]
                act_row = row + ACTIONS[action][1]
                if not conv.square_within_grid((act_col, act_row)):
                    forbidden.append((state, action))
        params.forbid_state_actions(forbidden)
        self.q_table = QTable(params)

        self.position_topic_name = f"{robot_name}_q_goal"
        self.position_topic = self.create_publisher(String, self.position_topic_name, qos_profile_sensor_data)
        self.info_topic_name = f"{robot_name}_q_info"
        self.info_topic = self.create_publisher(String, self.info_topic_name, qos_profile_sensor_data)
        self.create_subscription(Odometry, f"/{robot_name}/odom", self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(InterfaceButtons, f"/{robot_name}/interface_buttons", self.button_callback, qos_profile_sensor_data)

        self.position = Point()
        square = self.conv.odom_to_grid(Point())
        self.current_state = self.conv.grid_to_state(square)
        self.action_message = self.make_action_message(square, random.choice(ACTIONS))

    def make_action_message(self, sq1: Tuple[int, int], sq2: Tuple[int, int]) -> str:
        updated_square = add_squares(sq1, sq2)
        odom = self.conv.grid_to_odom(updated_square)
        return f"({odom.x}, {odom.y})"
    
    def button_callback(self, msg: InterfaceButtons):
        if self.action_message == "None":
            reward = None
            if msg.button_1.is_pressed:
                reward = BUTTON_1_REWARD
            elif msg.button_2.is_pressed:
                reward = BUTTON_2_REWARD
            elif msg.button_power.is_pressed:
                reward = 0.0

            if reward is not None:
                action = self.q_table.sense_act_learn(self.current_state, reward)
                self.action_message = self.make_action_message(self.conv.state_to_grid(self.current_state), ACTIONS[action])
                out = String()
                out.data = f"""After {self.q_table.total_updates} updates:
position: ({self.position.x:.2f}, {self.position.y:.2f}) {self.conv.odom_to_grid(self.position)}
state: {self.current_state}
reward: {reward:.2f}
action: {action} ({ACTIONS[action]}) {self.action_message}
"""
                for state in range(self.q_table.num_states()):
                    sq = self.conv.state_to_grid(state)
                    out.data += f"{state} {sq}:"
                    for action in range(self.q_table.num_actions()):
                        aq = (sq[0] + ACTIONS[action][0], sq[1] + ACTIONS[action][1])
                        if not self.conv.square_within_grid(aq):
                            aq = "Undef "
                        out.data += f" act:{action} {aq} ({self.q_table.q[state][action]:6.2f} {self.q_table.visits[state][action]:3d})"
                    out.data += "\n"
                self.info_topic.publish(out)

    def odom_callback(self, msg: Odometry):
        self.position = msg.pose.pose.position
        state = self.conv.grid_to_state(self.conv.odom_to_grid(self.position))
        if state != self.current_state:
            self.action_message = "None"
            self.current_state = state
        out = String()
        out.data = self.action_message 
        self.position_topic.publish(out)

        
def main(stdscr):
    rclpy.init()
    q_node = QRunningNode(sys.argv[1], QParameters(), GridConverter(width=1.5, height=1.5, cell_size=0.5))
    sensor_node = FuzzyGoalNode(sys.argv[1], q_node.position_topic_name)
    curses_node = CursesNode(q_node.info_topic_name, 2, stdscr)
    drive_node = FuzzyDriveNode(sys.argv[1], sensor_node.output_topic)
    run_curses_nodes(stdscr, [q_node, drive_node, curses_node, sensor_node])
    rclpy.shutdown()
            

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 qmapper.py robot_name")
    else:
        print(f"Odometry reset:\nros2 service call /{sys.argv[1]}/reset_pose irobot_create_msgs/srv/ResetPose\n")  
        input("Type enter once odometry is reset")
        curses.wrapper(main)
```

Answer the following questions:
* In what states are certain actions forbidden? How would you characterize
those states and actions?
* How does the node determine the robot's current state?
* How does the Q-Table get updated? What data is necessary for the 
update, and how is this data acquired?
* How do the actions selected by Q-learning get transformed into 
  ROS2 commands?
* When the program runs, what is the overall structure of interactions
between the robot and the human determining its rewards?
  
  
## Experimentation
* Test the program. Experiment with giving the robot rewards. Overall, how
does the robot behave? Do you observe its behavior being impacted by 
rewards?
* Having played around with the program informally, design a more formal
experiment. For your experiment, determine the following:
  * Total number of Q-Learning updates.
  * Policy for determining how rewards will be administered.
    * This policy need not be consistent - the same state could receive
      different rewards at different times.
  * Values to use for the following parameters:
    * Discount
    * Target visits
    * Rate constant
    * Epsilon
    * Size of grid and cells, in turn determining the number of states.
* Plan to run three experiments. Each experiment should involve a variation
  of one or more of the above parameters. Write down hypotheses about what
  you expect to happen with regard to these variations.
* Run your experiments and record the results. How closely did your observations
  match what you hypothesized?
* Describe two possible applications of this concept in a practical 
  context of robot programming.
