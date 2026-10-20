---
layout: work
type: Module
num: 7
worktitle: Reinforcement Learning
---


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
* In states `6`, `7`, and `1`, the robot receives a reward of 0.25.
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

Copy and paste the following Python code into `grid.py`:
```
from geometry_msgs.msg import Point
import unittest

UP = 0
RIGHT = 1
DOWN = 2
LEFT = 3

HEADINGS = [math.pi/2, 0, -math.pi/2, -math.pi]

class GridConverter:
    def __init__(self, width: float, height: float, cell_size: float):
        self.cols = int(width / cell_size)
        self.rows = int(height / cell_size)
        
    def num_states(self) -> int:
        return self.cols * self.rows
	
    def grid_to_state(self, grid_square: tuple[int, int]) -> int:
        col, row = grid_square
        return # YOUR CODE HERE 
    
    def state_to_grid(self, state_num: int) -> tuple[int, int]:
        return # YOUR CODE HERE
        
    def forbidden_actions(self) -> list[tuple[int, int]]:
        forbidden = []
        for i in range(self.cols):
            forbidden.append((self.grid_to_state((i, 0)), DOWN))
            forbidden.append((self.grid_to_state((i, self.rows - 1)), UP))
        for i in range(self.rows):
            forbidden.append((self.grid_to_state((0, i)), LEFT))
            forbidden.append((self.grid_to_state((self.cols - 1, i)), RIGHT))
        return forbidden
```


Considering anew the state-encoding table we saw earlier, complete the 
implementations of `grid_to_state()` and `state_to_grid()` so that they 
work correctly for the nine-state example given in the table.

|       | Column 0 | Column 1 | Column 2 |
| ----  | -------- | -------- | -------- |
| Row 0 | 0 (0, 0) | 1 (1, 0) | 2 (2, 0) |
| Row 1 | 3 (0, 1) | 4 (1, 1) | 5 (2, 1) |
| Row 2 | 6 (0, 2) | 7 (1, 2) | 8 (2, 2) |

Test them with the following test code:

```
class GridConverterTest(unittest.TestCase):
    def test_state_to_square(self):
        conv = GridConverter(1.5, 1.5, 0.5)
        self.assertEqual(conv.cols, 3)
        self.assertEqual(conv.rows, 3)
        expected = [
            (0, 0), (1, 0), (2, 0), 
            (0, 1), (1, 1), (2, 1), 
            (0, 2), (1, 2), (2, 2)
        ]
        for state in range(9):
            grid_square = conv.state_to_grid(state)
            self.assertEqual(expected[state], grid_square)
            self.assertEqual(conv.grid_to_state(grid_square), state)
			

if __name__ == '__main__':
    unittest.main()
```

Now that we can convert grid positions to and from Q-states, we will next work 
out how to transform odometry readings to and from grid positions. Let's begin
by adding three lines to the constructor so that it looks like the following:

```
    def __init__(self, width: float, height: float, cell_size: float):
        self.cols = int(width / cell_size)
        self.rows = int(height / cell_size)
        self.cell_size = cell_size
        self.max_odom_x = width / 2.0
        self.max_odom_y = height / 2.0
```

* Imagine that the `width` and `height` are both `10.0` and `cell_size` is 
  `2.0`. What are:
  * `self.cols`?
  * `self.rows`?
  * `self.max_odom_x`?
  * `self.max_odom_y`?
* In what cell column and cell row would you expect the odometry origin 
(position (0.0, 0.0)) to land? Why?
* In which cell columns and rows would you expect the following odometry
positions to land?
  * (4.5, 2.0) 
  * (-3.1, -0.5)
  * (0.0, 0.0)
  * (0.8, -0.8) 
  * (1.0, 1.0) 
  * (-1.0, -1.0)
* If we start with a given cell column and row, is there exactly one 
  corresponding pair of odometry values or are there many possible pairs?
  Explain your answer.
* For each of the following pairs of cell columns and rows, state the
  odometry value you would want to produce, with reference to your previous
  answer:
  * (4, 3)
  * (0, 2)
  * (2, 2)
  * (3, 3)
* Based on your previous answers, what is your understanding of the 
  relationship between odometry coordinates and grid coordinates in this 
  implementation?

Based on your answers to the above questions, complete the implementations of
the `odom_to_grid()` and `grid_to_odom()` methods:
```
    def odom_to_grid(self, position: Point) -> tuple[int, int]:
        col = # YOUR CODE HERE
        row = # YOUR CODE HERE
        return col, row
    
    def grid_to_odom(self, grid_square: tuple[int, int]) -> Point:
        col, row = grid_square
        p = Point()
        p.x = # YOUR CODE HERE
        p.y = # YOUR CODE HERE
        return p
```

Add these tests in order to assess your implementations:
```
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
```


## A Q-Learning implementation

Examine the Python implementation of Q-Learning below. Copy and paste it
into `qlearning.py`:
```
import random
from typing import Sequence, Tuple


class QParameters:
    def __init__(self, target_visits: int=2, discount: float=0.5, num_states: int=2, num_actions: int=2):
        self.target_visits = target_visits
        self.discount = discount
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
        self.discount = params.discount
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
        return 1 / (1 + self.visits[state][action])

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
        return below_target

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
* Walk through the implementation of the `sense_act_learn()` method. How does
  this method implement the update function given earlier?
* From how the code is written, what do you think it means to **explore**?
* For each of the two alternatives for action selection presented earlier:
  * Which method implements this alternative?
  * How does that method implement this alternative?
* How does the program's approach to action selection compare with what you
discussed as a possible option for combining the two alternative approaches?
* Why do you think we might want to forbid some actions in certain states?

## A Q-Learning Node

### The GridNode class
We will start by creating a node that navigates between squares. The code 
below relies upon `grid.py` that we completed above, `robot_pose.py` from
[Module 1]({{site.baseurl}}/modules/intro1.html), and `fuzzy.py` from 
[Module 5]({{site.baseurl}}/modules/fuzzy.html). It employs a fuzzy logic
controller to navigate the robot from one grid square to another. Create
a file named `grid_node.py` and copy the code below into it:

```
import sys, curses, math, random, copy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from robot_pose import RobotPose, odom2pose, turn_from_to
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import InterfaceButtons
from grid import GridConverter, UP, DOWN, LEFT, RIGHT, HEADINGS
from fuzzy import fuzzify, defuzzify, f_and, f_or, f_not


ANGLE_TOLERANCE = math.pi/24
DISTANCE_TOLERANCE = 0.05
MAX_FORWARD = 0.25
MAX_TURN = 0.5
MAX_ALIGN = math.pi/8


class GridNode(Node):
    def __init__(self, robot_name: str, width: float, height: float, cell_size: float):
        super().__init__(f"{robot_name}_GridNode")
        self.create_subscription(Odometry, f"{robot_name}/odom", self.odom_callback, qos_profile_sensor_data)
        self.motors = self.create_publisher(TwistStamped, f"{robot_name}/cmd_vel_stamped", qos_profile_sensor_data)
        self.grid = GridConverter(width, height, cell_size) 

        self.current_action = None
        self.pose = None
        self.goal_pose = None

        self.last_key = None

    def grid_col_row(self) -> tuple[int, int]:
        if self.pose is not None:
            return self.grid.odom_to_grid(self.pose.point())

    def state(self):
        col_row = self.grid_col_row()
        if col_row is not None:
            return self.grid.grid_to_state(col_row)

    def odom_callback(self, odom: Odometry):    
        self.pose = odom2pose(odom)
        if self.goal_pose is not None:
            self.head_for_goal()

    def make_twist(self) -> TwistStamped:
        t = TwistStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = self.get_clock().now().to_msg()
        return t

    def update_goal(self, action: int):
        self.current_action = action
        self.goal_pose = copy.copy(self.pose)
        self.goal_pose.theta = HEADINGS[self.current_action]        
        self.goal_pose.move(self.grid.cell_size)

    def head_for_goal(self):
        target_angle = self.pose.turn_to(self.goal_pose.x, self.goal_pose.y)
        distance = self.pose.distance_to(self.goal_pose.x, self.goal_pose.y)
        if distance < DISTANCE_TOLERANCE:
            self.final_heading_align()        
        else:
            self.go_to_goal(target_angle, distance)    

    def final_heading_align(self):
        offset = turn_from_to(self.pose.theta, self.goal_pose.theta)
        if abs(offset) < ANGLE_TOLERANCE:
            self.goal_pose = None
        else:
            t = self.make_twist()
            offset_left, offset_right = fuzzy_angle_left_right(offset)
            t.twist.angular.z = defuzzy_angle(offset_left, offset_right)
            self.motors.publish(t)

    def go_to_goal(self, target_angle: float, distance: float):
        fuzzy_left, fuzzy_right = fuzzy_angle_left_right(target_angle)
        fuzzy_distance = f_and(fuzzify(distance, 0.0, self.grid.cell_size), 
                               f_not(f_or(fuzzy_left, fuzzy_right)))
        t = self.make_twist()
        t.twist.angular.z = defuzzy_angle(fuzzy_left, fuzzy_right)
        t.twist.linear.x = defuzzify(fuzzy_distance, 0.0, MAX_FORWARD)
        self.motors.publish(t)

    def process_keystroke(self, k: str):
        self.last_key = k
        if k.isdigit():
            self.update_goal(int(k))

    def running(self) -> bool:
        return self.last_key != 'q'

    def display(self, stdscr):
        stdscr.addstr(0, 0, "0: up, 1: right, 2: down, 3: left")
        if self.last_key is not None:
            stdscr.addstr(1, 0, self.last_key)
        display_padded(stdscr, 2, f"pose     : {self.pose}")
        display_padded(stdscr, 3, f"goal_pose: {self.goal_pose}")
        display_padded(stdscr, 4, f"action   : {self.current_action}")
        if self.pose is not None:
            display_padded(stdscr, 5, f"state    : {self.state()}; ({self.grid_col_row()})")


def display_padded(stdscr, row: int, s: str):
    height, width = stdscr.getmaxyx()
    stdscr.addstr(row, 0, f"{s}{' ' * (width - len(s) - 1)}")


def fuzzy_angle_left_right(angle: float) -> tuple[float, float]:
    fuzzy_left = fuzzify(angle, ANGLE_TOLERANCE, MAX_ALIGN)
    fuzzy_right = fuzzify(-angle, ANGLE_TOLERANCE, MAX_ALIGN)
    return fuzzy_left, fuzzy_right
    

def defuzzy_angle(fuzzy_left: float, fuzzy_right: float) -> float:
    if fuzzy_left > fuzzy_right:
        return defuzzify(fuzzy_left, 0.0, MAX_TURN)
    else:
        return defuzzify(fuzzy_right, 0.0, -MAX_TURN)


def main(stdscr):
    rclpy.init()
    node = GridNode(sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()
    while node.running():
        try:
            k = stdscr.getch()
            if k != -1:
                node.process_keystroke(chr(k))
            node.display(stdscr)
            rclpy.spin_once(node, timeout_sec=0.0)
        except curses.error as e:
            if str(e) != 'no input':
                stdscr.addstr(0, 0, traceback.format_exc())

    rclpy.shutdown()
    node.destroy_node()
    curses.nocbreak()
    curses.echo()
    stdscr.refresh()


if __name__ == '__main__':
    if len(sys.argv) < 5:
        print("Usage: python3 grid_node.py robot_name width height cell_size")
    else:
        curses.wrapper(main)
```

### Q-Learning Node Code

Examine the Python implementation of a Q-Learning ROS2 node. Copy and paste it
into `qnode.py`:

```
import sys, curses, math, random, copy
import rclpy
from rclpy.qos import qos_profile_sensor_data
from robot_pose import RobotPose, odom2pose, turn_from_to
from irobot_create_msgs.msg import InterfaceButtons
from qlearning import QParameters, QTable
from grid import HEADINGS
import grid_node


class QNode(grid_node.GridNode):
    def __init__(self, robot_name: str, target_visits: int, discount: float):
        super().__init__(robot_name, 1.2, 1.2, 0.4)
        self.create_subscription(InterfaceButtons, f"{robot_name}/interface_buttons", self.button_callback, qos_profile_sensor_data)

        params = QParameters(target_visits, discount, num_states=self.grid.num_states(), num_actions=len(HEADINGS))
        params.forbid_state_actions(self.grid.forbidden_actions())
        self.qtable = QTable(params)
        self.started = False

    def button_callback(self, buttons: InterfaceButtons):
        if self.started:
            if self.goal_pose is None:
                reward = None
                if buttons.button_1.is_pressed:
                    reward = -1.0
                elif buttons.button_2.is_pressed:
                    reward = 0.25
                elif buttons.button_power.is_pressed:
                    reward = 0.0
                if reward is not None:
                    action = self.qtable.sense_act_learn(self.state(), reward)
                    self.update_goal(action)
        elif self.pose is not None and (buttons.button_1.is_pressed or buttons.button_2.is_pressed or buttons.button_power.is_pressed):
            self.started = True
            self.update_goal(random.randint(0, len(HEADINGS) - 1)) 

    def display(self, stdscr):
        super().display(stdscr)
        for state in range(self.qtable.num_states()):
            col, row = self.grid.state_to_grid(state)    
            values = [round(v, 2) for v in self.qtable.q[state]]
            stdscr.addstr(6 + self.grid.rows + state, 0, f"state {state} ({col}, {row}): qs: {values} visits:  {self.qtable.visits[state]}")


def main(stdscr):
    rclpy.init()
    node = QNode(sys.argv[1], int(sys.argv[2]), float(sys.argv[3]))
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()
    while node.running():
        try:
            k = stdscr.getch()
            if k != -1:
                node.process_keystroke(chr(k))
            node.display(stdscr)
            rclpy.spin_once(node, timeout_sec=0.0)
        except curses.error as e:
            if str(e) != 'no input':
                stdscr.addstr(0, 0, traceback.format_exc())

    rclpy.shutdown()
    node.destroy_node()
    curses.nocbreak()
    curses.echo()
    stdscr.refresh()


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: python3 qnode.py robot_name target_visits discount")
    else:
        curses.wrapper(main)
```

Answer the following questions. Note that you may have to consult 
`grid_node.py` in addition to `qnode.py` to answer some of them:
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
    * Size of grid and cells, in turn determining the number of states.
* Plan to run three experiments. Each experiment should involve a variation
  of one or more of the above parameters. Write down hypotheses about what
  you expect to happen with regard to these variations.
* Run your experiments and record the results. How closely did your observations
  match what you hypothesized?
* Describe two possible applications of this concept in a practical 
  context of robot programming.
