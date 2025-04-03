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

|   | 0 | 1 | 2 |
| - | - | - | - |
| 0 | 0 | 1 | 2 |
| 1 | 3 | 4 | 5 |
| 2 | 6 | 7 | 8 |

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
  * North, West, South, South, East, East, North, North, South, East, East, South, West, East
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
into Q-learning states.

```
from geometry_msgs.msg import Point
from typing import Tuple
import unittest

class GridConverter:
    def __init__(self, width: float, height: float, cell_size: float):
        self.cols = int(width / cell_size)
        self.rows = int(height / cell_size)
        self.cell_size = cell_size
        self.width = width
        self.height = height
        self.x_bound = width / 2.0
        self.y_bound = height / 2.0

    def square_within_grid(self, grid_square: Tuple[int, int]) -> bool:
        return 0 <= grid_square[0] < self.cols and 0 <= grid_square[1] < self.rows

    def from_odometry(self, position: Point) -> Tuple[int, int]:
        col = int((position.x + self.x_bound) / self.cell_size)
        row = int((position.y + self.y_bound) / self.cell_size)
        return col, row
    
    def from_grid(self, grid_square: Tuple[int, int]) -> Point:
        col, row = grid_square
        p = Point()
        p.x = col * self.cell_size - self.x_bound + self.cell_size / 2.0
        p.y = row * self.cell_size - self.y_bound + self.cell_size / 2.0
        return p
    
    def num_states(self) -> int:
        return self.cols * self.rows
    
    def state_num_for(self, grid_square: Tuple[int, int]) -> int:
        col, row = grid_square
        return row * self.cols + col
    
    def grid_square_for(self, state_num: int) -> Tuple[int, int]:
        return state_num % self.cols, state_num // self.cols


class GridConverterTest(unittest.TestCase):
    def test_state_to_square(self):
        conv = GridConverter(1.5, 1.5, 0.5)
        self.assertEqual(conv.cols, 3)
        self.assertEqual(conv.rows, 3)
        expected = [(x, y) for y in range(3) for x in range(3)]
        for state in range(9):
            grid_square = conv.grid_square_for(state)
            self.assertEqual(expected[state], grid_square)
            self.assertEqual(conv.state_num_for(grid_square), state)

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
            col, row = conv.from_odometry(p)
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
            odom = conv.from_grid((col, row))
            self.assertAlmostEqual(odom.x, expected_odom_x, 5)
            self.assertAlmostEqual(odom.y, expected_odom_y, 5)


if __name__ == '__main__':
    unittest.main()
```

## A Q-Learning implementation

<!-- From here
* Go over formula
* Go over learning rate
* Have them do some hand-calculation
* Then do an implementation with Button 1 as positive reward and 
  Button 2 as negative reward.
-->