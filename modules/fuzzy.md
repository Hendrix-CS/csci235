---
layout: work
type: Module
num: 4
worktitle: Fuzzy Logic
---

## Boolean Logic

The table below is a **truth table**, describing how logic operators produce their results:

| x     | y     | NOT x | NOT y | x AND y | x OR y |
| ----- | ----- | ----- | ----- | ------- | ------ |
| true  | true  | false | false | true    | true   |
| true  | false | false | true  | false   | true   |
| false | true  | true  | false | false   | true   |
| false | false | true  | true  | false   | false  |
<!-- Exploration -->

Imagine that a person who is 6' 4" is considered `tall`. Now imagine two people, 
Mickey (6' 4") and Donald (6' 2"). Determine the truth value of each of these statements:
1. Mickey is `tall` **and** Donald is `tall`.
2. Mickey is `tall` **or** Donald is `tall`.
3. Mickey is **not** `tall`.
4. Donald is **not** `tall`.

## Fuzzy Logic

Consider using real numbers in the range 0.0 to 1.0 instead of `false` and `true`.
In this scheme, `true` would correspond to 1.0, `false` would be 0.0, and values 
in between would correspond to varying levels of truth.

For example, imagine that it is `true` that a person who is 6' 4" is `tall`, and `false`
that a person who is 5' 8" is `tall`. 

<!-- Exploration -->
1. On a scale of 0.0 to 1.0, how true would it be that a person who is 6' 2" is `tall`?
2. How about someone 5' 10"?
3. How about someone 6' 0"?

<!-- Concept invention -->
Based on those insights, give fuzzy answers to these four questions you answered earlier:
1. Mickey is `tall` **and** Donald is `tall`.
2. Mickey is `tall` **or** Donald is `tall`.
3. Mickey is **not** `tall`.
4. Donald is **not** `tall`.

<!-- Concept invention -->
Based on those answers, give a mathematical definition for each of the following fuzzy operators:
1. **and**
2. **or**
3. **not**

<!-- Application -->

Create a file called `fuzzy.py`. Create Python definitions for the functions 
`f_and()`, `f_or()`, and `f_not()` that implement the above mathematical definitions.
A correct solution should pass the unit tests below.

```
import unittest

def f_and(v1: float, v2: float):
    # Your code here


def f_or(v1: float, v2: float):
    # Your code here


def f_not(value: float):
    # Your code here
    
class FuzzyTest(unittest.TestCase):
    def test_and_or_not(self):
        self.assertEqual(0.75, f_and(1.0, 0.75))
        self.assertEqual(1.0, f_or(1.0, 0.75))
        self.assertEqual(0.0, f_not(1.0))
        self.assertEqual(1.0, f_not(0.0))

if __name__ == "__main__":
    unittest.main()
```

## Fuzzification

<!-- Application -->
Following the above example, we will say that a height of 6' 4" (76") is `tall` (1.0), and a height 
of 5' 8" (68") is **not** `tall` (0.0). To **fuzzify** these values is to convert them from their 
original units to fuzzy values. 

Add the following function to `fuzzy.py`, and implement it:
```
def fuzzify(value: float, start: float, end: float):
    # Your code here.
```

Also add this unit-testing method to `FuzzyTest`:
```
    def test_fuzzify(self):
        for expected, height in [(1.0, 76), (0.75, 74), (0.5, 72), (0.25, 70),
                                 (0.0, 68), (1.0, 80), (0.0, 62)]:
            self.assertEqual(expected, fuzzify(height, 68, 76))
```

A correct solution should pass this additional unit test.


## Defuzzification

To transform a fuzzy value into a useful output, we **defuzzify** it. For instance, we might transform
a fuzzy value for **tall** into an inseam length as follows:

| `tall` | inseam |
| -----: | -----: |
| 1.0    | 36.0   |
| 0.75   | 34.5   |
| 0.5    |        |
| 0.25   |        |
| 0.0    | 30.0   |

<!-- Exploration -->
1. What should the inseam value be for `tall` = 0.5?
2. How about `tall` = 0.25?
<!-- Concept invention -->
3. Write down a mathematical formula for the inseam value given the value for `tall`, according to 
the pattern in this table.

<!-- Application --> 
Add the `defuzzify()` function below to `fuzzy.py`, and implement it.
```
def defuzzify(value: float, zero: float, one: float):
    # Your code here.
```

Also add this unit-testing method to `FuzzyTest`:
```
    def test_defuzzify(self):
        for inseam_size, fuzzy_height in [(36, 1.0), (34.5, 0.75), (33, 0.5), 
                                          (31.5, 0.25), (30, 0.0)]:
            self.assertEqual(inseam_size, defuzzify(fuzzy_height, 30, 36))
```

A correct solution should pass this additional unit test.


## Fuzzy IR Input Node

<!-- Application --> 

Copy the following code into a new file, `ir_fuzzy_input.py`. 

```
from typing import Any

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from irobot_create_msgs.msg import IrIntensityVector
import fuzzy


class FuzzyIrNode(Node):
    def __init__(self, robot_name: str, ir_fuzzy_start: int, ir_fuzzy_end: int):
        super().__init__(f'FuzzyIrNode_{robot_name}')
        self.create_subscription(IrIntensityVector, f"{robot_name}/ir_intensity", 
                                 self.ir_callback, qos_profile_sensor_data)
        self.output_topic = f'{robot_name}_ir_blocked'
        self.output = self.create_publisher(String, self.output_topic, qos_profile_sensor_data)
        self.debug_topic = f'{robot_name}_debug_topic'
        self.debug = self.create_publisher(String, self.debug_topic, qos_profile_sensor_data)
        self.ir_fuzzy_start = ir_fuzzy_start
        self.ir_fuzzy_end = ir_fuzzy_end

    def publish(self, publisher: Publisher, data: Any):
        output = String()
        output.data = f"{data}"
        publisher.publish(output)

    def ir_callback(self, msg: IrIntensityVector):
        out_msg = {}
        debug_msg = ""
        for reading in msg.readings:
            f = fuzzy.fuzzify(reading.value, self.ir_fuzzy_start, self.ir_fuzzy_end)
            out_msg[reading.header.frame_id] = f
            debug_msg += f"{reading.header.frame_id:32}{reading.value:>4}  {f:>.2f} {' ' * 10}\n"
        self.publish(self.output, out_msg)
        self.publish(self.debug, debug_msg)
```

1. What will a `FuzzyIrNode` object do when spun?
2. Based on your experience with the IR sensors, what value would you suggest for `ir_fuzzy_start`?
3. How about `ir_fuzzy_end`?


## Defuzzified motor controller node

<!-- Application --> 

Copy the following code into a new file, `ir_fuzzy_avoider.py`:

```
import sys, curses
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

from ir_fuzzy_input import FuzzyIrNode
from curses_runner import CursesNode, run_curses_nodes
import fuzzy


class FuzzyDriveNode(Node):
    def __init__(self, robot_name: str, fuzzy_topic: str, x_limit: float, z_limit: float):
        super().__init__(f"FuzzyDriveNode_{robot_name}")
        self.x_limit = x_limit
        self.z_limit = z_limit
        self.motors = self.create_publisher(TwistStamped, f"{robot_name}/cmd_vel_stamped", qos_profile_sensor_data)
        self.create_subscription(String, fuzzy_topic, self.fuzzy_callback, qos_profile_sensor_data)

    def fuzzy_callback(self, msg: String):
        fuzzy_values = eval(msg.data)
        blocked = {"left": 0.0, "right": 0.0, "ir_intensity": 0.0}
        for ir, value in fuzzy_values.items():
            for area in blocked:
                if area in ir:
                    blocked[area] = fuzzy.f_or(blocked[area], value)

        t = self.make_twist()
        t.twist.linear.x = fuzzy.defuzzify(fuzzy.f_not(blocked["ir_intensity"]), 0, self.x_limit)
        turn_limit = self.z_limit * (-1.0 if blocked["left"] > blocked["right"] else 1.0)
        t.twist.angular.z = fuzzy.defuzzify(fuzzy.f_or(blocked["left"], blocked["right"]), 0, turn_limit)
        self.motors.publish(t)
            
    def make_twist(self) -> TwistStamped:
        t = TwistStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = self.get_clock().now().to_msg()
        return t


def main(stdscr):
    cmd = parse_cmd_line_values()
    rclpy.init()
    sensor_node = FuzzyIrNode(sys.argv[1], cmd['min_ir'], cmd['max_ir'])
    curses_node = CursesNode(sensor_node.debug_topic, 2, stdscr)
    drive_node = FuzzyDriveNode(sys.argv[1], sensor_node.output_topic, cmd['x_limit'], cmd['z_limit'])
    run_curses_nodes(stdscr, [drive_node, curses_node, sensor_node])
    rclpy.shutdown()


def parse_cmd_line_values() -> Dict[str,float]:
    parsed = {}
    for arg in sys.argv:
        if '=' in arg:
            parts = arg.split('=')
            parsed[parts[0]] = float(parts[1])
    return parsed


if __name__ == '__main__':
    if len(sys.argv) < 6:
        print("Usage: python3 ir_fuzzy_avoider.py robot_name min_ir=value max_ir=value x_limit=value z_limit=value")
    else:
        curses.wrapper(main)
```

1. What will a `FuzzyDriveNode` object do when spun?
2. Based on your experience with the motors sensors, what value would you suggest for `x_limit`?
3. How about `z_limit`?
4. Write a command-line invocation of `ir_fuzzy_avoider.py` that will run it with the four values you specified.


## Running a fuzzy controller

<!-- Exploration -->

Copy `curses_runner.py` from `module2` into your `module4` folder. Then run `ir_fuzzy_avoider` with the
command-line arguments you specified.

1. Overall, how well does your robot perform as an obstacle avoider?

<!-- Concept invention -->
2. Do you think the `ir_fuzzy_start` value enables the robot to start turning when it should?
3. Do you think the `ir_fuzzy_end` value puts the robot into a sharp turn soon enough to avoid objects?
4. Do you think the `x_limit` value is fast enough to enable the robot to make progress but slow enough to
   give it ample time to avoid hitting objects?
5. Do you think the `z_limit` value is a good match to the `x_limit` value?

<!-- Application -->
6. Based on your observations, experiment with some different values for these four variables. For each variation
that you try, record its impact on the robot's performance. Create at least three distinct combinations of 
values. Which combination performed the best?  Why?


## Fuzzy navigation

