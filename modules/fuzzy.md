---
layout: work
type: Module
num: 5
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


def f_and(v1: float, v2: float) -> float:
    # Your code here


def f_or(v1: float, v2: float) -> float:
    # Your code here


def f_not(value: float) -> float:
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
def fuzzify(value: float, start: float, end: float) -> float:
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
def defuzzify(value: float, zero: float, one: float) -> float:
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

### Reverse defuzzification

Consider the following table for a fuzzy value for **short**:

| `short` | inseam |
| -----:  | -----: |
| 1.0     | 26.0   |
| 0.75    | 27.5   |
| 0.5     |        |
| 0.25    |        |
| 0.0     | 32.0   |

1. What should the inseam value be for `short` = 0.5?
2. How about `short` = 0.25?
3. Do you think `defuzzify()` would need to be modified to perform the above
   defuzzification of `short`? Why or why not?
3. Write an additional unit test for `defuzzify()` called 
   `test_defuzzify_short()`. This test will be similar
   to the previous test, except that the test will assess the values from 
   this table describing the defuzzification of `short`.
4. Run the additional test and make sure it works as expected.


## Distance Values

When thinking about navigating a robot from one location to another,
we define an **error** as the gap between where the robot is and where
we want it to be. There are two types of errors to consider:
1. The **distance error** is the distance between the robot's current 
location and goal location.
2. The **heading error** is the angular distance between the robot's 
current heading and a heading that, if the robot were to drive straight, 
would take the robot to the goal location.

Let's augment the `RobotPose` class to enable us to calculate these errors.
First, add the `distance_to()` method, which will calculate the distance error.
Use the Euclidean distance formula derived from the Pythagorean theorem:

```
    def distance_to(self, goal_x: float, goal_y: float) -> float:
        # Your code here
```

Next, let's add the `turn_to()` method, which will calculate the heading error.
Note that this is a two-part calculation:
* First, use `math.atan2()` to determine the heading offset between the robot's
  position and the goal position.
* Then, subtract the robot's heading and normalize the result to determine how
  far the robot needs to turn to correct the error.

```
    def turn_to(self, goal_x: float, goal_y: float) -> float:
        # Your code here
```

Here are unit tests to add:

```
    def test_distance_to(self):
        pose = RobotPose(1.0, -1.0, 0.0)
        for (gx, gy, d) in [(1.0, -1.0, 0.0), (4.0, 3.0, 5.0), (-4.0, -13.0, 13.0)]:
            self.assertAlmostEqual(pose.distance_to(gx, gy), d, places=3)

    def test_turn_to(self):
        pose = RobotPose(1.0, -1.0, math.pi / 2)
        for (gx, gy, d) in [(0.0,  0.0, math.pi / 4),
                            (0.0, -1.0, math.pi / 2),
                            (1.0,  0.0, 0.0),
                            (2.0,  0.0, -math.pi / 4),
                            (0.0, -2.0, 3 * math.pi / 4)]:
            self.assertAlmostEqual(pose.turn_to(gx, gy), d, places=3)
```

Answer the following questions:
1. We will say that the robot is **traveling** to its target when it has
not reached it. How would you define whether the robot is still traveling?
Why?
2. We will say that the robot is **left** of its target when turning with
a **positive** angular velocity will make the robot more closely aligned with its
target. How would you define whether the robot is left of its target? Why?
3. We will say that the robot is **right** of its target when turning with a
**negative** angular velocity will make the robot more closely aligned with its
target. How would you define whether the robot is right of its target? Why?
4. According to your definitions, is it ever possible for **left** and **right**
to **both** be true? Why or why not?
5. In terms of **left**, **right**, and **traveling**:
   * Under what circumstances should the linear x velocity be:
     * Zero? 
     * A positive number? 
   * Under what circumstances should the angular z velocity be:
     * Zero?
     * A positive number?
     * A negative number?
   * Explain all of your above answers

Create a new file (`align_go.py`) and copy the following code into it.
Write Python code to express your definitions of **traveling**, **left**,
and **right**, and assign the motor settings accordingly.

```
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from robot_pose import RobotPose, odom2pose
from nav_msgs.msg import Odometry


class DriveNode(Node):
    def __init__(self, robot_name: str, goal_x: float, goal_y: float):
        super().__init__(f"{robot_name}_DriveNode")
        self.motors = self.create_publisher(TwistStamped, f"{robot_name}/cmd_vel_stamped", qos_profile_sensor_data)
        self.create_subscription(Odometry, f"{robot_name}/odom", self.odom_callback, qos_profile_sensor_data)
        self.goal_x = goal_x
        self.goal_y = goal_y

    def odom_callback(self, odom: Odometry):
        t = TwistStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = self.get_clock().now().to_msg()

        pose = odom2pose(odom)
        traveling = # Write a boolean expression from your answer above
        left = # Write a boolean expression from your answer above
        right = # Write a boolean expression from your answer above

        # Using your definitions, write some `if` statements to assign
        # values for t.twist.linear.x and t.twist.angular.z that follow
        # your answers above.

        self.motors.publish(t)


def main():
    rclpy.init()
    node = DriveNode(sys.argv[1], float(sys.argv[2]), float(sys.argv[3]))
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: python3 align_go.py robot_name goal_x goal_y")
    else:
        main()

```

## Fuzzy Distance Values
* Recall that in fuzzy logic, concepts may be true, false, or partially true,
with true concepts having a value of 1.0, false concepts 0.0, and partially
true concepts between 0.0 and 1.0.
* What might be a useful fuzzy definition of **traveling**? Why?
* How about fuzzy definitions of **left** and **right**? Why?
* Make a copy of `align_go.py` called `fuzzy_align_go.py`.
  * To make a copy on the Linux command line, type `cp align_go.py fuzzy_align_go.py`.
* Add `from fuzzy import fuzzify, defuzzify, f_and, f_or, f_not` to the top.
* Replace your boolean definitions of **traveling**, **left**, and **right** with 
  calls to `fuzzify()` to create fuzzy-logic definitions of those terms.
* Write an assignment of a value to `t.twist.linear.x` in which you translate the
boolean logic you employed earlier into fuzzy logic, using `f_and`, `f_or`, and
`f_not` as appropriate.
* Write an assignment of a value to `t.twist.angular.z` in which you translate the
boolean logic you employed earlier into fuzzy logic, using `f_and`, `f_or`, and
`f_not` as appropriate.
* Test the resulting program. How does the robot's perfomance compare to `align_go.py`?

