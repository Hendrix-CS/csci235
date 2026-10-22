---
layout: work
type: Module
num: 5
worktitle: Fuzzy Logic
---

<!-- TODO
* Add a section about converging on a heading.
* Note that with traditional logic it is easy to overshoot.
* Then use fuzzy logic to hit it more precisely.
-->

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

As calculating the heading error is a multi-step process, we will first write
a couple of functions that will enable this to happen. The first function
is `shortest_angle_diff()`. To prepare for writing it, answer the following
questions:
* What is 3/4 &pi; - 1/4 &pi;, normalized to be positive (i.e., in the range 0 to 2 &pi;)?
* What is 1/4 &pi; - 3/4 &pi;, again normalized to be positive?
* Given the angles 3/4 &pi; and 1/4 &pi;, what are two possible angular 
distances between them?
* Which of the two distances is smaller?
* Repeat the above exercise with the angles -7/8 &pi; and 7/8 &pi;
* What would be a general algorithm for returning the shortest angle 
  difference between two angles?
* Complete `shortest_angle_diff()` below. Be sure to use 
`normalize_angle_positive()` from 
[Module 1]({{site.baseurl}}/modules/intro1.html) in your solution.


```
def shortest_angle_diff(angle1: float, angle2: float) -> float:
    """
    Find the shortest distance between two angles.
    Parameters will be in radians.
    Result will be in the range [0, math.pi]
    """
    # Your code here
```

Add the following unit test to ensure that your solution works:
```
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
```

Having found the shortest angle between two angles, we must now determine the 
proper sign for the angle that will enable us to correct the error. Given that 
our robot is facing a particular direction, we want to find out the angle it
would need to turn to reach a target heading. 
* Call `shortest_angle_diff(3/4 * math.pi, 1/4 * math.pi)` and record the
  answer.
* What is 3/4 &pi; + the answer - 1/4 &pi;?
* What is 3/4 &pi; - the answer - 1/4 &pi;?
* If 3/4 &pi; is the starting angle and 1/4 &pi; is the target heading, 
  would it be preferable to add or subtract the answer?
* If 1/4 &pi; is the starting angle and 3/4 &pi; is the target heading,
  would it be preferable to add or subtract the answer?
* Repeat this exercise with -7/8 &pi; and 7/8 &pi;. 
* Based on these answers, what would be a general algorithm for determining
whether one should make a left turn (positive angle) or a right turn 
(negative angle)?
* Complete `turn_from_to()` below:

```
def turn_from_to(starting_angle: float, goal_angle: float) -> float:
    """
    Return an angle that, if you were to add it to the starting angle,
    would produce the goal angle.
    The returned angle should be the smallest possible angle to 
    return in terms of absolute value.
    Parameters will be in radians.
    """
    # Your code here
```

Make sure it passes the following unit tests:
```
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
            (-0.51623311853, -0.05063084481, 0.465602273719),
            (-0.58370492313, -0.01613905879, 0.56756586434)
        ]:
            self.assertAlmostEqual(diff, turn_from_to(theta1, theta2), places=5)
```

Let's augment the `RobotPose` class to enable us to calculate these errors.
First, add the `distance_to()` method, which will calculate the distance error.
Use the Euclidean distance formula derived from the Pythagorean theorem:

```
    def distance_to(self, goal_x: float, goal_y: float) -> float:
        # Your code here
```

Calculating heading error is a multi-step process. First, we need to calculate
the difference between two angles. 

Next, let's add the `turn_to()` method, which will calculate the heading error.
Note that this is a two-part calculation:
* First, use `math.atan2()` to determine the heading offset between the robot's
  position and the goal position.
* Then, call `turn_from_to()` to determine how far and in what direction 
  the robot needs to turn to correct the error.

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
Examine the code and answer the following questions:
* What are the state variables assigned in `__init__()`?
* For each state variable assigned in `__init__()`, why is it a state variable
  and not a local variable? To answer this question, examine the rest of the
  code to see how each state variable is employed.
* Why might it be useful to display all of these state variables?
* Why might it be useful to have a pause button?

Write Python code to express your definitions of **traveling**, **left**,
and **right**, and assign the motor settings accordingly.

```
import sys, curses, math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from robot_pose import RobotPose, odom2pose
from nav_msgs.msg import Odometry


class DriveNode(Node):
    def __init__(self, robot_name: str, goal_x: float, goal_y: float):
        super().__init__(f"{robot_name}_DriveNode")
        self.create_subscription(Odometry, f"{robot_name}/odom", self.odom_callback, qos_profile_sensor_data)
        self.motors = self.create_publisher(TwistStamped, f"{robot_name}/cmd_vel_stamped", qos_profile_sensor_data)
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.running = True
        self.paused = False
        self.pose = None
        self.distance_to = None
        self.turn_to = None
        self.traveling = True
        self.left = False
        self.right = False

    def odom_callback(self, odom: Odometry):
        t = TwistStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = self.get_clock().now().to_msg()

        self.pose = odom2pose(odom)
        self.distance_to = self.pose.distance_to(self.goal_x, self.goal_y)
        self.turn_to = self.pose.turn_to(self.goal_x, self.goal_y)  

        self.traveling = # Write a boolean expression from your answer above
        self.left = # Write a boolean expression from your answer above
        self.right = # Write a boolean expression from your answer above

        if not self.paused:
            # Using your definitions, write some `if` statements to assign
            # values for t.twist.linear.x and t.twist.angular.z that follow
            # your answers above.

        self.motors.publish(t)

    def process_keystroke(self, k: str):
        if k == 'q':
            self.running = False
        elif k == 'p':
            self.paused = not self.paused


def main(stdscr):
    rclpy.init()
    node = DriveNode(sys.argv[1], float(sys.argv[2]), float(sys.argv[3]))
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()
    while node.running:
        try:
            k = stdscr.getch()
            if k != -1:
                k = chr(k)
                stdscr.addstr(2, 0, k)
                node.process_keystroke(k)
            if node.pose is not None:
                stdscr.addstr(0, 0, f"({node.pose.x:.2f}, {node.pose.y:.2f}): {node.pose.theta:.2f}   ")
                stdscr.addstr(1, 0, f"goal: {node.goal_x:.2f} {node.goal_y:.2f}")
                stdscr.addstr(2, 0, f"distance error: {node.distance_to:.2f}")
                stdscr.addstr(3, 0, f"heading error: {node.turn_to:.2f}")
                stdscr.addstr(4, 0, f"traveling? {node.traveling}  ")
                stdscr.addstr(5, 0, f"left? {node.left}  ")
                stdscr.addstr(6, 0, f"right? {node.right}  ")
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
        print("Usage: python3 align_go.py robot_name goal_x goal_y")
    else:
        curses.wrapper(main)

```

* Before running `align_go.py`, be sure to reset the odometry to (0, 0). 
  Use the command 
  `ros2 service call /[robot name]/reset_pose irobot_create_msgs/srv/ResetPose`
  to do so, substituting the name of your robot prior to `reset_pose`
* Test `align_go.py`. How well does it work? Do you need to momdify any of your
  definitions? If so, make a note of your modifications and their rationale in
  your journal.

## Fuzzy Distance Values
* Recall that in fuzzy logic, concepts may be true, false, or partially true,
with true concepts having a value of 1.0, false concepts 0.0, and partially
true concepts between 0.0 and 1.0.
* What might be a useful fuzzy definition of **traveling**? Why?
* How about fuzzy definitions of **left** and **right**? Why?
* Make a copy of `align_go.py` called `fuzzy_align_go.py`.
  * To make a copy on the Linux command line, type 
  `cp align_go.py fuzzy_align_go.py`.
* Add `from fuzzy import fuzzify, defuzzify, f_and, f_or, f_not` to the top.
* Replace your boolean definitions of **traveling**, **left**, and **right** 
  with calls to `fuzzify()` to create fuzzy-logic definitions of those terms.
* Write an assignment of a value to `t.twist.linear.x` in which you translate 
  the boolean logic you employed earlier into fuzzy logic, using `f_and`, 
  `f_or`, and `f_not`, and defuzzify it to a linear motion value using 
  `defuzzify()`.
* Write an assignment of a value to `t.twist.angular.z` in which you translate 
  the boolean logic you employed earlier into fuzzy logic, using `f_and`, 
  `f_or`, and `f_not`, and defuzzify it to a linear motion value using 
  `defuzzify()`.
* **NOTE**: Not all of your boolean logic will necessarily be translateable
  into fuzzy logic. In some cases you will need to retain the original 
  boolean logic. 
* How translateable was your boolean logic into fuzzy logic? And how do you
  anticipate changes to the robot's behavior from this translation?
* Test the resulting program. How does the robot's perfomance compare to 
  `align_go.py`? How accurately did you predict the changed behavior?
  

