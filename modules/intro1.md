---
layout: work
type: Module
num: 1
worktitle: Robot Pose
---

## Normalized angles

When writing software to control a mobile robot, it is extremely important
to represent the robot's **pose**. The pose is typically modeled as a triple:
its $x$ and $y$ coordinates, as well as its **heading**, represented in 
radians. Because of the heading, we will be working with angles a great deal.
Thus, it is useful to write some functions to help with handling them.

Examine each pair of angles in radians. Which ones are the same angle?
* 0, 2 &pi;
* 0, &pi;
* -&pi;, &pi;
* &pi;/4, 3&pi;/4
* &pi;/4, 5&pi;/4
* &pi;/4, 7&pi;/4
* &pi;/4, 9&pi;/4

Because, for any given angle, there are an infinite number of possible 
representations, we want to **normalize** angles to fall in a particular
range. As we want 0 radians to represent going straight, by using positive
and negative radians we can represent different turn directions. So it is
to our advantage to use the interval (-&pi;, &pi]. 

Read the Python program below. The function `find_normalized_angle()` needs 
to return an angle in the interval  (-&pi;, &pi]. The provided unit test
checks to make sure it has this property.

Copy this code into a Python editor. Implement a solution to 
`find_normalized_angle()` that passes the unit tests.

```
import unittest, math


def find_normalized_angle(angle: float) -> float:
    """
    Ensure that the angle in radians lies between -math.pi and math.pi
    """
    pass


class AngleMathTest(unittest.TestCase):
    def test_normalized_angle(self):
        for theta, normed in [
            (-math.pi, math.pi),
            (2 * math.pi, 0.0), 
            (3/2 * math.pi, -math.pi/2), 
            (9 * math.pi, math.pi),
            (-3 * math.pi, math.pi),
            (5/2 * math.pi, math.pi/2),
        ]:
            self.assertEqual(normed, find_normalized_angle(theta))


if __name__ == '__main__':
    unittest.main()
```

## Representing robot pose using a Python class

Read the partial class definition below:
```
class RobotPose:
    """
    Represents a robot pose (x, y, theta)
    theta should always be normalized
    """
    def __init__(self, x: float, y: float, theta: float):
        self.x = x
        self.y = y
        self.theta = find_normalized_angle(theta)

    def __repr__(self):
        return f"({self.x},{self.y},{self.theta})"

    def turn(self, angle: float):
	"""
        Adds angle to theta to represent a turn
        """
	pass

    def move(self, distance: float):
        """
        Updates x and y based on distance and self.theta
        """
        pass
```

