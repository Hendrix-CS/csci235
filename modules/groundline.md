---
layout: work
type: Module
num: 6
worktitle: Computer Vision I: Groundline
---

## Groundline

We employ computer vision to find the boundary between the ground and
objects resting on the ground. When you run `groundline_node`, it fills
its window with an image showing where the ground is located. A sample
image is given below.

<img src="{{site.baseurl}}/assets/images/groundline.jpg" />

The groundline itself represents, for each `x` coordinate, the number
of pixels of ground that are present. When an object is close to the 
robot, there will be very few pixels of ground present. When an object
is far from the robot, there will be a larger number of pixels of 
ground present. The `groundline_node` publishes this list
of pixel heights on the topic `robot_groundline`. 
(Substitute your robot's name for `robot`; on `frodo`, for instance,
the topic is `frodo_groundline`.) 

Run `groundline_node` as follows, substituting your robot's name for `robot_name`:
```
groundline_node robot_name
```

If, when you run the program, you see an error like this:
```
Error: Could not open device 0: V4L2 Error: Permission denied (os error 13)
```

Type this into the command prompt:
```
sudo chmod 666 /dev/video*
```
and enter your password when requested.

To get a feel for the groundline, do the following:
* Open three shells on your robot, in different tabs.
* In one shell, run `groundline_node robot_name`, substituting the
  name of your robot for `robot_name`.
* In a second shell, run `ros2 topic echo `robot_groundline`, 
  substituting the name of your robot for `robot`.
* In a third shell, run `curses_motor.py`.
* Drive your robot around for a while. How does the groundline change
as the robot drives around? What kinds of values do you see published
  on the `robot_groundline` topic?

## Finding floor regions

A **floor region** has the following characteristics:
* It is at least `min_width` pixels wide (working from a 160 x 120 image)
* All of the `x` coordinates in the region have a pixel height
of at least `min_height` pixels 
* The **center** of a floor region is the **x** coordinate that is in the middle.

Create a file `floor_region.py` and copy and paste the code below:
```
import unittest


class FloorRegion:
    def __init__(self, x_start: int, length: int):
        self.x_start = x_start
        self.length = length

    def __repr__(self):
        return f"FloorRegion({self.x_start}, {self.length})"

    def __eq__(self, other):
        return self.x_start == other.x_start and self.length == other.length

    def center(self):
        # YOUR CODE HERE


def find_regions(groundline: list[int], min_height: int, min_width: int) -> list[FloorRegion]:
    # YOUR CODE HERE
    

class FloorTest(unittest.TestCase):
    def test_region(self):
        line = [119, 9, 9, 9, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19, 19, 19, 19, 20, 20, 20, 20, 20, 20, 20, 20, 21, 21, 21, 21, 21, 21, 21, 21, 21, 22, 22, 22, 22, 22, 22, 22, 22, 23, 23, 23, 23, 23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 24, 25, 25, 26, 26, 28, 30, 33, 42, 42, 42, 42, 42, 42, 42, 42, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 119]
        regions = find_regions(line, 30, 16)
        self.assertEqual(regions, [FloorRegion(105, 54)])
        self.assertEqual(regions[0].center(), 132)


        line = [24, 25, 25, 26, 26, 28, 30, 33, 42, 42, 42, 42, 42, 42, 42, 42, 41, 41, 41, 41, 41, 12, 13, 13, 14, 14, 15, 42, 42, 42, 42,42, 42, 42, 42,42, 42, 42, 42,42, 42, 42, 42, 26, 28, 30, 33, 42, 42, 42, 42, 42, 42, 42, 42, 41,42, 42, 42, 42, 42, 42, 42, 42, 41]
        regions = find_regions(line, 30, 16)
        self.assertEqual(regions, [FloorRegion(27, 16), FloorRegion(45, 19)])
        self.assertEqual(regions[0].center(), 35)
        self.assertEqual(regions[1].center(), 54)


if __name__ == '__main__':
    unittest.main()
```

* Implement `find_regions()` and the `center()` method. 
* Test and make sure they work correctly.
* What strategies did you have to employ to get `find_regions()` to work properly?

## Groundline node

Create a new file `groundline.py` and copy and paste the code below into it:
```
import sys, curses, math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from floor_region import FloorRegion, find_regions

SCREEN_WIDTH = 160
SCREEN_HEIGHT = 120
MIN_WIDTH = 16
MIN_HEIGHT = 30


class DriveNode(Node):
    def __init__(self, robot_name: str):
        super().__init__(f"{robot_name}_DriveNode")
        self.create_subscription(String, f"{robot_name}_groundline", self.groundline_callback, qos_profile_sensor_data)
        self.motors = self.create_publisher(TwistStamped, f"{robot_name}/cmd_vel_stamped", qos_profile_sensor_data)
        self.running = True
        self.paused = False

    def groundline_callback(self, groundline_str: String):
        t = TwistStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = self.get_clock().now().to_msg()

        groundline = eval(groundline_str.data)
        regions = find_regions(groundline, MIN_HEIGHT, MIN_WIDTH)

        if not self.paused:
            # YOUR CODE HERE
            
        self.motors.publish(t)

    def process_keystroke(self, k: str):
        if k == 'q':
            self.running = False
        elif k == 'p':
            self.paused = not self.paused


def main(stdscr):
    rclpy.init()
    node = DriveNode(sys.argv[1])
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
            stdscr.addstr(0, 0, f"STATE INFORMATION HERE")
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
    if len(sys.argv) < 2:
        print("Usage: python3 groundline.py robot_name")
    else:
        curses.wrapper(main)

```

* Devise a strategy for using the groundline to create an obstacle-avoiding robot.
Write down your strategy in your journal.
* How would you need to modify the above program to implement your strategy? Pay attention
both to how the strategy will work as well as what information you want to display to 
give you feedback about how it is working.
* Modify the program according to your previous answer.
* To run `groundline.py`, first run `groundline_node` in a separate shell. Once it is 
up and running, start `groundline.py` in its own shell.
* Test your program. What worked well? What didn't work?
* What might be a good modification to your strategy to address what didn't work well?
* Implement and test your modification. Repeat this cycle of modification and testing
until its performance is satisfactory or until you have performed three modifications.

## To Submit
* `floor_region.py`
* `groundline.py`
