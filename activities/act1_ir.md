---
layout: work
type: Activity
num: 1
worktitle: The Infrared Sensors
---

Exploration
* Open a shell on your robot's Raspberry Pi
* type `ros2 topic list`
  * If it displays just two lines, type it again until you see a longer list

Now type: `ros2 topic echo /archangel/ir_intensity`
* After running a few seconds, type Control-C. It should stop.
* Examine the output
  * What information is contained in the output?
  * How many IR sensors does the robot have?

Now run the same line again:
* Let it run for a while
* wave your hand or foot in front of the robot
* hold it close to the robot, then move it further away slowly
* how do the `value` entries change in response to motion?

Open a text editor. Copy and paste one message (starting and ending with `---`) into it.
* What are the fields of this message?
* What is the meaning of each field?

To explore the sensor more systematically, we will write a Python program
to view the sensor values. Type in the following program using `nano`:

```
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import IrIntensityVector


class IrNode(Node):
    def __init__(self, node_name: str, namespace: str):
        super().__init__(node_name)
        if len(namespace) > 0 and not namespace.startswith('/'):
            namespace = f"/{namespace}"
        self.create_subscription(IrIntensityVector, f"{namespace}/ir_intensity", self.ir_callback, qos_profile_sensor_data)

    def ir_callback(self, msg: IrIntensityVector):
        print(msg)
        print()


def main():
    rclpy.init()
    node_name = sys.argv[1]
    robot = sys.argv[2]
    node = IrNode(node_name, robot)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python3 ir_viewer.py node_name robot_name")
    else:
        main()

```

Run the program as described in the `Usage` message. Use `IrNode` followed by your first
and last name for the `node_name`. (For example, Mickey Mouse would use the node name 
`IrNodeMickeyMouse`.)

While the program is running, open a second shell on your machine. Type `ros2 node list` in 
that shell. After doing so, use Control-C to halt your program. Then answer the following 
questions:
* What is a `Node`?
* What does it mean to "spin" a `Node`?
* What is a subscription?
* What is a callback?


As you did with the command-line output, copy
and paste one instance of this data structure output to a text file. Then rewrite the 
`ir_callback()` method as follows:
```
    def ir_callback(self, msg: IrIntensityVector):
        print(msg.header)
        for reading in msg.readings:
            print(reading)
        print()
```

Again, copy and paste one instance of output to a text file. Then answer the following questions:
* Why did the second version of the Python code result in more readable output?
* How did it achieve this? Carefully examine both the command-line output and the previous
  `ir_callback()` output for clues. 
* Are the timestamps for the header and the individual sensors identical? 
  * If so, why do you think they are redundant? 
  * If not, why do you think they differ?
  
Programming activities  
* Observe that the messages for the individual sensors are still very verbose. Using the 
  differences between the two versions of `ir_callback()` as a guide, write a new version of 
  `ir_callback()` that prints each message in the following format. Use Python format strings
  to achieve this appearance:
  
```
builtin_interfaces.msg.Time(sec=23639456, nanosec=40284135)
ir_intensity_side_left                   2
ir_intensity_left                        6
ir_intensity_front_left                 21
ir_intensity_front_center_left          74
ir_intensity_front_center_right          6
ir_intensity_front_right                 9
ir_intensity_right                       2
```
* The time gives the number of seconds since January 1, 1970. 
  * Modify the `IrNode` class so that it has an attribute storing the earliest timestamp it encounters.
  * Using this attribute, modify your output message to display the number of seconds since
    the first message was received, rather than since January 1, 1970. 
  * Once this version is working, divide the nanoseconds by 10^9, and add them to the 
    number of seconds.
  * The resulting output messages should be formatted as follows. Be sure to use Python's format
    string feature to ensure the resulting number of seconds goes out to exactly 9 decimal places.

```
12.862000815
ir_intensity_side_left                   2
ir_intensity_left                        4
ir_intensity_front_left                 19
ir_intensity_front_center_left          74
ir_intensity_front_center_right          5
ir_intensity_front_right                 3
ir_intensity_right                       1
```

TODO: More fun things:
* Calculate and display IR update rate per second
* Perform experiments to test IR values against a range of different surfaces
* Subscribe to battery and display level
* Subscribe to hazards

* Display 
