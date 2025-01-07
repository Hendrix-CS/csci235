---
layout: work
type: Activity
num: 1
worktitle: Sensors
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


class SensorNode(Node):
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
    node = SensorNode(node_name, robot)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python3 sensor_viewer.py node_name robot_name")
    else:
        main()

```

Run the program as described in the `Usage` message. Use `SensorNode` followed by your first
and last name for the `node_name`. (For example, Mickey Mouse would use the node name 
`SensorNodeMickeyMouse`.)

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
builtin_interfaces.msg.Time(sec=23718998, nanosec=222784006)
ir_intensity_side_left              0
ir_intensity_left                   5
ir_intensity_front_left           167
ir_intensity_front_center_left    104
ir_intensity_front_center_right     3
ir_intensity_front_right            6
ir_intensity_right                  0
```
* The time gives the number of seconds since January 1, 1970. 
  * Modify the `SensorNode` class so that it has an attribute storing the earliest timestamp it encounters.
  * Using this attribute, modify your output message to display the number of seconds since
    the first message was received, rather than since January 1, 1970. 
  * Once this version is working, divide the nanoseconds by 10^9, and add them to the 
    number of seconds.
  * If you wish to reference the data type for timestamps, use the following import
    * `from rclpy.time import Time`
  * The resulting output messages should be formatted as follows. Be sure to use Python's format
    string feature to ensure the resulting number of seconds goes out to exactly 9 decimal places.
```
11.785712313s
ir_intensity_side_left              0
ir_intensity_left                   2
ir_intensity_front_left           164
ir_intensity_front_center_left    106
ir_intensity_front_center_right     3
ir_intensity_front_right            3
ir_intensity_right                  0
```

Next, let's explore some more sensors. Run `ros2 topic list` and examine the output. Then 
answer the following questions:
* What topic is likely to give information about the amount of battery power remaining?
* Use `ros2 topic echo` to explore that topic. What information about the battery does the
  topic provide?
* What topic publishes messages more frequently: the IR topic or the battery topic?
* Modify `sensor_viewer.py` to also display the current battery power level. 
  * Add the following import to `sensor_viewer.py`:
    * `from sensor_msgs.msg import BatteryState`
  * Create a subscription to the topic. 
    * Be sure to create a callback method to handle each message that arrives.
  * Store the data from the less frequently encountered topic in an attribute of your 
    `SensorNode`. Then display the stored data alongside the most recently acquired data 
    in the callback for your more frequently encountered topic. 
  * Your output should look like the following:
```
** IR timestamp: 16.265703198s
ir_intensity_side_left              2
ir_intensity_left                   3
ir_intensity_front_left           168
ir_intensity_front_center_left    105
ir_intensity_front_center_right     5
ir_intensity_front_right            5
ir_intensity_right                  1
** Battery timestamp: 14.931958523s
Battery level: 100.00%
```
* Create a frequency tracker. In a new file called `frequency.py` (in the same directory as
  `sensor_viewer.py`), paste the following code:
  
```
import unittest


class Frequency:
    def __init__(self, count=0, timestamp=1):
        self.count = count
        self.timestamp = timestamp

    def record(self, timestamp: float):
        pass # Your code here

    def hz(self) -> float:
        pass # Your code here


class FrequencyTest(unittest.TestCase):
    def test1(self):
        f = Frequency()
        self.assertEqual(None, f.hz())
        for time, expected in [(0.5, 1 / 0.5), (1.0, 2 / 1.0), (2.0, 3 / 2.0)]:
            f.record(time)
            self.assertEqual(expected, f.hz())

    def test2(self):
        f = Frequency(10, 4)
        self.assertEqual(2.5, f.hz())
        for time, expected in [(5.5, 11 / 5.5), (6.0, 12 / 6.0), (7.0, 13 / 7.0)]:
            f.record(time)
            self.assertEqual(expected, f.hz())


if __name__ == '__main__':
    unittest.main()
```
* Implement the methods `record()` and `hz()` so that the unit tests pass.
* Frequencies are expressed in units of Hertz (Hz). Based on the unit tests and your 
  method implementations, give a one-sentence definition of Hz.
* Under what circumstances would Hz be undefined?
* Why might it be useful, as we did here, to define some of our code in a separate file
  outside our `Node` definition file?
* Extend `sensor_viewer.py` to display update frequencies for these sensors. 
  * To access the `Frequency` class, add the following `import` line:
    * `from frequency import Frequency`
  * Your output should look like the following. (**Note**: If `hz` is undefined, it should
    simply not appear in the output.)
```
** IR timestamp: 12.745317869 (62.5 hz)
ir_intensity_side_left              0
ir_intensity_left                   2
ir_intensity_front_left           169
ir_intensity_front_center_left    103
ir_intensity_front_center_right     6
ir_intensity_front_right            1
ir_intensity_right                  2
** Battery timestamp: 8.642997608 (0.2 hz)
Battery level: 96.00%
```
* Note that if `hz` is undefined, it should not appear in the output. Here is a sample output
  for that situation:
```
** IR timestamp: 0.000000000
ir_intensity_side_left              0
ir_intensity_left                   4
ir_intensity_front_left           163
ir_intensity_front_center_left    106
ir_intensity_front_center_right     0
ir_intensity_front_right            4
ir_intensity_right                  0
```
* Add a subscription to the `hazard_detection` topic. Add `from irobot_create_msgs.msg import HazardDetectionVector`
  to your imports. Then DO STUFF.

TODO: More fun things:
* Perform experiments to test IR values against a range of different surfaces
