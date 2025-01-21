---
layout: work
type: Module
num: 1
worktitle: ROS2 Nodes, Topics, and Subscriptions
---

## Getting Started

We will be programming our robots from their attached Raspberry Pi computers.
Each Raspberry Pi runs the GNU/Linux operating system. To work with the 
Raspberry Pi, you will need to open a remote shell.

From a Windows machine, to open a shell I recommend using 
[MobaXterm](https://mobaxterm.mobatek.net). From a Mac, open a Terminal 
window, then use ssh to open the remote shell.

## First topic: IR intensity

<!-- Exploration: IR Topic, Topics in general -->
* From the shell on your robot's Raspberry Pi, type `ros2 topic list`
  * If it displays just two lines, type it again until you see a longer list
* Each robot has a distinct name. When you see the longer topic list, look for a topic
  ending with `ir_intensity`. What is the name that appears before `ir_intensity`?
* Once the full list of topics appears, type 
  `ros2 topic echo /[your robot name]/ir_intensity`
* After running a few seconds, type Control-C. It should stop.
* Examine the output
  * What information is contained in the output?
  * How many IR sensors does the robot have?

<!-- Exploration: Values from IR Topic -->
Now run the same line again:
* Let it run for a while
* Wave your hand or foot in front of the robot.
* Hold it close to the robot, then move it further away slowly.
* How do the `value` entries change in response to motion?

<!-- Concept Invention: Topic Messages -->
Open a text editor. Copy and paste one message (starting and ending with `---`) into it.
* What are the fields of this message?
* What is the meaning of each field?

## Topics in Python

<!-- Exploration: Topics in Python -->
To explore the sensor more systematically, we will write a Python program
to view the sensor values. I recommend using the `micro` editor from within
the shell. To create this Python program using `micro`, type 
`micro sensor_viewer.py`. Once the editor opens, copy and paste the program
below:

```
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import IrIntensityVector


class SensorNode(Node):
    def __init__(self, robot_name: str):
        super().__init__(f'{robot_name}_SensorNode')
        self.create_subscription(IrIntensityVector, f"{robot_name}/ir_intensity", self.ir_callback, qos_profile_sensor_data)

    def ir_callback(self, msg: IrIntensityVector):
        print(msg)
        print()


def main():
    rclpy.init()
    node = SensorNode(sys.argv[1])
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 sensor_viewer.py robot_name")
    else:
        main()
```

Run the program as described in the `Usage` message.  While the program is 
running, open a second shell on your machine. Type `ros2 node list` in 
that shell. After doing so, use Control-C to halt your program. Then answer 
the following questions:
<!-- Concept Invention: ROS2 Topics -->
* What is a `Node`?
* What does it mean to "spin" a `Node`?
* What is a subscription?
* What is a callback?


As you did with the command-line output, copy and paste one instance of this 
data structure output to a text file. Then rewrite the 
`ir_callback()` method as follows:

```
    def ir_callback(self, msg: IrIntensityVector):
        print(msg.header)
        for reading in msg.readings:
            print(reading)
        print()
```

<!-- Exploration: Topic messages in Python -->
Again, copy and paste one instance of output to a text file. Then answer the following questions:
* Why did the second version of the Python code result in more readable output?
<!-- Concept Invention: Understanding topic messages -->
* How did it achieve this? Carefully examine both the command-line output and 
the previous `ir_callback()` output for clues. 
* Are the timestamps for the header and the individual sensors identical? 
  * If so, why do you think they are redundant? 
  * If not, why do you think they differ?
  
<!-- Application: Cleaning up topic messages in Python -->
Programming activities  
* Observe that the messages for the individual sensors are still very verbose. 
Using the differences between the two versions of `ir_callback()` as a guide, 
write a new version of `ir_callback()` that prints each message in the 
following format. Use Python format strings to achieve this appearance:
  
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

## Battery State
<!-- Exploration: Battery State Topic -->
Next, let's explore some more sensors. Run `ros2 topic list` and examine the 
output. Then answer the following questions:
* What topic is likely to give information about the amount of battery power 
remaining?
* Use `ros2 topic echo` to explore that topic. What information about the 
battery does the topic provide?
* What topic publishes messages more frequently: the IR topic or the battery 
topic?
<!-- Application: ROS2 Topics -->
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

## Frequencies
<!-- Exploration: Frequency -->
<!-- Exploration: Modules -->
* In a new file called `frequency.py` (in the same directory as 
`sensor_viewer.py`), paste the following code:
  
```
import unittest


class Frequency:
    def __init__(self, count=0, timestamp=0):
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
<!-- Concept Invention: Hertz -->
* Frequencies are expressed in units of Hertz (Hz). Based on the unit tests and your 
  method implementations, give a one-sentence definition of Hz.
* Under what circumstances would Hz be undefined?
<!-- Concept Invention: Modules -->
* Why might it be useful, as we did here, to define some of our code in a 
separate file outside our `Node` definition file?
<!-- Application: Frequency/Hertz -->
<!-- Application: Modules -->
* Extend `sensor_viewer.py` to display update frequencies for these sensors. 
  * To access the `Frequency` class, add the following `import` line:
    * `from frequency import Frequency`
  * Your output should look like the following. 
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
* Note that if `hz` is undefined, **it should not appear in the output**. Here is a sample 
output for that situation:
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

## Time-based Callbacks

Copy and paste the code below into a new file called `timer_demo.py`:
```
import sys
import rclpy
from rclpy.node import Node

class TimedNode(Node):
    def __init__(self, robot_name: str):
        super().__init__(f'{robot_name}_TimedNode')
        self.counter = 0
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.counter += 1
        print(f"count: {self.counter}")


def main():
    rclpy.init()
    node = TimedNode(sys.argv[1])
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 timer_demo.py robot_name")
    else:
        main()
```

<!-- Exploration: Timer callbacks -->
<!-- Application: Creating ROS2 nodes in Python -->
* What do you expect the program to do when it runs?
<!-- Concept formation: Timer callbacks -->
* Run the program. How frequently did it print the count?
* How would you modify the program so that it prints the count twice as often?
  * Implement and test your modification. Did it work as you predicted?
* How would you modify the program so that it prints the count half as often?
  * Implement and test your modification. Did it work as you predicted?
<!-- Application: Timer callbacks -->
* Create a copy of `sensor_viewer.py` called `timed_sensor_viewer.py`. 
  * To copy a file in the Unix shell, use the `cp` command: `cp sensor_viewer.py timed_sensor_viewer.py`
* Modify `timed_sensor_viewer.py` as follows:
  * Add a timer callback, set for executing twice per second.
  * Modify the IR callback so that it records the message rather than printing it.
  * Print both the IR message and the battery state message in the timer callback.
    * The format should be identical to the original.
    
    
## Publishing to a Topic
<!-- Exploration: Publishing Topics -->
Create a file named `ir_counter.py`, and copy the following code into it:
```
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import IrIntensityVector
from std_msgs.msg import String


class IrCounterNode(Node):
    def __init__(self, robot_name: str):
        super().__init__(f"{robot_name}_IrCounterNode")
        self.create_subscription(IrIntensityVector, f"{robot_name}/ir_intensity", self.ir_callback, qos_profile_sensor_data)
        self.counter = 0
        self.topic_name = f'{robot_name}_msgs'
        self.topic = self.create_publisher(String, self.topic_name, qos_profile_sensor_data)

    def ir_callback(self, msg: IrIntensityVector):
        output = String()
        output.data = f"{self.counter}"
        self.counter += 1
        self.topic.publish(output)
```

<!-- Concept Invention: Publishing Topics -->
What do you think `IrCounterNode` will do when it spins?

Create a file named `ir_count_printer.py`, and copy the following code into it:
```
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ir_counter import IrCounterNode
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor


class PrintNode(Node):
    def __init__(self, print_topic: str):
        super().__init__(f"PrintNode_{print_topic}")
        self.create_subscription(String, print_topic, self.callback, qos_profile_sensor_data)

    def callback(self, msg: String):
        print(msg.data)


def main():
    rclpy.init()
    robot_name = sys.argv[1]
    node = IrCounterNode(robot_name)
    printer = PrintNode(node.topic_name)
    executor = MultiThreadedExecutor()
    for n in (node, printer):
        executor.add_node(n)
    while True:
        try:
            executor.spin_once()
        except:
            break
    executor.shutdown()
    for n in (node, printer):
        n.destroy_node()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 ir_count_printer.py robot_name")
    else:
        main()
```

<!-- Concept Invention: Publishing to a topic -->
Run the `ir_count_printer.py` program. While it is running, open a second 
shell and type `ros2 topic list`. Then answer the following questions:
* What did the program do when it ran?
  * Was it consistent with what you expected `IrCounterNode` to do?
* What impact did running this program have on the output of `ros2 topic list`?
* What have you learned about ROS2 topics from this exercise?

<!-- Application: Separating modules from sensor_viewer.py -->
Using these two programs and `timed_sensor_viewer.py` as resources, recreate 
`timed_sensor_viewer.py` as two nodes: a publisher (in `sensor_messenger.py`) and a 
subscriber (in `sensor_printer.py`). Rather than directly printing the 
IR and battery messages straight to the console, it should instead create a 
string containing that message and publish it. Printing will be handled by 
the subscriber.


## Useful IR Values
<!-- Exploration: IR Values -->
* Select at least five different surfaces to assess the IR sensor values. 
For each surface:
  * Discuss why it is likely to be encountered by the robot.
  * Describe the surface. What color is it? Include photographs.
  * Record IR readings for the surface at distances 1 cm, 5 cm, 10 cm, 20 cm, 
and 50 cm from the sensor.
<!-- Concept Invention: IR Values and Surfaces -->
* Which surfaces are easiest to detect? Hardest to detect?
* For each surface, what is the farthest distance at which it is arguably "detected"?
