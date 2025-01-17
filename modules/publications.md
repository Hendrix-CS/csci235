---
layout: work
type: Module
num: 2
worktitle: ROS2 Publications
---

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
    def __init__(self, node_name: str, namespace: str):
        super().__init__(node_name)
        if len(namespace) > 0 and not namespace.startswith('/'):
            namespace = f"/{namespace}"
        self.create_subscription(IrIntensityVector, f"{namespace}/ir_intensity", self.ir_callback, qos_profile_sensor_data)
        self.counter = 0
        self.topic_name = f'{namespace}_msgs'
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
    def __init__(self, node_name: str, print_topic: str):
        super().__init__(node_name)
        self.create_subscription(String, print_topic, self.callback, qos_profile_sensor_data)

    def callback(self, msg: String):
        print(msg.data)


def main():
    rclpy.init()
    robot = sys.argv[1]
    node = IrCounterNode(f"{robot}_IrCounterNode", robot)
    printer = PrintNode(f"{robot}_PrintNode", node.topic_name)
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
Using these two programs and `sensor_viewer.py` as resources, recreate 
`sensor_viewer.py` as two nodes: a publisher (in `sensor_messenger.py`) and a 
subscriber (in `sensor_printer.py`). Rather than directly printing the 
IR and battery messages straight to the console, it should instead create a 
string containing that message and publish it. Printing will be handled by 
the subscriber.

## Console User Interfaces

<!-- Exploration: Curses UI -->
Create a file called `curses_demo.py` and copy and paste the code below into it:
```
import curses

def main(stdscr):
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()
    counter = 0

    stdscr.addstr(0, 0, "Starting...")    
    stdscr.refresh()
    running = True
    while running:
        try:
            k = stdscr.getch()
            if k != -1 and chr(k) == 'q':
                running = False            

            stdscr.addstr(3, 5, f"count: {counter}")
            stdscr.refresh()
            counter += 1
        except curses.error as e:
            if str(e) != 'no input':
                stdscr.addstr(0, 0, traceback.format_exc())
                stdscr.refresh()
        

if __name__ == '__main__':
    curses.wrapper(main)
```

Run the program. Then answer the following questions:
* What does the program do?
<!-- Concept Invention: Curses -->
* What is the purpose of the `curses` library?
* How might the `curses` library be useful for our ROS2 programs? In 
  particular, what advantages might it have over our previous approach to I/O?

<!-- Concept Invention: Example Console UI for ROS2 -->
Create a file called `curses_runner.py` and copy and paste the code below into 
it:
```
from typing import List
import curses, sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ir_counter import IrCounterNode
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

class CursesNode(Node):
    def __init__(self, node_name: str, print_topic: str, first_line: int, stdscr):
        super().__init__(node_name)
        self.create_subscription(String, print_topic, self.callback, qos_profile_sensor_data)
        self.first_line = first_line
        self.stdscr = stdscr

    def callback(self, msg: String):
        lines = msg.data.split('\n')
        for i, line in enumerate(lines):
            self.stdscr.addstr(i + self.first_line, 0, line)


def run_curses_nodes(stdscr, nodes: List[Node]):
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()
    executor = MultiThreadedExecutor()
    for n in nodes:
        executor.add_node(n)
    
    running = True
    while running:
        try:
            executor.spin_once()
            k = stdscr.getch()
            if k != -1 and chr(k) == 'q':
                running = False
        except curses.error as e:
            if str(e) != 'no input':
                stdscr.addstr(0, 0, traceback.format_exc())

    executor.shutdown()
    for n in nodes:
        n.destroy_node()    

    curses.nocbreak()
    curses.echo()
    stdscr.refresh()
```

Then create `curses_printer.py` and copy and paste the code below into it:
```
from typing import List
import curses, sys

import rclpy
from ir_counter import IrCounterNode
from curses_runner import run_curses_nodes, CursesNode


def main(stdscr):
    rclpy.init()
    robot = sys.argv[1]
    node = IrCounterNode(f'{robot}_IrCounterNode', robot)
    printer = CursesNode(f'{robot}_CursesNode', node.topic_name, 2, stdscr)
    run_curses_nodes(stdscr, [node, printer])
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 curses_printer.py robot_name")
    else:
        curses.wrapper(main)
```

Answer the following questions:
* Carefully examine `curses_runner.py`:
  * How is it similar to `curses_demo.py`? 
  * How does the `CursesNode` work?
* Now examine `curses_printer.py`:
  * How does it interact with `curses_runner.py`?
  * It creates two ROS2 nodes. How do they interact with each other and with `curses`?
<!-- Application: Curses Console UI for sensor values -->
<!-- Application: Modules -->
* What modifications need to be made to `curses_printer.py` to display 
  the sensor information from `sensor_messenger.py`?
* Make a copy of `curses_printer.py` called `curses_sensor.py`. Apply the
  modifications you described above. Then run the program.
* Compare the experience of running `curses_sensor.py` with 
  `sensor_printer.py`. Which of them is a more usable user interface? Why?

## Publishing motor commands

<!-- Exploration: cmd_vel from the command line -->
Type the command below into the command line:
```
ros2 topic pub -r 1 archangel/cmd_vel_stamped geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}"
```

* What happens when you run this command?
  * To stop, use Control-C
* Try `-r 2` instead of `-r 1`. How does its behavior differ?
  * Also try `-r 4`. What impact does it have?
* Change the angular z value from 0.5 to 2.5. What happens?
  * Now try 3.5, 4.5, and 5.5. What happens with each value?
* Now try a negative value for angular z. What happens?
* Change the angular z value to 0.0, and the linear x value to 0.1. What happens?
* Now try linear x at 0.3, then 0.5, then 0.7. What happens with each of these changes?
* Next, try linear x at 0.3 and angular z at 0.5. What happens?
<!-- Concept Invention: Create your own Twists -->
* Devise and publish a `TwistStamped` message that causes the robot to drive in a circle with a radius 
  of two meters.

## Writing a motor publisher

<!-- Exploration: cmd_vel from Python -->

<!-- Concept Invention: Understanding Twists -->
<!-- Application: Integrating motor commands -->

