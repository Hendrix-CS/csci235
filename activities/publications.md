---
layout: work
type: Activity
num: 2
worktitle: ROS2 Publications
---

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
    node_name = sys.argv[1]
    robot = sys.argv[2]
    node = IrCounterNode(f'{node_name}_sensor', robot)
    printer = PrintNode(node_name, node.topic_name)
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
    if len(sys.argv) < 3:
        print("Usage: python3 ir_count_printer.py node_name robot_name")
    else:
        main()
```

Run the `ir_count_printer.py` program. While it is running, open a second shell 
and type `ros2 topic list`. Then answer the following questions:
* What did the program do when it ran?
  * Was it consistent with what you expected `IrCounterNode` to do?
* What impact did running this program have on the output of `ros2 topic list`?
* What have you learned about ROS2 topics from this exercise?

Using these two programs and `sensor_viewer.py` as resources, recreate 
`sensor_viewer.py` as two nodes: a publisher (in `sensor_counter.py`) and a subscriber
(in `sensor_count_printer.py`). Rather than directly printing the IR and battery messages
straight to the console, it should instead create a string containing that message and
publish it. Printing will be handled by the subscriber.

Examine the following program:
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
* [Have them do some modifications to understand it better]
* Overall, what is the purpose of the `curses` library?
* How might the `curses` library be useful for our ROS2 programs?

Create a file called `curses_runner.py` and copy and paste the code below into it:
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


def curses_runner(nodes: List[Node], stdscr):
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
```

Then create `curses_printer.py` and copy and paste the code below into it:
```
from typing import List
import curses, sys

import rclpy
from ir_counter import IrCounterNode
from curses_runner import curses_runner, CursesNode


def main(stdscr):
    rclpy.init()
    node_name = sys.argv[1]
    robot = sys.argv[2]
    node = IrCounterNode(f'{node_name}_sensor', robot)
    printer = CursesNode(node_name, node.topic_name, 2, stdscr)
    curses_runner([node, printer], stdscr)
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python3 curses_printer.py node_name robot_name")
    else:
        curses.wrapper(main)
```

Answer the following questions:
* What do you think these two programs will do?
* How does the `CursesNode` work?
