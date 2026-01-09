---
layout: work
type: Module
num: 2
worktitle: ROS2 Publications
---

## Publishing motor commands

<!-- Exploration: cmd_vel_stamped from the command line -->
Type the command below into the command line:
```
ros2 topic pub -r 1 [your robot name]/cmd_vel_stamped geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}"
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
  of 50 centimeters.
* What tactics did you employ to determine the radius of the circle the robot traversed?

## Motor commands in Python

Create a new folder to contain today's code:
```
mkdir module2
cd module2
```

Create a file named `pub_motor.py`, and copy the following code into it:
```
import curses, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped


class DriveNode(Node):
    def __init__(self, robot_name: str):
        super().__init__(f"{robot_name}_DriveNode")
        self.motors = self.create_publisher(TwistStamped, f"{robot_name}/cmd_vel_stamped", qos_profile_sensor_data)
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        t = TwistStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = self.get_clock().now().to_msg()
        t.twist.angular.z = 1.0
        self.motors.publish(t)


def main():
    rclpy.init()
    node = DriveNode(sys.argv[1])
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 pub_motor.py robot_name")
    else:
        main()
```

Answer the following questions:
<!-- Exploration: Python TwistStamped -->
* Examine the program. What do you expect it to do when it runs?
* Run the program. What did it do? Did it meet your expectations?
<!-- Concept invention: direction of rotation from a TwistStamped -->
* Replace `t.twist.angular.z = 1.0` with `t.twist.angular.z = -1.0`.
  * What do you expect will happen when it runs?
  * Run the program. What did it do? Did it meet your expectations?
* Try `2.0` instead of `1.0`. Also try `0.5`. What happens?
<!-- Exploration: Drive forward -->
<!-- Application: cmd_vel from the command line -->
* How might you modify the `TwistStamped` object so that the robot drives forward?
* Perform the modification. Continue to experiment until it works as you expect.
* Modify the program so that the robot drives in a circle of radius 50 centimeters, using the 
  command-line `TwistStamped` message you created earlier.

## Console User Interfaces

<!-- Exploration: Curses UI -->
Create a file called `curses_demo.py` and copy and paste the code below into it:
```
import curses, traceback

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

    curses.nocbreak()
    curses.echo()
    stdscr.refresh()
        

if __name__ == '__main__':
    curses.wrapper(main)
```

Run the program. Then answer the following questions:
* What does the program do?
* Change the `3` in `stdscr.addstr()` to `10`. Then run the program again. What happens?
* Change the `5` in `stdscr.addstr()` to `20`. Then run the program again. What happens?
<!-- Concept Invention: Curses -->
* What can you conclude about the `curses` coordinate system from this?
* What is the purpose of the `curses` library?
* How might the `curses` library be useful for our ROS2 programs? In 
  particular, what advantages might it have over our previous approach to I/O?

## Console UI for ROS2
<!-- Concept Invention: Example Console UI for ROS2 -->
Copy `ir_counter.py` from `module1` into `module2`. Then create a file 
called `curses_runner.py` and copy and paste the code below into it:
```
from typing import List
import curses, traceback, sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

class CursesNode(Node):
    def __init__(self, print_topic: str, first_line: int, stdscr):
        super().__init__(f"CursesNode_{print_topic}")
        self.create_subscription(String, print_topic, self.callback, qos_profile_sensor_data)
        self.first_line = first_line
        self.stdscr = stdscr

    def callback(self, msg: String):
        lines = msg.data.split('\n')
        for i, line in enumerate(lines):
            self.stdscr.addstr(i + self.first_line, 0, line)


def run_curses_nodes(stdscr, nodes: List[Node]):
    executor = MultiThreadedExecutor()
    startup(stdscr, executor, nodes)
    run_loop(stdscr, executor)
    shutdown(stdscr, executor, nodes)


def startup(stdscr, executor: MultiThreadedExecutor, nodes: List[Node]):
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()
    for n in nodes:
        executor.add_node(n)


def run_loop(stdscr, executor: MultiThreadedExecutor):
    while True:
        try:
            executor.spin_once()
            k = stdscr.getch()
            if k != -1 and chr(k) == 'q':
                return
        except curses.error as e:
            if str(e) != 'no input':
                stdscr.addstr(0, 0, traceback.format_exc())


def shutdown(stdscr, executor: MultiThreadedExecutor, nodes: List[Node]):
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
    node = IrCounterNode(sys.argv[1])
    printer = CursesNode(node.topic_name, 2, stdscr)
    run_curses_nodes(stdscr, [node, printer])
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 curses_runner.py robot_name")
    else:
        curses.wrapper(main)
```

Answer the following questions:
* Carefully examine `curses_runner.py`:
  * How is it similar to `curses_demo.py`? 
  * How do objects of the `CursesNode` class work?
  * What does the `startup()` function do?
  * What does the `run_loop()` function do?
  * What does the `shutdown()` function do?
* Now examine `curses_printer.py`:
  * How does it interact with `curses_runner.py`?
  * It creates two ROS2 nodes. How do they interact with each other and with `curses`?
<!-- Application: Curses Console UI for sensor values -->
<!-- Application: Modules -->
* What modifications would need to be made to `curses_printer.py` to display 
  the sensor information from `sensor_messenger.py`?
* Copy `sensor_messenger.py` and `frequency.py` from the `module1` folder into 
  the `module2` folder. If you are in the `module2` folder, the following
  `cp` commands should work:
```
cp ../module1/sensor_messenger.py .
cp ../module1/frequency.py .
```  
* Make a copy of `curses_printer.py` called `curses_sensor.py`. 
* Apply the modifications you described above. Then run the program.
* Compare the experience of running `curses_sensor.py` with 
  `sensor_printer.py`. Which of them is a more usable user interface? Why?

## Processing Keystrokes in `curses`

A **thread** represents a distinct stream of execution. Each ROS2 node runs in its own
thread in parallel with the other nodes. This enables them to handle their callbacks
immediately when events happen, regardless of what else is happening in the rest of
the program.
* Go back and examine the `startup()` function in `curses_runner.py`. By what
mechanism are the nodes placed into separate threads?

Because code in different threads can run in an unpredictable order, one must be careful
when communicating between threads. We will use the 
[`Queue`](https://docs.python.org/3/library/queue.html) class for this purpose.
Add the following `import` to `curses_runner.py`:
```
from queue import Queue
```

The `put()` method adds an item to a `Queue`. The `get()` method removes the oldest item
from the queue. The `empty()` method returns `True` if there is nothing in the `Queue`, 
and `False` otherwise.

As `curses` is running in its own thread, we will use `Queue` objects to send keyboard
inputs to nodes that are intended to respond to them.

Copy and paste into `curses_runner.py` this modified version of `startup()`:
```
def startup(stdscr, executor: MultiThreadedExecutor, nodes: List[Node], key_nodes: List[Node]):
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()
    for n in nodes:
        executor.add_node(n)
        if hasattr(n, 'key_queue') and type(n.key_queue) == Queue:
            key_nodes.append(n)
```

Next, copy and paste into `curses_runner.py` this modified version of `run_loop()`:
```    
def run_loop(stdscr, executor: MultiThreadedExecutor, key_nodes: List[Node]):
    while True:
        try:
            executor.spin_once()
            k = stdscr.getch()
            if k != -1:
                if chr(k) == 'q':
                    return
                else:
                    for key_node in key_nodes:
                        key_node.key_queue.put(k)
        except curses.error as e:
            if str(e) != 'no input':
                stdscr.addstr(0, 0, traceback.format_exc())
```

Finally, copy and paste into `curses_runner.py` this modified version of `run_curses_nodes()`:
```
def run_curses_nodes(stdscr, nodes: List[Node]):
    executor = MultiThreadedExecutor()
    key_nodes = []
    startup(stdscr, executor, nodes, key_nodes)
    run_loop(stdscr, executor, key_nodes)
    shutdown(stdscr, executor, nodes)
```

Answer the following questions:
<!-- Exploration: Analyzing attributes in Python objects -->
* What do you think the `hasattr()` function does?
<!-- Concept formation: Analyzing attributes in Python objects -->
* What is the overall effect of the `if` statement containing `hasattr()`?
<!-- Application: Analyzing attributes in Python objects -->
* What would you need to do when designing a class to ensure it is included in the 
  `key_nodes` list?
* What is the purpose of the `key_nodes` list?
* How is that purpose achieved?

Create a new Python program named `key_timer_demo.py`. Copy and paste the following code into it:
```
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

import curses
from queue import Queue
from curses_runner import run_curses_nodes, CursesNode

class KeyNode(Node):
    def __init__(self, print_topic: str):
        super().__init__(f"KeyNode_{print_topic}")
        self.topic = self.create_publisher(String, print_topic, qos_profile_sensor_data)
        self.create_timer(0.25, self.timer_callback)
        self.key_queue = Queue()

    def timer_callback(self):
        if not self.key_queue.empty():
            msg = self.key_queue.get()
            key = chr(msg)
            output = String()
            output.data = f"Received key {key}"
            self.topic.publish(output)


def main(stdscr):
    rclpy.init()
    topic = f"{sys.argv[1]}_keys"
    curses_node = CursesNode(topic, 2, stdscr)
    key_node = KeyNode(topic)
    run_curses_nodes(stdscr, [curses_node, key_node])
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 key_timer_demo.py robot_name")
    else:
        curses.wrapper(main)
```

Examine the program. Then answer the following questions:
<!-- Application: Queues -->
* How will `key_node` interact with `curses_node`?
* Based on that interaction, how do you expect the program to behave?
* Run the program. Was your previous answer correct? If not, elaborate.

## Remote controlled robot
<!-- Application: -->
Make a copy of `key_timer_demo.py` called `key_motor_demo.py`. Make the following modifications:
* Add an import for `TwistStamped`.
* Write two functions `forward()` and `turn()`. Each should take a motor speed as a 
  parameter and return a `TwistStamped`.
  * Don't worry about setting the time stamp in the function - do that later.
* Pass in and retain as object state the robot's name.
* Add a motor publisher in the constructor.
* In the timer callback:
  * If the key is a `w`, publish a message created by the `forward()` function.
  * If the key is an `a`, turn left by publishing a message created by the `turn()` function.
  * If the key is a `d`, turn right by publishing a message created by the `turn()` function.
  * When publishing any of these messages, be sure to set the time stamp.
* Test out the resulting program.
  * How responsive is the robot to your commands?
* Make the following changes to the program:
  * Stop publishing the keystroke from `KeyNode`.
  * Create a `SensorNode` object (from `sensor_messenger.py`) and add it to the node list.
  * Have the `CursesNode` subscribe to the `SensorNode` topic, as we did in `curses_printer.py`.
  * When processing key inputs in `KeyNode`, empty the queue completely.
* How does the behavior of the program and robot change with these modifications?
* Experiment with the program for a while. Drive the robot forward as much as you can, turning
  whenever necessary to avoid hitting something. Using both your experience experimenting with the program,
  as well as your IR data from the previous module, answer the following questions:
  * Based exclusively on IR values, when should the robot turn left?
  * When should it turn right?
  * When it is it okay for it to drive forward?
  * In answering these questions, assume that the actions you defined for the `w`, `a`, 
    and `d` keys are the only possible actions for the robot to take.

## Autonomous Robot
Make a copy of `key_motor_demo.py` called `motor_sensor_demo.py`. Modify it as follows:
* Rename `KeyNode` to be `AvoidNode`. Make sure to change both the name of the class as
  well as its constructor call.
* Have `AvoidNode` subscribe to the `ir_intensity` topic.
* In the callback method for the `ir_intensity` topic, examine the IR values and implement the policy
  you described at the end of the last section.
* Let the robot run for 60 seconds.
  * How often does it successfully avoid hitting obstacles? 
  * How often, and in what circumstances, does it hit obstacles anyway?
* Based on your observations, devise two variations of your policy. A variation might involve 
  different thresholds for IR values, using different IR sensors in different ways, 
  different motor speeds, and so forth. 
* Run the robot with each variation for 60 seconds from the 
  same starting point. Then answer the following questions for each policy variation:
  * How did it perform in comparison with your original policy?
  * To what aspects of the variation do you attribute the difference in performance (if any)?
