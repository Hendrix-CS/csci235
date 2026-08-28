---
layout: work
type: Module
num: 3
worktitle: ROS2 Nodes, Topics, Publications, and Subscriptions
---

## ROS2 on the Unix command line

We will be programming our robots from their attached Raspberry Pi computers.
Each Raspberry Pi runs the GNU/Linux operating system. To work with the
Raspberry Pi, you will need to open a remote shell.

From a Windows machine, to open a shell I recommend using
[MobaXterm](https://mobaxterm.mobatek.net). From a Mac, open a Terminal
window, then use ssh to open the remote shell.

* From the shell on your robot's Raspberry Pi, type `ros2 topic list`
  * If it displays just two lines, type it again until you see a longer list
* Look for a topic ending in `cmd_vel_stamped`. For instance, on a robot 
  named `archangel` the topic would be `/archangel/cmd_vel_stamped`.
<!-- Exploration: cmd_vel_stamped from the command line -->                                                                 
* Type the command below into the command line:
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

## Publishing motor commands in Python

To explore motor programming more systematically, we will write a Python program
to send motor messages. I recommend using the `micro` editor from within
the shell. To create this Python program using `micro`, type
`micro pub_motor.py`. Once the editor opens, copy and paste the program
below:

```
import sys
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

The `rclpy` library (ROS Client Library for Python) enables Python programs to 
interact with ROS2. Examine the program and answer the following questions:
* What do you think a `Node` is in this context?
* What does it mean to "spin" a `Node`?
* What happens in `main()`?
* How often do you think `timer_callback()` is called?
<!-- Exploration: Python TwistStamped -->
* What do you think will happen on each call to `timer_callback()`?
* Run the program. What did it do? Did it meet your expectations?
<!-- Concept invention: direction of rotation from a TwistStamped -->
* Replace `t.twist.angular.z = 1.0` with `t.twist.angular.z = -1.0`.
  * What do you expect will happen when it runs?
  * Run the program. What did it do? Did it meet your expectations?
* Try `2.0` instead of `1.0`. Also try `0.5`. What happens?
* Try replacing `1.0` in the call to `self.create_timer()` with `0.1`. 
  What happens?
<!-- Exploration: Drive forward -->
<!-- Application: cmd_vel from the command line -->
* How might you modify the `TwistStamped` object so that the robot drives forward?
* Perform the modification. Continue to experiment until it works as you expect.
* Modify the program so that the robot drives in a circle of radius 50 
  centimeters, using the command-line `TwistStamped` message you created 
  earlier.

## Viewing messages on the command line

* Run `ros2 topic list` once again on the command line. 
* Look for the `interface_buttons` topic. For instance, on a robot named 
  `archangel` the topic would be `/archangel/interface_buttons`.
* Run the following command in the command line: 
  `ros2 topic echo /[your robot name]/interface_buttons`
  where you replace `[your robot name]` with the name of your robot.
* What is displayed on the command line?
* As it continues running, push the leftmost button and hold it down for a few 
  seconds.  How does the output change?
* Now push down the rightmost button and hold it down for a few seconds. How
  does the output change?

## Subscription to the buttons in Python

Type `micro sub_button.py` to create a new Python file. Then copy and paste the following program:
```
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import InterfaceButtons


class ButtonNode(Node):
    def __init__(self, robot_name: str):
        super().__init__(f"{robot_name}_ButtonNode")
        self.create_subscription(InterfaceButtons, f"{robot_name}/interface_buttons", 
            self.button_callback, qos_profile_sensor_data)

    def button_callback(self, buttons: InterfaceButtons):
        print(buttons)


def main():
    rclpy.init()
    node = ButtonNode(sys.argv[1])
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 sub_button.py robot_name")
    else:
        main()
```

Examine the program and answer the following questions:
* In what ways is this program similar to `pub_motor.py`? 
* In what ways does it differ?
* Based on your answers to the previous two questions, what do you expect
  the program to do when it runs?
* Run the program. What does it do? How does it compare to running the
  `ros2` command to show the button status?
* Modify the `print()` in `button_callback()` method as follows:
  `print(buttons.button_1.is_pressed)`
* Run the program. How does it behave now? 
* Run the ROS2 command from the previous section again, carefully examining
  the output. Based on insight you draw from this, modify the program
  to display the status of both buttons, clearly labeled.
* What can you conclude about how to access data from a subscribed object
  from this exercise?


## Controlling motors with buttons

Create a new Python program called `button_motor.py` along the following lines:
* It should contain a single ROS2 node.
* The node should have a subscription to the buttons and a publisher for 
  the motors.
* When the left button is pushed, the robot should drive forward.
* When the right button is pushed, the robot should rotate left.
* When neither button is pushed, the robot should stand still.

## Controlling motors using curses

In an **event-driven program**, the flow of the program is determined by
**events** that occur outside the program. An **event loop** listens
for events, then **handles** each event as it occurs.

When we were [first introduced](modules/intro1.html) to `curses`, we
implemented the event loop manually. On each iteration of the loop,
we call `stdscr.getch()` to check for **key events**. When they occur,
we then handle them accordingly.

In an `rclpy` program, calling `spin()` starts an event loop in which
**callbacks** are executed when timing and sensor events (like button 
presses) arrive.

To make `rclpy` and `curses` work together, we have to integrate their
event handlers into a single event loop. In the program below, we build
the event loop atop `curses`, invoking `rclpy.spin_once()` to run the 
`rclpy` event loop one time.

Read the code below, then answer the questions that follow.

```
import sys, curses
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from irobot_create_msgs.msg import InterfaceButtons


class KeyDriveNode(Node):
    def __init__(self, robot_name: str):
        super().__init__(f"{robot_name}_DriveNode")
        self.motors = self.create_publisher(TwistStamped, f"{robot_name}/cmd_vel_stamped", qos_profile_sensor_data)
        self.running = True
        self.create_subscription(InterfaceButtons, f"{robot_name}/interface_buttons", self.button_callback, qos_profile_sensor_data)
        self.left_active = False
        self.right_active = False

    def button_callback(self, buttons: InterfaceButtons):
        self.left_active = buttons.button_1.is_pressed
        self.right_active = buttons.button_2.is_pressed

    def process_keystroke(self, k: str):
        t = TwistStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = self.get_clock().now().to_msg()
        if k == 'q':
            self.running = False
        elif k == 'w':
            t.twist.linear.x = 1.0
        elif k == 'a':
            t.twist.angular.z = 1.0
        elif k == 'd':
            t.twist.angular.z = -1.0
        self.motors.publish(t)    


def main(stdscr):
    rclpy.init()
    node = KeyDriveNode(sys.argv[1])
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
            stdscr.addstr(0, 0, f"Left Pressed?  {node.left_active}  ")
            stdscr.addstr(1, 0, f"Right Pressed? {node.right_active}  ")
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
        print("Usage: python3 curses_motor.py robot_name")
    else:
        curses.wrapper(main)
```

* What keystrokes will activate the robot's motors? Describe the process by
  which each keystroke causes a particular motor behavior.
* What is the sequence of events that cause robot button status to be 
  displayed on the screen?
* Compare and contrast how `curses` events and `rclpy` events are handled. In
  your answer, carefully track the control flow that leads to the code that
  handles each type of event.
* Run the program. Drive the robot around a bit. Push the buttons. Did the
  program behave how you expected when you read the code?
* How might you modify this program to have the following features:
  * When the user presses the `u` key, the robot's **forward** speed **increases**.
  * When the user presses the `j` key, the robot's **forward** speed **decreases**.
  * When the user presses the `i` key, the robot's **turning** speed **increases**.
  * When the user presses the `k` key, the robot's **turning** speed **decreases**.
  * The robot's current forward and turning speeds should be displayed via curses.
  * In your answer, discuss both where the key presses will be handled and how 
    the current speeds will be stored and accessed.
* Implement this new feature and test it out. How did it work? Fix any bugs
  you encounter and make sure the program works properly.

## To submit
* `pub_motor.py` modified for the robot to drive in a circle.
* `sub_button.py` modified for the status of each button to be shown and 
   clearly labeled.
* `curses_motor.py` with the enhanced features.
