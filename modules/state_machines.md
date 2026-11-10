---
layout: work
type: Module
num: 10
worktitle: State Machines
---

## A Simple State Machine

The diagram below describes a **state machine** for controlling a robot.

<img src="{{site.baseurl}}/assets/images/state1.png" />

1. How would you describe the behavior of a robot that followed the above
diagram?
2. If you were provided the above diagram and told to create a ROS2 node
that corresponded with it, how would you:
* Specify a motor command for `forward`?
* Specify a motor command for `left`?
* Determine if the robot is `in_bounds`?
* Determine if the robot is `out_of_bounds`?
* Structure the resulting node in terms of subscriptions, publications,
and callbacks?

Create a new file called `state1.py` and copy the code below into it:
```
from statemachine import StateChart, State

class BoundedPatrol(StateChart):
    allow_event_without_transition = False

    forward = State(initial=True)
    left = State()
    finished = State(final=True)

    out_of_bounds = (forward.to(left) | left.to(left))
    in_bounds = (forward.to(forward) | left.to(forward))
    quit = (forward.to(finished) | left.to(finished))


if __name__ == '__main__':
    b = BoundedPatrol()
    print(f"{b:md}")

    while b.configuration != {b.finished}:
        cmd = input("'out_of_bounds', 'in_bounds', or 'quit': ")
        b.send(cmd)
        print(b.configuration)
```

1. Run the program. How well does its behavior correspond to the diagram?
2. Delete the `final=True` parameter from the `finished` line. How do you
interpret the message that the program outputs?
3. Restore the `final=True` parameter, but insert a comment character (`#`)
before the line `quit = (forward.to(finished) | left.to(finished))`. How do 
you interpret the message that the program outputs? 
 * Once you have answered this question, remove the comment character, run
   the program again, and make sure it has returned to its prior behavior.
4. Based on these results, how would you define the following terms?
 * `transition`
 * `outgoing transition`
 * `incoming transition`
 * `final state`
 * `unreachable state`
 
The current state configuration is a `set`, as this permits the possibility
of creating state machines that are in more than one state at a time. That
is beyond the scope of what we are doing, so we will modify the `BoundedPatrol` 
class to make clear the current state by adding the following method:
```
    def state_name(self):
        return next(iter(self.configuration)).id
```

This enables us to modify our `while` loop condition as follows:
```
    while b.state_name() != 'finished':
```

Make these two modifications, then test the program and ensure it still 
works.

## A ROS2 State Machine Node

Create a new program called `state_patrol_1.py`, and copy the code below
into it:
```
from state1 import BoundedPatrol
import sys, curses, math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from robot_pose import RobotPose, odom2pose, normalize_angle_symmetric
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import HazardDetectionVector


ANGULAR_VELOCITY = 0.5
LINEAR_VELOCITY = 1.0
DISTANCE_THRESHOLD = 0.5
HEADING_THRESHOLD = 0.25


def add_twist(machine: BoundedPatrol, t: TwistStamped):
    state = machine.state_name()
    if state == 'forward':
        t.twist.linear.x = LINEAR_VELOCITY
    else:
        t.twist.angular.z = ANGULAR_VELOCITY


class BoundedPatrolNode(Node):
    def __init__(self, robot_name: str, distance_threshold: float, heading_threshold: float):
        super().__init__(f"{robot_name}_BoundedPatrolNode")

        self.motors = self.create_publisher(TwistStamped, f"/{robot_name}/cmd_vel_stamped", qos_profile_sensor_data)
        self.create_subscription(Odometry, f"/{robot_name}/odom", self.odom_callback, qos_profile_sensor_data)

        self.machine = BoundedPatrol()

        self.distance_threshold = distance_threshold
        self.heading_threshold = heading_threshold
        self.start = None
        self.msg = None
        self.distance = 0.0
        self.heading = 0.0

    def running(self):
        return self.machine.state_name() != 'finished'

    def odom_callback(self, odom: Odometry):
        pose = odom2pose(odom)
        if self.start is None:
            self.start = pose
        else:
            self.send_odom_transition(pose)
            self.publish_twist()

    def send_odom_transition(self, pose: RobotPose):
        self.distance = pose.distance_to(self.start.x, self.start.y)
        self.heading = pose.turn_to(self.start.x, self.start.y)
        if self.distance > self.distance_threshold and abs(self.heading) > self.heading_threshold:
            self.msg = "out_of_bounds"
        else:
            self.msg = "in_bounds"
        self.machine.send(self.msg)

    def publish_twist(self):
        t = TwistStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = self.get_clock().now().to_msg()
        add_twist(self.machine, t)
        self.motors.publish(t)

    def process_keystroke(self, key: str):
        if key == 'q':
            self.machine.send("quit")

    def display(self, stdscr):
        stdscr.addstr(1, 0, f"state:      {self.machine.state_name()}{' ' * 10}")
        stdscr.addstr(2, 0, f"transition: {self.msg}{' ' * 10}")
        stdscr.addstr(3, 0, f"distance:   {self.distance:.2f}{' ' * 10}")
        stdscr.addstr(4, 0, f"heading:    {self.heading:.2f}{' ' * 10}")
        stdscr.refresh()


def main(stdscr):
    rclpy.init()
    node = BoundedPatrolNode(sys.argv[1], DISTANCE_THRESHOLD, HEADING_THRESHOLD)
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()
    while node.running():
        try:
            k = stdscr.getch()
            if k != -1:
                node.process_keystroke(chr(k))
            node.display(stdscr)
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
        print("Usage: python3 state_patrol_1.py robot_name")
    else:
        curses.wrapper(main)
```

1. Compare the program to the answers you gave earlier about creating a
ROS2 node that conforms to the state machine. How is this implementation
similar to and different from what you envisioned?
2. Run the program. How does the robot's observed behavior conform to the 
state machine we created earlier?

## An upgraded state machine

Consider the following state diagrams:

### State Diagram 1

<img src="{{site.baseurl}}/assets/images/state2a.png" />

### State Diagram 2

<img src="{{site.baseurl}}/assets/images/state2b.png" />

Make a copy of `state1.py` called `state2.py`. Modify the states and 
transitions to conform to the following state diagram:



## States

Create a new file called `state_machine.py`, and copy and paste the following code into it:

```
from typing import Callable, Dict, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data


class StateNode(Node):
    def __init__(self, robot_name: str, input_topic: str, start_state: str, transition_table: Dict[str,Callable[[Tuple[str,str]],str]]):
        super().__init__(f"StateNode_{robot_name}")
        self.state = start_state
        self.last_input = None
        self.last_debug = None
        self.transition_table = transition_table
        self.topic_name = f"{robot_name}_state"
        self.debug_topic = f"{robot_name}_state_debug"
        self.create_subscription(String, input_topic, self.input_callback, qos_profile_sensor_data) 
        self.create_timer(0.25, self.timer_callback)
        self.output = self.create_publisher(String, self.topic_name, qos_profile_sensor_data)
        self.debug = self.create_publisher(String, self.debug_topic, qos_profile_sensor_data)

    def input_callback(self, msg: String):
        msg = eval(msg.data)
        self.last_input = msg['input']
        self.last_debug = msg['debug']

    def timer_callback(self):
        if self.last_input is not None:
            update = self.transition_table[self.state](self.last_input)
            if update is not None:
                self.state = update
            output = String()
            output.data = self.state
            self.output.publish(output)
            output.data = f"""
State: {self.state}{' ' * 10}
Input: {self.last_input}{' ' * 10}
Debug: {self.last_debug}
"""
            self.debug.publish(output)
            self.last_input = None
```

Examine the program. Imagine that `start_state` has the value `"forward"`. Imagine that
`transition_table` has the following value:
```
{'forward': forward_transition,
 're_enter': re_enter_transition,
 'left':    turn_transition,
 'right':   turn_transition}
```

Also imagine the following function definitions:
```
def forward_transition(input: Tuple[str,str]) -> str:
    if input[0] == 'out_of_bounds':
        if input[1] == 'neg_heading':
            return 'left'
        elif input[1] == 'pos_heading':
            return 'right'


def turn_transition(input: Tuple[str,str]) -> str:
    if input[1] == 'aligned':
        return 're_enter'


def re_enter_transition(input: Tuple[str,str]) -> str:
    if input[0] == 'in_bounds':
        return 'forward'
```

<!-- Exploration: How state machines work -->
<!-- Application: Symbolic inputs -->
Imagine as well that it subscribes to the topic published by an `OdometryNode`. 
Answer the following questions:
* What would be an input that would transition the `StateMachine` object to the `left` state?
* What would be an input that would transition the `StateMachine` object in the `left` state
  to the `re_enter` state?
* What would be an input that would transition the `StateMachine` object in the `re_enter` state
  to the `forward` state?
* What would be a sequence of inputs that would transition the `StateMachine` object through
  the following state sequence, starting from `forward`: 
  * `right`, `re_enter`, `forward`, `left`, `re_enter`, `forward`
<!-- Concept invention: Emergent behavior of a state machine -->
* Describe in an abstract way the behavior you would expect of a robot controlled by 
  this `StateMachine` object.

Now create a file called `simple_patrol.py`, and copy and paste the code below:
```
import sys, curses, math
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

from odometry_patrol import OdometryNode
from state_machine import StateNode
from curses_runner import CursesNode, run_curses_nodes


class DriveNode(Node):
    def __init__(self, robot_name: str, state_topic: str):
        super().__init__(f"DriveNode_{robot_name}")
        self.motors = self.create_publisher(TwistStamped, f"{robot_name}/cmd_vel_stamped", qos_profile_sensor_data)
        self.create_subscription(String, state_topic, self.state_callback, qos_profile_sensor_data)

    def state_callback(self, msg: String):
        if msg.data in ('forward', 're_enter'):
            self.publish_forward(0.5)
        elif msg.data == 'left':
            self.publish_turn(1.0)
        elif msg.data == 'right':
            self.publish_turn(-1.0)

    def make_twist(self) -> TwistStamped:
        t = TwistStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = self.get_clock().now().to_msg()
        return t

    def publish_forward(self, speed: float):
        t = self.make_twist()
        t.twist.linear.x = speed
        self.motors.publish(t)

    def publish_turn(self, speed: float):
        t = self.make_twist()
        t.twist.angular.z = speed
        self.motors.publish(t)


def forward_transition(input: Tuple[str,str]) -> str:
    if input[0] == 'out_of_bounds':
        if input[1] == 'neg_heading':
            return 'left'
        elif input[1] == 'pos_heading':
            return 'right'


def turn_transition(input: Tuple[str,str]) -> str:
    if input[1] == 'aligned':
        return 're_enter'


def re_enter_transition(input: Tuple[str,str]) -> str:
    if input[0] == 'in_bounds':
        return 'forward'


def main(stdscr):
    rclpy.init()
    sensor_node = OdometryNode(sys.argv[1], 0.5, math.pi / 32)
    state_node = StateNode(sys.argv[1], sensor_node.topic_name, 'forward', {
        'forward': forward_transition,
        're_enter': re_enter_transition,
        'left': turn_transition,
        'right': turn_transition
    })
    curses_node = CursesNode(state_node.debug_topic, 2, stdscr)
    drive_node = DriveNode(sys.argv[1], state_node.topic_name)
    run_curses_nodes(stdscr, [drive_node, state_node, curses_node, sensor_node])
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 simple_patrol.py robot_name")
    else:
        curses.wrapper(main)
```

<!-- Concept invention: State machine behavior -->
Examine the program, and answer the following questions:
* Based both on the program text above as well as your analysis of `odometry_patrol.py` and
  `state_machine.py`, what do you expect the robot will do when you run `simple_patrol.py`?
* Run the program. Did the robot behave as you expected? Were there any surprises?

## New Topic: Hazards

<!-- Exploration: hazard_detection from the command line -->
Type the command below into the command line:
```
ros2 topic echo /[your robot name]/hazard_detection
```

* Press the bumper directly at the front of the robot. What does it display?
* Press the bumper in different places, throughout its coverage of the front half of the
  robot. What does it display when it is touched in different places?
<!-- Concept invention: which bump sensors does it have? -->
* How many distinct bump sensors does the iRobot Create3 have?
  * What are their names?
<!-- Exploration: Other hazards -->
* Pick up the robot. What messages does it display?
<!-- Concept invention: pick-me-up-sensors -->
* How many distinct cliff-detection sensors does the iRobot Create3 have?
  * What are their names?
* What else can the iRobot Create3 sense to determine that it is not entirely 
  on the ground?
  * What are their names?

## Hazard Detection in Python

<!-- Application: hazard_detection topic -->
Add the following import to `sensor_messenger.py`:
```
from irobot_create_msgs.msg import HazardDetectionVector
```

Then perform the following additional modifications to `sensor_messenger.py`:
* Add a subscription to the `hazard_detection` topic.
* Define the callback function that you name in the subscription.
* The `SensorNode` will report the most recent detected hazard along with 
  the time (in seconds since startup) when it occurred. Add attributes to
  the `SensorNode` class to enable this.
* Add the stored information you identified above to the `String` object
  that the `SensorNode` publishes.
* Drive the robot around for a while, periodically encountering hazards. 
  Ensure that its hazard reporting corresponds to what it encounters.
  
  <!-- Future idea -->
  <!-- Have an exercise where they make the patrol robot stop when it encounters something.
  It doesn't move again until both hazard and IR are free -->


## Hazard-avoiding state machine
<!-- Application: Symbolic inputs and State machines -->

Consider the following desired behavior:
* If nothing exceptional occurs, the robot drives forward.
* If the robot encounters a hazard, the robot turns 90 degrees.
  * If the hazard is to its left, it turns right.
  * If the hazard is to its right, it turns left.
  * If the hazard is in front of it, it turns in the same direction it most recently turned.
* Once the robot completes its 90 degree turn, it drives forward again.

Answer the following questions:
* What would be a set of states that would comprehensively represent the above behavior?
  * **Hint**: There is no need to use more than four states.
* What motor commands would you associate with each state?
* What symbolic inputs would you use to represent the situations that cause these states 
  to change? 
  * **Hint**: There is no need to use more than four distinct possible inputs.
* Create a table with a column for each state and a row for each input symbol. In each table
  entry, write the state that the robot will enter when the input for its row is received 
  while in the state of its column. 
  * **Note**: With no more than four states and four inputs, your table should have 
    no more than 16 entries.
* Create two files: `avoid_hazard_input.py` and `hazard_avoider.py`. Model 
  `avoid_hazard_input.py` on `odometry_patrol.py` and model `hazard_avoider.py` on 
  `simple_patrol.py`. Encode your inputs, state transitions, and motor outputs in
  those two files following the specification you just created.
* Test your program. Continue to modify it until it matches your specification. Overall,
  how did the program following your specification perform?
* What would you say are the benefits and challenges of the state machine approach to 
  programming a robot?
