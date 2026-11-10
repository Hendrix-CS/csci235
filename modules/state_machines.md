---
layout: work
type: Module
num: 10
worktitle: State Machines
---

# A Simple State Machine

The diagram below describes a **state machine** for controlling a robot.

## State Diagram 1

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

Create a new file called `patrol_machine_1.py` and copy the code below into it:
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

# A ROS2 State Machine Node

Create a new program called `patrol_node_1.py`, and copy the code below
into it:
```
from patrol_machine_1 import BoundedPatrol
import sys, curses, math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from robot_pose import RobotPose, odom2pose, normalize_angle_symmetric
from nav_msgs.msg import Odometry


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
        print("Usage: python3 patrol_node_1.py robot_name")
    else:
        curses.wrapper(main)
```

1. Compare the program to the answers you gave earlier about creating a
ROS2 node that conforms to the state machine. How is this implementation
similar to and different from what you envisioned?
2. Run the program. How does the robot's observed behavior conform to the 
state machine we created earlier?

# Upgrading the State Machine

Consider the following state diagrams:

## State Diagram 2A

<img src="{{site.baseurl}}/assets/images/state2a.png" />

## State Diagram 2B

<img src="{{site.baseurl}}/assets/images/state2b.png" />

Answer the following questions:
1. What do State Diagrams 2A and 2B have in common in comparison with State
Diagram 1?
2. How would you concisely describe the behavior specified by State Diagrams
2 and 3?
3. How do State Diagrams 2A and 2B differ?
4. Which of State Diagrams 2A and 2B seems more likely to exhibit the most
effective behavior? Why?
5. Make a copy of `patrol_machine_1.py` called `patrol_machine_2.py`. 
Modify the states and transitions to conform to your preferred diagram 
(2A or 2B). Test `patrol_machine_2.py` and make sure it works as you 
expect. Were there any changes you made that you found surprising or 
noteworthy?
6. Imagine modifying `patrol_node_1.py` to support your preferred diagram
(2A or 2B). What modifications would you need to make?
7. Make a copy of `patrol_node_1.py` called `patrol_node_2.py`. Modify
as necessary to support your preferred diagram. Try it out and observe how
your robot behaves. Did it behave as you expected?  
8. Make a copy of `patrol_machine_2.py` called `patrol_machine_2_other.py`. 
Modify `patrol_machine_2_other.py` to reflect the diagram you preferred 
less. What were the modifications? 
9. How would `patrol_node_2.py` also need to be modified to accommodate
the other diagram? Make a copy of `patrol_node_2.py` called 
`patrol_node_2_other.py` with the necessary modifications.
10. Try out `patrol_node_2_other.py` and observe how your robot behaves. 
Did it behave as you expected?
11. Do you think your choice of preferred diagram was warranted? Why or why 
not?

# Hazard-avoiding State Machine
<!-- Application: Symbolic inputs and State machines -->

Consider the following desired behavior:
* If nothing exceptional occurs, the robot drives forward.
* If the robot encounters a hazard, the robot turns 45 degrees.
  * If the hazard is to its left, it turns right.
  * If the hazard is to its right, it turns left.
  * If the hazard is in front of it, it turns in the same direction it most recently turned.
* Once the robot completes its 45 degree turn, it drives forward again.

Answer the following questions:
1. What would be a set of states that would comprehensively represent the 
  above behavior?
  * **Hint**: There is no need to use more than five states.
2. What motor commands would you associate with each state?
3. What transitions would you use to represent the situations that cause 
  these states to change? 
  * **Hint**: There is no need to use more than five distinct transitions.
4. Create a file `hazard_machine.py` that defines a state machine using
your states and transitions. Test it and modify it until it works as you
expect. 
5. Create a file `hazard_node.py` that defines a ROS2 node that implements
the state machine from `hazard_machine.py`. Test it and observe the robot's
behavior, modifying both programs until the robot behaves as specified 
above. 
6. How challenging was it to create the hazard-avoiding robot using a state
machine? Did you find it preferable to employ a state machine for this task?
7. What would you say are the benefits and challenges of the state machine 
approach to programming a robot?
