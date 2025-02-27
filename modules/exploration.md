---
layout: work
type: Module
num: 5
worktitle: Exploration
---

## Testing mapper_node

Before starting, check with Dr. Ferrer to make sure `mapper_node` is installed
on your robot. Then open two shells. In one shell, type the following on the command line, 
substituting the name of your robot for `robot_name`:

```
mapper_node robot_name
```

Then, in the other shell, type the following:
```
ros2 topic list
```

Towards the end of the list, you should see something like this:
```
robot_name_trajectory_map
```

Then, type this, using the exact string it printed above for your `trajectory_map`:
```
ros2 topic echo robot_name_trajectory_map
```

You should see something like this:
```
FILL IN HERE
```

## Obstacle-avoiding robot

Create a new folder called `module5`. Copy `odometry_math.py` from `module3` and 
`curses_runner.py` from `module2` into it.

Let's examine an obstacle-avoiding robot program that puts together several different
ideas we have explored recently. First, examine the input-encoding node. Create a file
called `avoid_input.py`, and copy and paste the code below into it.

```
from typing import Any
import math

from odometry_math import find_roll_pitch_yaw, find_angle_diff
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.msg import IrIntensityVector

def publish_string(publisher: Publisher, data: Any):
    output = String()
    output.data = f"{data}"
    publisher.publish(output)


class AvoidInputNode(Node):
    def __init__(self, robot_name: str, ir_turn: int):
        super().__init__(f'AvoidInputNode_{robot_name}')
        self.create_subscription(IrIntensityVector, f"{robot_name}/ir_intensity", self.ir_callback, qos_profile_sensor_data)
        self.create_subscription(HazardDetectionVector, f"{robot_name}/hazard_detection", self.hazard_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, f"{robot_name}/odom", self.odom_callback, qos_profile_sensor_data)
        self.output_topic = f'{robot_name}_ir_blocked'
        self.output = self.create_publisher(String, self.output_topic, qos_profile_sensor_data)
        self.ir_turn = ir_turn
        self.left_target = None
        self.right_target = None

    def avoiding(self) -> bool:
        return self.avoiding_left() or self.avoiding_right()

    def avoiding_left(self) -> bool:
        return self.left_target is not None
    
    def avoiding_right(self) -> bool:
        return self.right_target is not None
    
    def avoid_left(self):
        self.left_target = self.yaw + math.pi/2     

    def avoid_right(self):
        self.right_target = self.yaw - math.pi/2
    
    def avoid_no_longer(self):
        self.left_target = self.right_target = None

    def ir_callback(self, msg: IrIntensityVector):
        for reading in msg.readings:
            if reading.value >= self.ir_turn:
                if 'left' in reading.header.frame_id:
                    self.avoid_right()
                if 'right' in reading.header.frame_id:
                    self.avoid_left()

    def hazard_callback(self, msg: HazardDetectionVector):
        for d in msg.detections:
            name = d.header.frame_id
            if 'left' in name or 'front' in name:
                self.avoid_right()
            if 'right' in name or 'front' in name:
                self.avoid_left()               

    def odom_callback(self, msg: Odometry):
        r, p, self.yaw = find_roll_pitch_yaw(msg.pose.pose.orientation)
        left_reached = self.avoiding_left() and find_angle_diff(self.yaw, self.left_target) > 0
        right_reached = self.avoiding_right() and find_angle_diff(self.yaw, self.right_target) < 0

        if left_reached or right_reached:
            self.avoid_no_longer()
        
        msg = {'left_pending': self.avoiding_left(), 'right_pending': self.avoiding_right()}
        publish_string(self.output, msg)
```

1. What will an `AvoidInputNode` object do when spun?
2. Based on your experience with the IR sensors, what value would you suggest for `ir_turn`?

Copy the following code into a new file, `ir_fuzzy_avoider.py`:

```
import sys, curses
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

from avoid_input import AvoidInputNode, publish_string
from curses_runner import CursesNode, run_curses_nodes


class DriveNode(Node):
    def __init__(self, robot_name: str, input_topic: str, x_limit: float, z_limit: float):
        super().__init__(f"DriveNode_{robot_name}")
        self.avoid_topic = f"{robot_name}_map_avoiding"
        self.avoid_publisher = self.create_publisher(String, self.avoid_topic, qos_profile_sensor_data)
        self.x_limit = x_limit
        self.z_limit = z_limit
        self.motors = self.create_publisher(TwistStamped, f"{robot_name}/cmd_vel_stamped", qos_profile_sensor_data)
        self.create_subscription(String, input_topic, self.input_callback, qos_profile_sensor_data)
        self.last_turn_left = False

    def input_callback(self, msg: String):
        t = self.make_twist()
        avoid_msg = "clear"
        sensor_values = eval(msg.data)
        if all([not v for k, v in sensor_values]):
            t.twist.linear.x = self.x_limit
        else:
            turn_limit = self.z_limit * (-1.0 if sensor_values['left_pending'] or self.last_turn_left else 1.0)
            self.last_turn_left = turn_limit > 0
            t.twist.angular.z = turn_limit
            avoid_msg = "avoid"
        publish_string(self.avoid_publisher, avoid_msg)
        self.motors.publish(t)
            
    def make_twist(self) -> TwistStamped:
        t = TwistStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = self.get_clock().now().to_msg()
        return t


def main(stdscr):
    cmd = parse_cmd_line_values()
    rclpy.init()
    sensor_node = AvoidInputNode(sys.argv[1], cmd['ir_turn'])
    curses_node = CursesNode(sensor_node.output_topic, 2, stdscr)
    drive_node = DriveNode(sys.argv[1], sensor_node.output_topic, cmd['x_limit'], cmd['z_limit'])
    run_curses_nodes(stdscr, [drive_node, curses_node, sensor_node])
    rclpy.shutdown()


def parse_cmd_line_values() -> Dict[str,float]:
    parsed = {}
    for arg in sys.argv:
        if '=' in arg:
            parts = arg.split('=')
            parsed[parts[0]] = float(parts[1])
    return parsed


if __name__ == '__main__':
    if len(sys.argv) < 5:
        print("Usage: python3 avoid_drive.py robot_name ir_turn=value x_limit=value z_limit=value")
    else:
        curses.wrapper(main)
```

1. What will a `DriveNode` object do when spun?
2. Based on your experience with the motors, what values would you suggest for `x_limit` and `z_limit`?
3. Run the program. How does it perform as an obstacle avoider? Experiment with its parameters
until you are reasonably satisfied with its performance.

