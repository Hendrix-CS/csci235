---
layout: work
type: Module
num: 8
worktitle: Image Recognition
---

## Building an image database

Run the `photographer` program as follows:

```
photographer proj1 clear blocked
```

Position the robot in four locations that are clear of obstacles, and four
locations that are filled with obstacles. Take a picture with the appropriate
label at each location. (To take a picture, type the Enter key.)

To test how well it classifies, open two shells. In one shell, start the node:

```
knn_photo_node robot_name proj1
```

In another shell, monitor the node's output:

```
ros2 topic echo /robot_name_image_label
```

In both cases, replace `robot_name` with the name of your robot.

Test the classifier. How does it perform? 

You may find that it doesn't classify images quite the way you had in mind.
Continue to add training samples for underrepresented labels until you find
its performance reasonably satisfactory.

## Rewarding images

Copy all of the files from Module 7 except `qrunner.py`, which you will replace
with the following:

```
from grid import GridConverter, add_squares
from qlearning import QTable, QParameters
from goal_fuzzy_input import FuzzyGoalNode
from goal_fuzzy_navigator import FuzzyDriveNode
from curses_runner import CursesNode, run_curses_nodes

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Point
from irobot_create_msgs.msg import InterfaceButtons

from typing import Tuple
import random
import sys
import curses
import subprocess, atexit, time

ACTIONS = [(0, -1), (0, 1), (1, 0), (-1, 0)]

rewards = {
    'proj1/clear': 1.0,
    'proj1/blocked': 1.0
}

class QRunningNode(Node):
    def __init__(self, robot_name: str, params: QParameters, conv: GridConverter):
        super().__init__(f"{robot_name}_q_runner")
        self.conv = conv
        params.set_num_states_actions(conv.num_states(), len(ACTIONS))
        forbidden = []
        for state in range(params.num_states):
            col, row = conv.state_to_grid(state)
            for action in range(params.num_actions):
                act_col = col + ACTIONS[action][0]
                act_row = row + ACTIONS[action][1]
                if not conv.square_within_grid((act_col, act_row)):
                    forbidden.append((state, action))
        params.forbid_state_actions(forbidden)
        self.q_table = QTable(params)

        self.position_topic_name = f"{robot_name}_q_goal"
        self.position_topic = self.create_publisher(String, self.position_topic_name, qos_profile_sensor_data)
        self.info_topic_name = f"{robot_name}_q_info"
        self.info_topic = self.create_publisher(String, self.info_topic_name, qos_profile_sensor_data)
        self.create_subscription(Odometry, f"/{robot_name}/odom", self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(String, f"/{robot_name}_image_label", self.image_label_callback, qos_profile_sensor_data)

        self.position = Point()
        square = self.conv.odom_to_grid(Point())
        self.current_state = self.conv.grid_to_state(square)
        self.action_message = self.make_action_message(square, random.choice(ACTIONS))

    def make_action_message(self, sq1: Tuple[int, int], sq2: Tuple[int, int]) -> str:
        updated_square = add_squares(sq1, sq2)
        odom = self.conv.grid_to_odom(updated_square)
        return f"({odom.x}, {odom.y})"
    
    def image_label_callback(self, msg: String):
        if self.action_message == "None":
            reward = rewards[msg.data]
            action = self.q_table.sense_act_learn(self.current_state, reward)
            self.action_message = self.make_action_message(self.conv.state_to_grid(self.current_state), ACTIONS[action])
            out = String()
            out.data = f"""After {self.q_table.total_updates} updates:
position: ({self.position.x:.2f}, {self.position.y:.2f}) {self.conv.odom_to_grid(self.position)}
state: {self.current_state}
reward: {reward:.2f}
action: {action} ({ACTIONS[action]}) {self.action_message}
"""
            for state in range(self.q_table.num_states()):
                sq = self.conv.state_to_grid(state)
                out.data += f"{state} {sq}:"
                for action in range(self.q_table.num_actions()):
                    aq = (sq[0] + ACTIONS[action][0], sq[1] + ACTIONS[action][1])
                    if not self.conv.square_within_grid(aq):
                        aq = "Undef "
                    out.data += f" act:{action} {aq} ({self.q_table.q[state][action]:6.2f} {self.q_table.visits[state][action]:3d})"
                out.data += "\n"
            self.info_topic.publish(out)

    def odom_callback(self, msg: Odometry):
        self.position = msg.pose.pose.position
        state = self.conv.grid_to_state(self.conv.odom_to_grid(self.position))
        if state != self.current_state:
            self.action_message = "None"
            self.current_state = state
        out = String()
        out.data = self.action_message 
        self.position_topic.publish(out)

        
def main(stdscr):
    rclpy.init()
    q_node = QRunningNode(sys.argv[1], QParameters(), GridConverter(width=1.5, height=1.5, cell_size=0.5))
    sensor_node = FuzzyGoalNode(sys.argv[1], q_node.position_topic_name)
    curses_node = CursesNode(q_node.info_topic_name, 2, stdscr)
    drive_node = FuzzyDriveNode(sys.argv[1], sensor_node.output_topic)
    run_curses_nodes(stdscr, [q_node, drive_node, curses_node, sensor_node])
    rclpy.shutdown()
            

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 qrunner.py robot_name project_name")
    else:
        print(f"Odometry reset:\nros2 service call /{sys.argv[1]}/reset_pose irobot_create_msgs/srv/ResetPose\n")  
        input("Type enter once odometry is reset")
        process = subprocess.Popen(['/home/robotics/bin/knn_photo_node', sys.argv[1], sys.argv[2]])
        atexit.register(lambda: process.terminate())
        time.sleep(1)
        input("Type enter for robot to start")        
        curses.wrapper(main)
```

Try it out for a while. How does the robot perform at heading towards cells with
"good" images and avoiding cells with "bad" images? Construct some experiments
(as you did in Module 7) to assess its performance.

## Further application

Devise a scenario involving **three** categories of images. Build a suitable image
database and test out the q-learner. Modify `qrunner.py` to give appropriate rewards
to the appropriate labels. Construct some experiments
(as you did in Module 7) to assess its performance.

<!-- Concept 

* Students are given a program that takes pictures and annotates them
with labels. We can also view the pictures.
* Students specify rewards associated with each picture.
* Then we run Q-Learning
  * At each time step, we ask the image database for our current reward.
  * We then observe the emergent behavior.
  
TODO list:
* Write a program to take the pictures.
  * Start program with a list of labels.
  * During program, current label is highlighted.
  * Take a picture by pressing a key.
* Write a program to view the pictures.
  * Pick a label.
  * Then you can scan through all the pictures with that label.
  * You can hit a key to change a picture's label.
* Write a ROS2 node that monitors incoming images, publishing the 
  best-matching label at each image acquisition.
* Put together the q-learner.
-->

