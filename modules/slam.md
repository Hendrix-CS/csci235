---
layout: work
type: Module
num: 9
worktitle: SLAM
---

# SLAM

SLAM (**S**imultaneous **L**ocalization **A**nd **M**apping) is a large
family of techniques for simultaneously determing a robot's position and
a map of its environment. In this module, we will use the BitSLAM ROS2 node
to supply our robots with both a map and a corrected pose based on the map.

The node subscribes to a topic, `bitslam_obstacles`, that contains 
information about the robot's poses and the obstacles it encounters. The 
topic is posed in the robot's namespace; for example, if the robot's name 
is `elrond`, the topic would be `/elrond/bitslam_obstacles`. 

The node publishes useful information on the following topics:
* `bitslam_pose`
  * Current estimated pose for the robot, correcting the `odom` topic 
    readings with the map
* `bitslam_on_path` 
  * The `bitslam_on_path` topic publishes the following messages:
    * `on_path: 1.1 meters`
      * Distance to unvisited space in line with the robot's heading - it may be 
        any value, not just 1.1 meters
    * `obstacle: 1.1 meters`
      * Distance to an obstacle from the map in line with the robot's heading - 
        it may be any value, not just 1.1 meters
* `bitslam_status` 
  * `Mapping` if the SLAM algorithm is in the process of creating a map
  * `Finished` if the SLAM algorithm has completed a map
  * `Failed` if the SLAM algorithm is unable to create a map
* `bitslam_occupancy_grid`
  * Read the documentation for the ROS2 [Occupancy Grid]
    (https://docs.ros.org/en/jazzy/p/nav_msgs/msg/OccupancyGrid.html)
  * Also read the documentation for [MapMetaData]
    (https://docs.ros.org/en/jazzy/p/nav_msgs/msg/MapMetaData.html)
* **Note**: All of the `bitslam` topic messages are ROS2 `String` objects,
  except for `bitslam_occupancy_grid`, which is a ROS2 `OccupancyGrid` object.

## JSON

JSON (**J**ava**S**cript **O**bject **N**otation) is a standardized format
for enconding objects as strings. It is a useful format for sending ROS2
messages in a language-independent manner. We will be using JSON objects to
communicate on the `bitslam_obstacles` topic. 

In Python, we can build JSON objects as dictionaries. We can then `import json`
and use `json.dumps()` to encode a JSON object as a string. All of these
objects have two top-level fields: `header` and `data`. The `header` field
will contain a copy of the `header` of the source message. The `data` field
will contain the main JSON object.

To build a `Pose` message from an `Odometry` message:
* Convert the `Odometry` message to a [`RobotPose`](modules/intro1.html) 
object using [`odom2pose`](modules/nodes2.html)
* Imagine a `RobotPose` object at `(1.0, 2.0)` with heading `1.57`. Its JSON
representation will be:
```
{"Pose":{"pos":{"coords":[1.0,2.0]},"theta":1.57}}
```
* Based on the above structure, how would you encode a `RobotPose` object 
named `pose`?
* If the `Odometry` object is named `odom`, the full JSON object would look
like this: 
```
{"header": odom.header, "data": {"Pose":{"pos}":{"coords":[1.0,2.0]},"theta":1.57}}}
```
* What will the full JSON object look like for a `RobotPose` object named
`pose`?

To build a `Collision` message from a `HazardDetection` message:
* The sensor headings are:
  * Front bump sensor: 0.0
  * Front left bump sensor: &pi;/4 
  * Front right bump sensor: -&pi;/4
  * Left bump sensor: &pi;/2
  * Right bump sensor: -&pi;/2
  * The left and right bump sensor headings are &pi;/2 and -&pi;/2
* The JSON object for the front bump sensor is:
```
{"Collision":{"distance":0.2032,"heading":0.0,"noise":{"stdev_distance":0.01,"stdev_heading":0.39269908169872414}}}
```
* What would the JSON objects look like for the other four bump sensors?

## Exploring

Here is a skeleton of a program called `slam_bump_turn.py`:
```
import sys, curses, math, time, json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from irobot_create_msgs.msg import HazardDetectionVector
from robot_pose import RobotPose, odom2pose


class ExplorerNode(Node):
    def __init__(self, robot_name: str, stdscr):
        super().__init__(f"{robot_name}_ExplorerNode")
        self.running = True
        self.paused = False

        # YOUR CODE HERE
        #
        # Create publishers for:
        #   cmd_vel_stamped
        #   bitslam_obstacles
        #
        # Subscribe to:
        #   odom
        #   hazard_detection
        #   bitslam_on_path

    def odom_callback(self, odom: Odometry):
        # YOUR CODE HERE
        #
        # Publish a Pose message on bitslam_obstacles based on odom
        # If the robot has not encountered a hazard, go straight
        # Otherwise, rotate right

    def hazard_callback(self, hazards: HazardDetectionVector):
        # YOUR CODE HERE
        #
        # For each detected hazard
        #   If it is a bump
        #     Publish a Collision message on bitslam_obstacles
        #     Record that the robot needs to turn

    def on_path_callback(self, on_path: String):
        # YOUR CODE HERE
        # 
        # If the robot is turning, and is on a path to unvisited space,
        # mark the robot as not turning anymore.        

    def handle_key(self, k: str):
        if k == 'q':
            self.running = False
        if k == 'p':
            self.paused = not self.paused


def main(stdscr):
    rclpy.init()
    node = ExplorerNode(sys.argv[1], stdscr)
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()
    while node.running:
        try:
            k = stdscr.getch()
            if k != -1:
                k = chr(k)
                node.handle_key(k)
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
        print("Usage: python3 slam_bump_turn.py robot_name")
    else:
        curses.wrapper(main)
```

To run this program, you first need to run the BitSLAM node. Run it as
follows:
```
bit_slam_node --robot=robot_name
```

Once it is running, start up `slam_bump_turn.py`. Observe it running for
a while, and answer the following questions:
* How thoroughly does it seem to be exploring the room in which it is
  running? 
  
## Saving the observations

Modify `slam_bump_turn.py` so that, as it publishes each message to 
`bitslam_obstacles`, it will also append the message to a file. To ensure
a unique filename for each run, use `time.time()` as part of the filename.
(The function `time.time()` returns the number of seconds since January 1, 
1970, and is useful for creating unique identifiers.) Remember that to 
append to a file in Python, you can use the following syntax:
```
with open(filename, 'a') as f:
    f.write("An additional line.\n")
```

## Publishing the observations

Create a new file called `slam_transcript_node.py`. This node will read 
the transcript of messages saved in the previous step and publish them to
`bitslam_obstacles`, thus enabling us to simulate the robot's exploration
without actively driving the robot. Below is a skeleton of the implementation:

```
import sys, curses
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class TranscriptNode(Node):
    def __init__(self, robot_name: str, transcript_filename: str, stdscr):
        super().__init__(f"{robot_name}_TranscriptNode")
        self.running = True
        # YOUR CODE HERE
        #
        # Create a publisher for bitslam_obstacles
        # Create a timer 
        # Create a list of messages to publish
        # Open the transcript file and load all the messages into that list.
        # Create a means of tracking where we are in the list

    def timer_callback(self):
        # YOUR CODE HERE
        #
        # Identify and publish the next message.
        # Also display the message on the screen using curses.
        # If we are out of messages, end the program.

    def handle_key(self, k: str):
        if k == 'q':
            self.running = False


def main(stdscr):
    rclpy.init()
    node = TranscriptNode(sys.argv[1], sys.argv[2], stdscr)
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()
    while node.running:
        try:
            k = stdscr.getch()
            if k != -1:
                k = chr(k)
                node.handle_key(k)
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
    if len(sys.argv) < 3:
        print("Usage: python3 slam_transcript_node.py robot_name transcript_filename")
    else:
        curses.wrapper(main)
```
    
## Visualizing the map

Write another Python program called `curses_map.py`. This program will
subscribe to `bitslam_occupancy_grid` and display the occupancy grid on
the screen using `curses`. Carefully study [the OccupancyGrid data type]
(https://docs.ros.org/en/jazzy/p/nav_msgs/msg/OccupancyGrid.html) as well as
[MapMetaData](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/MapMetaData.html).

* How is a map represented as an occupancy grid?
* What would need to be done to represent an occupancy grid visually in
  `curses` if the map size fits completely within the `curses` window?
  * What if the map size does not fit within the `curses` window?

## Testing the visualization

* Run `slam_transcript_node.py`, `bit_slam_node`, and `curses_map.py` in 
tandem. How effective is the map visualization?
* Now run `slam_bump_turn.py`, `bit_slam_node`, and `curses_map.py` in tandem.
How effectively are you able to visualize the incremental construction of the
map?