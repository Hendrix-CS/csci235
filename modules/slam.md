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

Topics where this node publishes useful information include:
* `bitslam_pose`
  * Current estimated pose for the robot, correcting the `odom` topic 
    readings with the map
* `bitslam_status` 
  * `Mapping` if the SLAM algorithm is in the process of creating a map
  * `Finished` if the SLAM algorithm has completed a map
  * `Failed` if the SLAM algorithm is unable to create a map
* `bitslam_occupancy_grid`
  * Read the documentation for the ROS2 [Occupancy Grid](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/OccupancyGrid.html)
  * Also read the documentation for [MapMetaData](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/MapMetaData.html)
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

To convert a Header to JSON, we can use this function:
```
def header_json(header: Header) -> dict:
    return {"stamp": {"sec": header.stamp.sec, "nanosec": header.stamp.nanosec}, "frame_id": header.frame_id}
```

To build a `Pose` message from an `Odometry` message:
* Convert the `Odometry` message to a [`RobotPose`](modules/intro1.html) 
object using [`odom2pose`](modules/nodes2.html)
* Imagine a `RobotPose` object at `(1.0, 2.0)` with heading `1.57`. Its JSON
representation will be:
```
{"header": header_json(odom.header), "data": "(pose,1.0,2.0,1.57)"}
```
* Based on the above structure, how would you encode a `RobotPose` object 
named `pose`? *Hint: I recommend using an f-string.*

To build a `Collision` message from a `HazardDetection` message:
* The sensor headings are:
  * Front bump sensor: 0.0
  * Front left bump sensor: &pi;/4 
  * Front right bump sensor: -&pi;/4
  * Left bump sensor: &pi;/2
  * Right bump sensor: -&pi;/2
  * The left and right bump sensor headings are &pi;/2 and -&pi;/2
* Other information includes:
  * Distance to the collision: we will use `0.2032`, which is the robot's radius in meters
  * Distance noise: we will estimate the noise for the collision distance as +/- 1 cm
  * Heading noise: we will estimate the noise for the collision heading as +/- `0.39269908169872414` radians
* The JSON object for the front bump sensor is:
```
{"header": header_json(h.header), "data": f"(collision,0.2032,0.0,0.01,0.39269908169872414)"}
```
* What would the JSON objects look like for the other four bump sensors?
* When you are ready to encode a JSON object as a string, use `json.dumps()` and pass
in the JSON object.

## Exploring

Make a copy of your favorite obstacle-avoiding program (e.g. `bump_turn_45.py`) as 
`slam_bump_turn.py`. Augment it as follows:
* Add the import `from std_msgs.msg import String` in order to be able to
  publish `String` messages.
* Add a publisher for `bitslam_obstacles` that publishes `String` messages.
* Make sure it subscribes to `odom` and `hazard_detection` at a minimum.
  * In the callback for `odom`, publish a `pose` message, as defined 
    above, to `bitslam_obstacles`
  * In the callback for `hazard_detection`, publish a `collision` message, 
    as defined above, to `bitslam_obstacles`
* As it publishes each message to `bitslam_obstacles`, it will also append 
  the message to a file. To ensure a unique filename for each run, use 
  `import time` to use the `time` module and then use `time.time()` as part 
  of the filename. (The function `time.time()` returns the number of seconds 
  since January 1, 1970, and is useful for creating unique identifiers.) 
  Remember that to append to a file in Python, you can use the following syntax:
```
with open(filename, 'a') as f:
    f.write("An additional line.\n")
```
* Reset the odometry: `ros2 service call /robot_name/reset_pose irobot_create_msgs/srv/ResetPose`
* Run your program for a while. 
* Echo the `bitslam_obstacles` topic to see what it is publishing, and make sure 
  those values make sense.
  
### Incorporating IR sensors (optional)
* If you would also like to incorporate obstacles observed by the IR sensors,
  you can use [this diagram](https://iroboteducation.github.io/create3_docs/hw/mechanical/#ir-proximity-sensors)
  to obtain their headings.

## Publishing the observations

Create a new file called `slam_transcript_node.py`. This node will read 
the transcript of messages saved in the previous step and publish them to
`bitslam_obstacles`, thus enabling us to simulate the robot's exploration
without actively driving the robot. Below is a skeleton of the implementation:

```
import sys, curses, traceback
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

Run this program and make sure it is publishing the messages correctly. Make
sure it reaches the end of your file.
    
## Visualizing the map

Write another Python program called `curses_map.py`. This program will
subscribe to `bitslam_occupancy_grid` and display the occupancy grid on
the screen using `curses`. Carefully study 
[the OccupancyGrid data type](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/OccupancyGrid.html) 
as well as [MapMetaData](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/MapMetaData.html).

* How is a map represented as an occupancy grid?
* To visualize the map, we need to determine which occupancy grid cell to
  use for each curses cell. How should we determine this?
  
Here is a skeleton for `curses_map.py`:
```
import sys, curses, traceback
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData


class ShowGridNode(Node):
    def __init__(self, robot_name: str, stdscr):
        super().__init__(f"{robot_name}_ShowGridNode")
        self.running = True
        self.stdscr = stdscr
        self.create_subscription(OccupancyGrid, f"/{robot_name}/bitslam_occupancy_grid", self.grid_callback, qos_profile_sensor_data)

    def grid_callback(self, grid: OccupancyGrid):
        # YOUR CODE HERE
        #
        # Use your answers above to write this method

    def handle_key(self, k: str):
        if k == 'q':
            self.running = False


def main(stdscr):
    rclpy.init()
    node = ShowGridNode(sys.argv[1], stdscr)
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
        print("Usage: python3 curses_map.py robot_name")
    else:
        curses.wrapper(main)
```
  
To test this out, you need to do the following:
* Reset the odometry: `ros2 service call /robot_name/reset_pose irobot_create_msgs/srv/ResetPose`
* Run `bit_slam_node`: `bit_slam_node --robot=robot_name`
* Run `curses_map.py`
* Run `slam_transcript_node.py`


## Final testing

To see the robot mapping live:
* Reset the odometry: `ros2 service call /robot_name/reset_pose irobot_create_msgs/srv/ResetPose`
* Run `bit_slam_node`: `bit_slam_node --robot=robot_name`
* Run `curses_map.py`
* Run `slam_bump_turn.py` 

## Final thoughts
* In your journal, write down three idea about how you might use these maps in a larger project.