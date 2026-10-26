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
  * `on_path` if the robot's current estimated pose has it aiming for an
    unvisited location
  * `off_path` if the robot's current estimated pose has it aiming for an
    obstacle somewhere in the distance
* `bitslam_status` 
  * `Mapping` if the SLAM algorithm is in the process of creating a map
  * `Finished` if the SLAM algorithm has completed a map
  * `Failed` if the SLAM algorithm is unable to create a map
* `bitslam_occupancy_grid`
  * Read the documentation for the ROS2 [Occupancy Grid]
    (https://docs.ros.org/en/jazzy/p/nav_msgs/msg/OccupancyGrid.html)
  * Also read the documentation for [MapMetaData]
    (https://docs.ros.org/en/jazzy/p/nav_msgs/msg/MapMetaData.html)

## An obstacle avoider

Examine `bump_turn_45.py` from [Module 4](modules/nodes2.html). Make a copy
called `bump_turn_slam.py` with the following modifications:
* Each time it receives an `odom` message, it should publish a JSON message
  to `bitslam_obstacles` in the following format:
```

```
* Each time it receives a hazard message, it should publish a JSON message
  to `bitslam_obstacles` in the following format:
```
```
* Whenever it publishes either message on `bitslam_obstacles`, save the 
  message in a Python list.
* When it is turning to avoid an obstacle, instead of turning 45 degrees it
  should turn until it receives an `on_path` message from `bitslam_on_path`.
* When the user hits the `q` key to exit:
  * It should publish a suitable message on `bitslam_save`
  * It should write a transcript file where each published JSON message is
    on a separate line of the file