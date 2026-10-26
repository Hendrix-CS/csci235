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

Create a ROS2 node that 