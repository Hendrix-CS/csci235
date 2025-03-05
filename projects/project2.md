---
layout: work
type: Project
num: 2
worktitle: Navigation
---

For this [project]({{site.baseurl}}/index.html#projects), you may program an iRobot Create3 robot using ROS2 nodes 
to do anything you like, subject to the following constraints:
* In an initial mapping phase, the robot must build a map of the area to be navigated, using any of the 
[mapmaking]({{site.baseurl}}/modules/exploration.html#avoid_drive_map)
[programs]({{site.baseurl}}/modules/planning.html#frontier) we have written in this unit.
* In the [navigation]({{site.baseurl}}/modules/planning.html#navigation) phase, the robot will travel between 
different locations in the mapped area. The reasons for the robot to travel between locations are up to you. 
Here are some types of reasons:
  * Human input:
    * Pushing one of the interface buttons on the robot.
    * Commands sent via `curses` keystrokes.
  * Environmental input:
    * Encountering an object that is not on the map.
* The robot's actions during the navigation phase should not be arbitrary - they should be in the context of 
  completing one or more useful tasks.