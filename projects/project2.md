---
layout: work
type: Project
num: 2
worktitle: Navigation
---

For this [project]({{site.baseurl}}/index.html#projects), you may program an 
iRobot Create3 robot using ROS2 nodes to do anything you like, subject to the 
following constraints:
* The robot's task must lend itself naturally to using one or more of the key
  ideas from this unit:
  * Q-Learning
  * HTN Planning
  * SLAM
* It follows from this that the robot's task must involve navigating from one
  location to another, making use of non-local state to inform the navigational
  decision.  
* The reasons for the robot to travel between locations are up to you. Here are 
  some types of reasons:
  * Human input:
    * Pushing one of the interface buttons on the robot.
    * Commands sent via `curses` keystrokes.
  * Environmental input:
    * Encountering an object that is not on the map.
* The robot's actions during the navigation phase should not be arbitrary - they should be in the context of 
  completing one or more useful tasks.
