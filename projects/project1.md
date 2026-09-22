---
layout: work
type: Project
num: 1
worktitle: A ROS2 Controller
---

For this [project]({{site.baseurl}}/index.html#projects), you may program an iRobot Create3 robot to do anything
you like, subject to the following constraints:
* The program logic should be encapsulated in a Python ROS2 node.
* The ROS2 node should maintain some persistent state information that visibly influences
  the robot's behavior.
* The robot's behavior must be visibly influenced by sensor values coming from at least three of the following topics:
  * `ir_intensity`
  * `odom`
  * `hazard_detection`
  * `interface_buttons`
  * `groundline`
* The ROS2 node should display pertinent state and sensor information using `curses`.
* The robot's task should be creative or interesting in some way.
 