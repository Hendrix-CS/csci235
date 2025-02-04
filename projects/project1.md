---
layout: work
type: Project
num: 1
worktitle: A ROS2 Controller
---

For this [project]({{site.baseurl}}/index.html#projects), you may program an iRobot Create3 robot to do anything
you like, subject to the following constraints:
* The program should use four ROS2 nodes, along the lines of the [final part of Module 3]({{site.baseurl}}/modules/state_machines.html#hazard-avoiding-state-machine):
  * A node that encodes inputs into symbolic values and publishes them.
  * A [`StateNode`]({{site.baseurl}}/modules/state_machines.html#states).
  * A node that publishes motor commands corresponding to states.
  * A ['CursesNode']({{site.baseurl}}/modules/publications.html#console-ui-for-ros2) to display useful information.
    * It may also handle keyboard commands.
* The robot must have at least four distinct states and four distinct inputs. It may have more.
* The robot's behavior must be influenced in some way by sensor values coming from at least three topics.
  * In Modules 1-3 we explored the [`ir_intensity`]({{site.baseurl}}/modules/nodes.html#first-topic-ir-intensity), 
  [`battery_state`]({{site.baseurl}}/modules/nodes.html#second-topic-battery-state)
  [`odom`]({{site.baseurl}}/modules/state_machines.html#new-topic-odometry), and 
  [`hazard_detection`]({{site.baseurl}}/modules/state_machines.html#new-topic-hazards) topics.
  * Other topics that might be interesting to employ in this project include:
    * `interface_buttons`
    * `dock_status`
    * `ir_opcode`
    