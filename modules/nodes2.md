---
layout: work
type: Module
num: 4
worktitle: iRobot Create3 Sensors
---

## Odometry

Open two shells on your robot. Type the command below into the first command line:
```
ros2 topic echo /[your robot name]/odom
```

Type this command into the second command line. Make sure your robot has some 
room to move before you run this command:
```
ros2 topic pub -r 5 /[your robot name]/cmd_vel_stamped geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: {linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

Let it run for a few seconds. Then stop the `odom` command, and after that stop the `cmd_vel_stamped` command.
Then answer the following questions:
* What information is published by the `odom` topic?
* How does that information change as the robot drives forward?
* Now write a ROS2 command for the robot to spin in place. Repeat the above steps.
  How does the `odom` information change as the robot spins?
<!-- Concept invention: Odometry -->
* What fields from the `odom` message are most relevant to determining the robot's
  position and orientation?
<!-- odometry resets -->

Make sure you have a copy of `robot_pose.py` in the current directory. Then
add the following import at the top:
```
from nav_msgs.msg import Odometry
```

Then add the following function:
```
def odom2pose(odom: Odometry) -> RobotPose:
    p = odom.pose.pose.position
    q = odom.pose.pose.orientation
    qy = q.w * q.z + q.x * q.y
    qx = q.w**2 + q.x**2 - q.y**2 - q.z**2
    return RobotPose(p.x, p.y, math.atan2(qy, qx))
```

Then add `from RobotPose import RobotPose, odom2pose` at the top of 
`curses_motor.py`. Then modify the program as follows:
* Add a subscription to the `odom` topic.
* In the callback function for the subscription, write code that calls 
  `odom2pose()` to convert the `Odometry` message to a `RobotPose` object,
  then display the `RobotPose` object in a suitable part of the `curses`
  window.
* Drive the robot around a bit. How does the coordinate space of a `RobotPose`
  correspond to the Euclidean plane? Relative to the robot's starting 
  position, where are the four Euclidean quadrants? How does this compare
  to thinking about the position of our simulated robot from 
  [Module 1](modules/intro1.html)?

## Hazards

Open a shell on your robot and type the following command:
```
ros2 topic echo /[your robot name]/hazard_detection
```

* Press the bumper directly at the front of the robot. What does it display?
* Press the bumper in different places, throughout its coverage of the front 
  half of the robot. What does it display when it is touched in different 
  places?
<!-- Concept invention: which bump sensors does it have? -->
* How many distinct bump sensors does the iRobot Create3 have?
  * What are their names?
<!-- Exploration: Other hazards -->
* Pick up the robot. What messages does it display?
<!-- Concept invention: pick-me-up-sensors -->
* How many distinct cliff-detection sensors does the iRobot Create3 have?
  * What are their names?
* What else can the iRobot Create3 sense to determine that it is not entirely
  on the ground?
  * What are their names?
* Add this import to `curses_motor.py`:
```
from irobot_create_msgs.msg import HazardDetectionVector
```
* Add a subscription to the `hazard_detection` topic. 
* In the callback function for the subscription, display the most recent
  detected hazard in the `curses` window.
* Run `curses_motor.py`. Press the bumper in various places, lift up the robot,
  and ensure that the correct hazards are displayed when they arise.

<!-- Students write some simple obstacle avoiders -->
<!-- Then an advanced one, using odometry -->

## Infrared sensors

<!-- Bring back lots of stuff from my original module -->
<!-- Invite them to enhance obstacle avoiders -->
