---
layout: work
type: Module
num: 4
worktitle: iRobot Create3 Sensors
---

## Odometry

### Odometry on the command line
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
* Why do you think it has both `pose` and `twist` fields? In light of your 
  answer to the previous question, what distinct roles do they play?
* Now write a ROS2 command for the robot to spin in place. Repeat the above steps.
  How does the `odom` information change as the robot spins?
<!-- Concept invention: Odometry -->
* What fields from the `odom` message are most relevant to determining the robot's
  position and orientation?
<!-- odometry resets -->
* Open a third command line. Type 
```
ros2 service call /[your robot name]/reset_pose irobot_create_msgs/srv/ResetPose
``` 
* How does this affect the odometry messages being published?

### Odometry in Python

Make sure you have a copy of `robot_pose.py` in the current directory. Then
add the following import at the top:
```
from nav_msgs.msg import Odometry
```

Then add the following function after the `RobotPose` class definition:
```
def odom2pose(odom: Odometry) -> RobotPose:
    p = odom.pose.pose.position
    q = odom.pose.pose.orientation
    qy = q.w * q.z + q.x * q.y
    qx = q.w**2 + q.x**2 - q.y**2 - q.z**2
    return RobotPose(p.x, p.y, math.atan2(qy, qx))
```

Then add `from robot_pose import RobotPose, odom2pose` and `from nav_msgs.msg 
import Odometry` at the top of `curses_motor.py`. Then modify the program as 
follows:
* Add `stdscr` as a parameter to `__init__`, and set up an instance variable
  to store the `stdscr` reference. This will enable us to display messages
  in the cursor window.
* Add a subscription to the `odom` topic.
* In the callback function for the subscription, write code that calls 
  `odom2pose()` to convert the `Odometry` message to a `RobotPose` object.
  * Display the `x`, `y`, and `theta` coordinates in the `curses` window.
    Use format strings to show only the first two decimal places. 
  * For review, if you have a Python assignment `x = 2.345` and you only want
    to display the first two decimal places, you can use a format string like
    this: `f"{x:.2f}"`
* Drive the robot around a bit. How does the coordinate space of a `RobotPose`
  correspond to the Euclidean plane? Relative to the robot's starting 
  position, where are the four Euclidean quadrants? How does this compare
  to thinking about the position of our simulated robot from 
  [Module 1](modules/intro1.html)?

## Hazards

### Hazards on the command line
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
* Exit the topic echo.

### Hazards in Python
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
Having built a program to display sensor values while driving the robot, let's
adapt these ideas to building a more autonomous robot. Create a new Python
program called `bump_turn_90.py` with the following features:
* The robot normally drives straight.
* However, if it encounters a hazard, it turns 90 degrees, then resumes
  driving forward.
* The program has a `curses` UI:
  * It displays its current odometry and whether it is contacting a hazard.
  * If you press the `q` key, the program ends.
  * There are no other controls for the robot.

## Infrared sensors

<!-- Bring back lots of stuff from my original module -->
<!-- Invite them to enhance obstacle avoiders -->
