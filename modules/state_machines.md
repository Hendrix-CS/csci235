---
layout: work
type: Module
num: 3
worktitle: State Machines
---

## New Topic: Hazards

<!-- Exploration: hazard_detection from the command line -->
Type the command below into the command line:
```
ros2 topic echo /[your robot name]/hazard_detection
```

* Press the bumper directly at the front of the robot. What does it display?
* Press the bumper in different places, throughout its coverage of the front half of the
  robot. What does it display when it is touched in different places?
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

## States

<!-- outline 
* Introduce hazard sensors
* Introduce being in different states based on last obstacle encountered
* Odometry
* Patrol task
-->