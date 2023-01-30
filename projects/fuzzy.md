---
layout: work
type: Project
num: 4
worktitle: Fuzzy Logic Behaviors
---

## Sample Programs


## Assignment

* Remove the bump sensors from your robot.
* Attach and wire two NXT Ultrasonic sensors to the left and right 
  sides of your robot.
* Implement the following programs: 

### `FuzzyFollower`
* Use the front ultrasonic sensor to maintain a target distance
  from an object. 
* If the object moves away from the robot, it should increase its speed until it catches up. 
* If the object moves towards the robot, it should back up. 
* All motions should be proportional to the distance between the robot and the object.

### `AvoidFuzzy`
* When all sonars are maximally clear, drive forward.
* Create fuzzy logic rules for each of the three sonars.
  * The front sonar rule should slow both wheels.
  * The left sonar rule should slow down the right wheel.
  * The right sonar rule should slow down the left wheel.
* Write a controller that defuzzifies the outputs of these rules
  to produce motor settings for each wheel.
  
### `AvoidFuzzyBack`
* Similar to `AvoidFuzzy`, except if the front sonar value 
  falls below a threshold, the robot should shift to a backing-up mode.
* The backing-up mode should employ a fuzzy rule for each sonar that
  determines the intensity and direction of backwards movement.
  
### `FuzzyPatrol`
* Reimplementation of `Patrol3` from [Project 3]({{site.baseurl}}/projects/modes.html).
* Instead of immediately turning around when an obstacle is sensed,
  use fuzzy logic to slow the robot down as it approaches the obstacle.
* If the robot gets too close to the obstacle, it should then turn around.

### Your choice
* Devise a task for your robot that is amenable to being solved by using fuzzy logic.


## Questions


