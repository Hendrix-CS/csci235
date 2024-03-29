---
layout: work
type: Project
num: 5
worktitle: Fuzzy Logic Behaviors
---

## Sample Program

* [`main.py`]({{site.baseurl}}/assets/programs/fuzzy/main.py)
* [`lib.py`]({{site.baseurl}}/assets/programs/fuzzy/lib.py)
* [`sonar3demo.py`]({{site.baseurl}}/assets/programs/fuzzy/sonar3demo.py)



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
* Your task should involve the robot switching between at least three distinct modes,
  at least two of which should be fuzzy.


## Questions
1. How did you go about creating effective fuzzy behaviors for each of
the above tasks?
2. How did you structure states and mode-selection to successfully 
incorporate fuzzy behaviors?
3. How did the fuzzy-logic robots perform in comparison to the non-fuzzy
versions on each of the tasks that were implemented both ways?
4. Why did you choose the task you chose for the free-choice part of the
project? How suitable did it prove to be in practice? How well did the 
system work?

