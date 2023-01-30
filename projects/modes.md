---
layout: work
type: Project
num: 3
worktitle: Mode Selection
---

## Sample Programs

* [`main.py`]({{site.baseurl}}/assets/programs/mode_selection_2/main.py)
* [`sonar_demo.py`]({{site.baseurl}}/assets/programs/mode_selection_2/sonar_demo.py)
* [`multi_sonar.py`]({{site.baseurl}}/assets/programs/mode_selection_2/multi_sonar.py)
* [`rotation_demo.py`]({{site.baseurl}}/assets/programs/mode_selection_2/rotation_demo.py)
* [`demo3.py`]({{site.baseurl}}/assets/programs/mode_selection_2/demo3.py)

## Part 1: Obstacle Avoidance

* In addition to the ultrasonic sensor and two touch sensors, attach and wire the color sensor
  pointed toward the ground.
* Implement the following programs: 

### `Avoid2Way`
* An obstacle avoider that uses both the bumpers and the sonar.
* Turns **should not** monitor the motor encoders.
* When a bumper is pressed, the robot should turn in a direction that helps the robot
  avoid further collisions.
* When the sonar senses a close object, it should turn in the same direction that 
  the most recent bumper hit induced. (If a bumper has not yet been hit, the direction
  it chooses does not matter.)
  
### `Avoid2WayDelay`
* Similar to `Avoid2Way`, except that when turning the robot **should** monitor the motor 
  encoders to enable it to take a longer turn. 
* Experiment with different turn durations until you find one that performs well.
 
 
## Part 2: Patrol
 
### `Patrol1`
* The robot should patrol along a line.
* It should drive for two meters, followed by a 180 degree turn.
* This behavior will repeat indefinitely.

### `Patrol2`
* Some cardboard will be provided for you to create a patrol track.
* The robot should patrol along the track. It will drive forthe length of the track, followed
  by a 180 degree turn. If it wanders off the track, as determined by the color
  it scans, it will realign itself to be back on course.

### `Patrol3`
* Similar to `Patrol2`, except that the robot will begin its turn early if it senses an 
  obstacle (with either bumpers or sonar).
  
## Part 3: Your choice
* Devise a task for your robot that is amenable to being solved by using the state machine
approach.
* The solution to the task should employ at least three distinct states.
* At least one of the states should have the possibility of transitioning to at least
  two possible destination states, depending on sensory conditions.
* Implement a state-machine program that performs the task.

## Questions

1. Using the same metrics you used from [Project 2]({{site.baseurl}}/projects/avoid.html), which works better: `Avoid2Way` or `Avoid2WayDelay`?
2. A *traversal* of the patrol line consists of driving the full two meters out, followed by returning to the original location. 
   Run the `Patrol` robot for each number of traversals given below. How close does it get to the original starting point? 
   How might you characterize the loss of precision from repeated traversals?
   * 1
   * 2
   * 4
3. What are some appropriate quantitative metrics for the patrol task? With regard to those
   metrics, how did your `Patrol1`, `Patrol2`, and `Patrol3` robots perform? 
4. What qualitative performance aspects of the patrol task are not captured by your 
   quantitative metrics? How did your robot perform with respect to those aspects?
5. What might be an application of the concept of the `Patrol3` robot?
6. Describe your customized task. How well did the robot perform on the task? As before, use both 
   quantitative metrics and qualitative analysis to discuss its performance.
7. From your experience of this lab, what advantages and disadvantages did you find for the state-machine approach to programming?


