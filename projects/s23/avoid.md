---
layout: work
type: Project
num: 2
worktitle: Avoiding Obstacles
---

## Sample Programs
* [`hello_world.py`]({{site.baseurl}}/assets/programs/hello_world.py)
* [`button_demo.py`]({{site.baseurl}}/assets/programs/button_demo.py)
* [`sensor_demo.py`]({{site.baseurl}}/assets/programs/sensor_demo.py)
* [`motor_demo_1.py`]({{site.baseurl}}/assets/programs/motor_demo_1.py)
* [`motor_demo_2.py`]({{site.baseurl}}/assets/programs/motor_demo_2.py)
* [`motor_demo_3.py`]({{site.baseurl}}/assets/programs/motor_demo_3.py)

## Assignment

* Be sure to wire up the motors.
* Attach and wire the ultrasonic sensor and two touch sensors.
* Implement the following programs: 
  * `ForwardBump`: Drives forward unless a front-mounted bump sensor is hit. When it is hit, it stops moving. If the button is released, forward motion resumes.
  * `BumpAvoid`: Drives forward. When the left bump sensor is pressed, it turns left. When the right bump sensor is pressed, it turns right.
  * `UltrasonicAvoid`: Drives forward. When the ultrasonic value is too low, it turns left.
  * `CombinedAvoid`: Drives forward. Uses both the bump sensors and the ultrasonic sensor to help it decide when to turn.
  * For the last three programs, experiment with them until, in your opinion, they work as well as they possibly can. Be sure to document this in terms of a performance metric, as discussed in the questions below.

## Questions

1. With a two-wheeled robot, three different styles of turns are possible. What are the three styles? For each style, in what situations is it beneficial? Detrimental? Support your answers with evidence from experimenting with your robots.
2. What is the minimum and maximum useful distance for the ultrasonic sensor? In what situations does it work well? When might it work poorly?
3. When creating an obstacle-avoiding robot that uses the ultrasonic sensor, what is the minimum distance at which to begin a safe turn? This value may vary depending upon the turning style; be sure to specify a value for each one.
4. What are the advantages and disadvantages of obstacle avoidance using only touch sensors? Only ultrasonic sensors? How much better is avoidance when using both together? Support your answer with observations of your robot.
5. What numerical metrics are appropriate for quantifying the performance of an obstacle-avoiding robot? How are they measured? How do they conform with our intuitions?
6. How did you employ your metrics to iteratively create your best obstacle-avoiding robot?
7. Is your best obstacle-avoiding robot *intelligent*? Why or why not?

