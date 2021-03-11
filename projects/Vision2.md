---
layout: work
type: Project
num: 6
worktitle: Groundline and PID Control
---

## Finding the Image Groundline

In this project, we will program the robot to respond to continuous (instead of discrete) 
feedback from an image. The continuous input will be the robot heading that maximizes 
perceived open space.

To find open space, the **groundline** is determined as follows:
* Each pixel color is classified as **floor** or **not-floor**.
* For each `x` coordinate in the image:
  * Set `y` to the maximum value
  * Decrease `y` as long as the color is **floor**
  * Stop when either:
    * `y` reaches zero (i.e. top of the image)
	* A total of **minNotFloor** pixels are encountered consecutively
	
The maximum perceived open space is the `x` value that minimizes `y`. If there is a tie,
`x` values closer to the center of the image are preferred.

## Steering Towards Maximum Space Using PID Control

### Proportional Control

The robot wants to set its motor levels such that the maximum perceived open space is as 
close to the center of the image as possible. The difference between the desired target
and the current target is the robot's **error**. Our goal is to write a program that 
minimizes this error value.

The error's units are in image pixels. To set the motor speeds appropriately, one must
specify a proportionality constant *P* that converts the error's units into motor speed 
units. This constant can be scaled in any desired manner to produce the target speed 
adjustments. Larger values of *P* result in more rapid but potentially less stable
adjustments. Smaller values of *P* result in less rapid but more stable adjustments.

### Integral Control

Ideally, the median error should be zero. If it is not, then the error has a **bias**. 
An integral controller maintains a running total of error values in order to determine the bias.
(If this total is zero, there is no bias.) The integral constant *I* is multiplied by this
running total and added to the error value from the proportional step.

### Derivative Control

If the system is oscillating excessively, it can be helpful to incorporate the difference 
between consecutive errors into the calculation. We subtract the previous error from the 
current error, multiply this difference by a third constant (*D*), and add it to our value 
from the previous two steps.

## Vision App

For this and all future projects, you will need to install the 
[Tracker2](https://github.com/gjf2a/Tracker2/releases/download/0.2.0/app-release.apk) app on your Android device.

<img src="https://hendrix-cs.github.io{{site.baseurl}}/assets/images/Groundline1.png" width=500>

## Task

Use the Groundline feature to build a PID controlled robot that navigates through 
two different indoor areas. Examples:
* A hallway
* A room strewn with obstacles

The robot should continue to seek open space as it drives around until it arrives
at a known location, recognized using the kNN classifier.

## Writeup



## Video

Submit videos of the robot performing each of the three tasks.