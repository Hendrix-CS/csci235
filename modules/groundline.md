---
layout: work
type: Module
num: 6
worktitle: Computer Vision I: Groundline
---

## Groundline

We employ computer vision to find the boundary between the ground and
objects resting on the ground. When you run `groundline_node`, it fills
its window with an image showing where the ground is located. A sample
image is given below.

The groundline itself represents, for each `x` coordinate, the number
of pixels of ground that are present. When an object is close to the 
robot, there will be very few pixels of ground present. When an object
is far from the robot, there will be a larger number of pixels of 
ground present. The `groundline_node` publishes this list
of pixel heights on the topic `robot_groundline`. 
(Substitute your robot's name for `robot`; on `frodo`, for instance,
the topic is `frodo_groundline`.) When you run the node, it will
fill its window with an image showing where the ground is located.

To get a feel for the groundline, do the following:
* Open three shells on your robot, in different tabs.
* In one shell, run `groundline_node robot_name`, substituting the
  name of your robot for `robot_name`.
* In a second shell, run `ros2 topic echo `robot_groundline`, 
  substituting the name of your robot for `robot`.
* In a third shell, run `curses_motor.py`.
* Drive your robot around for a while. How does the groundline change
as the robot drives around? What kinds of values do you see published
  on the `robot_groundline` topic?

## Finding floor regions

A **floor region** has the following characteristics:
* It is at least 16 pixels wide (working from a 160 x 120 image)
* All of the `x` coordinates in the region have a pixel height
of at least 30 pixels (1/4 of the image height)

Here is an example groundline:
