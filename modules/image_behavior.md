---
layout: work
type: Module
num: 11
worktitle: Image-Driven Behavior
---

# Classifying images

We have two sets of training images. Examine each set below.

## Blocked Images

### Image 1

<img src="{{site.baseurl}}/assets/images/blocked/photo_1.png" />

### Image 2

<img src="{{site.baseurl}}/assets/images/blocked/photo_2.png" />

### Image 3

<img src="{{site.baseurl}}/assets/images/blocked/photo_3.png" />

## Clear Images

### Image 1

<img src="{{site.baseurl}}/assets/images/clear/photo_1.png" />

### Image 2

<img src="{{site.baseurl}}/assets/images/clear/photo_2.png" />

### Image 3

<img src="{{site.baseurl}}/assets/images/clear/photo_3.png" />

We also have two images to classify:

## First test image

<img src="{{site.baseurl}}/assets/images/clear/photo_4.png" />

## Second test image

<img src="{{site.baseurl}}/assets/images/blocked/photo_4.png" />

## Image Similarity

1. Which image from the **combined** set of training images would
you say is **most** similar to the **first** test image?
2. Does that image belong to the **clear** or **blocked** label?
3. Which image from the **combined** set of training images would
you say is **most** similar to the **second** test image?
4. Does that image belong to the **clear** or **blocked** label?
5. Which **three** images from the **combined** set of training images would
you say are **most** similar to the **first** test image?
6. How many of those three images belong to the **clear** set?
How about the **blocked** set?
7. Which **three** images from the **combined** set of training images would
you say are **most** similar to the **second** test image?
8. How many of those three images belong to the **clear** set?
How about the **blocked** set?
9. What led to more accurate classifications: one images or three images?

This classification algorithm is known as **k nearest neighbor** 
(abbreviated **kNN**). To classify an image:
* Compare each image in the training set to the image to be classified.
* Sort them in order of how similar they are to the image to be classified.
* Select the top *k* most similar images.
* The most frequently occuring label is the classification.

# Building a Labeled Image Database

Run the `photographer` program as follows:

```
photographer proj1 in_bounds out_of_bounds
```

If, when you run the program, you see an error like this:
```
Error: Could not open device 0: V4L2 Error: Permission denied (os error 13)
```

Type this into the command prompt:
```
sudo chmod 666 /dev/video*
```
and enter your password when requested.

Select an area for your robot to patrol. Position the robot in four locations
where it would be in-bounds, and four locations where it would be 
out-of-bounds. Take a picture with the appropriate label at each location.
(To take a picture, type the Enter key.)

To test how well it classifies, open two shells. In one shell, start the node:

```
knn_photo_node robot_name proj1
```

In another shell, monitor the node's output:

```
ros2 topic echo /robot_name_image_label
```

In both cases, replace `robot_name` with the name of your robot.

Test the classifier. How does it perform? 

You may find that it doesn't classify images quite the way you had in mind.
Continue to add training samples for underrepresented labels until you find
its performance reasonably satisfactory.

The `photographer` program saves the images in folders as follows:
* Top-level folder: Name of your project
* Subfolders: Each label
* Files within subfolders: The images, in PNG format.

You can view the images using the MobaXTerm file browser, or you can copy them onto
your own computer as well.

# Deploying a kNN guided robot

We will deploy our robot to follow the simplest patrolling state machine
from [module 10](modules/state_machines.html). Create a new Python program
named `image_patrol_node_1.py`, and copy the code below into it:
```
from patrol_machine_1 import BoundedPatrol
import sys, curses, math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from robot_pose import RobotPose, odom2pose, normalize_angle_symmetric
from std_msgs.msg import String


ANGULAR_VELOCITY = 0.5
LINEAR_VELOCITY = 1.0


def add_twist(machine: BoundedPatrol, t: TwistStamped):
    state = machine.state_name()
    if state == 'forward':
        t.twist.linear.x = LINEAR_VELOCITY
    else:
        t.twist.angular.z = ANGULAR_VELOCITY


class BoundedPatrolNode(Node):
    def __init__(self, robot_name: str):
        super().__init__(f"{robot_name}_BoundedPatrolNode")

        self.motors = self.create_publisher(TwistStamped, f"/{robot_name}/cmd_vel_stamped", qos_profile_sensor_data)
        self.create_subscription(String, f"/{robot_name}_image_label", self.image_label_callback, qos_profile_sensor_data)

        self.machine = BoundedPatrol()
        self.msg = None

    def running(self):
        return self.machine.state_name() != 'finished'

    def image_label_callback(self, image_label: String):
        self.msg = image_label.data.split('/')[-1]
        self.machine.send(self.msg)
        self.publish_twist()

    def publish_twist(self):
        t = TwistStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = self.get_clock().now().to_msg()
        add_twist(self.machine, t)
        self.motors.publish(t)

    def process_keystroke(self, key: str):
        if key == 'q':
            self.machine.send("quit")

    def display(self, stdscr):
        stdscr.addstr(1, 0, f"state:      {self.machine.state_name()}{' ' * 10}")
        stdscr.addstr(2, 0, f"transition: {self.msg}{' ' * 10}")
        stdscr.refresh()


def main(stdscr):
    rclpy.init()
    node = BoundedPatrolNode(sys.argv[1])
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()
    while node.running():
        try:
            k = stdscr.getch()
            if k != -1:
                node.process_keystroke(chr(k))
            node.display(stdscr)
            rclpy.spin_once(node, timeout_sec=0.0)
        except curses.error as e:
            if str(e) != 'no input':
                stdscr.addstr(0, 0, traceback.format_exc())

    rclpy.shutdown()
    node.destroy_node()
    curses.nocbreak()
    curses.echo()
    stdscr.refresh()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 image_patrol_node_1.py robot_name")
    else:
        curses.wrapper(main)
```

1. Examine `image_patrol_node_1.py`. How does it differ from 
`patrol_node_1.py`?
2. To run `image_patrol_node_1.py`, you will also need to run `knn_photo_node`
in a separate shell. It does not matter which one you start first. Go ahead
and run the nodes. How well does the robot patrol the area? 
3. Are there any image categories that it consistently mis-labels? If so,
add (or remove) photographs until its performance improves noticeably.

# An Additional Task

1. Devise a new task for the robot that makes use of image classification
as one of its sensor inputs. The robot should also use two or more of the
following:
* A third image category
* Odometry
* IR sensors
* Hazard sensors
* Interface buttons

2. Devise a state machine for controlling the robot for this task. Implement
the state machine in Python.

3. Implement a ROS2 node to execute the state machine.

4. Test the ROS2 node. How well did it perform? Experiment with the images
and implementation until performance is acceptable for the task. If you are
uncertain about what constitutes "acceptable performance", speak with the
instructor.
