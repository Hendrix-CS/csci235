---
layout: work
type: Project
num: 5
worktitle: Vision-based Reactive Behaviors
---

## Vision

In this project, we will use images to control the robot's behavior. 

To create a vision-based controller:
* Use your tablet to take photographs of the robot's environment.
* Label each photograph.
* Build a k-nearest-neighbor classifier using the labeled photographs.
* Write an Arduino program to determine robot behavior based on the label assigned
  to the current image the tablet perceives.

## Vision App

For this and all future projects, you will need to install the 
[Tracker2]({{site.baseurl}}/assets/apps/app-release.apk) app on your Android device.

### Manager Screen
<img src="https://hendrix-cs.github.io{{site.baseurl}}/assets/images/App_Manager_View.png" width=500>

From the Manager Screen, you can:
* View photographs.
* Assign photographs to projects and labels.
* Delete unwanted photographs.
* Go to the Testing Screen
* Go to the Project/Label Editing Screen
* Go to the Robot View Screen

### Testing Screen
<img src="https://hendrix-cs.github.io{{site.baseurl}}/assets/images/App_Test_View.png" width=500>

From the Testing Screen, you can:
* Type in a vision command that you would like for your Arduino program to send.
* See a report of its estimated classification accuracy.
* Go back to the Manager Screen.

### Editing Screen
<img src="https://hendrix-cs.github.io{{site.baseurl}}/assets/images/App_Edit_View.png" width=500>

From the Project/Label Editing Screen, you can:
* Create new projects and labels.
* Rename existing projects and labels.
* Delete projects and labels.
* Go back to the Manager Screen.

### Robot View Screen
<img src="https://hendrix-cs.github.io{{site.baseurl}}/assets/images/App_Robot_View.png" width=500>

From the Robot View Screen, you can:
* Take pictures.
* Start the robot.
* Stop the robot.
* Go back to the Manager Screen.