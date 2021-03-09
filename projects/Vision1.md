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
[Tracker2](https://github.com/gjf2a/Tracker2/releases/download/0.1.2/app-release.apk) app on your Android device.

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

## Arduino App Interaction

The Arduino program below shows how to communicate with the app to control the robot.

	const int PWMA = 11;
	const int AIN1 = 12;
	const int AIN2 = 13;

	const int PWMB = 10;
	const int BIN1 =  9;
	const int BIN2 =  8;

	const int SPEED = 220;
	const int TURN = 150;

	enum motor {
	  A, B
	};

	void spin(motor mot, int speed) {
	  if (mot == A) {
		spin_help(PWMA, AIN1, AIN2, speed);
	  } else {
		spin_help(PWMB, BIN1, BIN2, speed);
	  }
	}

	void spin_help(int pwm, int in1, int in2, int speed) {
	  if (speed >= 0) {
		write_dir(in1, in2);
	  } else {
		write_dir(in2, in1);
	  }
	  analogWrite(pwm, abs(speed));
	}

	void write_dir(int hi, int lo) {
	  digitalWrite(hi, HIGH);
	  digitalWrite(lo, LOW);
	}

	bool running = false;

	void setup() {
	  Serial.begin(9600);
	  // Start a knn classifier with k=3, image width=20, image height=15
	  Serial.println("cv knn 3 Gameroom 20 15");
	}

	void loop() {
	  if (Serial.available() > 0) {
		String message = Serial.readStringUntil('\n');
		if (message == "start") {
		  Serial.println("cv resume 1");
		  running = true;
		} 
		if (message == "stop") {
		  spin(A, 0);
		  spin(B, 0);
		  Serial.println("cv pause");
		  running = false;
		}
		
		if (running) {
		  if (message == "Turn") {
			spin(A, -TURN);
			spin(B, TURN);
		  } else if (message == "Go") {
			spin(A, SPEED);
			spin(B, SPEED);
		  }
		}
	  }
	}
	
## Project 

Devise three vision-based controllers for your robot:
1. The first controller should avoid obstacles along the lines of the Arduino 
   example above. Use the example program in conjunction with your own collection
   of labeled photographs. Adjust speeds as necessary.

2. The second controller may be for any task you like. It should use a knn classifier
   with two labels.

3. The third controller may also be for any task you like. It should use a knn classifier
   with three or more labels. You may optionally also incorporate sonar.   
   
For tasks 2 and 3, be sure to describe criteria for good performance on the task
prior to testing the robot the first time. Include both quantitative and qualitative 
criteria. After gaining experience with the robot, you may want to adjust your criteria.
Be sure to record both the original and adjusted criteria.

## Writeup

1. How did the robot perform on the obstacle-avoidance task in comparison to the 
   sonar-based robot? 
   
2. How did the robot perform on the second task, in light of your criteria? What were 
   the original criteria? How did they change? Why?
   
3. Answer question 2 for the third task.

4. For each of the three tasks, how many images did you employ for each label? What
  image resolution did you use? What frame rates did your robot obtain?
  
5. Based on your answers to the above questions, what does it take to create an 
   effective vision-based robot behavior?

## Video

Submit videos of the robot performing each of the three tasks.