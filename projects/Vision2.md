---
layout: work
type: Project
num: 6
worktitle: Groundline and PID Control
---

## Finding the Image Groundline

In this project, we will program the robot to respond to continuous (instead of discrete) 
feedback from an image. The continuous input will be the robot heading that maximizes 
perceived open space. The perceived open space is found by calculating the **groundline**, 
a sequence of pixels where the floor meets the walls or other objects.

The app's groundline command is as follows:
```
cv groundline maxColors minNotFloor project label width height [photoNums]
```

The groundline is calculated as follows:
* Each pixel color is classified as **floor** or **not-floor**.
  * Pixel colors are grouped into `maxColors` clusters each for **floor** and **not-floor** pixels,
    using k-means with `k=maxColors`.
* For each `x` coordinate in the image:
  * Set `y` to the maximum value (`height - 1`)
  * While `y > 0` and the number of consecutive **not-floor** pixels is less than `minNotFloor`
    * Decrease `y` by 1
	
The maximum perceived open space is the `x` value that minimizes `y`. If there is a tie,
`x` values closer to the center of the image are preferred.

If no `photoNums` values are specified, all of the images with the given `project` and `label` are used
for determining the **floor** and **not-floor** pixel colors. If there are one or more `photoNums` values,
only the photos with the given numbers are used. The rectangle in the bottom center of the image is the source
of the **floor** pixels, and the two rectangles in the upper-left and upper-right corners are the sources of the
**not-floor** pixels. To view the rectangles, select the `Floor Sample` checkbox.

<img src="https://hendrix-cs.github.io{{site.baseurl}}/assets/images/Screenshot_Floor_Sample.png" width=500>


## Steering Towards Maximum Space Using PID Control

### Proportional Control

The robot wants to set its motor levels such that the maximum perceived open space is as 
close to the center of the image as possible. The difference between the desired target
and the current target is the robot's **error**. Our goal is to write a program that 
minimizes this error value.

The error's units are in image pixels. To set the motor speeds appropriately, one must
specify a proportionality constant `P` that converts the error's units into motor speed 
units. This constant can be scaled in any desired manner to produce the target speed 
adjustments. Larger values of `P` result in more rapid but potentially less stable
adjustments. Smaller values of `P` result in less rapid but more stable adjustments.

### Integral Control

Ideally, the median error should be zero. If it is not, then the error has a **bias**. 
An integral controller maintains a running total of error values in order to determine the bias.
(If this total is zero, there is no bias.) The integral constant `I` is multiplied by this
running total and added to the error value from the proportional step.

### Derivative Control

If the system is oscillating excessively, it can be helpful to incorporate the difference 
between consecutive errors into the calculation. We subtract the previous error from the 
current error, multiply this difference by a third constant (`D`), and add it to our value 
from the previous two steps.

### Sample Arduino Code

```
const int PWMA = 11;
const int AIN1 = 12;
const int AIN2 = 13;

const int PWMB = 10;
const int BIN1 =  9;
const int BIN2 =  8;

const long WIDTH = 40;
const long HEIGHT = 30;

const int MIN_SPEED = 120;
const int MAX_SPEED = 255;
const int SPEED_RANGE = MAX_SPEED - MIN_SPEED;

const float P = 1.0 / (WIDTH / 2);
const float I = 0.0;
const float D = 0.0;

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
long lastError = 0;
long totalError = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

float getAdjustment(long x) {
  long error = x - (WIDTH / 2);
  long diff = error - lastError;
  lastError = error;
  totalError += error;
  float adjustment = error * P + totalError * I + diff * D;
}

void assignSpeeds(motor slower, motor faster, float adjustment) {
  int offset = MAX_SPEED - int(adjustment * SPEED_RANGE);
  spin(slower, offset);
  spin(faster, MAX_SPEED);
}

void adjustMotors(float adjustment) {
  if (adjustment < 0) {
    assignSpeeds(B, A, -adjustment);
  } else {
    assignSpeeds(A, B, adjustment);
  }
}

void pidController(String message) {
  int space = message.indexOf(' ');
  long x = message.substring(0, space).toInt();
  long y = message.substring(space + 1).toInt();
  if (y > 2 * HEIGHT / 3) {
    spin(A, MIN_SPEED);
    spin(B, -MIN_SPEED);
  } else {
    float adjustment = getAdjustment(x);
    adjustMotors(adjustment);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n');
    if (message == "start") {
      Serial.println("cv groundline 30 2 Office Carpet " + String(WIDTH) + " " + String(HEIGHT) + " 1 2");
      running = true;
    } 
    if (message == "stop") {
      spin(A, 0);
      spin(B, 0);
      Serial.println("cv pause");
      running = false;
    }

    if (running) {
      pidController(message);
    }
  }
}
```

## Vision App

For this and all future projects, you will need to install the 
[Tracker2](https://github.com/gjf2a/Tracker2/releases) app on your Android device.

<img src="https://hendrix-cs.github.io{{site.baseurl}}/assets/images/Screenshot_Office_Groundline.png" width=500>

## Task

Use the Groundline feature to program a PID controlled robot that navigates through 
two different areas. Examples:
* A hallway
* A room strewn with obstacles

<!--
The robot should continue to seek open space as it drives around until it arrives
at a known location, recognized using a kNN or kmeans classifier. At that point, it 
should stop moving. 

To incorporate two different `cv` commands, you will need to alternate between them. Here is one possible
strategy for doing so:
* When the `start` button is pushed, send both `cv` commands. The first one sent will be `1`, and the second
one sent will be `2`. The second one will be active initially.
* Increment a counter on each `loop()` iteration.
* When the counter hits a certain value, send `cv resume 1`. That will pause the current `cv` command and 
  restart the other command.
* Reset the counter. When it hits a target value, send `cv resume 2` to restart the original `cv` command.
-->
## Writeup

1. What quantitative and qualitative criteria did you use for assessing the quality of your PID controllers?
2. How did you determine an appropriate value for `P`? In particular, how did you adjust `P` 
   in light of the criteria you stated in your answer to question 1? What value for `P` did you ultimately
   employ?
3. Did you employ a nonzero value for `I`? If so, what observations about the robot's behavior persuaded
   you to do so? To what degree did using a nonzero value for `I` prove helpful? What value for `I` did
   you ultimately employ?
4. Did you employ a nonzero value for `D`? If so, what observations about the robot's behavior persuaded
   you to do so? To what degree did using a nonzero value for `D` prove helpful? What value for `D` did
   you ultimately employ?
5. What tradeoffs are involved in deciding between using a discrete action and a continuous action? 
   Use specific examples from this week's tasks to support your answers.

## Video

Submit videos of the robot navigating each of the two areas.