---
layout: work
type: Project
num: 3
worktitle: Build and Drive the Robot
---

## Parts

To begin, the parts you will need for this robot are found in the list below:
* Arduino Uno R3[A000066]
* Adafruit TB6612 1.2A DC/Stepper Motor Driver
* Bread Board
* Jumper Wires
* Robot Car Chassis kit including:
  * Two DC motors 
  * Two wheels with counter weights for balancing
  * Various nuts and bolts for assembly
  * Caster Wheel
  * Battery Box

## Assembly

**Please read each step to completion before beginning to assemble.**

### Step 1

First, start with the two yellow DC motors. In your kit, there should be two aluminum mounting brackets with 
multiple holes drilled out. There will also be four long screws. To attach the aluminum block to the DC motor, 
align it with the holes on the side of the DC motor where the connectors are and push the screw through the 
opposite side and tighten with a bolt. Place the black disk on the same side as the mounting bracket.
 
Make sure these are in the correct orientation, meaning the bolt side will be pointing to the center of the 
chassis with the mounting block and connectors pointing towards the center as well. Also, make sure the mounting 
bracket screw holes at the up towards the bottom of the chassis. If it is in the wrong orientation, then the 
block and bolts will interfere with the wheels and not connect to the chassis correctly. 

<img src="https://hendrix-cs.github.io{{site.baseurl}}/assets/images/SoloMotor.jpg" width=500>

### Step 2

Now attach the DC motors to the chassis, where the connectors are pointed toward the front of the chassis. 
This will require 1cm screws and bolts. Put the screws through the top side of the chassis plate and bolts in the 
bottom. Then attach the mounting bracket to the end of the screws with the bolts still in place. The bolts act as 
spacers to make sure there is clearance for the disk, so it doesn't conflict with the chassis. Then connect positive 
and negative wires to the connectors on the DC motors. 

<img src="https://hendrix-cs.github.io{{site.baseurl}}/assets/images/ChasisPlateWithSpacers.jpg" width=500>

Then, feed the wires from under the chassis to the top through the T-shaped holes. 

<img src="https://hendrix-cs.github.io{{site.baseurl}}/assets/images/ChasisTopSide.jpg" width=500>

### Step 3

Next, attach the caster wheel to the bottom of the chassis. Using 1cm screws and the appropriate bolts, there are 
four holes on the back of the chassis plate to use.

### Step 4

Next, push the wheels onto the axis. Make sure to align the hole on the wheel to the shape of the axis and it is 
pushed as far as possible.

Here is how your robot should look from the bottom:

<img src="https://hendrix-cs.github.io{{site.baseurl}}/assets/images/CasterWheelBottom.jpg" width=500>

Here is how your robot should look from the top:

<img src="https://hendrix-cs.github.io{{site.baseurl}}/assets/images/CasterWheelTop.jpg" width=500>

### Step 5

The Arduino board runs computer programs that send and receive signals to and from electrical devices.
We will connect the Arduino with these devices by using a breadboard. Here is a diagram of our circuit.
We will explore each element of this circuit in the sections below.

<img src="https://hendrix-cs.github.io{{site.baseurl}}/assets/images/WiringDiagram.png" width=500>

#### Step 5.1

Each numbered row of the breadboard
electrically connects all devices wired to it. The `+` and `-` columns of the breadboard represent **Power**
and **Ground** respectively. The power source of each circuit will be wired to the `+` column, and the `-` column
will direct electricity to ground once it completes its traversal of the circuit.

There are two possible power sources for the Arduino and its connected devices:
* The Arduino receives power through its USB port, and sends power to other devices.
* The Arduino and other devices receive power from batteries. 

In both cases, the power pins on the Arduino are extremely important. When it is powered by USB, it conveys
power through the 5V pin or 3.3V pin. (We will be using the 5V pin.) When it is receiving power from a battery, it
receives that power from that same pin.

Wire: 

| Arduino | Breadboard | 
| ------- | ---------- |
| 5V Pin  | `+` column |
| GND Pin | `-` column |

**Note:** Wire the two `-` columns together so that they both provide grounding. Also wire the
two `+` columns together so that they both provide power.

#### Step 5.2

The motor driver chip enables us to regulate the direction and voltage for each motor. It is an intermediary
between the motors, power and ground, and the Arduino. We will begin by connecting the driver chip to power
and ground. 

Wire: 

| Motor Driver Pin       | Breadboard |
| ---------------------- | ---------- |
| Top `+` pin            | `+` column |
| Top `-` pin            | `-` column |
| Bottom edge Ground pin | `-` column |

Each motor has two wires. The direction of current flow through those wires determines the direction
the motor spins. By sending electricity in one direction through the
motors, they turn forwards. Sending electricity the other direction turns the motors backwards.
The direction of flow will be controlled by the motor driver chip, through the motor outputs on
its bottom edge.

Wire: 

| Motor Driver Pin | Motor Wire |
| ---------------- | ---------- |
| Motor A Out 1    | Left Motor 1 |
| Motor A Out 2    | Left Motor 2 |
| Motor B Out 1    | Right Motor 1 |
| Motor B Out 2    | Right Motor 2 |

#### Step 5.3 

Each motor is controlled by three input pins on the motor driver. The motor speed level is controlled by
a **Pulse-Width Modulation** (PWM) pin. Pulse-Width Modulation reduces electrical current by periodically stopping
the flow of electricity. We will send analog signals to those pins to indicate how fast we want the motors to 
turn.

The other two pins for each motor control the input direction through a circuit known as an **H-Bridge**. 
These are digital pins. Setting pin 1 high and pin 2 low spins the motor in one direction, and setting
pin 1 low and pin 2 high spins the motor in the other direction. The direction it spins depends on how the 
two motor wires are connected to the motor outputs.

We use six Arduino outputs to control these pins - three for each motor. A major constraint on the 
assignment of Arduino pins to motor driver pins is that only Arduino pins 3, 5, 6, 9, 10, and 11 are 
able to send PWM signals. Wire the pins as follows:

| Arduino Pin | Motor Driver Pin |
| ----------- | ---------------- |
|           8 | BIN2             | 
|           9 | BIN1             |
|          10 | PWMB             |
|          11 | PWMA             | 
|          12 | AIN1             |
|          13 | AIN2             |

## <a name="programming">Programming the Robot</a>

[Download](https://www.arduino.cc/en/software) and install the 
[Arduino IDE](https://www.arduino.cc/en/Guide/Environment) for your computer.

Each Arduino program is called a *sketch*. The code in each sketch is written in the
Arduino programming language. 

We will begin by initializing some constants.  As each Arduino pin is connected to one 
of the motor control pins, we name the constant after the pin label on the motor driver chip.
When we declare a new constant, we begin with the `const` keyword, followed by its **type**. 
We then give its name, an `=` sign, and then the value which we assign to that constant.

Finally, every statement in the Arduino language ends with a semicolon (`;`).

The following block of code sets up the constants corresponding to the motor pins:

```
const int PWMA = 11;
const int AIN1 = 12;
const int AIN2 = 13;

const int PWMB = 10;
const int BIN1 =  9;
const int BIN2 =  8;
```

Note that if we were to choose to wire these pins differently, we would want to redefine
our constants accordingly.

Next, we create an `enum` type to enumerate the motors:

```
enum motor {
  A, B
};
```

Having created these constants and data types, we are ready to write some functions. We will
write a series of small functions to control the motors. 

The `spin()` function is the primary function you will call to spin a motor. As 
with all Arduino functions, this function has a return type. The return type `void` signifies 
that it does not return a value. Curly braces (`{` and `}`) specify where the function's code
begins and ends. Each parameter has its type listed first, followed by its name.

The `spin()` function determines whether motor A or motor B is to be spun, and calls `spin_help` 
with the appropriate pins.

```
void spin(motor mot, int speed) {
  if (mot == A) {
    spin_help(PWMA, AIN1, AIN2, speed);
  } else {
    spin_help(PWMB, BIN1, BIN2, speed);
  }
}
```

The `spin_help()` function receives the specific pins to control as its parameters, in addition
to the motor speed and direction. It uses the built-in `analogWrite()` function to send the
speed level to the PWM pin. Depending on the direction of spin, it calls `write_dir()` with 
the input pins in the desired order.

```
void spin_help(int pwm, int in1, int in2, int speed) {
  if (speed >= 0) {
    write_dir(in1, in2);
  } else {
    write_dir(in2, in1);
  }
  analogWrite(pwm, abs(speed));
}
```

The `write_dir()` function uses the built-in `digitalWrite()` function to send signals to the
input pins. The motor will spin one direction if the first pin is high and the second one low,
and the other direction if they are reversed. If the wiring of your motors is not consistent
with what you want "forward" to signify, you can switch the AIN1 and AIN2 (or BIN1 and BIN2) 
wires on your Arduino.

```
void write_dir(int hi, int lo) {
  digitalWrite(hi, HIGH);
  digitalWrite(lo, LOW);
}
```

The `setup()` function is the starting point for the Arduino program when the power comes on.
We begin by specifying the communcation rate over the USB cable using the `Serial.begin(9600)`
function. Next, we use the built-in `delay()` function to pause for 2000 milliseconds (2 seconds)
before doing anything else. Then, we call the `spin()` function we wrote earlier to start up each motor.
For convenience, we define a constant `FULL_SPEED` representing the highest value we can use for the 
motor speed. We then pause for 10 seconds before stopping both motors.

```
const int FULL_SPEED = 255;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(2000);
  spin(A, FULL_SPEED);
  spin(B, FULL_SPEED);
  delay(10000);
  spin(A, 0);
  spin(B, 0);
}

```

## <a name="assignment">Assignment</a>

Once you have built your robot, answer the following questions:
* What is the robot's speed at the full 255 level? What is its speed at the 200, 150, and 100 levels? To measure
  its speed, determine the time it takes to drive five meters. For each speed, run your program five times, and 
  record all of your measurements.
* When the robot drives five meters, how far off from a straight line does it drive? Run your program five times and
  record those measurements.
* Write a program to drive the robot one meter, turn 90 degrees, and drive one more meter. How close can you get to
  a perfect 90 degree turn? How repeatable is it? How about a 45 degree turn?
* Devise three driving patterns for your robot. Each driving pattern should include at least three alternations of
  driving and turning. How accurately can you make the robot follow each pattern?
* Based on the above experiments, how well does your robot follow directions in the absence of sensor input?
  
## Submission

Submit a paper containing the following:
* Answers to the questions above.
* Source code for the three driving patterns.

Also submit a [link to a video of your robot in action](https://docs.google.com/forms/d/e/1FAIpQLSclzFbU1iCMsCVzT3jQBo3wyyRbUzYsfY7FxJcqNPbm5ONo-Q/viewform?usp=sf_link).

------------------------------------------------------------------------
