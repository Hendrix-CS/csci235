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

Please refer to the image below for the circuit diagram.

<img src="https://hendrix-cs.github.io{{site.baseurl}}/assets/images/WiringDiagram.png" width=500>

The only pieces that may require soldering are the DC motor driver and the connections to the DC motors. 
There can be faulty connections if not secure. Faulty connections will cause the motors to function 
improperly and provide incorrect results.

## <a name="programming">Programming the Robot</a>

[Download](https://www.arduino.cc/en/software) and install the 
[Arduino IDE](https://www.arduino.cc/en/Guide/Environment) for your computer.

Each Arduino program is called a *sketch*. The code in each sketch is written in the
Arduino programming language. 

We will begin by initializing some variables. When we declare a new variable, we 
begin by designating its *type*. We then give its name, an `=` sign, and then the
value with which we intend to initialize that variable.

Finally, every statement in the Arduino language ends with a semicolon (`;`).

The following block of code sets up variables corresponding to the motor pins:

```
int spd_A  = 11;
int A_in_1 = 12;
int A_in_2 = 13;

int spd_B  = 10;
int B_in_1 =  9;
int B_in_2 =  8;
```

The next two lines of code set up some useful speed values:

```
int fwd_speed = 200;
int trn_speed = 255;
```

Each Arduino sketch has a special `setup` function that is called when the program starts. As 
with all Arduino functions, this function has a return type. The return type `void` signifies 
that it does not return a value. Curly braces (`{` and `}`) specify where the function's code
begins and ends.

```
void setup() {
  Serial.begin(9600);
  
  move(1, fwd_speed, 0);
  move(2, fwd_speed, 0);
  
  delay(1000);
}
```

Let's explore each line in turn:
* `Serial.begin(9600);` This sets the communication speed for the cable from the Arduino back to 
  your computer. In future weeks, this will be the connection to your Android device.
* `move()` These two lines are calls to a function we will define shortly.
* `delay(1000);` This tells the Arduino to pause for 1000 milliseconds.

------------------------------------------------------------------------
