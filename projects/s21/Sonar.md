---
layout: work
type: Project
num: 4
worktitle: Sonar
---

## Sonar

<img src="https://hendrix-cs.github.io{{site.baseurl}}/assets/images/sonar.jpg" width=500>

The sonar sensor emits an ultrasonic ping and listens for its return. By multiplying the 
time it takes to hear the returned ping by the speed of sound, we can compute the distance to an object.

The sonar has four pins:
* `GND`: Ground
* `VCC`: Power
* `TRIG`: Sends a sonar ping when it receives a signal.
* `ECHO`: Listens for a sonar ping.

Wire the sonar as follows:
* Wire `GND` to the `-` column.
* Wire `VCC` to the `+` column.
* Wire `Trig` to Arduino pin 7.
* Wire `Echo` to Arduino pin 6.

<img src="https://hendrix-cs.github.io{{site.baseurl}}/assets/images/WiringDiagram.png" width=500>

Having wired the sonar, we read it as follows:
* Activate the `Trig` pin for 10 microseconds.
* [Listen for a pulse](https://www.arduino.cc/reference/en/language/functions/advanced-io/pulsein/) from the `Echo` pin.
* Multiply the pulse duration by the speed of sound. Divide by 2 to take into account that the pulse traveled the
  distance twice (one way out, one way return).
  
The following function implements this algorithm:
```
//Speed of sound, in mm/us
const float speedOfSound = 0.343;

float readSonar(int trigPin, int echoPin) {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH);
  return duration * speedOfSound / 2.;
}
```

To get a feel for how the sonar works in practice, it is helpful to have a way to view the sonar values.
Since the Arduino lacks a display, we can instruct it to send the values back to the host computer over
its USB cable. The following complete Arduino program, which includes the `readSonar()` function above, 
continually reads the sonar, sending one reading per second back to the host computer. To view the readings,
be sure to select the **Serial Monitor** from the **Tools** menu. It will bring up a window allowing you to view
everything the Arduino sends over the serial port.

```
//Speed of sound, in mm/us
const float speedOfSound = 0.343;

float readSonar(int trigPin, int echoPin) {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH);
  return duration * speedOfSound / 2.;
}

const int TRIGGER = 7;
const int ECHO = 6;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  float reading = readSonar(TRIGGER, ECHO);
  Serial.println(reading);
  delay(1000);
}
```

## Understanding the Sonar

Upload the above program to the Arduino. Grab a book or similar object, and position it in front of the sonar.
* What is the **shortest** distance from the sonar that it reliably detects the object?
* What is the **longest** distance from the sonar that it reliably detects the object?

## Employing the Sonar

Write an Arduino program that enables the robot to drive forward while avoiding obstacles. As you construct
and experiment with the program, work towards answering the following questions:

1. With a two-wheeled robot, three different styles of turns are possible. 
   1. What are the three styles? 
   2. For each style, in what situations is it beneficial? Detrimental? Support your answers with evidence from experimenting with your robot.
2. When creating an obstacle-avoiding robot that uses the sonar, what is the minimum distance at which to begin a safe turn? 
   This value may vary depending upon the turning style; be sure to specify a value for each of the three styles.
3. What numerical metrics are appropriate for quantifying the performance of an obstacle-avoiding robot? 
   1. How are they measured? 
   2. How do they conform with our intuitions?
4. How did you employ your metrics to iteratively create your best obstacle-avoiding robot?
5. Is your best obstacle-avoiding robot *intelligent*? Why or why not?

[Record a video](https://docs.google.com/forms/d/e/1FAIpQLSdOhoNctoprb8qkhFf1LMAhfe7fxLxZL5AhssO90eCUriKZjw/viewform?usp=sf_link) 
of your robot driving around and avoiding obstacles. Include several different shots of the 
robot that highlight its behavior in different situations.

------------------------------------------------------------------------
