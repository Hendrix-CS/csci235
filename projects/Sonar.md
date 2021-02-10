---
layout: work
type: Project
num: 4
worktitle: Sonar
---

## Sonar

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

------------------------------------------------------------------------
