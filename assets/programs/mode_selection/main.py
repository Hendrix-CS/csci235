#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import sonar_demo
import rotation_demo
import demo3

SPEED = 360
TOO_CLOSE = 200

class SensorMotor:
    def __init__(self):
        self.left = Motor(Port.A)
        self.right = Motor(Port.D)
        self.sonar = UltrasonicSensor(Port.S4)

    def too_close(self):
        return self.sonar.distance() < TOO_CLOSE

    def go_forward(self):
        self.left.run(SPEED)
        self.right.run(SPEED)

    def go_left(self):
        self.left.run(-SPEED)
        self.right.run(SPEED)

    def go_back(self):
        self.left.run(-SPEED)
        self.right.run(-SPEED)


def executor(start_mode):
    robot = SensorMotor()
    mode = start_mode
    while True:
        ev3.screen.clear()
        ev3.screen.draw_text(0, 0, mode.__name__)
        mode = mode(robot)


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
ev3.speaker.beep()
ev3.screen.draw_text(0,  0, "L: Sonar Demo")
ev3.screen.draw_text(0, 16, "R: Rotation Demo")
ev3.screen.draw_text(0, 32, "D: 3-Way Demo")

choices = {
    Button.LEFT:  sonar_demo.forward,
    Button.RIGHT: rotation_demo.forward,
    Button.DOWN:  demo3.forward
}

selection = None
while selection is None:
    pressed = ev3.buttons.pressed()
    if len(pressed) > 0:
        selection = choices[pressed[0]]

executor(selection)