#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()

import rotation_demo
import sonar_demo
import demo3
import multi_sonar
import lib

# Write your program here.
ev3.speaker.beep()

ev3.screen.draw_text(0,  0, "L: Sonar Demo")
ev3.screen.draw_text(0, 16, "R: Rotation Demo")
ev3.screen.draw_text(0, 32, "D: 3-Way Demo")
ev3.screen.draw_text(0, 48, "U: Multi Sonar")

choices = {
    Button.LEFT:  sonar_demo.check_close,
    Button.RIGHT: rotation_demo.check_close,
    Button.DOWN:  demo3.check_close,
    Button.UP:    multi_sonar.check_distance,
}

selection = None
while selection is None:
    pressed = ev3.buttons.pressed()
    if len(pressed) > 0:
        selection = choices[pressed[0]]

lib.executor(lib.SensorMotor(ev3), lib.go_forward, selection)