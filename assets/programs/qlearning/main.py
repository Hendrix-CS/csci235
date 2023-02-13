#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import lib, robot, forward_back, leave_alone

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
ev3.speaker.beep()
ev3.screen.draw_text(0, 0,  "L: Active")
ev3.screen.draw_text(0, 16, "R: Calm")

choices = {
    Button.LEFT: forward_back.params,
    Button.RIGHT: leave_alone.params
}

bot = robot.SensorMotor(ev3)
selection = None
while selection is None:
    pressed = ev3.buttons.pressed()
    if len(pressed) > 0 and pressed[0] in choices:
        selection = choices[pressed[0]]

lib.run_q(bot, selection)
