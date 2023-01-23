#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
import pybricks.nxtdevices

ev3 = EV3Brick()
ev3.screen.clear()
ev3.speaker.beep()

REFRESH = 2000

bumper = TouchSensor(Port.S1)
nxtsonar1 = pybricks.nxtdevices.UltrasonicSensor(Port.S2)
nxtsonar2 = pybricks.nxtdevices.UltrasonicSensor(Port.S3)
ev3sonar = UltrasonicSensor(Port.S4)

update_count = 0
while True:
    if update_count > REFRESH:
        update_count = 0
        ev3.screen.clear()
        ev3.screen.draw_text(0, 0,  "S1: " + str(bumper.pressed()))
        ev3.screen.draw_text(0, 16, "S2: " + str(nxtsonar1.distance()))
        ev3.screen.draw_text(0, 32, "S3: " + str(nxtsonar2.distance()))
        ev3.screen.draw_text(0, 48, "S4: " + str(ev3sonar.distance()))
    else:
        update_count += 1