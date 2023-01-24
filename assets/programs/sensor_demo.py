#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)

ev3 = EV3Brick()
ev3.screen.clear()
ev3.speaker.beep()

REFRESH = 2000

bumper = TouchSensor(Port.S1)
ev3sonar = UltrasonicSensor(Port.S4)

update_count = 0
while True:
    if update_count > REFRESH:
        update_count = 0
        ev3.screen.clear()
        ev3.screen.draw_text(0, 0,  "S1: " + str(bumper.pressed()))
        ev3.screen.draw_text(0, 16, "S4: " + str(ev3sonar.distance()))
    else:
        update_count += 1
