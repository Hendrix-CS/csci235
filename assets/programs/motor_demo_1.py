#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)

ev3 = EV3Brick()
ev3.screen.clear()
ev3.speaker.beep()

left = Motor(Port.A)
right = Motor(Port.D)

SPEED = 360
REFRESH = 10000

left.run(SPEED)
right.run(SPEED)
update_count = 0
while True:
    if update_count > REFRESH:
        update_count = 0
        ev3.screen.clear()
        ev3.screen.draw_text(0, 0, "Left:  " + str(left.angle()))
        ev3.screen.draw_text(0, 16, "Right: " + str(right.angle()))
    else:
        update_count += 1