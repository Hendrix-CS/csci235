#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)

ev3 = EV3Brick()
ev3.screen.clear()
ev3.screen.draw_text(0, 0, "Ready!")
ev3.speaker.beep()

left = Motor(Port.A)
right = Motor(Port.D)

SPEED = 360
button2speeds = {
    Button.LEFT:    (-SPEED,  SPEED),
    Button.RIGHT:   ( SPEED, -SPEED),
    Button.UP:      ( SPEED,  SPEED),
    Button.DOWN:    (-SPEED, -SPEED),
    Button.CENTER:  (0, 0),
    Button.LEFT_UP: (0, 0)
}

while True:
    pressed = ev3.buttons.pressed()
    if len(pressed) > 0:
        speed_left, speed_right = button2speeds[pressed[0]]
        left.run(speed_left)
        right.run(speed_right)