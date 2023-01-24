from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)

BACK_UP_DIST = 360

def forward(robot):
    robot.go_forward()
    while not robot.too_close():
        pass
    return back_up


def back_up(robot):
    robot.right.reset_angle(BACK_UP_DIST)
    robot.go_back()
    while robot.right.angle() > 0:
        pass
    return forward
