from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)

def forward(robot):
    robot.go_forward()
    while not robot.too_close():
        pass
    return left


def left(robot):
    robot.go_left()
    while robot.too_close():
        pass
    return forward
