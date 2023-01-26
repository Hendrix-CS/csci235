from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color

SPEED = 360
TOO_CLOSE = 200
REFRESH = 2000

def executor(robot, action, condition):
    action(robot)
    while True:
        robot.loops += 1
        update = condition(robot)
        if update:
            action, condition = update
            action(robot)
        if robot.loops > REFRESH:
            robot.loops = 0
            robot.show(action, condition)

class SensorMotor:
    def __init__(self, ev3):
        self.ev3 = ev3
        self.left = Motor(Port.A)
        self.right = Motor(Port.D)
        self.light = ColorSensor(Port.S1)
        self.sonar = UltrasonicSensor(Port.S4)
        self.loops = 0

    def values(self):
        return ["L:" + str(self.left.angle()), 
            "R:" + str(self.right.angle()), 
            "C:" + str(self.light.color()),
            "S:" + str(self.sonar.distance())]

    def show(self, action, condition):
        self.ev3.screen.clear()
        self.ev3.screen.draw_text(0, 0, action.__name__)
        self.ev3.screen.draw_text(0, 16, condition.__name__)
        y = 32
        for value in self.values():
            self.ev3.screen.draw_text(0, y, value)
            y += 16


def go_forward(robot):
    robot.left.run(SPEED)
    robot.right.run(SPEED)

def go_left(robot):
    robot.left.run(-SPEED)
    robot.right.run(SPEED)

def go_back(robot):
    robot.left.run(-SPEED)
    robot.right.run(-SPEED)

def too_close(robot):
    return robot.sonar.distance() < TOO_CLOSE
