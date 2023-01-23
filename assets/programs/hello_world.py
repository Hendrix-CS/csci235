#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick

ev3 = EV3Brick()
ev3.speaker.beep()
ev3.screen.draw_text(0, 0, "Hello, world!")
while True:
    pass