#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Stop, Direction, Button, Color

ev3 = EV3Brick()
ev3.screen.clear()
ev3.screen.draw_text(0, 0, "Press a button")

button2str = {
    Button.LEFT:    "Left",
    Button.RIGHT:   "Right",
    Button.CENTER:  "Center",
    Button.UP:      "Up",
    Button.DOWN:    "Down",
    Button.LEFT_UP: "Escape"
}

# Write your program here.
ev3.speaker.beep()
while True:
    pressed = ev3.buttons.pressed()
    if len(pressed) > 0:
        ev3.screen.clear()
        ev3.screen.draw_text(0, 0, button2str[pressed[0]])