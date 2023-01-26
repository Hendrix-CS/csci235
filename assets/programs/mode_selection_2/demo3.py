import lib

BACK_UP_DIST = 360
TURN_DIST = 140

def check_close(robot):
    if lib.too_close(robot):
        robot.right.reset_angle(BACK_UP_DIST)
        return lib.go_back, check_retreated

def check_retreated(robot):
    if robot.right.angle() <= 0:
        robot.right.reset_angle(0)
        return lib.go_left, check_turned

def check_turned(robot):
    if robot.right.angle() >= TURN_DIST:
        return lib.go_forward, check_close