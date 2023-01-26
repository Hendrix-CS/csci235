import lib

BACK_UP_DIST = 360

def check_close(robot):
    if lib.too_close(robot):
        robot.right.reset_angle(BACK_UP_DIST)
        return lib.go_back, check_retreated

def check_retreated(robot):
    if robot.right.angle() <= 0:
        return lib.go_forward, check_close