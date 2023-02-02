import lib

CLOSE = 100
FAR = 500
SPEED = 500

def fuzzy_condition(robot):
    front_close = lib.falling(robot.front_sonar.distance(), CLOSE, FAR)
    left_close  = lib.falling(robot.left_sonar.distance(),  CLOSE, FAR)
    right_close = lib.falling(robot.right_sonar.distance(), CLOSE, FAR)

    left_wheel  = lib.defuzzify(lib.f_or(front_close, left_close),  0, -SPEED)
    right_wheel = lib.defuzzify(lib.f_or(front_close, right_close), 0, -SPEED)

    robot.left.run(left_wheel)
    robot.right.run(right_wheel)