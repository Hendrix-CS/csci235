import lib

MODERATE = 270
SLOW = 180

def scoot_back(robot):
    robot.left.run(-MODERATE)
    robot.right.run(-MODERATE)

def careful_back(robot):
    robot.left.run(-SLOW)
    robot.right.run(-SLOW)

def check_distance(robot):
    distance = robot.sonar.distance()
    if distance < 200:
        return lib.go_back, check_distance
    elif distance < 400:
        return scoot_back, check_distance
    elif distance < 600:
        return careful_back, check_distance
    else:
        return lib.go_forward, check_distance
