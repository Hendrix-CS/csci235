import lib

def check_close(robot):
    if lib.too_close(robot):
        return lib.go_left, check_clear

def check_clear(robot):
    if not lib.too_close(robot):
        return lib.go_forward, check_close