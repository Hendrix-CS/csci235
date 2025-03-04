---
layout: work
type: Module
num: 6
worktitle: Planning
---

## To the frontier

From Module 5, copy over the following files:
* `binary_grid.py`
* `curses_mapper.py`
* `curses_runner.py`
* `map_viewer.py`
* `odometry_math.py`
* `avoid_input.py` as `explorer_input.py`
* `avoid_drive_map.py` as `explorer_drive.py`

Modify `explorer_drive.py` as follows:
* Instead of importing `from avoid_input`, import `from explorer_input`
* Change the constructor call for `sensor_node` as follows:
  * `sensor_node = AvoidInputNode(sys.argv[1], f"{sys.argv[1]}_trajectory_map")`
  
Modify `explorer_input.py` as follows:
* Add a `map_topic` parameter to the constructor for `AvoidInputNode`
* Add instance variables for the `map` and `position`. Initialize both to `None`.
* Add a subscription to this topic (of type `String`), as well as a callback function for it.
  * The callback function should use `eval` on the data from the message to create the
    map dictionary and set `self.map` to it.
* Add imports of the functions `find_goal_heading` and `find_normalized_angle` from `odometry_math`
* Modify the odometry handler as follows:
  * Set `self.position` to the position from the `Odometry` message.
* Modify `avoid()` as follows:
  * If either of `map` or `position` are still `None`, maintain the current implementation
    of setting the target one radian past the `yaw`.
  * Otherwise:
    * Our overall strategy is to aim the robot at a frontier space on the map as close
      as possible to directly behind it. To achieve this strategy:
      * Use `find_frontier()` to find the frontier of the map.
      * Set a target heading of the robot's yaw, rotated by 180 degrees (i.e., pi radians). Use
        `find_normalized_angle()` from `odometry_math` to make sure it is normalized between
        -pi and pi.
      * Loop through the frontier:
        * For each frontier point, use `find_goal_heading()` to see what angle the robot
          would need to turn towards to be aimed to it.
        * Use `find_angle_diff` to see if that frontier point is the closest possible to the target heading.
      * Set `self.target` to the frontier goal heading that is as close as possible.

Test this out to map an area. How does the robot's behavior compare with `avoid_drive_map.py`
from Module 5? What advantages and disadvantages does each approach have?

## Improved Map Viewer

In this verison of `map_viewer.py`, you can use the WASD keys to navigate, seeing the locations
of various points on the map. The cursor may occasionally disappear between rows - this is an
artifact of scaling to the terminal screen.

Replace `main()` with the following:
```
import sys, curses
import traceback

from binary_grid import BinaryGrid
from typing import Dict, Tuple, List
from geometry_msgs.msg import Point

def main(stdscr):
    with open(sys.argv[1]) as map_file:
        map_data = eval(map_file.read())
        display = MapDisplay(2, map_data)

        curses.cbreak()
        stdscr.nodelay(True)
        stdscr.clear()

        stdscr.refresh()
        running = True
        while running:
            try:
                k = stdscr.getch()
                if k != -1:
                    if chr(k) == 'q':
                        running = False
                    if chr(k) == 'w':
                        display.up()
                    if chr(k) == 'a':
                        display.left()
                    if chr(k) == 's':
                        display.down()
                    if chr(k) == 'd':
                        display.right()

                display.show(stdscr)
            except curses.error as e:
                if str(e) != 'no input':
                    stdscr.addstr(0, 0, traceback.format_exc())
                    stdscr.refresh()

        curses.nocbreak()
        curses.echo()
        stdscr.refresh()
```

Then add the `MapDisplay` class:
```
class MapDisplay:
    def __init__(self, start_row: int, map_data: Dict):
        self.free_space = BinaryGrid(map_data['free_space_grid'], map_data['columns'], map_data['rows'])
        self.obstacles = BinaryGrid(map_data['obstacles_grid'], map_data['columns'], map_data['rows'])
        self.highlight = meter2grid(map_data['x'], map_data['y'], map_data)
        self.start_row = start_row
        self.map_data = map_data

    def up(self):
        if self.highlight[1] - 1 >= 0:
            self.highlight = (self.highlight[0], self.highlight[1] - 1)

    def down(self):
        if self.highlight[1] + 1 < self.free_space.rows():
            self.highlight = (self.highlight[0], self.highlight[1] + 1)

    def left(self):
        if self.highlight[0] - 1 >= 0:
            self.highlight = (self.highlight[0] - 1, self.highlight[1])

    def right(self):
        if self.highlight[0] + 1 < self.free_space.cols():
            self.highlight = (self.highlight[0] + 1, self.highlight[1])

    def show(self, stdscr):
        rows, columns = stdscr.getmaxyx()
        rows -= self.start_row + 1
        columns -= 1
        display_rows = rows - self.start_row
        row_grid_slice = round_quotient_up(self.free_space.rows(), display_rows)
        col_grid_slice = round_quotient_up(self.free_space.cols(), columns)

        stdscr.addstr(0, 0, f"{grid2meter(self.highlight[0], self.highlight[1], self.map_data)}{' ' * 20}")

        for row in range(display_rows):
            grid_row = row * self.free_space.rows() // rows
            for col in range(columns):
                c = '.'
                grid_col = col * self.free_space.cols() // columns
                robot_col, robot_row = meter2grid(self.map_data['x'], self.map_data['y'], self.map_data)
                if grid_col <= robot_col <= grid_col + col_grid_slice and grid_row <= robot_row <= grid_row + row_grid_slice:
                    c = 'X'
                else:
                    free_square = self.free_space.any_1_values(grid_col, grid_row, col_grid_slice, row_grid_slice)
                    obstacle_square = self.obstacles.any_1_values(grid_col, grid_row, col_grid_slice, row_grid_slice)
                    frontier_square = has_frontier(self.free_space, self.obstacles, grid_col, grid_row, col_grid_slice, row_grid_slice)
                    if obstacle_square:
                        c = '*'
                    elif frontier_square:
                        c = 'F'
                    elif free_square:
                        c = 'o'
                view_row = row + self.start_row
                if (grid_col, grid_row) == self.highlight:
                    stdscr.addch(view_row, col, c, curses.A_REVERSE)
                else:
                    stdscr.addch(view_row, col, c)
```

And also add a revamped `display_curses()`:

```
def display_curses(stdscr, start_row: int, map_data: Dict):
    d = MapDisplay(start_row, map_data)
    d.show(stdscr)
```

Play around with it a bit using one or more of the maps you developed last time.
What insights about the map can you obtain from using the cursor to observe various
points?

## Navigation

Copy `fuzzy.py` and `goal_fuzzy_input.py` from Module 4. Then modify `goal_fuzzy_input.py`
as follows:
* Remove the `goal_x` and `goal_y` parameters from the constructor.
* Remove any lines of code that reference those parameters.
* Set `self.goal` to `None`.
* Create the following subscription:
  * `self.create_subscription(String, f"{robot_name}_waypoints", self.goal_callback, qos_profile_sensor_data)`
* In `self.goal_callback()`:
  * The `msg.data` field will contain a dictionary. Use `eval()` to unpack it.
  * The dictionary will have two entries:
    * `status` will have either `navigating` or `stopped` as values.
    * `waypoint` will have value `None` if `status` is `stopped`. Otherwise, it will 
      contain a pair of (x, y) coordinates towards which the robot should navigate.
    * If `waypoint` contains (x, y) coordinates, create a `Point()` object (from `geometry_msgs.msg`)
      and set its `x` and `y` instance variables to those (x, y) coordinates. Make `self.goal`
      this `Point()` object.
    * Otherwise, set `self.goal` to `None`.
* Modify the odometry callback as follows:
  * Make the first line `errors = {'left': 0.0, 'right': 0.0, 'distance': 0.0}`
  * Remove the later lines that read `errors = {}` and `errors['left'] = errors['right'] = 0.0`.
  * Write an `if` statement to ensure that all of the error calculations only occur if
    `self.goal` is not `None`.
  * Replace the debugging code with the following:
```
        debug = f"{msg.pose.pose.position}{' ' * 10}"
        debug += f"\ndistance: {errors['distance']:.2f}{' ' * 10}"
        debug += f"\nleft: {errors['left']:.2f}{' ' * 10}"
        debug += f"\nright: {errors['right']:.2f}{' ' * 10}"
        debug += f"\ngoal: {self.goal}"
        self.publish(self.debug, debug)
```
    
Copy `goal_fuzzy_navigator.py` from Module 4 and modify it as follows:
* At the top, add `import subprocess, atexit, time`
* Add a parameter `goal: str` to the constructor, and then add the line `self.goal = goal`
  within it.
* Add the following line in the constructor
  * `self.navigator = self.create_publisher(String, f"{robot_name}_goal", qos_profile_sensor_data)`
* Replace `button_callback()` with the following:
```
    def button_callback(self, msg: InterfaceButtons):
        if msg.button_2.is_pressed:
            msg = String()
            msg.data = self.goal
            self.navigator.publish(msg)
            self.override_stop = False
        elif msg.button_1.is_pressed or msg.button_power.is_pressed:
            self.override_stop = True
```
* Replace `main()` with the following:
```
def main(stdscr):
    cmd = parse_cmd_line_values()
    rclpy.init()
    sensor_node = FuzzyGoalNode(sys.argv[1], cmd['angle_limit'])
    curses_node = CursesNode(sensor_node.debug_topic, 2, stdscr)
    drive_node = FuzzyDriveNode(sys.argv[1], sensor_node.output_topic, cmd['x_limit'], cmd['z_limit'], f"({cmd['goal_x']}, {cmd['goal_y']})")
    run_curses_nodes(stdscr, [drive_node, curses_node, sensor_node])
    rclpy.shutdown()
```
* Replace `if __name__ == '__main__'` with the following:
```
if __name__ == '__main__':
    if len(sys.argv) < 8:
        print("Usage: python3 goal_fuzzy_navigator.py robot_name map_file_name goal_x=value goal_y=value angle_limit=value x_limit=value z_limit=value")
    else:
        robot_name = sys.argv[1] if len(sys.argv) > 1 else "robot_name"
        print(f"Odometry reset:\nros2 service call /{robot_name}/reset_pose irobot_create_msgs/srv/ResetPose\n")  
        input("Type enter once odometry is reset")
        process = subprocess.Popen(['/home/ferrer/bin/navigator_node', sys.argv[1], sys.argv[2]])
        atexit.register(lambda: process.terminate())
        time.sleep(1)
        input("Type enter for robot to start; then push button 2 to begin navigating")        
        curses.wrapper(main)
```

Using `map_viewer.py`, pick some points to navigate to on the map you created earlier. 
How does it perform with its navigation?