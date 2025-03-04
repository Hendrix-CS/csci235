---
layout: work
type: Module
num: 6
worktitle: Planning
---

## To the frontier

Have them modify the wanderer to aim for a frontier square in the correct general direction. Then
compare it to the mappability of the last version. This should hopefully be really easy.

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

Play around with it a bit using one or more of the maps you developed last time.

## Navigation



<!--

No time to debug my solution. Dropping...


## Exploration

Make a copy of `avoid_drive_map.py` called `explorer_drive.py` and a copy of
`avoid_input.py` called `explorer_input.py`. Modify them as follows:
* In `AvoidInputNode`, add a subscription to the `trajectory_map` topic to which the 
  `CursesMappingNode` is also subscribed.
* In the callback for that subscription:
  * Unpack the message into a Python dictionary using `eval()`.
  * Store the message in an instance variable set aside for this purpose.
* In `odom_callback()`:
  * In addition to storing the yaw, also store the robot's `position` in an instance
    variable.
* In the `avoid()` method:
  * Call `find_frontier()` from `map_viewer.py` to get a list of frontier points.
  * Traverse the list, finding the frontier point closest to the robot's last 
    recorded position. Use `find_euclidean_distance()` from `odometry_math.py` to help.
  * Save that frontier point as the robot's `target`.
  * If there are no frontier points, the robot should simply stop moving.
    * Publish a suitable message for `DriveNode`.
* Copy `fuzzy.py` from `module4` into `module5`.
* In `odom_callback()`, modify the published message as follows:
  * If the robot does not have a `target`, send a message that `DriveNode` will interpret
    as a signal to drive forward.
  * If the robot does have a target, take inspiration from `FuzzyGoalNode` in `module4`
    to fuzzify the robot's relationship to that target. Publish the fuzzified input. Once 
    the robot is within a few centimeters of the target, clear the target and allow the
    robot to wander once again.
* In `DriveNode`, rework `input_callback()` to either drive straight or to defuzzify the
  fuzzified input. Publish `avoid` whenever the robot transitions from going forward.
  Continue publishing `avoid` as long as the `x` velocity is zero. Once it is non-zero,
  publish `clear`.

Once your explorer is complete, map two areas. Save the maps and submit them along with
the rest of your materials for this module.
-->