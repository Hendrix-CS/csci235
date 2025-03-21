---
layout: work
type: Module
num: 5
worktitle: Exploration
---


## Obstacle-avoiding robot

Create a new folder called `module5`. Copy `odometry_math.py` from `module3` and 
`curses_runner.py` from `module2` into it.

Let's examine a simple obstacle-avoiding robot program. First, examine the 
input-encoding node. Create a file called `avoid_input.py`, and copy and 
paste the code below into it.

```
from typing import Any

from odometry_math import find_roll_pitch_yaw, find_angle_diff
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from irobot_create_msgs.msg import HazardDetectionVector

def publish_string(publisher: Publisher, data: Any):
    output = String()
    output.data = f"{data}"
    publisher.publish(output)


class AvoidInputNode(Node):
    def __init__(self, robot_name: str):
        super().__init__(f'AvoidInputNode_{robot_name}')
        self.create_subscription(HazardDetectionVector, f"{robot_name}/hazard_detection", self.hazard_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, f"{robot_name}/odom", self.odom_callback, qos_profile_sensor_data)
        self.output_topic = f'{robot_name}_ir_blocked'
        self.output = self.create_publisher(String, self.output_topic, qos_profile_sensor_data)
        self.target = None
        self.yaw = 0.0

    def avoiding(self) -> bool:
        return self.target is not None
    
    def avoid(self):
        self.target = self.yaw + 1.0
    
    def avoid_no_longer(self):
        self.target = None

    def hazard_callback(self, msg: HazardDetectionVector):
        for d in msg.detections:
            name = d.header.frame_id
            if 'left' in name or 'front' in name or 'right' in name:
                self.avoid()               

    def odom_callback(self, msg: Odometry):
        r, p, self.yaw = find_roll_pitch_yaw(msg.pose.pose.orientation)

        if self.avoiding():
            gap = find_angle_diff(self.yaw, self.target)
            if abs(gap) < math.pi / 32:
                self.avoid_no_longer()
        
        msg = {'pending': self.avoiding()}
        publish_string(self.output, msg)
```

Copy the following code into a new file, `avoid_drive.py`:

```
import sys, curses
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

from avoid_input import AvoidInputNode, publish_string
from curses_runner import run_curses_nodes, CursesNode


class DriveNode(Node):
    def __init__(self, robot_name: str, input_topic: str, x_limit: float, z_limit: float):
        super().__init__(f"DriveNode_{robot_name}")
        self.avoid_topic = f"{robot_name}_map_avoiding"
        self.avoid_publisher = self.create_publisher(String, self.avoid_topic, qos_profile_sensor_data)
        self.x_limit = x_limit
        self.z_limit = z_limit
        self.motors = self.create_publisher(TwistStamped, f"{robot_name}/cmd_vel_stamped", qos_profile_sensor_data)
        self.create_subscription(String, input_topic, self.input_callback, qos_profile_sensor_data)
        self.last_turn_left = False

    def input_callback(self, msg: String):
        t = self.make_twist()
        avoid_msg = "clear"
        sensor_values = eval(msg.data)
        if sensor_values['pending']:
            t.twist.angular.z = self.z_limit
            avoid_msg = "avoid"
        else:
            t.twist.linear.x = self.x_limit
        publish_string(self.avoid_publisher, avoid_msg)
        self.motors.publish(t)
            
    def make_twist(self) -> TwistStamped:
        t = TwistStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = self.get_clock().now().to_msg()
        return t


def parse_cmd_line_values() -> Dict[str,float]:
    parsed = {}
    for arg in sys.argv:
        if '=' in arg:
            parts = arg.split('=')
            parsed[parts[0]] = float(parts[1])
    return parsed


def main(stdscr):
    rclpy.init()
    sensor_node = AvoidInputNode(sys.argv[1])
    curses_node = CursesNode(sensor_node.output_topic, 2, stdscr)
    drive_node = DriveNode(sys.argv[1], sensor_node.output_topic, 0.3, 1.0)
    run_curses_nodes(stdscr, [drive_node, curses_node, sensor_node])
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 avoid_drive.py robot_name")
    else:
        curses.wrapper(main)
```

1. What will an `AvoidInputNode` object do when spun?
2. What will a `DriveNode` object do when spun?
3. What might be some reasons why a `DriveNode` would publish whether it is driving forward or
   avoiding an obstacle?
4. Run the program. What are its qualities and limitations compared with prior obstacle-avoiding
programs you have written?

## Maps

The Python dictionary below encodes a map. Create a file called `example_map` and copy and
paste this into it:
```
{'x': 0.7990859150886536, 'y': -0.7146968245506287, 'theta': 0.28728632915795493, 
'columns': 50, 'rows': 40, 'meters_per_cell': 0.1, 
'free_space_grid': [0, 279223176896970752, 34909494181888, 8573157376, 
                    17293822569104800768, 1152851135862669567, 281466386776064, 
                    68718428160, 18302628885650472896, 2305825417027650559, 
                    140735340871680, 8589803520, 18410715276690849776, 4499201580859399, 
                    1099377410048, 13835058055416377344, 9223090561878073343, 
                    562915593682944, 34355544064, 2088960, 70931694131085439, 
                    1924145348608, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
'obstacles_grid': [0, 2251799813685248, 17592186044416, 0, 0, 0, 0, 0, 0, 0, 
                   70368744177664, 0, 0, 0, 0, 0, 4096, 0, 0, 4096, 0, 1099511627776, 
                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}
```

Our maps are encoded as follows:
* The robot's current `x`, `y`, and `theta` coordinates, derived from the most 
  recent odometry message.
* The floating-point coordinates are mapped onto a grid. We store the number of 
  columns and rows in that grid, as well as the number of meters represented by
  each grid cell.
* Each grid cell value is either a zero or one. This enables a single integer to 
  store the values of many grid cells. In this implementation, each integer stores
  64 grid cell values.
* We maintain one grid of locations where the robot was able to move freely.
* We maintain a second grid of locations where the robot encountered an obstacle.

Create a file called `binary_grid.py` and copy and paste the following code into it:
```
from typing import List, Tuple

class BinaryGrid:
    def __init__(self, words: List[int], columns: int, rows: int, width:int=64):
        self.grid = [[bit_value(words, y * columns + x, width) for x in range(columns)] 
                     for y in range(rows)]
        
    def rows(self) -> int:
        return len(self.grid)
    
    def cols(self) -> int:
        return len(self.grid[0]) if self.rows() > 0 else 0
    
    def any_1_values(self, col: int, row: int, col_slice: int, row_slice: int) -> bool:
        for r in range(row, row + row_slice):
            for c in range(col, col + col_slice):
                if self.is_set(c, r):
                    return True
        return False
    
    def in_bounds(self, col: int, row: int) -> bool:
        return row >= 0 and col >= 0 and row < len(self.grid) and col < len(self.grid[0])
    
    def is_set(self, col: int, row: int) -> bool:
        return self.in_bounds(col, row) and self.grid[row][col] == 1
    
    def manhattan_neighbors(self, col: int, row: int) -> List[Tuple[int, int]]:
        result = []
        for drow, dcol in [(-1, 0), (1, 0), (0, 1), (0, -1)]:
            nrow = row + drow
            ncol = col + dcol
            if self.in_bounds(ncol, nrow):
                result.append((ncol, nrow))
        return result
    
    def all_columns_rows(self) -> List[Tuple[int, int]]:
        return [(col, row) for col in range(self.cols()) for row in range(self.rows())]

def bit_value(words: List[int], bit: int, width: int) -> int:
    i = bit // width
    m = bit % width
    return 1 if ((1 << m) & words[i]) > 0 else 0
```

The above program decodes a list of integers into a binary grid. The next program, 
`map_viewer.py`, displays a map for us:

```
import sys, curses
import traceback

from binary_grid import BinaryGrid
from typing import Dict, Tuple, List
from geometry_msgs.msg import Point

def main(stdscr):
    first_line = 2
    with open(sys.argv[1]) as map_file:
        map_data = eval(map_file.read())

        curses.cbreak()
        stdscr.nodelay(True)
        stdscr.clear()

        stdscr.refresh()
        running = True
        while running:
            try:
                k = stdscr.getch()
                if k != -1 and chr(k) == 'q':
                    running = False            

                display_curses(stdscr, first_line, map_data)
            except curses.error as e:
                if str(e) != 'no input':
                    stdscr.addstr(0, 0, traceback.format_exc())
                    stdscr.refresh()

        curses.nocbreak()
        curses.echo()
        stdscr.refresh()


def display_curses(stdscr, start_row: int, map_data: Dict):
    free_space = BinaryGrid(map_data['free_space_grid'], map_data['columns'], map_data['rows'])
    obstacles = BinaryGrid(map_data['obstacles_grid'], map_data['columns'], map_data['rows'])

    rows, columns = stdscr.getmaxyx()
    rows -= 2
    columns -= 2
    display_rows = rows - start_row
    row_grid_slice = round_quotient_up(free_space.rows(), display_rows)
    col_grid_slice = round_quotient_up(free_space.cols(), columns)
    for row in range(rows):
        grid_row = row * free_space.rows() // rows
        for col in range(columns):
            grid_col = col * free_space.cols() // columns
            robot_col, robot_row = meter2grid(map_data['x'], map_data['y'], map_data)
            if grid_col <= robot_col <= grid_col + col_grid_slice and grid_row <= robot_row <= grid_row + row_grid_slice:
                c = 'X'
            else:
                free_square = free_space.any_1_values(grid_col, grid_row, col_grid_slice, row_grid_slice)
                obstacle_square = obstacles.any_1_values(grid_col, grid_row, col_grid_slice, row_grid_slice)
                c = '.'
                if obstacle_square:
                    c = '*'
                elif free_square:
                    c = 'o'
            stdscr.addstr(row + start_row, col, c)


def meter2grid(x: float, y: float, map_data: Dict) -> Tuple[int,int]:
    col = int(x / map_data['meters_per_cell']) + map_data['columns'] // 2
    row = int(y / map_data['meters_per_cell']) + map_data['rows'] // 2
    return col, row


def grid2meter(col: int, row: int, map_data: Dict) -> Tuple[float, float]:
    x = (col - map_data['columns'] // 2) * map_data['meters_per_cell']
    y = (row - map_data['rows'] // 2) * map_data['meters_per_cell']
    return x, y


def round_quotient_up(dividend: int, divisor: int) -> int:
    return dividend // divisor + (dividend % divisor > 0)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 map_viewer.py map_file")
    else:
        curses.wrapper(main)
```

1. Run `map_viewer.py` with `example_map`. What does the image look like? 
2. Examine the code. What do the symbols `.`, `o`, `X`, and `*` each designate?

## Finding the frontier

One valuable use of a map is to find new places to explore. One possible suggestion is
to look for locations that are on the edge of unexplored space but not blocked by 
obstacles or the edge of the map. We will call these **frontier** locations.

First, add the code below to `map_viewer.py`:
```
# Replace display_curses() with the following:
def display_curses(stdscr, start_row: int, map_data: Dict):
    free_space = BinaryGrid(map_data['free_space_grid'], map_data['columns'], map_data['rows'])
    obstacles = BinaryGrid(map_data['obstacles_grid'], map_data['columns'], map_data['rows'])

    rows, columns = stdscr.getmaxyx()
    rows -= 2
    columns -= 2
    display_rows = rows - start_row
    row_grid_slice = round_quotient_up(free_space.rows(), display_rows)
    col_grid_slice = round_quotient_up(free_space.cols(), columns)
    for row in range(rows):
        grid_row = row * free_space.rows() // rows
        for col in range(columns):
            grid_col = col * free_space.cols() // columns
            robot_col, robot_row = meter2grid(map_data['x'], map_data['y'], map_data)
            if grid_col <= robot_col <= grid_col + col_grid_slice and grid_row <= robot_row <= grid_row + row_grid_slice:
                c = 'X'
            else:
                free_square = free_space.any_1_values(grid_col, grid_row, col_grid_slice, row_grid_slice)
                obstacle_square = obstacles.any_1_values(grid_col, grid_row, col_grid_slice, row_grid_slice)
                frontier_square = has_frontier(free_space, obstacles, grid_col, grid_row, col_grid_slice, row_grid_slice)
                c = '.'
                if obstacle_square:
                    c = '*'
                elif frontier_square:
                    c = 'F'
                elif free_square:
                    c = 'o'
            stdscr.addstr(row + start_row, col, c)


def is_frontier_point(free_space: BinaryGrid, obstacles: BinaryGrid, col: int, row: int) -> bool:
    # YOUR CODE HERE

                    
def has_frontier(free_space: BinaryGrid, obstacles: BinaryGrid, col: int, row: int, col_grid_slice: int, row_grid_slice: int) -> bool:
    for r in range(row, row + row_grid_slice):
        for c in range(col, col + col_grid_slice):
            if is_frontier_point(free_space, obstacles, c, r):
                return True
    return False


def find_frontier(map_data: Dict) -> List[Point]:
    free_space = BinaryGrid(map_data['free_space_grid'], map_data['columns'], map_data['rows'])
    obstacles = BinaryGrid(map_data['obstacles_grid'], map_data['columns'], map_data['rows'])
    frontier = []
    for col, row in free_space.all_columns_rows():
        if is_frontier_point(free_space, obstacles, col, row):
            p = Point()
            p.x, p.y = grid2meter(col, row, map_data)
            frontier.append(p)
    return frontier         
    
```

1. Given the way our maps are defined, how would you define whether a given grid location
is a frontier?
2. Complete the implementation of `is_frontier_point()` following your answer to the 
previous question.
3. Test your modified `map_viewer.py` and make sure it displays the frontiers properly.

## Testing mapper_node

Before continuing, check with Dr. Ferrer to make sure `mapper_node` is installed
on your robot. Then open two shells. In one shell, type the following on the command line, 
substituting the name of your robot for `robot_name`:

```
mapper_node robot_name
```

Then, in the other shell, type the following:
```
ros2 topic list
```

Towards the end of the list, you should see something like this:
```
robot_name_trajectory_map
```

Then, type this, using the exact string it printed above for your `trajectory_map`:
```
ros2 topic echo robot_name_trajectory_map
```

You should see something like this:
```
data: "{ 'x': 1.2440211772918701, \n'y': -0.18090949952602386, \n'theta': -2.002969721829994, \n'columns': 30, \n'rows': 30, \n'meters_per_c..."
---
data: "{ 'x': 1.2440211772918701, \n'y': -0.18090949952602386, \n'theta': -2.002969721829994, \n'columns': 30, \n'rows': 30, \n'meters_per_c..."
---
data: "{ 'x': 1.2440211772918701, \n'y': -0.18090949952602386, \n'theta': -2.002969721829994, \n'columns': 30, \n'rows': 30, \n'meters_per_c..."
---
data: "{ 'x': 1.2440211772918701, \n'y': -0.18090949952602386, \n'theta': -2.002969721829994, \n'columns': 30, \n'rows': 30, \n'meters_per_c..."
---
```

## <a name="avoid_drive_map">Live mapping</a>

Copy and paste the following code into a new file called `curses_mapper.py`:
```
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from map_viewer import display_curses

class CursesMappingNode(Node):
    def __init__(self, map_topic: str, first_line: int, stdscr):
        super().__init__(f"CursesMappingNode_{map_topic}")
        self.create_subscription(String, map_topic, self.callback, qos_profile_sensor_data)
        self.first_line = first_line
        self.stdscr = stdscr
        self.msg = {}

    def callback(self, msg: String):
        self.msg = eval(msg.data)
        display_curses(self.stdscr, 2, self.msg)
```

Next, make a copy of `avoid_drive.py` called `avoid_drive_map.py`, and modify the 
copy as follows:

```
# Add these import lines
import subprocess, atexit, time, datetime
from curses_mapper import CursesMappingNode

# Replace main() with the following:
def main(stdscr):
    rclpy.init()
    sensor_node = AvoidInputNode(sys.argv[1])
    curses_node = CursesMappingNode(f"{sys.argv[1]}_trajectory_map", 2, stdscr)
    drive_node = DriveNode(sys.argv[1], sensor_node.output_topic, 0.3, 1.0)
    run_curses_nodes(stdscr, [drive_node, curses_node, sensor_node])
    output_filename = f"map_{datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')}"
    with open(output_filename, 'w') as map_out:
        map_out.write(f"{curses_node.msg}\n")
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 avoid_drive_map.py robot_name [-width=value] [-height=value]")
    else:
        cmd = parse_cmd_line_values()
        width = cmd.get("-width", 3.0)
        height = cmd.get("-height", 3.0)
        print("To reset odometry:")
        print(f"ros2 service call /{sys.argv[1]}/reset_pose irobot_create_msgs/srv/ResetPose\n")
        input("Type enter once odometry is reset")
        process = subprocess.Popen(['/home/robotics/bin/mapper_node', sys.argv[1], f"-dim={width},{height}"])
        atexit.register(lambda: process.terminate())
        time.sleep(1)
        input("Type enter for robot to start")
        curses.wrapper(main)
```

1. Test the modified program. What do you observe in the terminal window as the robot moves?
2. Select an area to map. Let the robot drive around for a while. 
3. Stop the robot. Then view the map with `map_viewer.py`. What are some interesting aspects
   of the map?
4. Select two additional areas to map. Small rooms or sections of hallway are suitable ideas. 
Let the robot drive until it has largely covered the area. How long did the robot have to 
run to map out each area? What relationship do you see between the area covered and the
time necessary to map it?

Submit your maps along with the rest of your materials for this module.
