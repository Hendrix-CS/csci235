---
layout: work
type: Module
num: 2
worktitle: Autonomous Movement
---

## Generating a map

Create a new Python file entitled `map.py`. In that file, create a class 
called `GridMap`. Its constructor should start like the code below:

```
class GridMap:
    def __init__(self, height: int, width: int, num_obstacles: int):
        self.height = height
        self.width = width
```

* What Python data type might we use to store the locations of obstacles?
  * Keep in mind that we should be able to figure out whether a given square 
    is an obstacle very quickly.
* Create a field of that type. Then write a loop to add `num_obstacles` 
  randomly selected positions to it.
* Add a method: `is_open(self, row: int, col: int) -> bool`. It should return
  `True` if the given `row` and `col` are **not** an obstacle but are 
  in-bounds. It should otherwise return `False`.
* Add a method: `draw(self, stdscr)`. It should use `stdscr.addstr` to draw
  a `*` wherever there is an obstacle and a `.` wherever there is no obstacle.
* Write a `curses` `main()` function that does the following:
  * Randomly creates a `GridMap` object using the size of the window and 50
    obstacles. The constructor call should look like this:
```
    height, width = stdscr.getmaxyx()
    grid_map = GridMap(height - 1, width - 1, 50)
```
  * Runs until the user types the `q` key to quit.
  * This function should be based upon what you wrote in the 
    [previous module]({{site.baseurl}}/modules/intro1.html).

## Navigating around obstacles
* Create a copy of `grid_world.py` (`grid_world_2.py`) that you wrote in the 
  [previous module]({{site.baseurl}}/modules/intro1.html).
* In `grid_world_2.py`, complete the following function:
```
def blocked_ahead(pose: RobotPose, grid_map: GridMap) -> bool:
    """
    Returns True if the square ahead of pose is not open.
    """
    # Your code here
```
  * One possible strategy is to copy the `pose` object, move it one square,
    and see if this copied pose location is blocked.
  * To copy an object in Python, `import copy` and pass the object to copy to
    `copy.deepcopy()`
* Modify the implementation of the `w` key to call `blocked_ahead()` before 
  moving. If the robot is blocked, it should stay where it is.
* Run the program and make sure it works as expected.
* Next, complete this function:
```
def right_move(pose: RobotPose, grid_map: GridMap):
    """
    Turns right if blocked; otherwise drives forward.
    """
    # Your code here
```
  * In your solution, be sure to call `blocked_ahead()` as well as using the
    `turn()` and `move()` methods of the `RobotPose` class.
* Having written the `right_move()` function, we will designate the `x` key to 
  indicate that we want the robot to drive autonomously. Whenever the `x` is
  pressed, call `right_move()` to determine how the robot will move.
* Run the program and hold down the `x` key to watch the robot move 
  autonomously. What are some strengths and weaknesses of how it chooses its
  moves?

## A new approach
* Based on the weaknesses you identified with `right_move()`, brainstorm an
  alternative approach. Write a function to implement your idea, and have your
  function run whenever the user presses the `z` key.
* How well did your idea work in comparison with `right_move()`? What were its
  strengths and weaknesses? 
* Brainstorm one last idea for an alternative approach. Use the `c` key to run
  your idea. Again, what were its strengths and weaknesses, and how did it 
  compare to the other two approaches?
