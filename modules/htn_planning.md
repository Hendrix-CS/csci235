---
layout: work
type: Module
num: 8
worktitle: HTN Planning
---

# HTN Planning

A sequence of steps for a robot to perform is called a **plan**. A 
**path plan** is concerned only with locations to visit. A **task plan**
involves tasks to be performed at the locations to be visited. In this
module, we will focus on task planning using a Hierarchical Task Network
(HTN) planner. 

An HTN planner is provided the following:
* A description of the world state
* A list of **plan operators**: 
  * Each operator relies upon certain aspects of the state being true 
    (**preconditions**), and transforms the state so that certain things 
    become true and other things become false (**effects**).
  * Operators can be parameterized.
* A list of **plan methods**:
  * Each method describes how a particular goal is broken down into
    a hierarchy of other methods and plan operators.
    
## Plan operators

Answer the following questions in your journal:
* For each of the following plan operators, what might be some preconditions 
  and effects?
  * `move_from_to((0, 0), (0, 1))`
  * `move_from_to((1, 1), (1, 2))`
  * `pick_up(strawberry, (0, 1))`
  * `pick_up(apple, (1, 2))`
  * `put_down(banana, (2, 0))`
  * `put_down(apple, (1, 2))`
* Given the operators `move_from_to()`, `pick_up()`, and `put_down()`,
  in what situations are they **inverses** of each other? Explain your 
  answer in terms of preconditions and effects.
* Why might it be useful to split `move_from_to()` into two separate 
  operators: `turn_to(direction)` and `move_one_step(start)`?
* How would the robot's state representation differ if we split 
`move_from_to()` into those two separate actions?
  
## A starting state

The following Python program contains a state description. Create a new
file called `grid_problems.py` and copy the following code into it:

```
import sys
from pyhop_anytime import State
from pyhop_anytime.grid import Grid, Facing
from pyhop_anytime_examples.grid_package_world import make_grid_planner

probs = [State("Problem_0")]

probs[0].at = (1, 1)
probs[0].facing = Facing.EAST
probs[0].width = 3
probs[0].height = 3
probs[0].grid = Grid(3, 3)
probs[0].capacity = 2
probs[0].holding = []
probs[0].package_locations = [(0, 0), (2, 2)]
probs[0].package_goals = [(2, 0), (0, 2)]
```

The possible operators are:
* `turn_to`
* `move_one_step`
* `pick_up`
* `put_down`

Answer the following questions in your journal:
* For each of the four operators, which elements of the state:
  * Could serve as preconditions?
  * Could be altered by effects?
* What elements of the state will **never** be changed by **any** operators?

## Running a planner

Add the code below to the end of `grid_problems.py`:
```
if __name__ == '__main__':
    planner = make_grid_planner()
    prob_num = 0 if len(sys.argv) == 1 else int(sys.argv[1])
    plans = planner.anyhop_random_tracked(probs[prob_num],
                [('deliver_all_packages_from', probs[prob_num].at)], 3)
    if len(plans) == 0:
        print("Sorry, couldn't find any plans")
    else:
        for operator in plans[-1][0]:
            print(operator)
```

Run the program. Copy the plan it displays into your journal.

Notes:
* The number for `pick_up` and `put_down` references the specific package
  being moved. Each package is named by using consecutive integers starting
  at zero. So in this example there are two packages: Package 0 and Package 1.
* The `move_one_step` method has as a second parameter the robot's expected
  facing.
* The numbers after the `Facing` values represent the change in column and
  row for moving in the direction of that `Facing`.
* This planner is an example of an **anytime planner**. It is given a time
  budget (in this example, one second) and generates plans until the time
  limit is reached. This program grabs the last plan generated from the
  plan list and prints out its operators.
  
Answer the following questions in your journal:
* After each of the 17 steps in the generated plan, what state elements 
  change?
* Are the two packages at their goal locations by the end of the plan?

## Plan execution

Executing a plan requires creating a ROS2 node that checks operator 
preconditions by subscribing to topics and implements operator effects
by publishing to topics. We will build our plan execution node atop
`GridNode` from the [Q-learning module](modules/qlearning.html). The plan
execution algorithm will translate operators into action as follows:
* Before the node starts up, we call the planner to generate a plan.
  * If the planner fails to find a plan, we signal failure and exit, 
    without running the node at all.
* Implementing `turn_to()` can consist of recording the intended action.
  * Note that the coordinate system of the planner uses `NORTH`, `SOUTH`,
    `EAST`, and `WEST`. To which directions for `GridNode` do these 
    correspond?
* To implement `move_one_step()`, use the `update_goal()` method of 
  `GridNode`.
  * When `self.goal_pose` is `None`, the robot doesn't have a current goal;
    this is a way to determine that it got where it was going (or that it
    hasn't started executing the plan yet).
* Because our robots do not have a mechanism to pick things up and put
  things down, a human accompanying the robot will do that. 
* The robot will play one set of notes to request a pick-up and a 
  different set to request a put-down.
* A put-down package will be put aside on a table.
  * Once the package has been put aside, push Button 1.
  * Perhaps a better domain description would treat packages as obstacles,
    but the current domain description does not support this.
* A picked-up package will be placed atop the robot. 
  * Once the package is atop the robot, push Button 2.
* Boolean state variables can indicate whether a pick-up or put-down is
  in progress. 
  * A callback for the buttons can determine completion of the operator.
* We will need to store the plan as part of the state, as well as a 
  variable to indicate the current plan operator we are executing.
* We can monitor our progress using a timer callback.

Create a new program called `run_plan_node.py`. Below is a skeleton of
the program, with parts for you to fill in based on the design above:

```
import sys, curses, traceback
import grid_node, grid
import rclpy
from pyhop_anytime.grid import Facing
from pyhop_anytime_examples.grid_package_world import make_grid_planner
from grid_problems import probs
from irobot_create_msgs.msg import InterfaceButtons
from rclpy.qos import qos_profile_sensor_data


class RunPlanNode(grid_node.GridNode):
    def __init__(self, robot_name: str, plan: list[tuple]):
        super().__init__(robot_name, 1.2, 1.2, 0.4)
        self.create_subscription(InterfaceButtons, f"{robot_name}/interface_buttons", self.button_callback, qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_callback)
        self.plan = plan
        # Add any additional state variables you wish to initialize

    def timer_callback(self):
        # YOUR CODE HERE

    def button_callback(self, buttons: InterfaceButtons):
        # YOUR CODE HERE

    def display(self, stdscr):
        super().display(stdscr)
        # YOUR CODE HERE
        # * Add any other information you wish to display.


def main(stdscr):
    rclpy.init()
    prob_num = int(sys.argv[2])
    planning_time = int(sys.argv[3])
    planner = make_grid_planner()
    plans = planner.anyhop_random_tracked(probs[prob_num], [('deliver_all_packages_from', probs[prob_num].at)], planning_time)
    if len(plans) == 0:
        print("Could not find a plan")
    else:
        best_plan = plans[-1][0]
        node = RunPlanNode(sys.argv[1], best_plan)
        curses.cbreak()
        stdscr.nodelay(True)
        stdscr.clear()
        while node.running():
            try:
                k = stdscr.getch()
                if k != -1:
                    node.process_keystroke(chr(k))
                node.display(stdscr)
                rclpy.spin_once(node, timeout_sec=0.0)
            except curses.error as e:
                if str(e) != 'no input':
                    stdscr.addstr(0, 0, traceback.format_exc())

        rclpy.shutdown()
        node.destroy_node()
        curses.nocbreak()
        curses.echo()
        stdscr.refresh()


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: python3 run_plan_node.py robot_name planning_problem_number seconds_to_plan")
    else:
        curses.wrapper(main)
```

Once you have completed your plan executor, test it with our running example.
* Once the plan executor works, devise two additional scenarios and add them
  to `grid_problems.py`. 
* Test them out and make sure they work as expected.