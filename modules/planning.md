---
layout: work
type: Module
num: 6
worktitle: Planning
---

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
