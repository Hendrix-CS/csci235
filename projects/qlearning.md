---
layout: work
type: Project
num: 4
worktitle: Reinforcement Learning of Behavior
---

## Sample Program

* [`main.py`]({{site.baseurl}}/assets/programs/qlearning/main.py)
* [`lib.py`]({{site.baseurl}}/assets/programs/qlearning/lib.py)
* [`robot.py`]({{site.baseurl}}/assets/programs/qlearning/robot.py)
* [`forward_back.py`]({{site.baseurl}}/assets/programs/qlearning/forward_back.py)
* [`leave_alone.py`]({{site.baseurl}}/assets/programs/qlearning/leave_alone.py)

## Q-Learning Parameters
* `pause_ms` - Number of milliseconds from the start of an action
until the state is updated.
* `actions` - List of possible actions
* `state_func` - Encodes sensor values into discrete states
* `reward_func` - Determines the robot's reward
* `num_states` - Number of distinct states assigned by `state_func`
* `target_visits` - Number of visits to a state/action pair required
to complete the exploration phase
* `epsilon` - Probability of exploring during the exploitation phase
* `discount` - Ranges from 0.0 to 1.0, representing how much anticipated
future rewards are used in updating Q values
* `rate_constant` - Determines how quickly the learning rate decreases
* `max_steps` - Number of steps the robot will run until it stops

## Assignment

* Implement the following programs: 

### Learning to Avoid Obstacles

For this task, your robot's goals are:
* Drive forward as much as possible
* Avoid hitting objects

Develop a first learning robot as follows:
* Include [`lib.py`]({{site.baseurl}}/assets/programs/qlearning/lib.py) in 
  your project without any modifications.
* Include [`robot.py`]({{site.baseurl}}/assets/programs/qlearning/robot.py)
  in your project. 
  * Modify it as needed to correspond to your robot's configuration.
  * Add one or more functions to represent possible robot turns.
* Base your solution on [`forward_back.py`]({{site.baseurl}}/assets/programs/qlearning/forward_back.py).
  * Replace the `robot.go_back` function with one of your turn functions.
* Write a `main.py` that sets everything up and calls `lib.run_q()`
* Run three experiments with the robot in the same obstacle-strewn 
  area with the same starting point.
  * For each experiment, run the robot for 200 iterations.
  * Record the total reward the robot earns for each experiment.

For your second learning robot:
* Replace the turn function you employed on the previous step with 
  a different turn function.
* Again, run three experiments for 200 iterations each, recording
  the total reward the robot earns.
  
For your third learning robot:
* Replace the `BUMPED` state with two distinct alternatives:
  * `LEFT_BUMP`
  * `RIGHT_BUMP`
* Modify `find_state()` to return the correct state for each of
  the given bump sensor configurations.
* Update `num_states` in the parameters to be `4`.
* Modify `reward()` accordingly as well.
* The `actions` should include a forward move, a left turn, and a 
  right turn. The forward move should be first.
* Consider setting `epsilon` to a positive value, perhaps `0.05` or `0.1`.
  It is easy for the robot to get "stuck" on a suboptimal action in this
  task.
* Run three experiments for 200 iterations each, recording the total
  reward the robot earns.

For your fourth learning robot:
* Use a single `BUMPED` state, as you did in the first two variations.
* Replace the `OBJECT` state, modifying `find_state()` accordingly, with the following three states:
  * `CLOSE_OBJECT` (0-200 mm)
  * `MID_OBJECT` (200-400 mm)
  * `FAR_OBJECT` (400-600 mm)
* Update `num_states` in the parameters to be `5`.
* `actions` should include a forward move and one turn.
* Run three experiments for 200 iterations each, recording the total
  reward the robot earns.
  
For your fifth learning robot:
* Put together any combination of ideas, either from the above variations or
  your own new ideas, that you hypothesize will result 
  in an effective obstacle avoider.
* Draw whatever inspiration you like from the previous four versions.
* Run three experiments for 200 iterations each, recording the total
  reward the robot earns.

### Your choice
* Devise a task for your robot that is amenable to being solved by 
  using reinforcement learning.
* You will need to create a `find_state()` function and a `reward()`
  function suitable for the task.
* You will also need to select which actions will be available for
  the robot to employ.
* The color sensor can be a valuable source of additional sensor inputs
  to create a more interesting task.
* Create at least two variations of the task parameters, and compare
  across three experiments the total reward earned with each variation.

## Questions
1. What were the total rewards earned for each of your obstacle-avoiding
robots?
2. What type of turn was most effective for your robot, in terms of the total
reward? Was the total reward a good metric for the effectiveness of
the turn?
3. What impact did you observe from considering the bump sensors 
separately?
4. What impact did you observe from considering different sonar 
distances separately?
5. In general, is the total reward a useful metric for the effectiveness
of an obstacle avoider? If not, should the reward function be modified?
6. How suitable was Q-Learning for your chosen task? Support your answer
with observations of its performance, including its total reward.
7. What variations of parameters did you attempt for your chosen task?
What was the impact of those variations?
8. Generally speaking, when do you think Q-Learning would be preferable
to hand-coding a robot's behavior? When would hand-coding behavior
be preferable? Support your answer with observations from your
experiments.
9. In comparison to our earlier implementations, to what extent does a 
Q-Learning robot exhibit intelligence? Support your answer with observations
of your experiments.


