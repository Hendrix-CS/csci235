---
layout: work
type: Module
num: 7
worktitle: Reinforcement Learning
---

<!-- Concept 

Q-learning states are squares based on odometry.
Positive rewards for low IR values.
Small positive reward for maintaining the current direction of movement.
Negative rewards for high IR values.
Negative rewards for hitting things.

One could then path-plan by using BFS or A* on the Q-states.

-->

## Reinforcement Learning

**Reinforcement learning** is a technique for teaching a robot how to act
based on its experiences. The robot receives **rewards** for having 
certain experiences. Over time, it learns to select actions that will 
maximize its rewards.

The specific reinforcement learning algorithm we will employ is 
**Q-learning**. In Q-learning, the robot has a table of states and actions.
For every pair of state and action, the table stores the **expected reward**
of performing that action in that state. The goal of the Q-learning 
algorithm is to employ experience to calculate the expected reward.

For the first set of exercises, consider the following states and actions:

### States

|        | Left | Center | Right |
| ---    | ---- | -----  | ----- |
| Up     | A    | B      | C     |
| Center | D    | E      | F     |
| Down   | G    | H      | I     |

### Actions

* North
  * From `D`, `E`, or `F`, go to `A`, `B`, or `C`
  * From `G`, `H`, or `I`, go to `D`, `E`, or `F`
* South
  * From `A`, `B`, or `C`, go to `D`, `E`, or `F`
  * From `D`, `E`, or `F`, go to `G`, `H`, or `I`
* East
  * From `A`, `D`, or `G`, go to `B`, `E`, or `H`
  * From `B`, `E`, or `H`, go to `C`, `F`, or `I`
* West
  * From `B`, `E`, or `H`, go to `A`, `D`, or `G`
  * From `C`, `F`, or `I`, go to `B`, `E`, or `H`

### Rewards
* In states `G`, `H`, and `B`, the robot receives a reward of 5.
* In states `A`, `F`, and `I`, the robot receives a reward of -4.
* In states `C`, `D`, and `E`, the robot receives a reward of zero.

### Example
* Imagine the robot starts in state `E` and takes the following actions:
  * North, West, South, South, East, East, North, North, South, East, East, South, West, East
* At each step, record the state and the **current** and **total** rewards the robot has
earned.
* For each state:
  * How many times did the robot visit the state?
  * What is the **best** action the robot could take from that state, in 
    terms of maximizing rewards?
  * What is the **worst** action the robot could take from that state?
* When visiting a state, there may be a trade-off between immediate rewards
and future rewards. Among these nine states, which ones have the most striking
trade-off between immediate and future rewards? Why?
* Should the expected reward for taking an action in a state depend more
on the immediate or future reward? Why or why not? 
* What might be a way of balancing immediate and future rewards?

## Q-Learning

In Q-Learning, we update expected rewards using the following formula:
* discount * self.q[new_state][self.best_action(new_state)] + reward

<!-- From here
* Go over formula
* Go over learning rate
* Have them do some hand-calculation
* Then do an implementation with Button 1 as positive reward and 
  Button 2 as negative reward.
-->