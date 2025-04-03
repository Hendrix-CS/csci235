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

