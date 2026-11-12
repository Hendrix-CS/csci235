---
layout: work
type: Module
num: 12
worktitle: Image-Driven Rewards
---

# Learning to Patrol

1. Select an area for your robot to patrol.
2. Divide the are into grid cells, as we did in the
[q-learning module](modules/qlearning.html).
3. Create a set of labeled images using `photographer`, with some images
labeled `in_bounds` and other images labeled `out_of_bounds`. Position the
robot to take the pictures from grid cells appropriate to the labels.
4. Make a copy of `qnode.py` called `qnode_patrol.py`. Modify `qnode_patrol.py`
so that rewards are determined by the most recently received image label,
with `in_bounds` labels receiving a significant positive reward and 
`out_of_bounds` labels receiving a significant negative reward.
5. Determine a set of q-learning parameters to use for the first experiment.
Run the experiment. How did the robot perform? Where would you give credit
or blame to your parameter selections?
6. Create one variation of q-learning parameters for the second experiment.
Run the experiment. How did the robot perform? Where would you give credit
or blame to your parameter selections?
7. Create a final variation of q-learning parameters for the third 
experiment. Run the experiment. How did the robot perform? Where would you give credit
or blame to your parameter selections?
8. Assess the utility of image-guided q-learning for the purpose of this
task. How did it perform compared to the state machine version?