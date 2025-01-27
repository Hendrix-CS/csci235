---
layout: work
type: Project
num: 2
worktitle: Symbolic AI Planning
---

## HTN Planning

For this project, we will use the [Anytime Pyhop](https://github.com/gjf2a/pyhop_anytime) planner.

Pyhop is a hierarchical task network (HTN) planner. To use an HTN planner, one must specify the following:

* **State**: Complete description of the current world state. In Pyhop, you can use an arbitrary Python data structure to describe the state.
* **Operators**: Each operator describes a state transformation. Operators can optionally include preconditions. If a precondition is not met, the operator will fail. In Pyhop, you will write one Python function for each operator.
* **Methods**: Methods encode a planning algorithm that decomposes a task into operators and other methods. In Pyhop, you will write one Python function for each method.

## Example

The example below shows a state description of part of the 3rd floor of the M.C. Reynolds building. 
Several rooms are all connected to a hallway. The lounge is further connected to the copy room.
A robot is in room 312 and has not yet visited any other rooms.

```
# State
state = State('3rd-floor')
state.visited = {'robot': []}
state.loc = {'robot': 'mcrey312'}
state.connected = {'mcrey312': ['hallway', 'mcrey314'], 
                   'hallway': ['mcrey312', 'mcrey314', 'lounge'], 
                   'mcrey314': ['mcrey312', 'hallway'], 
                   'lounge': ['hallway', 'copyroom'], 
                   'copyroom': ['lounge']}
```

In this example, there is only one operator: `go`. The `go` operator moves an entity from one room to 
an adjacent room, and records the adjacent room as having been visited.

It makes sure the entity is in the starting room and has not already visited the ending room.
It also makes sure the rooms are connected.

```
def go(state, entity, start, end):
    if state.loc[entity] == start and end in state.connected[start] and end not in state.visited[entity]:
        state.loc[entity] = end
        state.visited[entity].append(end)
        return state
```

There is also only one method in this example. If the start and end are the same, it signals success.
If they are connected, it posts as a task the `go` operator. Otherwise, it posts a list of alternative
tasks: travel from `start` to a neighbor, then recursively post `find_route` to travel from that 
neighbor to the end.

```
def find_route(state, entity, start, end):
    if start == end:
        return TaskList(completed=True)
    elif state.connected[start] == end:
        return TaskList([('go', entity, start, end)])
    else:
        return TaskList([[('go', entity, start, neighbor), ('find_route', entity, neighbor, end)] for neighbor in state.connected[start]])
```

## Anytime Planning

Pyhop employs a search strategy known as depth-first search to find a plan. When presented with multiple options, 
as in the third alternative above, it aggressively makes choices until it has a complete plan. Here is one plan
that the planner might produce in response to the task ``:
```
[('go', 'robot', 'mcrey312', 'mcrey314'), 
 ('go', 'robot', 'mcrey314', 'hallway'), 
 ('go', 'robot', 'hallway', 'lounge'), 
 ('go', 'robot', 'lounge', 'copyroom')]
```

With properly designed methods, this should produce a plan if one exists, but it is not guaranteed to be the 
shortest possible plan. If time permits, the planner can go back and try other alternatives, and see if they 
produce better plans. This is known as *anytime planning*. Here is an example of a shorter plan:

```
[('go', 'robot', 'mcrey312', 'hallway'), 
 ('go', 'robot', 'hallway', 'lounge'), 
 ('go', 'robot', 'lounge', 'copyroom')]])
```

In an anytime planner, a plan is ready to return as soon as the first depth-first search completes. An anytime 
planner will backtrack and try alternative plans as long as time is available. The multiple options in the third
method step above constitute a *nondeterministic choice*. These nondeterministic choices are the alternatives 
available to the anytime planner.

## Assignment

Create two distinct planning domains using Pyhop and the Python programming language. One domain must include the 
following as a minimum:

* Two distinct types of objects.
* Two operators.
* Two methods.
* Two distinct starting states, at least one of which yields a plan containing at least five steps.

The other domain must include the following as a minimum:
* As before, two distinct types of objects.
* Three operators.
* Three methods.
  * At least one of these methods must have another method in its task decomposition.
  * Either that method or a different method must also have a nondeterministic choice in its task decomposition.
* Two distinct starting states, at least one of which yields a plan containing at least ten steps.

The intention is for the first domain to be relatively easy to construct. It is acceptable for the second domain
to be a more complex variation of the first domain.

## Details

After completing your domains, write a short paper (2-3 pages) containing the following:

* An introductory paragraph or two describing your target domains and why you initially believed that they were suitable for planning.
* For each domain, one to two paragraphs describing how you encoded that domain. 
  Pay particular attention to discussing the tradeoffs involved in the encoding. 
  * What domain features are represented explicitly? 
  * What domain features have been abstracted away? 
  * Why did you perform those abstractions? 
  * How good were the plans that were produced? 
  * What criteria do you have for "a good plan"?
* Planning is a quintessential example of classical AI as Brooks discussed in 
  [Elephants Don't Play Chess]({{site.baseurl}}/readings/Brooks1990.pdf).
  Now that you have hands-on experience with implementing a classical AI system, what is your view of the criticism
  that Brooks levels at classical AI?
* Write a concluding paragraph discussing the utility of planning in general. Under what circumstances is it worthwhile to build and use a planner? When is it not appropriate? Justify your answers based on the experiences in this project you described earlier in the paper.
* Submit each of your Python files via Teams. Also submit your paper in PDF format.


------------------------------------------------------------------------
