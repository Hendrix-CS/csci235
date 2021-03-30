---
layout: work
type: Project
num: 7
worktitle: Maps
---

## Part 1: Topological Maps

A **topological map** is a data structure that represents locations and 
routes between them. 

The robot can recognize if it has reached a location using a **knn** or **kmeans**
classifier. It can transit between locations either also using the same classifier
or using a groundline finder.

### Part 1A: Traveling to a target

Write a program to drive your robot towards a target as follows:
* At the start of the program, the robot should be placed several meters from
  the target, aimed in its general direction.
* Once the program starts, the robot should use a groundline to drive towards
  the target.
* The robot should periodically (every few seconds) switch from using the 
  groundline to using a **knn**/**kmeans** classifier to determine if it has
  reached the target. If so, it should stop moving.
  
### Part 1B: Multiple targets

Write a program that enables your robot to navigate among three corners of 
an "L" shape:
* At the start of the program, the robot should be placed at a starting vertex
  of the "L", aimed at the next vertex, which should be a few meters away. It
  should drive towards it using the approach from Part 1A.
* When the robot reaches the next vertex, it should turn towards the third 
  vertex, again using the technique from Part 1A to get here.
* When the robot reaches the third vertex, it should turn around and drive
  back towards the second vertex.
* Once it reaches the second vertex, it should turn back towards the starting
  vertex and drive towards it. Once it reaches that vertex, it should stop.
  
## Part 2: Metric Maps

*TBA*

