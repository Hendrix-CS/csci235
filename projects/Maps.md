---
layout: work
type: Project
num: 7
worktitle: Maps
---

## Option 1: Topological Maps

A **topological map** is a data structure that represents locations and 
routes between them. 

The robot can recognize if it has reached a location using a **knn** or **kmeans**
classifier. It can transit between locations either also using the same classifier
or using a groundline finder.

### 1A: Traveling to a target

Write a program to drive your robot towards a target as follows:
* At the start of the program, the robot should be placed several meters from
  the target, aimed in its general direction.
* Once the program starts, the robot should drive towards the target, either by 
  using a groundline or a kmeans/knn classifier.
  * If using a groundline, the robot should periodically (every few seconds) switch 
    from using the groundline to using a **knn**/**kmeans** classifier to determine 
	if it has reached the target. If so, it should stop moving.
  * If using a knn/kmeans classifier, there should be separate classifications for
    adjusting left, adjusting right, and recognizing the goal area.
  
### 1B: Multiple targets

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
  
## Option 2: Metric Maps

A **metric map** is a data structure that represents positions in terms of
specific measured quantities. The simplest form of a metric map (which is what
we will employ) is an **occupancy grid**. Each cell in the grid represents a 
certain fixed distance, and is either marked **filled** or **clear**. 

Groundline points can be used to fill in an occupancy grid. To do so requires
calibrating the relationship between `x` and `y` values and metric distances. 
The Calibration screen enables you to do so as follows:
* Place a meter stick one meter away from your robot.
* Place another meter stick two meters away from your robot.
* Take a photo of the scene.
* Go to the Calibration screen.
* Adjust the red and cyan meter lines so that they overlap with the meter sticks in your image.
* Record the `width` and `height` values for each meter stick for later use.

<img src="https://hendrix-cs.github.io{{site.baseurl}}/assets/images/Calibration.png" width=500>

A **particle filter** represents a set of guesses ("particles") about the robot's 
location and surroundings. Each particle contains the following data:
* Estimated (x, y) distance from starting position
* Estimated heading
* Estimated occupancy grid

To set up a particle filter that derives its occupancy grid points from the 
groundline:

    cv particle maxColors minNotFloor maxJump numParticles minR minTheta maxR maxTheta cellsPerMeter width height meter1height meter1width meter2height meter2width project label [photoNums]

The following parameters configure the groundline, and work the same way:
* `maxColors`
* `minNotFloor`
* `maxJump`
* `width`
* `height`
* `project`
* `label`
* `photoNums`

Here is a guide to the remaining parameters:
* `numParticles`: The total number of particles in the filter.
* `cellsPerMeter`: The number of occupancy grid cells per meter.
* `minR`, `minTheta`, `maxR`, `maxTheta`: When the robot reports an estimate of its
  movement, these values indicate the expected precision of that estimate. 
  When creating a particle, it will add `minR` and `minTheta` to the estimate, and
  then add a random value (uniformly scaled between `min` and `max`) to that sum.
* `meter1width`, `meter1height`, `meter2width`, `meter2height`: Calibration values from above.

To send a motion estimate to the app:

    msg r theta
    
The app sends back two different types of messages to the Arduino:
* `heading x y`: The same message as the groundline.
* `pos x y theta`: Position estimate from the particle filter, using the best particle.
  It only sends this message in response to the sending of a motion estimate.
  
## Assignment

* Select to implement either a metric map or a topological map. If you select a 
metric map, the robot task is the same as for the topological map.
* Try your best to get it to complete the localization task. Have the robot speak a
message out loud whenever it reaches a destination.
* In your writeup, discuss how you configured your robot and describe in detail
  how it performed.
* As always, submit representative videos to document its performance.
* Also in your writeup, discuss why you chose the topological or metric approach.

