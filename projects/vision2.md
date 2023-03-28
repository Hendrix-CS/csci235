---
layout: work
type: Project
num: 7
worktitle: Identifying Locations
---

## Part 1: Classifying Locations

Create labels for three distinct locations. Program your robot to identify its 
current location, and based on that location, pick the direction to travel to the
next location in a clockwise manner. When it finds itself in the second location,
it will continue to the third location, then back to the first location, continuing
the cycle indefinitely.

## Part 2: Alternative Recognizers

Several alternative kNN distance functions are now available in the app.
Experiment with them. Based on your observations, determine which distance
function is most effective for identifying your locations.

## Part 3: Additional Locations

Introduce two additional locations. Each should only be adjacent to one of the
previous three selected locations. Create a menu on the EV3 that allows a user
to select one of the five locations. The robot should then navigate to that 
location. Once it arrives, it should again create the menu and allow a human to
select a new destination.

## Questions
1. How did you arrange your environment and select your locations to make them 
distinct for the classifier? How effective were your arrangements?
2. Which kNN distance function worked best for you? What evidence do you have that
supports that conclusion?
3. How reliably was the robot able to navigate among the five locations? Give 
at least three specific examples to support your answer.
