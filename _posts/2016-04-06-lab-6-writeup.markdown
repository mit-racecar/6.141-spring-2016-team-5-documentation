---
layout:     post
title:      "Lab 6 Writeup"
categories: writeups
cover:      /assets/images/lab6/complex_rrt.png
published:  false
---

<!-- TODO: introduction -->

<!--more-->

## Selecting a Mapping Strategy

In order to effectively find paths, the robot needs a model of its surroundings in which to plan.  This entails both knowledge of where obstacles are located in the world and where the robot is in the world.  We investigated two different potential alternatives for this.  The first was a divided approach, which localized using a known map and the AMCL library and created an ad-hoc local map based on the laser scans and the transform determined by AMCL.  The alternative was simply to use hector-slam to both create a map and localize within it.

The first approach has the advantage of being less computationally expensive, because not all of the functions of SLAM were actually required for the task.  However, it is more complicated to implement than simply using an integrated SLAM package.  Conversely, SLAM involves more computation than is necessary but required only using a single pre-implemented package.  On testing hector-slam on the robot, we found that it was able to run sufficiently fast despite the extra computation, so we went with the second option.

## Selecting a Pathfinding Algorithm

The team looked over the pathfinding algorithms covered in class in order to select one to implement. The choices were potential fields, probabilistic roadmap (PRM), and rapidly-expanding random trees (RRT). In an ideal universe we would have created implementation stubs for each of these and evaluated which one worked best in simulation in order to pick one for the final race. In the non-ideal universe where we have one week to write implementation code while balancing other classes, we had to make a choice of one algorithm to implement.

In order to do this, we judged the algorithms on ease of implementation as well as the completeness of the algorithm. Along these metrics, potential fields seemed to be the easiest to implement, followed by the RRT and PRM being somewhat equivalent in difficulty. As for completeness, potential fields seemed to be the worst due to the possibility of getting stuck in a concave obstacle between the car and the goal. PRM is worse than RRT because it cannot handle dynamic obstacles as well. As a result, we picked the RRT to implement because we expected it to result in the most complete solution.


## Path planning with RRTs

There are four key parts to a Rapidly-expanding Random Tree planner:

1. `sample` - A method to sample a point to try
2. `distance` - A metric for determining the distance or cost between a visited point and an unvisited point
3. `extend` - A rule for extending towards a new location from an existing one
4. `done` - A condition for when to stop

We started by writing the core algorithm that wraps these four operations as an abstract class (using pythons `abc` module), which allowed us to swap out these functions without rewriting the entire RRT each time.

### A simple RRT

For a simply RRT, we filled out these key parts as:

1. `sample`:
  * With a $$\frac{1}{20}$$ chance, choose the goal location
  * Otherwise, choose a random point in the bounding box circumscribed around a circle 1.5&times; the diameter of the smallest circle passing through the start and end points of the planner, as in the below interactive diagram. <object type="image/svg+xml" data="{{ site.baseurl }}/assets/images/lab6/sample_simple.svg" width="300px" height="100px"></object>
2. `distance` - Use the cartesian distance metric, $$\sqrt{x^2 + y^2}$$
3. `extend` - Linear interpolate up to 0.5m towards the goal, stopping early if the line hits a wall
4. `done` - Stop if the distance to the goal is less than 0.1m


![a visualization of the simple rrt]({{ site.baseurl }}/assets/images/lab6/simple_rrt.png)


### Modelling the car more accurately

<!-- remarks about how the car is non-holonomic -->


1. `sample`
  * With a $$\frac{1}{20}$$ chance, choose the goal location
  * With a $$\frac{1}{4}$$ chance, choose a random _free_ point from the bounding box between the goal and the nearest point on the tree so far, defined in the same way as in the other RRT
  * Otherwise, choose a random free point using the same rules as before <!-- insert diagram, possibly inline svg --> 
2. `distance`
   Return the length of the shortest circular arc connecting the existing point to the new one. If such an arc would be too tight for the car to follow, return $$\infty$$
3. `extend` - _Spherically_ interpolate up to 0.25m along the arc found in the distance function, stopping early if _any part of the car_ would be intersecting a wall
4. `done` - As before

![The start of the complex rrt]({{ site.baseurl }}/assets/images/lab6/complex_rrt_1.png)
![The next step]({{ site.baseurl }}/assets/images/lab6/complex_rrt_2.png)
![The next step]({{ site.baseurl }}/assets/images/lab6/complex_rrt_3.png)
![The next step]({{ site.baseurl }}/assets/images/lab6/complex_rrt_4.png)
![The final path of the rrt]({{ site.baseurl }}/assets/images/lab6/complex_rrt.png)

### Updating the `path_follower` for usage with RRT
<!-- Assigned to Winterg --> 

