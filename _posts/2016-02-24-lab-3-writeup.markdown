---
layout:     post
title:      "Lab 3 Writeup"
categories: writeups
cover:      /assets/images/lab3-point-clouds.png
---


In this week's lab, the goal was to create a rudimentary pipeline for following a wall while checking for collisions in front. To accomplish this, we divided the task into natural functionality segments both to better organize our code structure as well as to add modularity for future functionality to build off of.  In addition, we continued to add infrastructure to make writing ROS code with good practices easier.

<iframe width="560" height="315" src="https://www.youtube.com/embed/iGUTtHEP7PU" frameborder="0" allowfullscreen></iframe>

<!--more-->

![our node graph]({{ site.baseurl }}/assets/images/lab3-rosgraph.dot.svg)

## Table of Contents
{:.no_toc}

* This list element is automatically replaced with the TOC
{:toc}


## Estimation

![rviz results from simulated laser scan]({{ site.baseurl }}/assets/images/lab3-point-clouds.png)

### Scan Parser

The lowest level of functionality is the scan parser node, which takes input directly from the scanner and manipulates it into a form more useful for the rest of the nodes to use.  First, the laser scan is divided into 3 segments: front, left, and right.  The division between the three scans is determined by input from the ROS parameter service.  Each of these reduced laser scans is republished to its own topic so other nodes can look only at the portion of the scan they require, and many nodes can look at each trimmed scan without recomputing the trim operation.

The other operation the scan parser completes is to transform the `LaserScan`s into `PointCloud2`s using the [`laser_geometry`](http://wiki.ros.org/laser_geometry) package.  The point cloud format enables a node like the wall detector to access the the scan information in a potentially more convenient way, for example if it is going to perform regression or other operations on the points of the scan in cartesian space.  A point cloud is published for each of the trimmed down scans as well.

The scan parser is also responsible for ignoring points which are the robot sighting itself. It does this by dropping samples outside of a particular angle band.

### Object Detector

The object detector looks at the laser scan from the front of the robot to check for any close objects which could cause a collision.  Using the trimmed scan from the scan parser, it counts the points within a distance threshold and publishes a boolean indicating whether there are sufficiently many points within this threshold.  The threshold for distance and for the number within that distance are determined by input from the ROS parameter service.

### Wall Detector

The wall detector detects the nearest point to the obstacle in trimmed scanner data from the right side, publishing the location of this point in cartesian space relative to the frame of the robot.  The desired quantities of angle and range to the wall are approximated by the angle and range of this shortest beam, recoverable from the cartesian coordinates simply by converting back to polar.

We construct two of these objects using a roslaunch file containing the following:

```xml
<node pkg="reflexive_control" name="right_wall_detector" type="wall_detector.py">
  <remap from="laser/scan" to="/tokyo/laser/scan/right"/>
  <remap from="wall_detection" to="/tokyo/wall_detection/right"/>
</node>
<node pkg="reflexive_control" name="front_wall_detector" type="wall_detector.py">
  <remap from="laser/scan" to="/tokyo/laser/scan/front"/>
  <remap from="wall_detection" to="/tokyo/wall_detection/front"/>
</node>
```

## Control

### Wall Follower

The wall follower node implements a simple two-state proportional controller to follow a wall to its right at a fixed distance.  Given the input vector from the wall detector, it first extracts the distance and angle to the wall encoded in the point message.  Given some target distance and angle from the wall, the node multiplies each error by a gain constant and publishes the result to the racecar’s drive system as the angle component of the ackermann command message.  The target distance and angle as well as the gain constants are determined by the ROS parameter service.

### Safety Controller

Our `safety_controller_node` acts as a safe and transparent way of disabling robot movement when the robot has detected that an object has encroached closer than a specified safety margin from the front of the robot. To achieve this, our `safety_controller_node` works independently of our higher-level robot logic and outputs “STOP” commands to the robot’s servos through the output MUX when it detects that an object has encroached upon the safety barrier.

To detect objects in front of the robot, the `safety_controller` node subscribes to the output of the `front_wall_detector` node and, on every update from `front_wall_detector`, decides whether an object has encroached too close to the front of the robot. If an object has encroached too close to the front of the robot, the `safety_controller_node` will override all outgoing movement commands to the servos with a default `AckermannSteeringStamped` message that disables all driving movement by using the `ackermann_cmd_mux/input/safety` topic which takes precedent on all other robot actuator commands.

![Safety Controller Node Hookup Diagram]({{ site.baseurl }}/assets/images/lab3-safety_controller.png)

## Putting it all together

### `rospyext`

There are two things we wanted to avoid doing when writing ros nodes:
hard-coding constants in the source code, rather than allowing them to be configured them from the launch file
Naming the parameter in the source code something different from the one in rosparam
Accidently forgetting to hook a subscriber up, or hooking the same handler up twice by accident

We decided to solve this by writing a wrapper for some of `rospy`’s key functions, called `rospyext`. This used a handful of advanced python features - metaclasses, descriptors, and decorators. The end result is that instead of our modules looking like this:

```python
class ObjectDetectorNodeBefore(object):
    def __init__(self):
        self.pub_detect = Publisher('/object_detection', Bool)
        Subscriber('/laser/scan', LaserScan, self.sub_scan)
        self.distance_param = rospy.get_param('~distance_thresh', 1)

    def sub_scan(self, scan):
        detected = min(scan) < self.distance_thresh
        self.pub_detect.publish(detected)
```
They look like this:

```python
class ObjectDetectorNodeAfter(Node):
    pub_detect = Publisher('/object_detection', Bool)
    distance_thresh = Param(float, default=1)

    @Subscriber('/laser/scan', LaserScan)
    def sub_scan(self, scan):
        detected = min(scan) < self.distance_thresh
        self.pub_detect.publish(detected)
```

### Namespace Manipulation

We discovered that the topic names used by the simulator did not align with those used by the hardware. In particular many topics which are in `/racecar` in the simulator are in `/vesc` on the car. The most important of which so far is `/racecar/ackermann_cmd_mux/input/*`.

Our approach to being able to run the same node code in the simulator as on the car with the least amount of manual tuning was to use remapping. Nodes use the simulator topic layout as canon, and a standard launch file `wall_follower_sim.launch` works in the simulator. For running on the car, a launch file `wall_follower_car.launch` first remaps the topics we use in `/vesc` and `/scan` to their `/racecar` equivalents and then includes `wall_follower_sim.launch`.

Additionally, to get the simulator’s namespace to adhere as closely to the robot’s operating namespace, we also created a new Gazebo simulator launch file `racecar_tunnel_with_mux.launch` that also boots a `ackermann_cmd_mux` MUX under the `/racecar` namespace. Using this modified launch file, we can now post commands to `ackermann_cmd_mux/input/safety` and `ackermann_cmd_mux/input/navigation` and expect the simulated robot to prioritize these commands in the correct manner.

## Open source contributions

* ![](https://github-shields.com/github/ros/ros_comm/pull/743.svg) - Fixes to `rospy.numpy_msg`
* ![](https://github-shields.com/github/ros-visualization/rqt_common_plugins/pull/354.svg) - Fixes to rqt node graph

Our presentation slides can be found [on google slides](https://docs.google.com/presentation/d/1XHG0e0SMnvZHWL35wB0ebjpf-7GNpNpY7OfzQ7E2a1w/edit?usp=sharing)
