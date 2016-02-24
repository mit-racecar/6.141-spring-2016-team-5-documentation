---
layout:     post
title:      "Lab 3 Writeup"
categories: writeups
cover:      /assets/images/lab3-point-clouds.png
---


In this week's lab, the goal was to create a rudimentary pipeline for following a wall while checking for collisions in front. To accomplish this, we divided the task into natural functionality segments both to better organize our code structure as well as to add modularity for future functionality to build off of.  In addition, we continued to add infrastructure to make writing ROS code with good practices easier.

![our node graph]({{ site.baseurl }}/assets/images/.)

<!--more-->
##Estimation
![rviz results from simulated laser scan]({{ site.baseurl }}/assets/images/.)
###Scan Parser
The lowest level of functionality is the scan parser node, which takes input directly from the scanner and manipulates it into a form more useful for the rest of the nodes to use.  First, the laser scan is divided into 3 segments: front, left, and right.  The division between the three scans is determined by input from the ROS parameter service.  Each of these reduced laser scans is republished to its own topic so other nodes can look only at the portion of the scan they require, and many nodes can look at each trimmed scan without recomputing the trim operation.  The other operation the scan parser completes is to transform the `LaserScan`s into `PointCloud2`s using the `laser_geometry` package.  The point cloud format enables a node like the wall detector to access the the scan information in a potentially more convenient way, for example if it is going to perform regression or other operations on the points of the scan in cartesian space.  A point cloud is published for each of the trimmed down scans as well.
###Object Detector
The object detector looks at the laser scan from the front of the robot to check for any close objects which could cause a collision.  Using the trimmed scan from the scan parser, it counts the points within a distance threshold and publishes a boolean indicating whether there are sufficiently many points within this threshold.  The threshold for distance and for the number within that distance are determined by input from the ROS parameter service.
###Wall Detector
The wall detector detects the nearest point to the obstacle from scanner data, and publishes the location of this point in cartesian space, relative to the frame of the robot.

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
##Control
###Wall Follower
The wall follower node implements a simple two-state proportional controller to follow a wall to its right at a fixed distance.  Given the input vector from the wall detector, it first extracts the distance and angle to the wall encoded in the point message.  Given some target distance and angle from the wall, the node multiplies each error by a gain constant and publishes the result to the racecar’s drive system as the angle component of the ackermann command message.  The target distance and angle as well as the gain constants are determined by the ROS parameter service.
###Safety Controller

##Putting it all together
###`rospyext`
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


###Namespace Manipulation
We discovered that the topic names used by the simulator did not align with those used by the hardware.

## Open source contributions

* ![](https://github-shields.com/github/ros/ros_comm/pull/743.svg) - Fixes to `rospy.numpy_msg` 
* ![](https://github-shields.com/github/ros-visualization/rqt_common_plugins/pull/354.svg) - Fixes to rqt node graph

