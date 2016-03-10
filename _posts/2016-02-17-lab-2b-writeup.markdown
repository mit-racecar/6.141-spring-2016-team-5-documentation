---
layout:     post
title:      "Lab 2B Writeup"
date:       2016-02-17 19:45:48.740
categories: writeups
cover:      /assets/images/lab2b-laser-sim.jpg
---

Lab 2B was focused on setting up the team's tools and environment in order to effectively develop software for the robot.  Several tasks involved setting up the software development environment on the user end in order to facilitate communication, and code-sharing between team members and with the staff, while others involved setting up the software on the physical robot in order to work correctly.

<!--more-->

## Table of Contents
{:.no_toc}

* This list element is automatically replaced with the TOC
{:toc}

## Module 1

Module 1 tasks set up our development environment.  The hosts file on the VM was updated to enable proper connection to the robot through the access point.  In terms of code sharing, MIT's github enterprise allows more freedom of use than public github for the purposes of this project.  Most group members set up SSH keys for authentication as well, to avoid the clumsiness of using tokens to authenticate.

Joining the class-wide organization on MIT github allows the team to get base software updates from the staff and potentially even revert to a baseline working state.  We created an organization for our team as well to house the repositories for all of our code, including personalized forks of the staff code.

Finally, each team member set up a local ROS workspace to build all staff and team code in the same overlay for use in the ROS ecosystem.  

## Module 2

For module 2, the tasks were designed to test the infrastructure set up in module 1.  Specifically, we verified that the gazebo simulator for the racecar properly dealt with the tunnel map and car controls.  In addition, we used the joypad to manually control the robot's movement, configuring the VM to properly recognize input from the controller in the process.

In doing this, unexpected behavior was encountered.  Gazebo often improperly simulates the robot's movement, resulting in sidways translations of the car rather than the proper movement resulting from Ackermann steering.  The fact that all team members encountered this problem separately using the software forked directly from the staff rpositories indicates that this is an underlying problem of the racecar simulator or gazebo rather than of anything local. Another potential issue is that the performance of Gazebo in a VM is relatively poor. It was fast enough to accomplish the tasks for this lab, but this might be a problem in the future when more sensor data is involved.

Finally, we verified that the simulator was interacting properly with the ROS ecosystem by checking the sensors' published topics.  In particular we verified that the simulated laser scan data seen through RViz was reasonable given the robot's position.

## Module 3

The final module of lab 2B involved setting up the software on the robot's end.  Unfortunately, our team ran into problems completing this portion because of the failure of the access point, likely as a result of being used with the wrong power supply.  Our 12v router was found suspiciously close to 19v power supply.

This delayed our progress for many hours while we attempted to work around the missing router. We experienced and continue to experience difficulty connecting a display to the racecar. At one point we got a shell to show up on the monitor, but all subsequent attempts to do so failed. We attempted to use an ethernet cable to directly connect to the robot, but this did not work. This could be due to not having a crossover cable, or other networking issues. Attempts to connect to the racecar while it was wired to a neighboring team's AP failed probably because the racecar's wired interface is used to connecting to a sensor or because it requires a wireless connection to be connected to.

Eventually with the help of Abhishek, who brainwashed another router into our configuration, we were able to connect to the robot and complete some of the goals of this module. We updated the racecar software and the ZED camera settings. We reached the point where the ZED camera needs calibration. We did not reach subsequent goals of testing the updated sensor drivers.

## Additional Items

Our team also set up communication infrastructure beyond the lab's directions.  We created a team-wide Slack page for communication and collaboration on non-code assignments, with integration from github to provide additional notification for new commits.

[Here's a link to our lab presentation slide deck](https://docs.google.com/presentation/d/1uv5sRoH_Ll5dtq8-jLKJIxxPPaZvK0B958jsGzH6fNs/edit?usp=sharing).
