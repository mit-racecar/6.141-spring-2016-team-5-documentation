---
layout:     post
title:      "Lab 3 Writeup"
categories: writeups
cover:      /assets/images/lab3-point-clouds.jpg
published:  false
---

# A simpler ROS API

```python
from rospyext import *

class ObjectDetectorNode(Node):
    pub_detect = Publisher('/object_detection', Bool, queue_size=10)

    distance_thresh  = Param(float, default=1)

    @Subscriber('/laser/scan', LaserScan)
    def sub_scan(self, scan):
    	detected = ...

        # process scan data
        self.pub_detect.publish(detected)

```

# Estimation
## Splitting the scans

![](/assets/images/lab3-point-clouds.jpg)

## Wall and object detection


# Control


# Results

<!-- [Here's a link to our lab presentation slide deck](). -->
