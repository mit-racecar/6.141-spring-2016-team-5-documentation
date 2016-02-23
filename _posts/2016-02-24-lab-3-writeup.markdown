---
layout:     post
title:      "Lab 3 Writeup"
categories: writeups
published:  false
cover:      /assets/images/lab3-point-clouds.png
---

<!--more-->

## A simpler ROS API

```python
from rospyext import *

class ObjectDetectorNode(Node):
    pub_detect = Publisher('/object_detection', Bool)
    distance_thresh = Param(float, default=1)

    @Subscriber('/laser/scan', LaserScan)
    def sub_scan(self, scan):
        detected = min(scan) < self.distance_thresh
        self.pub_detect.publish(detected)
```

## Estimation

### Splitting the scans

![]({{ site.baseurl }}/assets/images/lab3-point-clouds.png)

### Wall and object detection


## Control


## Results

<!-- [Here's a link to our lab presentation slide deck](). -->
