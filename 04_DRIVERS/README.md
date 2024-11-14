---
layout: default
title_url: /04_DRIVERS/README.html
title: "ROS2 Drivers"
description: "Compiling and Installing ROS2 Drivers"
---

**Installing ROSRider Drivers**

The ROSRider package provides the necessary ROS nodes and drivers to interact with the ROSRider board. It allows you to control motors, read sensor data, and perform other tasks using ROS.

While previous versions of ROSRider utilized middleware to integrate with ROS, our latest iterations have transitioned to native C++ drivers. This shift eliminates the performance overhead associated with middleware, resulting in faster and more predictable system behavior. By directly interfacing with the hardware, we can optimize resource utilization and minimize latency, ensuring optimal performance for your robotics applications.

The ROSRider firmware incorporates a timing control feature that synchronizes itself to the host computer's polling rate, resulting in a latency of 1-2 milliseconds. By actively adjusting its internal clock based on received packets, the ROSRider firmware ensures precise timing and synchronization, preventing data inconsistencies and errors that can occur due to timing drifts. This guarantees that ROS packets are delivered on time, ensuring reliable and efficient system operation.

**Installation Instructions**

> mkdir -p rosrider_ws/src  
> cd rosrider_ws/src  
> git clone https://github.com/acadadev/rosrider.git

**Build the Workspace**

> cd ..
> colcon build
> source devel/setup.bash

running the driver
checking the output
checking odometry output



[TODO: add line to .bashrc on the robot]
[TODO: simulations]
[TODO: move status to debug section]

__Next Chapter:__ [Parameters](../05_PARAMETERS/README.md)
