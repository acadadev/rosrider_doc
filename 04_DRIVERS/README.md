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

1. Create a ROS Workspace

	> mkdir -p rosrider_ws/src  
	> cd rosrider_ws/src  

	This creates a directory called `rosrider_ws` and a subdirectory named `src` within it. The `src` directory is the standard location for ROS package source code.

2. Clone the ROSRider Repository:

	> git clone https://github.com/acadadev/rosrider.git

3. Initialize ROS Dependency System (if not done during ROS installation)

	> sudo rosdep init
	> rosdep update

4. Install ROS Dependencies

	> cd ..  
	> rosdep install --from-paths src -y --ignore-src

	This command retrieves and installs all the necessary dependencies required by the ROSDriver package based on the cloned source code in the src directory.


5. Build the Workspace

	> colcon build  
	> source devel/setup.bash

6. Adding ROSRider to Your Bash Environment

	To ensure that your system can find the necessary ROSDriver components, you'll need to add the following line to your Bash configuration file:

	> source ~/rosrider_ws/install/setup.bash

---

instrumnting robot to turn lidar on off. ros2rpi

running the driver  

checking diag output
checking odometry output  

[TODO: simulations]
[TODO: move status to debug section]

__Next Chapter:__ [Parameters](../05_PARAMETERS/README.md)
