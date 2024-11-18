---
layout: default
parent_url: /08_PROCEDURES/README.html
parent_title: "Procedures in Robotics"
title_url: /08_PROCEDURES/01_PID/README.html
title: "PID Tuning"
description: "Detailed demonstration of the PID tuning procedure"
---


#### PID Tuning

PID tuning is a crucial process in robotics to optimize a system's response to inputs. By carefully adjusting the proportional (P), integral (I), and derivative (D) gains, engineers can fine-tune the system's behavior. This document outlines a step-by-step guide to PID tuning using the ROSRider platform. We will leverage the power of RQTreconfigure to dynamically adjust PID parameters and RQTplot to visualize the system's response in real-time. By following this procedure, you can achieve precise and efficient control of your robot.

To start the ROSRider driver, execute the following command on the robot:

```bash
ros run rosrider_node rosrider_node.launch.py
```

<div style="display: flex; justify-content: space-around; margin: 25px 0;">
   <img src="../../images/rosrider_node_launch.gif" alt="ROSRider node launch" style="box-shadow: 0px 4px 8px rgba(0, 0, 0, 0.2);">
</div>

To view the list of parameters for the ROSRider node, use the following command:

```bash
ros2 param list /rosrider_node
```		

To launch the RQT Reconfigure tool, execute the following command:

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

Next, we'll modify the `kP` parameter, which controls the proportional gain for the left motor.

Let's verify the parameter change by requesting it from the ROS Parameter Server.




---

    ros2 param get /rosrider_node LEFT_KP

---



---

<p><strong>If you are running Ubuntu on a virtual machine</strong>, install the package <code class="docutils literal notranslate"><span class="pre">open-vm-tools</span></code> in addition to the normal Ubuntu dependencies:</p>
<div class="highlight notranslate"><div class="highlight"><pre><span></span>sudo apt install open-vm-tools
</pre></div>
</div>

---

<clipboard-copy for="blob-path">Copy</clipboard-copy>
<div id="blob-path">src/index.js</div>

--