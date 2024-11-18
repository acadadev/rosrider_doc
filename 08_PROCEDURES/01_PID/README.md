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



<div id="blob-path">
  blabla
<clipboard-copy for="blob-path" class="btn btn-sm BtnGroup-item">
  Copy
</clipboard-copy>
</div>

---

<button class="copy-button" data-clipboard-target="#code-snippet">Copy</button>
<pre id="code-snippet">
    ros2 param get /rosrider_node LEFT_KP
</pre>

---

<div class="highlight"><pre id="codecell1">
  <span></span>sudo apt install open-vm-tools
</pre>

<button class="copybtn o-tooltip--left" data-tooltip="Copy" data-clipboard-target="#codecell1">
      <svg xmlns="http://www.w3.org/2000/svg" class="icon icon-tabler icon-tabler-copy" width="44" height="44" viewBox="0 0 24 24" stroke-width="1.5" stroke="#000000" fill="none" stroke-linecap="round" stroke-linejoin="round">
  <title>Copy to clipboard</title>
  <path stroke="none" d="M0 0h24v24H0z" fill="none"></path>
  <rect x="8" y="8" width="12" height="12" rx="2"></rect>
  <path d="M16 8v-2a2 2 0 0 0 -2 -2h-8a2 2 0 0 0 -2 2v8a2 2 0 0 0 2 2h2"></path>
</svg>
</button>

</div>