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

<p><strong>If you are running Ubuntu on a virtual machine</strong>, install the package</p>


---


<div class="highlight notranslate"><div id="blob-path" class="highlight"><pre><span></span>sudo apt install open-vm-tools
</pre></div>

<clipboard-copy aria-label="Copy" class="ClipboardButton btn btn-invisible js-clipboard-copy m-2 p-0 d-flex flex-justify-center flex-items-center" data-copy-feedback="Copied!" data-tooltip-direction="w" value="npm install
npm test" tabindex="0" role="button">

<svg aria-hidden="true" height="16" viewBox="0 0 16 16" version="1.1" width="16" data-view-component="true" class="octicon octicon-copy js-clipboard-copy-icon">
    <path d="M0 6.75C0 5.784.784 5 1.75 5h1.5a.75.75 0 0 1 0 1.5h-1.5a.25.25 0 0 0-.25.25v7.5c0 .138.112.25.25.25h7.5a.25.25 0 0 0 .25-.25v-1.5a.75.75 0 0 1 1.5 0v1.5A1.75 1.75 0 0 1 9.25 16h-7.5A1.75 1.75 0 0 1 0 14.25Z"></path><path d="M5 1.75C5 .784 5.784 0 6.75 0h7.5C15.216 0 16 .784 16 1.75v7.5A1.75 1.75 0 0 1 14.25 11h-7.5A1.75 1.75 0 0 1 5 9.25Zm1.75-.25a.25.25 0 0 0-.25.25v7.5c0 .138.112.25.25.25h7.5a.25.25 0 0 0 .25-.25v-7.5a.25.25 0 0 0-.25-.25Z"></path>
</svg>

</clipboard-copy>
</div>

---

