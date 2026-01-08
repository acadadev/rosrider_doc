---
layout: default
title_url: /07_DIAGRAM/README.html
title: "Controller Diagram"
description: "ROSRider Motor Control Theory of Operation"
---

__PID Controller__

<div style="display: flex; justify-content: space-around; margin: 25px 0;">
  <figure style="width: 60%; margin: 0; text-align: center;">
    <img src="../images/sch/ROSRider_Controller_Diagram_PID.png" alt="ROSRider PID Loop, Classic Mode" style="width: 100%;">
    <figcaption>ROSRider PID Loop, Classic Mode</figcaption>
  </figure>
</div>

__Cascaded Current Controller__

The cascaded PID controller utilizes a nested loop architecture, where the outer loop calculates the 
desired current reference, and the inner faster loop calculates current error and adjusts the output.

This design allows the system to correct minor disturbances immediately within the inner loop before they affect the overall system stability.

<div style="display: flex; justify-content: space-around; margin: 25px 0;">
  <figure style="width: 100%; margin: 0; text-align: center;">
    <img src="../images/sch/ROSRider_Controller_Diagram_Cascaded.png" alt="ROSRider PID Loop, Cascaded Current Control Mode" style="width: 100%;">
    <figcaption>ROSRider PID Loop, Cascaded Current Control Mode</figcaption>
  </figure>
</div>

__Next Chapter:__ [Procedures](../08_PROCEDURES/README.md)