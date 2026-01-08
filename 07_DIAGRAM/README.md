---
layout: default
title_url: /07_DIAGRAM/README.html
title: "Controller Diagram"
description: "ROSRider Motor Control Theory of Operation"
---

<div style="
    background-color: #f8f9fa;
    border-left: 6px solid #4CAF50;
    color: #333333;
    padding: 25px;
    border-radius: 4px;
    margin: 30px 0;
    box-shadow: 0 2px 8px rgba(0,0,0,0.08);
    font-family: 'Ubuntu', sans-serif;
">
    <div style="font-size: 1.4em; font-weight: bold; color: #4CAF50; margin-bottom: 5px;">
        > Theory of Operation
    </div>
</div>

__PID Controller__

<div style="display: flex; justify-content: space-around; margin: 25px 0;">
  <figure style="width: 60%; margin: 0; text-align: center;">
    <img src="../images/sch/ROSRider_Controller_Diagram_PID.png" alt="ROSRider PID Loop, Classic Mode" style="width: 100%;">
    <figcaption>ROSRider PID Loop, Classic Mode</figcaption>
  </figure>
</div>

__Cascaded Current Controller__

The cascaded PID controller utilizes a nested loop architecture, where the outer loop calculates the 
desired current reference, and the inner faster loop modulates the motor voltage to strictly regulate torque.

This inner stage employs a **physics-based feedforward strategy**, injecting calculated terms for resistive voltage drop and estimated Back-EMF (BEMF) directly into the control output.
By utilizing DSP-filtered velocity and current data to predict the motor's electrical requirements, the PI controller is relieved of the bulk control effort and focuses solely on disturbance rejection.
This decoupling of electrical dynamics from mechanical load simplifies tuning and ensures the system remains robust and stable even if the robot's weight or inertia changes significantly.

<div style="display: flex; justify-content: space-around; margin: 25px 0;">
  <figure style="width: 100%; margin: 0; text-align: center;">
    <img src="../images/sch/ROSRider_Controller_Diagram_Cascaded.png" alt="ROSRider PID Loop, Cascaded Current Control Mode" style="width: 100%;">
    <figcaption>ROSRider PID Loop, Cascaded Current Control Mode</figcaption>
  </figure>
</div>

__Next Chapter:__ [Procedures](../08_PROCEDURES/README.md)