---
layout: default
title_url: /07_DIAGRAM/README.html
title: "Controller Diagram"
description: "ROSRider Motor Control Theory of Operation"
---

<div class="sl">
    <div class="sl1">
        > PID Controller
    </div>
</div>

The standard control mode uses a single-loop PID architecture augmented with predictive compensation terms.
To improve tracking during dynamic maneuvers, the controller applies **velocity and acceleration feedforwards,**
essentially supplying the energy needed for the motion profile before any error occurs.
We also implemented a **physics-based Stribeck friction model** to counteract mechanical resistance like stiction and viscous drag.
This effectively linearizes the motor response, ensuring the control loop is driving the load rather than just fighting against friction thresholds.  

<div class="img_dv">
  <figure class="img_fg60">
    <img src="../images/sch/ROSRider_Controller_Diagram_PID.png" alt="ROSRider PID Loop, Classic Mode" style="width: 100%;">
  </figure>
</div>

<div class="sl">
    <div class="sl1">
        > Cascaded Current Controller
    </div>
</div>

The cascaded PID controller utilizes a nested loop architecture, where the outer loop calculates the 
desired current reference, and the inner faster loop modulates the motor voltage to strictly regulate torque.

This inner stage employs a **physics-based feedforward strategy**, injecting calculated terms for resistive voltage drop and estimated Back-EMF (BEMF) directly into the control output.
By utilizing DSP-filtered velocity and current data to predict the motor's electrical requirements, the PI controller is relieved of the bulk control effort and focuses solely on disturbance rejection.
This decoupling of electrical dynamics from mechanical load simplifies tuning and ensures the system remains robust and stable even if the robot's weight or inertia changes significantly.

<div class="img_dv">
  <figure class="img_fg90">
    <img src="../images/sch/ROSRider_Controller_Diagram_Cascaded.png" alt="ROSRider PID Loop, Cascaded Current Control Mode" style="width: 100%;">
  </figure>
</div>

__Next Chapter:__ [Physics](../07_PHYSICSREADME.md)