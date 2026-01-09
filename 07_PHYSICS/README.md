---
layout: default
title_url: /07_PHYSICS/README.html
title: "Physics"
description: "Feedforwards, Friction Model and Cascaded Current Control Explained"
---

<div class="sl">
    <div class="sl1">
        > Physics
    </div>
</div>

### Velocity Feedforward

Velocity feedforward prevents the controller from lagging by proactively supplying the voltage necessary
to counteract the motor’s Back Electromotive Force (Back-EMF) at a given speed.
In standard operation, this acts as a direct voltage adder proportional to the target velocity.  

Crucially, in ***Cascaded mode***, this feedforward is not ignored but rather architecturally
shifted to the inner loop to improve physical accuracy; because the motor’s generated
Back-EMF naturally "fights back" against the driving current, the inner loop explicitly calculates and 
injects a Back-EMF compensation term (`EB_ff`).  

This ensures that the voltage required to simply maintain speed is supplied immediately, 
allowing the current controller to focus entirely on torque generation rather
than wasting gain trying to overcome the motor's own generated voltage.

### Acceleration Feedforward

Acceleration feedforward compensates for the system's mechanical inertia by injecting an
immediate control effort whenever a change in velocity is requested,
rather than waiting for a position or velocity error to accumulate.  

In the standard control mode, this is applied as a direct voltage
contribution (`ff_acceleration`) calculated from the rate of change
in the target speed.  

When the controller operates in Cascaded mode, this logic adapts by converting
the acceleration demand into an equivalent current target (`control_effort_amps`); 
using the motor's armature resistance (`R_arm`), the system calculates exactly
how much extra current is required to generate the torque needed for that specific acceleration,
adding this directly to the inner loop’s reference target.  

### Friction Model


__Next Chapter:__ [Troubleshooting](../10_DEBUG/README.md)