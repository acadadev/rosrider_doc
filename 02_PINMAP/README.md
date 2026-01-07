---
layout: default
title_url: /02_PINMAP/README.html
title: "Connections and Pinmaps"
description: "Connections and Pinmaps of ROSRider Card"
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
        > The Blueprint for Your Build.
    </div>
    <div style="font-size: 1.1em; font-style: italic; line-height: 1.5;">
        "Detailed pinouts and electrical specifications."
    </div>
</div>

{% capture tab1 %}

![ROSRider Port Diagram](../images/ROSRider4D_portmap.png)

{% endcapture %}

{% capture tab2 %}

__Left Motor Connector__

This standard 6-wire encoder motor connector is compatible with various motors and can be used with standard cables. The ROSRider card supports both dual-phase and single-phase encoders, with the encoder type configurable via software. For single-phase encoders, connect to the Encoder Phase A pin and leave Encoder Phase B unconnected.  

![Left Motor Connector](../images/pinmap/dwg_left_motor.png)

{% endcapture %}

{% capture tab3 %}

__Right Motor Connector__

The right motor connector is a mirrored version of the left motor connector.

![Right Motor Connector](../images/pinmap/dwg_right_motor.png)

{% endcapture %}

{% capture tab4 %}

__Servo Connector__

The ROSRider card can control two standard servos. The servos are powered by auxiliary power, which must be enabled via software before use.

![Servo Connector](../images/pinmap/dwg_servo.png)

{% endcapture %}

{% capture tab5 %}

__AUX Power Port__

Software controllable auxillary power port, will generate 5V when turned on.

![AUX Power Port](../images/pinmap/dwg_power_aux.png)

{% endcapture %}

{% capture tab6 %}

__Power Control Port__

This is an internal connector, that can be used to control buttons on the board remotely. The connector is not soldered, and left for future expansion.

![Power Control Port](../images/pinmap/dwg_power_control.png)

{% endcapture %}

{% capture tab7 %}

__Communications Port__

This pin header can be used for both I2C0 and serial port communication. The reset pin is decoupled with a 150nF capacitor to isolate it from the internal reset.

![Communications Port](../images/pinmap/dwg_comm.png)

{% endcapture %}

{% capture tab8 %}

__I2C QWIC Port A__

This is a standard QWIC port, which can be connected by a standard QWIC cable. The VCC from QWIC cable is used to wake up the board from hibernate state and is isolated from device.

![I2C QWIC Port A](../images/pinmap/dwg_qwic_a.png)

{% endcapture %}

{% capture tab9 %}

__I2C QWIC Port B__

There are two identical QWIC ports, that can be used to connect other QWIC devices, or piggyback another ROSRider card.

![I2C QWIC Port B](../images/pinmap/dwg_qwic_b.png)

{% endcapture %}

{% capture tab10 %}

__Serial Port__

Serial Port debug connector, this can be connected to ROS2RPI card for accessing serial port. **DO NOT** connect to a QWIC port, as this may short the power supply.

![Serial Port](../images/pinmap/dwg_serial.png)

{% endcapture %}

{% capture tab11 %}

__Power Connector__

XT30 Power Connector. Apply maximum 12V.

![Power Connector](../images/pinmap/dwg_xt30.png)

{% endcapture %}

{% capture tab12 %}

__Battery__

Use CR1225 3V coin battery.

![CR1225 Battery](../images/pinmap/dwg_cr1225.png)

{% endcapture %}

{% include tabs.html 
   tab1_title="Diagram" 
   tab1_content=tab1
   tab2_title="Left" 
   tab2_content=tab2
   tab3_title="Right" 
   tab3_content=tab3
   tab4_title="Servo" 
   tab4_content=tab4
   tab5_title="AUX" 
   tab5_content=tab5
   tab6_title="Power Control" 
   tab6_content=tab6
   tab7_title="Comms" 
   tab7_content=tab7
   tab8_title="I2C A" 
   tab8_content=tab8
   tab9_title="I2C B" 
   tab9_content=tab9
   tab10_title="Serial" 
   tab10_content=tab10
   tab11_title="Power" 
   tab11_content=tab11
   tab12_title="Battery" 
   tab12_content=tab12
%}

The subsequent chapter provides clear diagrams and instructions for connecting to popular platforms like Raspberry Pi and NVIDIA Jetson.

__Next Chapter:__ [Connecting ROSRider to Host Computer](../03_CONNECT/README.md)