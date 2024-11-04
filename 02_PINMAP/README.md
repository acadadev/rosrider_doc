---
layout: default
title_url: /02_PINMAP/README.html
title: "Pinmap"
description: "Pinmap and Connections"
---

**Board Port Diagram**

ROSRider card is equipped with number of ports and connectors, for connecting to encoder motors, and other peripherals.  
Pinmaps of each connector and port will be illustrated in detail.  

Below is a diagram of ports of ROSRider control card:  
  
[![ROSRider Pinmap](../images/ROSRider4D_portmap.png)](https://acada.dev/products)

**List of Ports**

### Left Motor Connector

![Left Motor Connector](../images/pinmap/con_left_motor.png)
| -------- | ------- |
| A        | Motor Negative |
| B        | Motor Positive |
| C        | 3V3 |
| D        | Encoder Phase B |
| E        | Encoder Phase A |
| F        | Ground |


### Right Motor Connector

| -------- | ------- |
| ![Right Motor Connector](../images/pinmap/con_right_motor.png) |
<table><thead><tr><th>Z-value</th><th>06</th></tr></thead><tbody><tr><td>Protocol</td><td>04 05</td></tr></tbody>  </table> 
|


| ![Servo Connector](../images/pinmap/con_servo.png) | Servo connector |
| ![AUX Power Port](../images/pinmap/con_power_aux.png) | AUX Power connector |
| ![Power Control Port](../images/pinmap/con_power_control.png) | Power control port |
| ![Communications Port](../images/pinmap/con_comm.png) | Communications port |
| ![QWIC A Port](../images/pinmap/con_qwic_a.png) | QWIC A Port |
| ![QWIC B Port](../images/pinmap/con_qwic_b.png) | QWIC B Port |
| ![SPI Port](../images/pinmap/con_spi.png) | SPI Port |
| ![Serial Port](../images/pinmap/con_serial.png) | Serial Port |
| ![Power Connector](../images/pinmap/con_xt30.png) | Power Connector |
| ![Battery](../images/pinmap/con_battery.png) | Battery |


__Next Chapter:__ [Connecting ROSRider to Host Computer](../03_CONNECT/README.md)