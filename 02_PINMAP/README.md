---
layout: default
title_url: /02_PINMAP/README.html
title: "Connections and Pinmaps"
description: "Connections and Ports of ROSRider Card"
---

## Board Port Diagram

ROSRider card is equipped with number of ports and connectors, for connecting to encoder motors, and other peripherals.  
Pinmaps of each connector and port will be illustrated in detail.  

Below is a diagram of ports of ROSRider control card:  
  
[![ROSRider Pinmap](../images/ROSRider4D_portmap.png)](https://acada.dev/products)

## Left Motor Connector

![Left Motor Connector](../images/dwg/dwg_left_motor.png)

## Right Motor Connector

![Right Motor Connector](../images/dwg/dwg_right_motor.png)

## Servo Connector

![Servo Connector](../images/dwg/dwg_servo.png)

ROSRider card can control two standard servos. Servos are powered via auxillary power, and before servos can be used, auxillary power must be switched on through software.

## AUX Power Port

![AUX Power Port](../images/dwg/dwg_power_aux.png)

Software controllable auxillary power port, will generate 5V when turned on.

## Power Control Port

![Power Control Port](../images/dwg/dwg_power_control.png)

## Communications Port

![Communications Port](../images/dwg/dwg_comm.png)

Both I2C0 and Serial Port can be accessed using this pin header. Reset pin is decoupled with a 150nF capacitor to internal reset.

## I2C QWIC Port A

![I2C QWIC Port A](../images/dwg/dwg_qwic_a.png)

This is a standard QWIC port, which can be connected by a standard QWIC cable. There are two QWIC ports, that can be used to connect
other QWIC devices, or piggyback another ROSRider card. The VCC from QWIC cable is used to wake up the board from hibernate state and is isolated from device.

## I2C QWIC Port B

![I2C QWIC Port B](../images/dwg/dwg_qwic_b.png)

## SPI Port

![SPI Port](../images/dwg/dwg_spi.png)

SPI port has been added for future expansion, and not supported by firmware at this point.

## Serial Port

![Serial Port](../images/dwg/dwg_serial.png)

Serial Port debug connector, this can be connected to ROS2RPI card for accessing serial port. **DO NOT** connect to a QWIC port, as this may short the power supply.

## Power Connector

![Power Connector](../images/dwg/dwg_xt30.png)

XT30 Power Connector. Apply maximum 12V.

## Battery

![CR1225 Battery](../images/dwg/dwg_battery.png)

Use CR1225 coin battery.

__Next Chapter:__ [Connecting ROSRider to Host Computer](../03_CONNECT/README.md)