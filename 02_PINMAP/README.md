---
layout: default
title_url: /02_PINMAP/README.html
title: "Connections and Pinmaps"
description: "Connections and Ports of ROSRider Card"
---

<style type="text/css">

	.markdown-body h2 {
		border-bottom: 3px solid #eaecef !important;
	}

</style>

ROSRider card is equipped with number of ports and connectors, for connecting to encoder motors, and other peripherals.  
Pinmaps of each connector and port will be illustrated in detail.  

Below is a diagram of ports of ROSRider control card:  
  
[![ROSRider Pinmap](../images/ROSRider4D_portmap.png)](https://acada.dev/products)

## Left Motor Connector

This is a standard 6 wire encoder motor connector, that is compatible with many motors and can be connected with standard cables. ROSRider card can be used with dual phase or single phase encoders, and encoder type can be configured via software. If single phase encoder is used, use Encoder Phase A pin, and leave Encoder Phase B pin unconnected.

![Left Motor Connector](../images/pinmap/dwg_left_motor.png)

## Right Motor Connector

Right motor connector is mirrored version of Left motor connector.

![Right Motor Connector](../images/pinmap/dwg_right_motor.png)

## Servo Connector

ROSRider card can control two standard servos. Servos are powered via auxillary power, and before servos can be used, auxillary power must be switched on through software.

![Servo Connector](../images/pinmap/dwg_servo.png)

## AUX Power Port

Software controllable auxillary power port, will generate 5V when turned on.

![AUX Power Port](../images/pinmap/dwg_power_aux.png)

## Power Control Port

This is an internal connector, that can be used to control buttons on the board remotely. The connector is not soldered, and left for future expansion.

![Power Control Port](../images/pinmap/dwg_power_control.png)

## Communications Port

Both I2C0 and Serial Port can be accessed using this pin header. Reset pin is decoupled with a 150nF capacitor to internal reset.

![Communications Port](../images/pinmap/dwg_comm.png)

## I2C QWIC Port A

This is a standard QWIC port, which can be connected by a standard QWIC cable. The VCC from QWIC cable is used to wake up the board from hibernate state and is isolated from device.

![I2C QWIC Port A](../images/pinmap/dwg_qwic_a.png)

## I2C QWIC Port B

There are two identical QWIC ports, that can be used to connect other QWIC devices, or piggyback another ROSRider card.

![I2C QWIC Port B](../images/pinmap/dwg_qwic_b.png)

## SPI Port

SPI port has been added for future expansion, and not supported by firmware at this point.

![SPI Port](../images/pinmap/dwg_spi.png)

## Serial Port

Serial Port debug connector, this can be connected to ROS2RPI card for accessing serial port. **DO NOT** connect to a QWIC port, as this may short the power supply.

![Serial Port](../images/pinmap/dwg_serial.png)

## Power Connector

XT30 Power Connector. Apply maximum 12V.

![Power Connector](../images/pinmap/dwg_xt30.png)

## Battery

Use CR1225 3V coin battery.

![CR1225 Battery](../images/pinmap/dwg_cr1225.png)

__Next Chapter:__ [Connecting ROSRider to Host Computer](../03_CONNECT/README.md)