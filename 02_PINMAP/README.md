---
layout: default
title_url: /02_PINMAP/README.html
title: "Connections, Pinmaps and Hardware Specifications"
description: "Connections and Specifications of ROSRider Card"
---

<style type="text/css">

	.markdown-body h2 {
		border-bottom: 3px solid #eaecef !important;
	}

</style>

Equipped with multiple ports and connectors, the ROSRider card enables connection to encoder motors and other peripherals. Pinout diagrams for each port are shown below:
  
[![ROSRider Pinmap](../images/ROSRider4D_portmap.png)](https://acada.dev/products)

### Left Motor Connector

This standard 6-wire encoder motor connector is compatible with various motors and can be used with standard cables. The ROSRider card supports both dual-phase and single-phase encoders, with the encoder type configurable via software. For single-phase encoders, connect to the Encoder Phase A pin and leave Encoder Phase B unconnected.

![Left Motor Connector](../images/pinmap/dwg_left_motor.png)

### Right Motor Connector

The right motor connector is a mirrored version of the left motor connector.

![Right Motor Connector](../images/pinmap/dwg_right_motor.png)

### Servo Connector

The ROSRider card can control two standard servos. The servos are powered by auxiliary power, which must be enabled via software before use.

![Servo Connector](../images/pinmap/dwg_servo.png)

### AUX Power Port

Software controllable auxillary power port, will generate 5V when turned on.

![AUX Power Port](../images/pinmap/dwg_power_aux.png)

### Power Control Port

This is an internal connector, that can be used to control buttons on the board remotely. The connector is not soldered, and left for future expansion.

![Power Control Port](../images/pinmap/dwg_power_control.png)

### Communications Port

This pin header can be used for both I2C0 and serial port communication. The reset pin is decoupled with a 150nF capacitor to isolate it from the internal reset.

![Communications Port](../images/pinmap/dwg_comm.png)

### I2C QWIC Port A

This is a standard QWIC port, which can be connected by a standard QWIC cable. The VCC from QWIC cable is used to wake up the board from hibernate state and is isolated from device.

![I2C QWIC Port A](../images/pinmap/dwg_qwic_a.png)

### I2C QWIC Port B

There are two identical QWIC ports, that can be used to connect other QWIC devices, or piggyback another ROSRider card.

![I2C QWIC Port B](../images/pinmap/dwg_qwic_b.png)

### SPI Port

SPI port has been added for future expansion, and not supported by firmware at this point.

![SPI Port](../images/pinmap/dwg_spi.png)

### Serial Port

Serial Port debug connector, this can be connected to ROS2RPI card for accessing serial port. **DO NOT** connect to a QWIC port, as this may short the power supply.

![Serial Port](../images/pinmap/dwg_serial.png)

### Power Connector

XT30 Power Connector. Apply maximum 12V.

![Power Connector](../images/pinmap/dwg_xt30.png)

### Battery

Use CR1225 3V coin battery.

![CR1225 Battery](../images/pinmap/dwg_cr1225.png)

Armed with the knowledge of the pinmap, the next step is to physically connect ROSRider to your hardware. The subsequent chapter provides clear diagrams and instructions for connecting to popular platforms like Raspberry Pi and NVIDIA Jetson.

__Next Chapter:__ [Connecting ROSRider to Host Computer](../03_CONNECT/README.md)