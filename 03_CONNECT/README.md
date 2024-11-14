---
layout: default
title_url: /03_CONNECT/README.html
title: "Connecting ROSRider to Host Computer"
description: "Connecting ROSRider Control Card to Host Computer"
---

**Connecting ROSRider to Raspberry PI**

This wiring diagram illustrates direct I2C communication with the Raspberry PI.

<div style="display: flex; justify-content: space-around; margin: 25px 0;">
   <img src="../images/rpi4b_wiring.png" alt="Connecting to Raspberry PI diagram" style="box-shadow: 0px 4px 8px rgba(0, 0, 0, 0.2);">
</div>

This image depicts the actual wiring configuration for direct I2C communication between the Raspberry Pi and ROSRider.

<div style="display: flex; margin: 25px 0;">
   <img src="../images/caretta_bare.jpg" alt="Connecting to Raspberry PI" style="width:50%; box-shadow: 0px 4px 8px rgba(0, 0, 0, 0.2);">
</div>

**Connecting ROSRider to NVIDIA Jetson**

The NVIDIA Jetson has two I2C ports accessible via the header. To ensure proper communication with I2C devices, it's crucial to configure the I2C voltage level. This is accomplished using a jumper (J514) on the board. To select 3.3V I2C, position the jumper accordingly.

To ensure proper I2C communication, verify that I2C is enabled at the kernel level and that your user account has the necessary permissions to access the I2C bus on the host computer.

There are two I2C ports on the Jetson AGX:

<div style="display: flex; margin: 25px 0;">
   <img src="../images/jetson_agx_i2c_0.png" alt="Jetson AGX I2C Port 1" style="width:48%; box-shadow: 0px 4px 8px rgba(0, 0, 0, 0.2);">
   <img src="../images/jetson_agx_i2c_1.png" alt="Jetson AGX I2C Port 2" style="width:48%; box-shadow: 0px 4px 8px rgba(0, 0, 0, 0.2);">
</div>

Here is an excellent video on connecting I2C devices to Jetson: <a href="https://www.youtube.com/watch?v=7_H7tzcdBMU]" title="I2C - NVIDIA Jetson AGX Xavier">https://www.youtube.com/watch?v=7_H7tzcdBMU]</a>. This video provides a comprehensive guide on the steps involved, from identifying I2C pins to configuring software and starting device usage.


**Detecting I2C Devices**

For troubleshooting purposes, you may want to detect I2C devices connected to your computer. To do this, you can use the `i2cdetect` command in the terminal.

Install the `i2c-tools` package to access I2C devices:

```console
sudo apt install i2c-tools
```

Use the i2cdetect command to scan the I2C bus and identify connected devices:

```console
sudo i2cdetect -y -r 1
```

<div style="display: flex; margin: 25px 0;">
<img src="../images/i2cdetect.png" alt="Detecting I2C Devices" style="box-shadow: 0px 4px 8px rgba(0, 0, 0, 0.2);">
</div>

This command will display a table of I2C addresses. The default address of the ROSRider is `0x3c`, and the default address of the ROS2RPI card is `0x20`.

Note: If you only have a ROSRider connected directly, you should only see the address `0x3c`.

Once you've confirmed the presence of the ROSRider on the I2C bus, you can proceed with installing the necessary drivers.

__Next Chapter:__ [ROS2 Drivers](../04_DRIVERS/README.md)