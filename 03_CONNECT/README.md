---
layout: default
title_url: /03_CONNECT/README.html
title: "Connecting ROSRider to Host Computer"
description: "Connecting ROSRider Control Card to Host Computer"
---

**Connecting ROSRider to Host Computer**

This wiring diagram illustrates direct I2C communication with the Raspberry PI.

<p align="center">
<img src="../images/rpi5_wiring.png" alt="Connecting to Raspberry PI diagram">
</p>

This image depicts the actual wiring configuration for direct I2C communication between the Raspberry Pi and ROSRider.

<p align="center">
<img src="../images/caretta_bare.jpg" alt="Connecting to Raspberry PI on CARETTA">
</p>

The NVIDIA Jetson has two I2C ports accessible via the header. To ensure proper communication with I2C devices, it's crucial to configure the I2C voltage level. This is accomplished using a jumper (J514) on the board. To select 3.3V I2C, position the jumper accordingly.

To ensure proper I2C communication, verify that I2C is enabled at the kernel level and that your user account has the necessary permissions to access the I2C bus on the host computer.

**Optional**

Install the `i2c-tools` package to access I2C devices:

> `sudo apt install i2c-tools`

**Detecting I2C Devices**

Use the i2cdetect command to scan the I2C bus and identify connected devices:

> sudo i2cdetect -y -r 1

You should see:

>     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f  
>00:                         -- -- -- -- -- -- -- --  
>10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --  
>20: 20 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --  
>30: -- -- -- -- -- -- -- -- -- -- -- -- 3c -- -- --  
>40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --  
>50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --  
>60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --  
>70: -- -- -- -- -- -- -- --  

0x3c is default address of ROSRider. 0x20 is the default adress of ROS2RPI card. If you have only ROSRider and connected directly, you should only see 0x3c.

After you can see ROSRider on the I2C bus, you may proceed to install drivers.

**ROS2RPI board connection**

[TODO: explanation of power cycling]
[TODO: nvidia agx connection diagram]

__Next Chapter:__ [ROS2 Drivers](../04_DRIVERS/README.md)