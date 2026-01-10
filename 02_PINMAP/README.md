---
layout: default
title_url: /02_PINMAP/README.html
title: "Pinmaps and Connections"
description: "Pinmaps and Connections of ROSRider Card"
---

<div class="img_dv">
  <figure class="img_fg60">
    <img src="../images/rosrider/ROSRider4D_portmap.png" alt="ROSRider Port Diagram" style="width: 100%;">
  </figure>
</div>

{% capture tab1 %}

__Left Motor Connector__

This standard 6-wire encoder motor connector is compatible with various motors and can be used with standard cables.

![Left Motor Connector](../images/pinmap/dwg_left_motor.png)

{% endcapture %}

{% capture tab2 %}

__Right Motor Connector__

The right motor connector is a mirrored version of the left motor connector.

![Right Motor Connector](../images/pinmap/dwg_right_motor.png)

{% endcapture %}

{% capture tab3 %}

__Servo Connector__

The ROSRider card can control two standard servos. The servos are powered by auxiliary power, which must be enabled via software before use.

![Servo Connector](../images/pinmap/dwg_servo.png)

{% endcapture %}

{% capture tab4 %}

__AUX Power Port__

Software controllable auxillary power port, will generate 5V when turned on.

![AUX Power Port](../images/pinmap/dwg_power_aux.png)

{% endcapture %}

{% capture tab5 %}

__Power Control Port__

This is an internal connector, that can be used to control buttons on the board remotely. The connector is not soldered, and left for future expansion.

![Power Control Port](../images/pinmap/dwg_power_control.png)

{% endcapture %}

{% capture tab6 %}

__Communications Port__

This pin header can be used for both I2C0 and serial port communication. The reset pin is decoupled with a 150nF capacitor to isolate it from the internal reset.

![Communications Port](../images/pinmap/dwg_comm.png)

{% endcapture %}

{% capture tab7 %}

__I2C QWIC Port A__

This is a standard QWIC port, which can be connected by a standard QWIC cable. The VCC from QWIC cable is used to wake up the board from hibernate state and is isolated from device.

![I2C QWIC Port A](../images/pinmap/dwg_qwic_a.png)

{% endcapture %}

{% capture tab8 %}

__I2C QWIC Port B__

There are two identical QWIC ports, that can be used to connect other QWIC devices, or piggyback another ROSRider card.

![I2C QWIC Port B](../images/pinmap/dwg_qwic_b.png)

{% endcapture %}

{% capture tab9 %}

__Serial Port__

Serial Port debug connector, this can be connected to ROS2RPI card for accessing serial port. **DO NOT** connect to a QWIC port, as this may short the power supply.

![Serial Port](../images/pinmap/dwg_serial.png)

{% endcapture %}

{% capture tab10 %}

__Power Connector__

XT30 Power Connector. Apply maximum 12V.

![Power Connector](../images/pinmap/dwg_xt30.png)

{% endcapture %}

{% capture tab11 %}

__Battery__

Use CR1225 3V coin battery.

![CR1225 Battery](../images/pinmap/dwg_cr1225.png)

{% endcapture %}

{% include tabs.html
   tab1_title="Left Motor" 
   tab1_content=tab1
   tab2_title="Right Motor" 
   tab2_content=tab2
   tab3_title="Servo" 
   tab3_content=tab3
   tab4_title="AUX" 
   tab4_content=tab4
   tab5_title="Power Control" 
   tab5_content=tab5
   tab6_title="Comms" 
   tab6_content=tab6
   tab7_title="I2C A" 
   tab7_content=tab7
   tab8_title="I2C B" 
   tab8_content=tab8
   tab9_title="Serial" 
   tab9_content=tab9
   tab10_title="Power" 
   tab10_content=tab10
   tab11_title="Battery" 
   tab11_content=tab11
%}

<div class="sl">
    <div class="sl1">
        > Connecting ROSRider to Raspberry PI
    </div>
</div>

This wiring diagram illustrates direct I2C communication with the Raspberry PI.
To enable I2C communication on a Raspberry Pi running Ubuntu, first ensure I2C is enabled by adding
`dtparam=i2c_arm=on` to `/boot/firmware/config.txt` and rebooting.  

Use jumper wires to connect your Raspberry PI using GPIO pins 2 (SDA) and 3 (SCL).
After installation, verify the connection using `sudo i2cdetect -y 1` to detect connected devices.  

<div class="img_dv">
  <figure class="img_fg_60">
    <img src="../images/rosrider/RPI5_ROSRider.png" alt="Raspberry PI Wiring Diagram" style="width: 100%;">
    <figcaption>Raspberry PI Wiring Diagram.</figcaption>
  </figure>
</div>

<div class="sl">
    <div class="sl1">
        > Detecting I2C Devices
    </div>
</div>

For troubleshooting purposes, you may want to detect I2C devices connected to your computer.
To do this, you can use the `i2cdetect` command in the terminal. Run the following command to install i2c-tools: `sudo apt install i2c-tools`.
Use the `i2cdetect` command to scan the I2C bus and identify connected devices:  

<div class="highlight notranslate position-relative">
  <div class="highlight">
    <pre id="command-i2cdetect"><span></span>sudo i2cdetect -y -r 1</pre>
  </div>
  <clipboard-copy style="position:absolute; right:8px; top:8px;" for="command-i2cdetect">
    <svg aria-hidden="true" height="16" viewBox="0 0 16 16" version="1.1" width="16" data-view-component="true" class="octicon octicon-copy js-clipboard-copy-icon">
    <path d="M0 6.75C0 5.784.784 5 1.75 5h1.5a.75.75 0 0 1 0 1.5h-1.5a.25.25 0 0 0-.25.25v7.5c0 .138.112.25.25.25h7.5a.25.25 0 0 0 .25-.25v-1.5a.75.75 0 0 1 1.5 0v1.5A1.75 1.75 0 0 1 9.25 16h-7.5A1.75 1.75 0 0 1 0 14.25Z"></path><path d="M5 1.75C5 .784 5.784 0 6.75 0h7.5C15.216 0 16 .784 16 1.75v7.5A1.75 1.75 0 0 1 14.25 11h-7.5A1.75 1.75 0 0 1 5 9.25Zm1.75-.25a.25.25 0 0 0-.25.25v7.5c0 .138.112.25.25.25h7.5a.25.25 0 0 0 .25-.25v-7.5a.25.25 0 0 0-.25-.25Z"></path>
    </svg>
    <svg aria-hidden="true" height="16" viewBox="0 0 16 16" version="1.1" width="16" data-view-component="true" class="octicon octicon-check js-clipboard-check-icon color-fg-success d-none">
    <path d="M13.78 4.22a.75.75 0 0 1 0 1.06l-7.25 7.25a.75.75 0 0 1-1.06 0L2.22 9.28a.751.751 0 0 1 .018-1.042.751.751 0 0 1 1.042-.018L6 10.94l6.72-6.72a.75.75 0 0 1 1.06 0Z"></path>
    </svg>
  </clipboard-copy>
</div>

<div style="img_dv">
   <img class="replay" src="../images/rosrider/i2c_detect.gif" alt="Detecting I2C Devices">
</div>

This command will display a table of I2C addresses. The default address of the ROSRider is `0x3C`, and the default address of the ROS2RPI card is `0x20`.
If you only have a ROSRider connected directly, you should only see the address `0x3C`.

<div class="sl">
    <div class="sl1">
        > Connecting ROSRider to NVIDIA Jetson
    </div>
</div>

The NVIDIA Jetson has two I2C ports accessible via the header.
To ensure proper communication with I2C devices, it's crucial to configure the I2C voltage level.
This is accomplished using a jumper (J514) on the board. To select 3.3V I2C, position the jumper accordingly.
To ensure proper I2C communication, verify that I2C is enabled at the kernel level and that your user account has the necessary permissions to access the I2C bus on the host computer.

There are two I2C ports on the Jetson AGX:

<div class="img_dv">
  <figure class="img_fg45">
    <img class="img_sh" src="../images/rosrider/jetson_agx_i2c_0.png" alt="Jetson AGX I2C Port 1" style="width: 100%;">
    <figcaption>Jetson AGX I2C Port 0</figcaption>
  </figure>
  <figure class="img_fg45">
    <img class="img_sh" src="../images/rosrider/jetson_agx_i2c_1.png" alt="Jetson AGX I2C Port 2" style="width: 100%;">
    <figcaption>Jetson AGX I2C Port 1</figcaption>
  </figure>
</div>

__Next Chapter:__ [Updating Firmware](../06_FIRMWARE/README.md)