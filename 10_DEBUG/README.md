---
layout: default
title_url: /10_DEBUG/README.html
title: "Troubleshooting"
description: "Error codes and conditions on the ROSRider control card"
---

__OTHER CHAPTERS__

slam, navigation and others, must be adsorbed by new chapters, or
some chapters are overlapping.

we an skip the intro, and remove simulations. and just focus on basic robotics concepts in ros.
start the other ros2rpi doc asap, it will take little time

__TODO__

- ros2rpi config explanation, is it 0x33 for default
- couple of rosparam commands
- led blink documentation - red
- sysctl commands
- rosrider_smbus
- connecting servos
- connecting rosrider with qwic


__Troubleshooting__

[TODO: explain output and status and also that fault_left and fault_right are not problem]

__Status Registers__

#### PWR_STATUS

b7	CMD_TIMEOUT  
b6	POWER_BAD  
b5	RIGHT_AMP  
b4	LEFT_AMP  
b3	MAIN_FUSE  
b2	OVER_VOLTAGE  
b1	UNDER_VOLTAGE  
b0	AUX_PWR  

#### MTR_STATUS

b7	FAULT_RIGHT  
b6	FAULT_LEFT  
b5	RIGHT_REV (0=REV, 1=FWD)  
b4	LEFT_REV  
b3	MODE_2  
b2	MODE_1  
b1      DRIVE_MODE_MSB  
b0	DRIVE_MODE_LSB  

MODE_BRAKE = 00  
MODE_PWM   = 01  
MODE_VEL   = 10  
MODE_PID   = 11  

#### SYS_STATUS

b7	EPROM_INIT_OK=0  
b6	RESTART_REQUIRED=1  
b0	EEPROM_WRITE_WRITE_OK=0  

__Return to the introduction:__ [Introduction](../README.md)
