---
layout: default
title_url: /10_DEBUG/README.html
title: "Troubleshooting"
description: "Error codes and conditions on the ROSRider control card"
---

<div class="sl">
    <div class="sl1">
        > Troubleshooting
    </div>
</div>

__OTHER CHAPTERS__

- slam
- navigation
- cartographer


__Connecting Servos__

TODO:

__LED Blink Codes__


| Color | Visual Pattern | Duration | System State Description                                                     |
|-------|----------------|----------|------------------------------------------------------------------------------|
| Green | 3x Blink       | 50ms     | Restart Complete: The system has successfully rebooted.                      |
| Red   | 3x Blink       | 50ms     | Request Acknowledged: Soft Reset, Hard Reset, or Hibernate request received. |
| Green | Solid ON       | N/A      | Active: Commands have been successfully received.                            |
| Red   | Solid ON       | N/A      | System Disabled: A critical status check is required.                        |

__ROSRider SMBUS__

Python code to send commands to ROSRider Board is available at [https://github.com/acadadev/rosrider_smbus](https://github.com/acadadev/rosrider_smbus)
ROSRider SMBus enables you to write your own python programs to control ROSRider board without the ROS drivers.  

Also a set of utilities is included that allows you to:

 - Send System Control Commands
 - Set Parameters
 - Set POSIX time
 - Set PID and PWM targets
 - Profile Motor Torque Constant

__TODO__

- make videos of pid debug tool and put some screenshots between things, maybe even a video.
- reamp parameters

- if initial update fail, then what happens?

- ros2rpi config explanation, is it 0x33 for default
-  Explanation of parameter 0x33 in rosrider, 0x0 to cancel.


__Troubleshooting__

[TODO: explain output and status and also that fault_left and fault_right are not problem]

__Status Registers__

***PWR_STATUS***

| Bit | Function      |
|-----|---------------|
| b7  | CMD_TIMEOUT   | 
| b6  | POWER_BAD     |
| b5  | RIGHT_AMP     | 
| b4  | LEFT_AMP      |
| b3  | MAIN_FUSE     |
| b2  | OVER_VOLTAGE  |
| b1  | UNDER_VOLTAGE |  
| b0  | AUX_PWR       |

***MTR_STATUS***

| Bit | Function       |
|-----|----------------|
| b7  | FAULT_RIGHT    | 
| b6  | FAULT_LEFT     |
| b5  | RIGHT_REV      | 
| b4  | LEFT_REV       |
| b3  | MODE_2         |
| b2  | MODE_1         |
| b1  | DRIVE_MODE_MSB |  
| b0  | DRIVE_MODE_LSB |

***DRIVE_MODE***

MODE_BRAKE = 00  
MODE_PWM   = 01  
MODE_VEL   = 10  
MODE_PID   = 11  

***SYS_STATUS***

| Bit | Function                  |
|-----|---------------------------|
| b7  | EPROM_INIT_OK = 0         | 
| b6  | RESTART_REQUIRED = 1      |
| b1  | INITIAL_UPDATE_ERROR = 1  |  
| b0  | EEPROM_WRITE_WRITE_OK = 0 |


__Return to the introduction:__ [Introduction](../README.md)
