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

Two standard servos can be controlled separately then main locomotion. `AUX_PWR` must be on in order for servos to power up.

__LED Blink Codes__

| Color | Visual Pattern | Duration | System State Description                                                     |
|-------|----------------|----------|------------------------------------------------------------------------------|
| Green | 3x Blink       | 50ms     | Restart Complete: The system has successfully rebooted.                      |
| Red   | 3x Blink       | 50ms     | Request Acknowledged: Soft Reset, Hard Reset, or Hibernate request received. |
| Green | Solid ON       | N/A      | Active: Commands have been successfully received.                            |
| Red   | Solid ON       | N/A      | System Disabled: A critical status check is required.                        |

// TODO: check status by using rosrider_smbus
// TODO: rarely, an initial update problem can lead to system disabled
// TODO: if system disabled,  it can also be initial update failed. rosrider_smbus read_status.py will tell you exactly what is happening.

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
- revisit parameters, add more description
- explain friction and feedforwards

__Troubleshooting__

// TODO: FAULT_LEFT and FAULT_RIGHT after

__Status Registers__

***PWR_STATUS***

| Bit | Function      | Description                                   |
|-----|---------------|-----------------------------------------------|
| b7  | CMD_TIMEOUT   | Command has time out, the robot will not move |
| b6  | POWER_BAD     | Main MCU Power Supply Problem, Battery Check  |
| b5  | RIGHT_AMP     | Left AMP Limit Triggered, Issue Soft-Reset    |
| b4  | LEFT_AMP      | Right AMP Limit Triggered, Issue Soft-Reset   |
| b3  | MAIN_FUSE     | Main AMP Limit exceeded, Issue Soft-Reset     |
| b2  | OVER_VOLTAGE  | Battery Over Voltage, System Disabled         |
| b1  | UNDER_VOLTAGE | Battery Under Voltage, System Disabled        |
| b0  | AUX_PWR       | Auxillary Power Supply ON                     |

***MTR_STATUS***

| Bit | Function       | Description                        |
|-----|----------------|------------------------------------|
| b7  | FAULT_RIGHT    | Left Motor Fault, will be ON < 12V |
| b6  | FAULT_LEFT     | Left Motor Fault, will be ON < 12V |
| b5  | RIGHT_REV      | Right Motor Connected Reverse      |
| b4  | LEFT_REV       | Left Motor Connected Reverse       |
| b3  | MODE_2         | Low Side Decay / High Side Decay   |
| b2  | MODE_1         | Coast Mode / Brake Mode            |
| b1  | DRIVE_MODE_MSB | Drive Mode Left Bit                |
| b0  | DRIVE_MODE_LSB | Drive Mode Right Bit               |

***DRIVE_MODE***

| Value | Mode       | Description                                                         |
|-------|------------|---------------------------------------------------------------------|
| 0     | MODE_BRAKE | Brakes applied                                                      |
| 1     | MODE_PWM   | PWM Mode, Device accepts PWM Commands                               |
| 2     | MODE_VEL   | VEL Mode, Device accepts linear.x in m / s and angular.z in rad / s |
| 3     | MODE_PID   | PID Mode, Device accepts Target Velocities in rad / s               |

***SYS_STATUS***

| Bit | Function             | Description                                                                                |
|-----|----------------------|--------------------------------------------------------------------------------------------|
| b7  | EPROM_INIT_OK = 0    | If this bit is set, EPROM has not initialized. System Disabled                             |
| b6  | RESTART_REQUIRED     | After updating certain variables, this bit will be set requesting reset from driver        |
| b1  | INITIAL_UPDATE_ERROR | If a parameter value from EEPROM can not pass input validation at startup. System Disabled |
| b0  | EEPROM_WRITE_ERROR   | Denotes EEPROM write error                                                                 |

__Return to the introduction:__ [Introduction](../README.md)