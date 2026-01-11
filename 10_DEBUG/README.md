---
layout: default
title_url: /10_DEBUG/README.html
title: "Troubleshooting"
description: "Error codes and conditions on the ROSRider control card"
---

### ROSRider SMBUS

The ROSRider SMBus library allows for direct control of the ROSRider board using Python, independent of standard ROS drivers.
The source code is available at [https://github.com/acadadev/rosrider_smbus](https://github.com/acadadev/rosrider_smbus)

This package includes a suite of utilities enabling you to:

 - Send System Control Commands
 - Set Parameters
 - Set POSIX time
 - Set PID and PWM Targets
 - Profile Motor Torque Constant

<div class="sl">
    <div class="sl1">
        > LED Blink Codes
    </div>
</div>

| Color | Visual Pattern | Duration | System State     | Description                                           |
|-------|----------------|----------|------------------|-------------------------------------------------------|
| Green | 3X Blink       | 50ms     | Restart Complete | The system has successfully rebooted                  |
| Red   | 3X Blink       | 50ms     | Acknowledged     | Soft Reset, Hard Reset, or Hibernate request received |
| Green | Solid ON       | N/A      | Active           | Commands have been successfully received              |
| Red   | Solid ON       | N/A      | System Disabled  | A critical status check is required                   |


A red LED indicates a system fault. While rare, these errors are usually recoverable.
Please check the status codes listed below to diagnose the problem and find the appropriate solution.  

### Status Registers

| PWR_STATUS    | Bit | Description                                   |
|---------------|-----|-----------------------------------------------|
| CMD_TIMEOUT   | b7  | Command has time out, the robot will not move |
| POWER_BAD     | b6  | Main MCU Power Supply Problem, Battery Check  |
| RIGHT_AMP     | b5  | Left AMP Limit Triggered, Issue Soft-Reset    |
| LEFT_AMP      | b4  | Right AMP Limit Triggered, Issue Soft-Reset   |
| MAIN_FUSE     | b3  | Main AMP Limit exceeded, Issue Soft-Reset     |
| OVER_VOLTAGE  | b2  | Battery Over Voltage, System Disabled         |
| UNDER_VOLTAGE | b1  | Battery Under Voltage, System Disabled        |
| AUX_PWR       | b0  | Auxillary Power Supply ON                     |

Send a SYSCTL command to ***Soft-Reset*** the board in case of over-current events (`RIGHT_AMP`, `LEFT_AMP`, `MAIN_FUSE`),
or ***Hard-Reset*** the board for critical power supply faults (`POWER_BAD`, `OVER_VOLTAGE`, `UNDER_VOLTAGE`).  

---

| MTR_STATUS     | Bit | Description                        |
|----------------|-----|------------------------------------|
| FAULT_RIGHT    | b7  | Left Motor Fault, will be ON < 12V |
| FAULT_LEFT     | b6  | Left Motor Fault, will be ON < 12V |
| RIGHT_REV      | b5  | Right Motor Connected Reverse      |
| LEFT_REV       | b4  | Left Motor Connected Reverse       |
| MODE_2         | b3  | Low Side Decay / High Side Decay   |
| MODE_1         | b2  | Coast Mode / Brake Mode            |
| DRIVE_MODE_MSB | b1  | Drive Mode Left Bit                |
| DRIVE_MODE_LSB | b0  | Drive Mode Right Bit               |

Although `FAULT_RIGHT` and `FAULT_LEFT` are monitored by the hardware (active below 12V), they are redundant in this implementation.
The software utilizes a more precise method for monitoring bus voltage and current, so these specific flags can be safely ignored.  

---

| DRIVE_MODE | Mode | Description                                                         |
|------------|------|---------------------------------------------------------------------|
| MODE_BRAKE | 0    | Brakes applied                                                      |
| MODE_PWM   | 1    | PWM Mode, Device accepts PWM Commands                               |
| MODE_VEL   | 2    | VEL Mode, Device accepts linear.x in m / s and angular.z in rad / s |
| MODE_PID   | 3    | PID Mode, Device accepts Target Velocities in rad / s               |

For standard ROS operations, use `MODE_PID`. `MODE_PWM` is reserved specifically for Torque Constant calibration.
Please note that while `MODE_VEL is functional`, it remains untested and is not currently supported by the driver.  

---

| SYS_STATUS           | Bit | Description                                                                                |
|----------------------|-----|--------------------------------------------------------------------------------------------|
| EPROM_INIT_OK = 0    | b7  | If this bit is set, EPROM has not initialized. System Disabled                             |
| RESTART_REQUIRED     | b6  | After updating certain variables, this bit will be set requesting reset from driver        |
| INITIAL_UPDATE_ERROR | b1  | If a parameter value from EEPROM can not pass input validation at startup. System Disabled |
| EEPROM_WRITE_ERROR   | b0  | Denotes EEPROM write error                                                                 |

If `INITIAL_UPDATE_ERROR` you can try to send a SYSCTL command to ***restore factory defaults***  

__OTHER CHAPTERS__

- slam
- navigation
- cartographer

__TODO__

- revisit parameters, add more description
- Procedures
- rosrider_diag, vis-pid
- rosrider_diag, parameter-manager
- rosrider_node documentation on github
- make videos of pid debug tool