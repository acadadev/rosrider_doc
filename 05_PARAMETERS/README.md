---
layout: default
title_url: /01_INTRO/README.html
title: "Parameters"
description: "Parametric Configuration of the ROSRider control card"
---

A YAML file is a human-readable text format used to store configuration parameters in ROS. The provided YAML file defines parameters that influence the behavior of the ROSRider node, such as motor control, sensor settings, and communication protocols.

The ROS Parameter Server is a mechanism that allows you to dynamically configure parameters in your ROS nodes. When you launch a ROS node with a YAML file, the parameters defined in the file are loaded into the ROS Parameter Server. This makes them accessible to other nodes and allows you to modify them at runtime using tools like rosparam or rqt_reconfigure.

The ROSRider node reads the configuration parameters from the YAML file and stores them in its non-volatile memory (EEPROM). This allows the ROSRider to retain its configuration settings even after a power cycle. Dynamic parameters, such as PID control gains, can be adjusted on-the-fly using tools like `rosparam` or `rqt_reconfigure`, and the changes are immediately applied to the ROSRider.

The following YAML file defines the configuration parameters for the ROSRider node. This file is crucial for customizing the behavior and performance of the ROSRider, tailoring it to specific robotic applications.

```yaml
/rosrider_node:
  ros__parameters:
    I2C_ENABLED: True
    ODOM_FRAME_ID: 'odom'
    BASE_FRAME_ID: 'base_footprint'
    BROADCAST_TF2: True
    PUB_ODOMETRY: True
    PUB_JOINTS: True
    PUB_DIAGNOSTICS: True
    ROS2RPI_CONFIG: 0x33
    AUTO_SYNC: True
    DEBUG: False
    CONFIG_FLAGS: 48
    UPDATE_RATE: 20
    ALLOWED_SKIP: 3
    PWM_DIV: 64
    DRIVE_MODE: 3
    MONITOR_RATE: 100
    PWM_SCALE: 256
    PWM_FRQ: 50
    MAX_IDLE_SECONDS: 3600
    UPPER_LIMIT: 255
    INTEGRAL_LIMIT: 224
    ENCODER_PPR: 48
    RTC_TRIM: 0x7FFF
    LEFT_FORWARD_DEADZONE: 8
    LEFT_REVERSE_DEADZONE: 8
    RIGHT_FORWARD_DEADZONE: 8
    RIGHT_REVERSE_DEADZONE: 8
    GEAR_RATIO: 65.0
    WHEEL_DIA: 0.0685
    BASE_WIDTH: 0.160
    MAIN_AMP_LIMIT: 3.6
    BAT_VOLTS_HIGH: 15.0
    BAT_VOLTS_LOW: 6.0
    MAX_RPM: 120.0
    LEFT_AMP_LIMIT: 1.6
    RIGHT_AMP_LIMIT: 1.6
    LEFT_KP: 1.2
    LEFT_KI: 0.8
    LEFT_KD: 0.01
    RIGHT_KP: 1.2
    RIGHT_KI: 0.8
    RIGHT_KD: 0.01
    GAIN: 1.0
    TRIM: 0.0
    MOTOR_CONSTANT: 1.0
```

**General configuration**

| Parameter | Type | Explanation | Default |
| -------- | -------- | -------- | -------- |
| I2C_ENABLED  | bool | Enables or disables I2C communication. For development only. | True |
| ODOM_FRAME_ID | string | Sets the frame ID for the odometry data. | `odom` |
| BASE_FRAME_ID | string | Sets the frame ID for the robot's base frame. | `base_footprint` |
| BROADCAST_TF2 | bool | Enables or disables TF2 broadcast. | True |
| PUB_ODOMETRY | bool | Enables or disables odometry data publication. | True |
| PUB_JOINTS | bool | Enables or disables joint state publication. | True |
| PUB_DIAGNOSTICS | bool | Enables or disables diagnostic data publication. | True |
| ROS2RPI_CONFIG | uint8 | Configuration for the ROS2RPI board (if used). | 0x33 |
| AUTO_SYNC | bool | Enables automatic clock synchronization. | True |
| DEBUG | bool | Enables or disables debug mode. | False |
| CONFIG_FLAGS | uint8 | Configuration flags for specific features. | 48 |
| UPDATE_RATE | uint8 | Desired update rate for the control loop. | 10 |
| ALLOWED_SKIP | uint8 | Maximum number of allowed skipped data packets. | 3 |
| PWM_DIV | uint8 | PWM Frequency divider | 64 |
| DRIVE_MODE | uint8 | Drive mode configuration. | 3 |
| MONITOR_RATE | uint8 | Rate at which current sensor data is monitored. | 100 |
| PWM_SCALE | uint16 | PWM scaling factor. | 256 |
| PWM_FRQ | uint16 | PWM frequency. |  50 |
| MAX_IDLE_SECONDS | uint16 | Maximum idle seconds before entering hibernate mode. | 3600 |
| RTC_TRIM | uint32 | Real-Time Clock trim value. | 0x7FFF |

**Motor Control**

| Parameter | Type | Explanation | Default |
| -------- | -------- | -------- | -------- |
| UPPER_LIMIT | uint16 | Maximum PWM output value. | 255 |
| INTEGRAL_LIMIT | uint16 |  Integral term limit for PID control. | 224 |
| ENCODER_PPR | uint16 | Encoder pulses per revolution. | 48 |
| LEFT_FORWARD_DEADZONE | int16 | Deadzone for the left motor's forward direction. | 8 | 
| LEFT_REVERSE_DEADZONE | int16 | Deadzone for the left motor's reverse direction. | 8 |
| RIGHT_FORWARD_DEADZONE | int16 | Deadzone for the right motor's forward direction. | 8 |
| RIGHT_REVERSE_DEADZONE | int16 | Deadzone for the right motor's reverse direction. | 8 |

**Odometry Configuration**

| Parameter | Type | Explanation | Default |
| -------- | -------- | -------- | -------- |
| GEAR_RATIO | float | Gear ratio of the motors. | 65.0 |
| WHEEL_DIA | float | Diameter of the wheels. | 0.0685 |
| BASE_WIDTH | float | Distance between the wheels. | 0.168 |

**Power and Safety Limits**

| Parameter | Type | Explanation | Default |
| -------- | -------- | -------- | -------- |
| MAIN_AMP_LIMIT | float | Maximum current draw for the main power supply. |
| BAT_VOLTS_HIGH | float | Maximum battery voltage. |
| BAT_VOLTS_LOW | float | Minimum battery voltage. |
| MAX_RPM | float | Maximum motor RPM. |
| LEFT_AMP_LIMIT | float | Maximum current limit for the left motor. |
| RIGHT_AMP_LIMIT | float | Maximum current limit for the right motor. |

**PID Control Parameters**

| Parameter | Type | Explanation | Default |
| -------- | -------- | -------- | -------- |
| LEFT_KP | float | PID proportional for left motor. | 1.2 |
| LEFT_KI | float | PID integral for left motor. | 0.8 |
| LEFT_KD | float | PID differential for left motor. | 0.01 |
| RIGHT_KP | float | PID proportional for right motor. | 1.2 |
| RIGHT_KI | float | PID integral for right motor. | 0.8 |
| RIGHT_KD | float | PID differential for right motor. | 0.01 |

**Drive Trim Parameters**







| GAIN | float |
| TRIM | float |
| MOTOR_CONSTANT | float |

gain trim motor constant explanied

add defaults, maybe
config_flags explanation
ros2rpi config explanation
allowed skip before what

explain pwm frequency and pwm divider

is it 0x33 for default

word about datatypes 





__Next Chapter:__ [Updating Firmware](../06_FIRMWARE/README.md)