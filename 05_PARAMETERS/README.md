---
layout: default
title_url: /01_INTRO/README.html
title: "Parameters"
description: "Parametric Configuration of the ROSRider control card"
---



'''
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
'''

__Next Chapter:__ [Updating Firmware](../06_FIRMWARE/README.md)