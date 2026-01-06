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


{% capture tab1 %}
```yaml
### DRIVER config
DRIVE_MODE: 3
CONFIG_FLAGS: 112
UPDATE_RATE: 20
### PWM config  
PWM_DIV: 16
PWM_SCALE: 256
PWM_FRQ: 1000
```
{% endcapture %}

{% capture tab2 %}
```yaml
### MOTOR config  
GEAR_RATIO: 65.0
ENCODER_PPR: 48
WHEEL_DIA: 0.0685
BASE_WIDTH: 0.174
MAX_RPM: 160.0
```
{% endcapture %}

{% capture tab3 %}
```yaml
### PID config  
UPPER_LIMIT: 240
LEFT_KP: 8.0
LEFT_KI: 6.0
LEFT_KD: 0.0
RIGHT_KP: 8.0
RIGHT_KI: 6.0
RIGHT_KD: 0.0
TANH_DIV: 2.0
SIGM_DIV: 10.0
### Anti windup
K_FB_WINDUP: 0.5
### Feed forwards
K_FF_VEL: 0.12
K_FF_ACCEL: 0.08
```
{% endcapture %}

{% capture tab4 %}
```yaml
### CASCADED loop
INNER_LIMIT: 240
CURRENT_KP: 8.0
CURRENT_KI: 6.0
CURRENT_MULTIPLIER_LEFT: 4.8
CURRENT_MULTIPLIER_RIGHT: 4.8
CURRENT_OMEGA_K_LEFT: -2.4
CURRENT_OMEGA_K_RIGHT: -2.4
R_ARM: 2.0
### TORQUE constant
LEFT_KT: 0.016
LEFT_KT_W: -0.008
RIGHT_KT: 0.016
RIGHT_KT_W: -0.008
```
{% endcapture %}

{% capture tab5 %}
```yaml
# stribeck friction model  
STATIC_KICK: 0.8  
COULOMB_RUN: 0.2  
STRIBECK_WIDTH: 2.1  
VISCOUS_FRICTION: 0.064  
VISCOUS_FRICTION_LIMIT: 1.2  
EB_FF_LIMIT: 12.0  
SCV_OMEGA_THRESHOLD: 0.05  
SCV_LATCH_THRESHOLD: 1.0
# dead zones  
LEFT_FORWARD_DEADZONE: 12  
LEFT_REVERSE_DEADZONE: 12  
RIGHT_FORWARD_DEADZONE: 12  
RIGHT_REVERSE_DEADZONE: 12
```
{% endcapture %}

{% capture tab6 %}
```yaml
# trim model
TRIM_GAIN: 1.0
TRIM_MOTOR_K: 1.0
TRIM_CONSTANT: 0.0
```
{% endcapture %}

{% capture tab7 %}
```yaml
# electrical limits
MAIN_AMP_LIMIT: 3.6
BAT_VOLTS_HIGH: 15.0
BAT_VOLTS_LOW: 6.0
LEFT_AMP_LIMIT: 2.4
RIGHT_AMP_LIMIT: 2.4
INA219_CAL: 8192
# adc bias calibration
CS_LEFT_OFFSET: 0
CS_RIGHT_OFFSET: 0
```
{% endcapture %}

{% capture tab8 %}
T8
{% endcapture %}

{% capture tab9 %}

{% endcapture %}

{% capture tab10 %}
```yaml
# filtering
VOLTAGE_FILTER: True
BEMF_FILTERED_OMEGA: True
PID_FILTERED_OMEGA: True
SCV_FILTERED_OMEGA: True
CURRENT_OMEGA_FILTER: True
```

{% endcapture %}

{% capture tab11 %}
```yaml
# features
AUTO_SYNC: True
AUTO_BIAS: True
AUTO_BRAKE: False
# adc
ADC_SYNC: True
ADC_MULTIPHASE: True
ADC_BIPHASE: False
# inner loop
CASCADED: True
# physics, acceleration, velocity and friction
OUTER_FEEDFORWARD: True
OUTER_SCV: True
# experimental
CROSS_COUPLED_CONTROL: True
```
{% endcapture %}

{% capture tab12 %}
```yaml
# filter configs
OMEGA_FILTER_TYPE: 1
CURRENT_FILTER_TYPE: 3
OUTPUT_FILTER_TYPE: 0
# synchronization
SYNC_KP: 256
SYNC_KI: 4
SYNC_LIMIT: 4096
SYNC_INTERVAL: 8
DT_I2C: 32
DT_THRESHOLD: 2
# experimental
CROSS_KP: 4.0
CROSS_K_LEFT: 1.0
CROSS_K_RIGHT: 1.0
# topic parameters
CMD_VEL_TOPIC: 'cmd_vel_nav'
# system parameters
I2C_ENABLED: True
ODOM_FRAME_ID: 'odom'
BASE_FRAME_ID: 'base_footprint'
BROADCAST_TF2: True
PUB_ODOMETRY: True
PUB_JOINTS: True
PUB_DIAGNOSTICS: True
ROS2RPI_CONFIG: 0x33 # 0x00 # 0x0F # 0x33
I2C_ADDRESS: 0x3c
DEBUG: False
RTC_TRIM: 0x7FFF
ALLOWED_SKIP: 3
MONITOR_RATE: 100
MAX_IDLE_SECONDS: 1800
```
{% endcapture %}

{% include tabs.html 
   tab1_title="Driver & PWM" 
   tab1_content=tab1
   tab2_title="Motor" 
   tab2_content=tab2
   tab3_title="PID" 
   tab3_content=tab3
   tab4_title="CASCADED" 
   tab4_content=tab4
   tab5_title="Friction/Deadzone" 
   tab5_content=tab5
   tab6_title="Trim Model" 
   tab6_content=tab6
   tab7_title="Electrical" 
   tab7_content=tab7
   tab8_title="Calib/Trim" 
   tab8_content=tab8
   tab9_title="Booleans" 
   tab9_content=tab9
   tab10_title="Filters" 
   tab10_content=tab10
   tab11_title="Sync/Exp" 
   tab11_content=tab11
   tab12_title="System" 
   tab12_content=tab12 
%}

```yaml

```

__General configuration__

| parameter | type | description | default |
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
| ALLOWED_SKIP | uint8 | Command timeout in units of 1 / UPDATE_RATE. | 3 |
| PWM_DIV | uint8 | PWM Frequency divider | 64 |
| DRIVE_MODE | uint8 | Drive mode configuration. | 3 |
| MONITOR_RATE | uint8 | Rate at which current sensor data is monitored. | 100 |
| PWM_SCALE | uint16 | PWM scaling factor. | 256 |
| PWM_FRQ | uint16 | PWM frequency. |  50 |
| MAX_IDLE_SECONDS | uint16 | Maximum idle seconds before entering hibernate mode. | 3600 |
| RTC_TRIM | uint32 | Real-Time Clock trim value. | 0x7FFF |

__Motor Control__

| parameter | type | description | default |
| -------- | -------- | -------- | -------- |
| UPPER_LIMIT | uint16 | Maximum PWM output value. | 255 |
| INTEGRAL_LIMIT | uint16 |  Integral term limit for PID control. | 224 |
| ENCODER_PPR | uint16 | Encoder pulses per revolution. | 48 |
| LEFT_FORWARD_DEADZONE | int16 | Deadzone for the left motor's forward direction. | 8 | 
| LEFT_REVERSE_DEADZONE | int16 | Deadzone for the left motor's reverse direction. | 8 |
| RIGHT_FORWARD_DEADZONE | int16 | Deadzone for the right motor's forward direction. | 8 |
| RIGHT_REVERSE_DEADZONE | int16 | Deadzone for the right motor's reverse direction. | 8 |

__Odometry Configuration__

| parameter | type | description | default |
| -------- | -------- | -------- | -------- |
| GEAR_RATIO | float | Gear ratio of the motors. | 65.0 |
| WHEEL_DIA | float | Diameter of the wheels. | 0.0685 |
| BASE_WIDTH | float | Distance between the wheels. | 0.168 |

__Power and Safety Limits__

| parameter | type | description | default |
| -------- | -------- | -------- | -------- |
| MAIN_AMP_LIMIT | float | Maximum current draw for the main power supply. | 3.6 |
| BAT_VOLTS_HIGH | float | Maximum battery voltage. | 15.0 |
| BAT_VOLTS_LOW | float | Minimum battery voltage. | 6.0 |
| MAX_RPM | float | Maximum motor RPM. | 120.0 |
| LEFT_AMP_LIMIT | float | Maximum current limit for the left motor. | 1.6 |
| RIGHT_AMP_LIMIT | float | Maximum current limit for the right motor. | 1.6 |

__PID Control Parameters__

| parameter | type | description | default |
| -------- | -------- | -------- | -------- 
| LEFT_KP | float | PID proportional for left motor. | 1.2 |
| LEFT_KI | float | PID integral for left motor. | 0.8 |
| LEFT_KD | float | PID differential for left motor. | 0.01 |
| RIGHT_KP | float | PID proportional for right motor. | 1.2 |
| RIGHT_KI | float | PID integral for right motor. | 0.8 |
| RIGHT_KD | float | PID differential for right motor. | 0.01 |

__Drive Trim Parameters__

| parameter | type | description | default |
| -------- | -------- | -------- | -------- |
| GAIN | float | Overall gain factor. | 1.0 |
| TRIM | float | Trim value for motor output. | 0.0 |
| MOTOR_CONSTANT | float | Motor constant for output calculation. | 1.0 |

In the context of motor control, the motor constant is a proportionality factor that relates the input voltage to the output speed or torque. By adjusting the motor constant, we can compensate for differences in motor performance, such as variations in motor efficiency or mechanical load.

The trim parameter, on the other hand, is used to introduce a small offset to the motor's output. This can be helpful in compensating for slight misalignments in the robot's mechanical structure or differences in motor characteristics.

In the given equations:

```c
MotorConstantLeft = (GAIN + TRIM) / MOTOR_CONSTANT;
MotorConstantRight = (GAIN - TRIM) / MOTOR_CONSTANT;
```

The `MotorConstantLeft` and `MotorConstantRight` values are used to multiply the algorithm output (typically from a PID controller) to determine the appropriate PWM values for the left and right motors. These adjusted motor constants account for variations in motor performance and mechanical factors, ensuring precise and coordinated motor control.

- `MotorConstantLeft` and `MotorConstantRight` are the adjusted motor constants for the left and right motors, respectively.
- `GAIN` is a global gain factor that scales the overall motor output.
- `TRIM` is a small value used to adjust the motor output.
- `MOTOR_CONSTANT` is the nominal motor constant.

By adjusting the TRIM parameter, we can effectively fine-tune the motor outputs to ensure accurate and precise robot motion, even in the presence of minor variations in motor performance or mechanical alignment.    

__Understanding the Configuration Flag__

The `CONFIG_FLAGS` parameter in the ROSRider configuration file is a bitmask that controls various hardware settings. By setting specific bits within this flag, you can configure different aspects of the ROSRider's behavior.

Here's a breakdown of the individual bits and their corresponding functionalities:

| bit | function | description |
| ----- | ----- | ----- |
| 0   | LEFT_REVERSE | inverts the direction of the left motor |
| 1   | RIGHT_REVERSE | inverts the direction of the left motor |
| 2   | LEFT_SWAP | swaps the phase order of the left encoder |
| 3   | RIGHT_SWAP | swaps the phase order of the left encoder |
| 4   | LEFT_ENC_AB | selects the AB phase encoding for the left encoder |
| 5   | RIGHT_ENC_AB | selects the AB phase encoding for the right encode |
| 6   | MODE1 | Brake mode |
| 7   | MODE2 | High side decay |

The configuration flags allow you to customize the behavior of the ROSRider to match your specific hardware setup. Here's a breakdown of their functions:

- Reversing Motor Direction: The `LEFT_REVERSE` and `RIGHT_REVERSE` flags allow you to invert the direction of the motors, useful for correcting wiring mistakes or physical misorientations.
- Swapping Encoder Phases: The `LEFT_SWAP` and `RIGHT_SWAP` flags allow you to correct the phase order of the encoders, ensuring accurate position and velocity measurements.
- Selecting Encoder Mode: The `LEFT_ENC_AB` and `RIGHT_ENC_AB` flags determine whether to use AB phase encoding or single-phase encoding for the respective encoders. For AB phase encoding, both A and B phase signals are used, while for single-phase encoding, only the A phase signal is required.

By carefully configuring these flags, you can ensure that the ROSRider can work with a variety of motor and encoder configurations, providing flexibility and adaptability in your robotics projects.

__Understanding Command Timeout__

The ROSRider employs a command timeout mechanism to ensure safe operation and prevent unintended movement. This mechanism monitors the frequency of incoming commands from the host computer. If the system fails to receive a command within a specified time frame, it enters a state that restrains movement.

The `ALLOWED_SKIP` parameter in the ROSRider configuration determines the maximum number of consecutive command cycles that can be skipped before triggering the timeout. This value, when multiplied by the inverse of the `UPDATE_RATE` (measured in milliseconds), sets the overall timeout duration. For instance, if `ALLOWED_SKIP` is set to 3 and the `UPDATE_RATE` is 20Hz, the timeout duration would be 150 milliseconds.

__Understanding PWM Frequency and its Impact on Motor Control__

PWM (Pulse-Width Modulation) is a technique used to control the average power supplied to a device by turning the power on and off rapidly. In the context of motor control, PWM is used to vary the speed and direction of a motor.

***The Role of PWM Frequency:***

The PWM frequency, measured in Hertz (Hz), determines how often the power is switched on and off. A higher frequency results in smoother motor control and reduced audible noise. However, excessively high frequencies can lead to increased power losses and potential interference with other electronic components.

***Selecting the Optimal PWM Frequency:***

The optimal PWM frequency depends on several factors, including:

- Motor Type: Different motor types have different optimal frequency ranges. Brushed DC motors typically require lower frequencies, while brushless DC motors may benefit from higher frequencies.
- Desired Performance: Higher frequencies can lead to smoother and quieter operation, but they may also increase power consumption and complexity of the control system.
- Hardware Limitations: The microcontroller and power electronics used in the system may have limitations on the maximum achievable PWM frequency.

***The Impact of PWM Frequency on ROSRider:***

The `PWM_FRQ` parameter in the ROSRider configuration file plays a crucial role in determining the performance and efficiency of your robot's motors. By carefully selecting this value, you can optimize motor smoothness, responsiveness, power consumption, and electromagnetic interference (EMI).

A higher PWM frequency generally leads to:

- Smoother Motor Operation: More frequent switching reduces motor torque ripple.
- Improved Responsiveness: Faster reaction to control inputs.

However, a higher frequency can also result in:

- Increased Power Dissipation: More switching losses in the motor driver.
- Higher EMI: Increased electromagnetic interference.

***The Role of PWM_DIV:***

To achieve the desired PWM frequency, you'll need to set the `PWM_DIV` parameter appropriately. This parameter divides the system clock to generate the PWM clock. A higher PWM_DIV value results in a lower PWM clock frequency.

- This setting determines the clock frequency for the PWM module.
- It's a hardware-based division of the system clock (80000000).

***Balancing Performance and Efficiency:***

The optimal PWM frequency depends on various factors, including:

- Motor Characteristics: The type and size of the motors.
- Load Conditions: The expected load on the motors.
- Desired Performance: The required level of smoothness, responsiveness, and efficiency.

***Practical Considerations:***

- Start with a Moderate Frequency: Begin with a moderate PWM frequency and gradually increase it if needed.
- Monitor Motor Performance: Observe the motor's behavior and adjust the frequency accordingly.
- Consider Power Dissipation and EMI: If power consumption or EMI becomes a concern, reduce the PWM frequency.
- Experiment and Fine-Tune: The best PWM frequency may vary depending on your specific application.

By carefully considering these factors and adjusting the PWM_DIV and PWM_FRQ parameters, you can optimize your ROSRider's performance and efficiency.

***ROS2RPI_CONFIG Parameter***

When operating the ROSRider card in conjunction with the ROS2RPI card on a Raspberry Pi platform, the driver provides the capability to transmit commands to the ROS2RPI card. 
This functionality is particularly valuable for controlling peripheral devices, such as lidar units, during the driver initialization sequence.  

If the ROSRider card is deployed independently (standalone configuration), set this parameter to 0. For configurations involving the ROS2RPI card, refer to the [ROS2RPI documentation](https://docs.acada.dev/ros2rpi_doc) for appropriate parameter selection.

---

Stay tuned for our next chapter, where we'll guide you through the process of updating your ROSRider's firmware.

__Next Chapter:__ [Updating Firmware](../06_FIRMWARE/README.md)
