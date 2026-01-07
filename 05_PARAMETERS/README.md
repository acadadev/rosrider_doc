---
layout: default
title_url: /01_SPECS/README.html
title: "Parameters"
description: "Parametric Configuration of the ROSRider control card"
---

A YAML file is a human-readable text format used to store configuration parameters in ROS. The provided YAML file defines parameters that influence the behavior of the ROSRider node, such as motor control, sensor settings, and communication protocols.

The ROS Parameter Server is a mechanism that allows you to dynamically configure parameters in your ROS nodes. When you launch a ROS node with a YAML file, the parameters defined in the file are loaded into the ROS Parameter Server. This makes them accessible to other nodes and allows you to modify them at runtime using tools like rosparam or rqt_reconfigure.

The ROSRider node reads the configuration parameters from the YAML file and stores them in its non-volatile memory (EEPROM). This allows the ROSRider to retain its configuration settings even after a power cycle. Dynamic parameters, such as PID control gains, can be adjusted on-the-fly using tools like `rosparam` or `rqt_reconfigure`, and the changes are immediately applied to the ROSRider.

The following YAML file defines the configuration parameters for the ROSRider node. This file is crucial for customizing the behavior and performance of the ROSRider, tailoring it to specific robotic applications.


{% capture tab1 %}

__Driver Configuration__

```yaml
DRIVE_MODE: 3          # MODE_PID 3 for ROS
CONFIG_FLAGS: 112      #
UPDATE_RATE: 20        # 10, 16, 20, 32, 50, 64
```

| Parameter    | Type  | Description                         | Default |
|--------------|-------|-------------------------------------|---------|
| DRIVE_MODE   | uint8 | Drive Mode Configuration, 3 for ROS | 3       |
| CONFIG_FLAGS | uint8 | Hardware Config Bitmask             | 48      |
| UPDATE_RATE  | uint8 | Outer PID Loop Update Rate          | 20      |

__Understanding the Configuration Flag__

The `CONFIG_FLAGS` parameter in the ROSRider configuration file is a bitmask that controls various hardware settings. By setting specific bits within this flag, you can configure different aspects of the ROSRider's behavior.

Here's a breakdown of the individual bits and their corresponding functionalities:

| Bit | Function      | Description                                        |
|-----|---------------|----------------------------------------------------|
| 0   | LEFT_REVERSE  | inverts the direction of the left motor            |
| 1   | RIGHT_REVERSE | inverts the direction of the left motor            |
| 2   | LEFT_SWAP     | swaps the phase order of the left encoder          |
| 3   | RIGHT_SWAP    | swaps the phase order of the left encoder          |
| 4   | LEFT_ENC_AB   | selects the AB phase encoding for the left encoder |
| 5   | RIGHT_ENC_AB  | selects the AB phase encoding for the right encode |
| 6   | MODE1         | Brake mode                                         |
| 7   | MODE2         | High side decay                                    |

The configuration flags allow you to customize the behavior of the ROSRider to match your specific hardware setup. Here's a breakdown of their functions:

- Reversing Motor Direction: The `LEFT_REVERSE` and `RIGHT_REVERSE` flags allow you to invert the direction of the motors, useful for correcting wiring mistakes or physical misorientations.
- Swapping Encoder Phases: The `LEFT_SWAP` and `RIGHT_SWAP` flags allow you to correct the phase order of the encoders, ensuring accurate position and velocity measurements.
- Selecting Encoder Mode: The `LEFT_ENC_AB` and `RIGHT_ENC_AB` flags determine whether to use AB phase encoding or single-phase encoding for the respective encoders. For AB phase encoding, both A and B phase signals are used, while for single-phase encoding, only the A phase signal is required.

By carefully configuring these flags, you can ensure that the ROSRider can work with a variety of motor and encoder configurations, providing flexibility and adaptability in your robotics projects.

{% endcapture %}

{% capture tab2 %}

__PWM Configuration__

```yaml
PWM_DIV: 16  
PWM_SCALE: 256  
PWM_FRQ: 1000  
```

| Parameter | Type   | Description                         | Default |
|-----------|--------|-------------------------------------|---------|
| PWM_DIV   | uint8  | Drive Mode Configuration, 3 for ROS | 64      |
| PWM_SCALE | uint16 | Hardware Config Bitmask             | 256     |
| PWM_FRQ   | uint16 | Outer PID Loop Update Rate          | 50      |

__Understanding PWM Frequency and its Impact on Motor Control__

PWM (Pulse-Width Modulation) is a technique used to control the average power supplied to a device by turning the power on and off rapidly.
In the context of motor control, PWM is used to vary the speed and direction of a motor.

__The Role of PWM Frequency:__

The PWM frequency, measured in Hertz (Hz), determines how often the power is switched on and off. 
A higher frequency results in smoother motor control and reduced audible noise. 
However, excessively high frequencies can lead to increased power losses and potential interference with other electronic components.

__Selecting the Optimal PWM Frequency:__

The optimal PWM frequency depends on several factors, including:

- Motor Type: Different motor types have different optimal frequency ranges. Brushed DC motors typically require lower frequencies, while brushless DC motors may benefit from higher frequencies.
- Desired Performance: Higher frequencies can lead to smoother and quieter operation, but they may also increase power consumption and complexity of the control system.
- Hardware Limitations: The microcontroller and power electronics used in the system may have limitations on the maximum achievable PWM frequency.

__The Impact of PWM Frequency on ROSRider:__

The `PWM_FRQ` parameter in the ROSRider configuration file plays a crucial role in determining the performance and efficiency of your robot's motors.
By carefully selecting this value, you can optimize motor smoothness, responsiveness, power consumption, and electromagnetic interference (EMI).

A higher PWM frequency generally leads to:

- Smoother Motor Operation: More frequent switching reduces motor torque ripple.
- Improved Responsiveness: Faster reaction to control inputs.

However, a higher frequency can also result in:

- Increased Power Dissipation: More switching losses in the motor driver.
- Higher EMI: Increased electromagnetic interference.

__The Role of PWM_DIV:__

To achieve the desired PWM frequency, you'll need to set the `PWM_DIV` parameter appropriately.
This parameter divides the system clock to generate the PWM clock. A higher PWM_DIV value results in a lower PWM clock frequency.

- This setting determines the clock frequency for the PWM module.
- It's a hardware-based division of the system clock (80000000).

__Balancing Performance and Efficiency:__

The optimal PWM frequency depends on various factors, including:

- Motor Characteristics: The type and size of the motors.
- Load Conditions: The expected load on the motors.
- Desired Performance: The required level of smoothness, responsiveness, and efficiency.

__Practical Considerations:__

- Start with a Moderate Frequency: Begin with a moderate PWM frequency and gradually increase it if needed
- Monitor Motor Performance: Observe the motor's behavior and adjust the frequency accordingly
- Consider Power Dissipation and EMI: If power consumption or EMI becomes a concern, reduce the PWM frequency
- Experiment and Fine-Tune: The best PWM frequency may vary depending on your specific application
- Synchronized ADC: Limit PWM Frequency 1-2kHZ

By carefully considering these factors and adjusting the PWM_DIV and PWM_FRQ parameters, you can optimize your ROSRider's performance and efficiency.

{% endcapture %}

{% capture tab3 %}

__Motor Configuration__

```yaml
GEAR_RATIO: 65.0  
ENCODER_PPR: 48  
WHEEL_DIA: 0.0685  
BASE_WIDTH: 0.174  
MAX_RPM: 160.0  
```

| Parameter   | Type   | Description                    | Default |
|-------------|--------|--------------------------------|---------|
| GEAR_RATIO  | float  | Gear ratio of the motors.      | 65.0    |
| ENCODER_PPR | uint16 | Encoder pulses per revolution. | 48      |
| WHEEL_DIA   | float  | Diameter of the wheels.        | 0.0685  |
| BASE_WIDTH  | float  | Distance between the wheels.   | 0.168   |
| MAX_RPM     | float  | Distance between the wheels.   | 90.0    |

{% endcapture %}

{% capture tab4 %}

__PID Configuration__

```yaml
UPPER_LIMIT: 240  
LEFT_KP: 8.0  
LEFT_KI: 6.0  
LEFT_KD: 0.0  
RIGHT_KP: 8.0  
RIGHT_KI: 6.0  
RIGHT_KD: 0.0  
K_FB_WINDUP: 0.5  
```

| Parameter   | Type   | Description                      | Default |
|-------------|--------|----------------------------------|---------|
| UPPER_LIMIT | uint16 | Maximum Controller PWM output    | 192     |
| LEFT_KP     | float  | PID proportional for left motor  | 2.4     |
| LEFT_KI     | float  | PID integral for left motor      | 1.2     |
| LEFT_KD     | float  | PID differential for left motor  | 0.0     |
| RIGHT_KP    | float  | PID proportional for right motor | 2.4     |
| RIGHT_KI    | float  | PID integral for right motor     | 1.2     |
| RIGHT_KD    | float  | PID differential for right motor | 0.0     |
| K_FB_WINDUP | float  | Anti windup coefficient          | 0.5     |

{% endcapture %}

{% capture tab5 %}

__Cascaded Loop__

```yaml
INNER_LIMIT: 240  
CURRENT_KP: 8.0  
CURRENT_KI: 6.0  
CURRENT_MULTIPLIER_LEFT: 4.8  
CURRENT_MULTIPLIER_RIGHT: 4.8  
CURRENT_OMEGA_K_LEFT: -2.4  
CURRENT_OMEGA_K_RIGHT: -2.4  
R_ARM: 2.0
```

| Parameter               | Type   | Description                           | Default |
|-------------------------|--------|---------------------------------------|---------|
| INNER_LIMIT             | uint16 | Maximum Controller PWM output         | 192     |
| CURRENT_KP              | float  | Inner Loop Current Error Proportional | 2.4     |
| CURRENT_KI              | float  | Inner Loop Current Error Integral     | 1.2     |
| CURRENT_MULTIPLIER_LEFT | float  | Current Multiplier Left               | 4.8     |
| CURRENT_MULTIPLIER_LEFT | float  | Current Multiplier Right              | 4.8     |
| CURRENT_OMEGA_K_LEFT    | float  | Current Omega Compensation Left       | 0.0     |
| CURRENT_OMEGA_K_RIGHT   | float  | Current Omega Compensation Right      | 0.0     |
| R_ARM                   | float  | Motor Armature Resistance             | 2.0     |

__Torque Constant__

```yaml
LEFT_KT: 0.016
LEFT_KT_W: -0.008
RIGHT_KT: 0.016
RIGHT_KT_W: -0.008
```

| Parameter  | Type  | Description                              | Default |
|------------|-------|------------------------------------------|---------|
| LEFT_KT    | float | Torque Constant for Left Motor           | 0.016   |
| LEFT_KT_W  | float | Torque Constant Omega Compensation Left  | -0.008  |
| RIGHT_KT   | float | Torque Constant for Right Motor          | 0.016   |
| RIGHT_KT_W | float | Torque Constant Omega Compensation Right | -0.008  |

{% endcapture %}

{% capture tab6 %}

__Physics Feed Forwards__

```yaml
OUTER_FEEDFORWARD: True
K_FF_VEL: 0.12  
K_FF_ACCEL: 0.08  
```

| Parameter         | Type    | Description                    | Default |
|-------------------|---------|--------------------------------|---------|
| OUTER_FEEDFORWARD | boolean | Enable Outer Loop Feedforwards | False   |
| K_FF_VEL          | float   | Velocity Feedforward / s       | 0.16    |
| K_FF_ACCEL        | float   | Acceleration Feedforward / s²  | 0.12    | 
{% endcapture %}

{% capture tab7 %}

__Stribeck Friction Model__

```yaml
OUTER_SCV: True
STATIC_KICK: 0.8  
COULOMB_RUN: 0.2  
STRIBECK_WIDTH: 2.1  
VISCOUS_FRICTION: 0.064  
VISCOUS_FRICTION_LIMIT: 1.2  
EB_FF_LIMIT: 12.0  
SCV_OMEGA_THRESHOLD: 0.05  
SCV_LATCH_THRESHOLD: 1.0  
```

| Parameter              | Type    | Description                                            | Default |
|------------------------|---------|--------------------------------------------------------|---------|
| OUTER_SCV              | boolean | Enable SCV if not deadzone will be used                | False   |
| STATIC_KICK            | float   | Initial Kick in Volts                                  | 6.0     |
| COULOMB_RUN            | float   | Minimum energy for motor to turn in Volts              | 3.0     |
| STRIBECK_WIDTH         | float   | Statick Kick Decay rate                                | 64      | 
| VISCOUS_FRICTION       | float   | Viscous Friction Coefficient                           | 0.001   | 
| VISCOUS_FRICTION_LIMIT | float   | Viscous Friction Limit                                 | 1.2     | 
| EB_FF_LIMIT            | float   | Calculated BEMF Limit in Volts                         | 12.0    | 
| SCV_OMEGA_THRESHOLD    | float   | Below this threshold SCV will not be triggered         | 0.05    | 
| SCV_LATCH_THRESHOLD    | float   | Below this threshold Static kick will not be triggered | 1.0     | 

__Dead Zones__

```yaml
LEFT_FORWARD_DEADZONE: 12  
LEFT_REVERSE_DEADZONE: 12  
RIGHT_FORWARD_DEADZONE: 12  
RIGHT_REVERSE_DEADZONE: 12  
```

| Parameter              | Type  | Description                  | Default |
|------------------------|-------|------------------------------|---------|
| LEFT_FORWARD_DEADZONE  | int16 | Left Motor Forward Deadzone  | 0       |
| LEFT_REVERSE_DEADZONE  | int16 | Left Motor Reverse Deadzone  | 0       |
| RIGHT_FORWARD_DEADZONE | int16 | Right Motor Forward Deadzone | 0       |
| RIGHT_REVERSE_DEADZONE | int16 | Right Motor Reverse Deadzone | 0       |
{% endcapture %}

{% capture tab8 %}

__Trim Model__

```yaml
TRIM_GAIN: 1.0  
TRIM_MOTOR_K: 1.0  
TRIM_CONSTANT: 0.0  
```

| Parameter     | Type  | Description                           | Default |
|---------------|-------|---------------------------------------|---------|
| TRIM_GAIN     | float | Overall Gain Factor                   | 1.0     |
| TRIM_MOTOR_K  | float | Motor constant for output calculation | 1.0     |
| TRIM_CONSTANT | float | Trim value for motor output           | 1.0     | 

In the context of motor control, the motor constant is a proportionality factor that relates the input voltage to the output speed or torque. By adjusting the motor constant, we can compensate for differences in motor performance, such as variations in motor efficiency or mechanical load.

The trim parameter, on the other hand, is used to introduce a small offset to the motor's output. This can be helpful in compensating for slight misalignments in the robot's mechanical structure or differences in motor characteristics.

In the given equations:

```c
MotorConstantLeft = (GAIN + TRIM) / MOTOR_CONSTANT;
MotorConstantRight = (GAIN - TRIM) / MOTOR_CONSTANT;
```

The `MotorConstantLeft` and `MotorConstantRight` values are used to multiply the algorithm output (typically from a PID controller) to determine the appropriate PWM values for the left and right motors. These adjusted motor constants account for variations in motor performance and mechanical factors, ensuring precise and coordinated motor control.

- `MotorConstantLeft` and `MotorConstantRight` are the adjusted motor constants for the left and right motors, respectively
- `TRIM_GAIN` is a global gain factor that scales the overall motor output
- `TRIM_MOTOR_K` is the nominal motor constant
- `TRIM_CONSTANT` is a small value used to adjust the motor output


By adjusting the TRIM parameter, we can effectively fine-tune the motor outputs to ensure accurate and precise robot motion, even in the presence of minor variations in motor performance or mechanical alignment.    

{% endcapture %}

{% capture tab9 %}

__Filter Configuration__

```yaml
OMEGA_FILTER_TYPE: 1  
CURRENT_FILTER_TYPE: 3  
OUTPUT_FILTER_TYPE: 0  
```

| Parameter           | Type  | Description          | Default |
|---------------------|-------|----------------------|---------|
| OMEGA_FILTER_TYPE   | uint8 | Velocity Filter Type | 1       |
| CURRENT_FILTER_TYPE | uint8 | Current Filter Type  | 3       |
| OUTPUT_FILTER_TYPE  | uint8 | Output Filter Type   | 0       | 

__Omega Filter__

| Name            | ID | Filter Type                           | Details                      |
|-----------------|----|---------------------------------------|------------------------------|
| EWMA4           | 0  | Exponentially Weighted Moving Average | Last 4 Values                | 
| EWMA8           | 1  | Exponentially Weighted Moving Average | Last 8 Values                |
| EWMA16          | 2  | Exponentially Weighted Moving Average | Last 16 Values               |
| BIQUAD_20HZ_2HZ | 3  | Biquad Filter                         | 20HZ update rate, 2HZ cutoff |
| BIQUAD_20HZ_4HZ | 4  | Biquad Filter                         | 20HZ update rate, 4HZ cutoff |

__Current Filter__

| Name            | ID | Filter Type                           | Details                      |
|-----------------|----|---------------------------------------|------------------------------|
| EWMA4           | 0  | Exponentially Weighted Moving Average | Last 4 Values                | 
| EWMA8           | 1  | Exponentially Weighted Moving Average | Last 8 Values                |
| EWMA16          | 2  | Exponentially Weighted Moving Average | Last 16 Values               |

__Output Filter__

| Name | ID | Filter Type |
|------|----|-------------|
| None | 0  | None        |
| TANH | 1  | Tanh        |
| SIGM | 2  | Sigmoid     |

```yaml
TANH_DIV: 2.0  
SIGM_DIV: 10.0  
```

| Parameter | Type  | Description                     | Default |
|-----------|-------|---------------------------------|---------|
| TANH_DIV  | float | Tanh filter prescaler           | 2.0     |
| SIGM_DIV  | float | Sigmoid output filter prescaler | 10.0    |

__Filter Use__

```yaml
VOLTAGE_FILTER: True  
BEMF_FILTERED_OMEGA: True  
PID_FILTERED_OMEGA: True  
SCV_FILTERED_OMEGA: True  
CURRENT_OMEGA_FILTER: True  
```

| Parameter            | Type    | Description                                           | Default |
|----------------------|---------|-------------------------------------------------------|---------|
| VOLTAGE_FILTER       | boolean | Use Filtered voltage for Cascaded Loop Calculations   | False   |
| BEMF_FILTERED_OMEGA  | boolean | Use Filtered Omega for Cascaded Loop Calculations     | False   |
| PID_FILTERED_OMEGA   | boolean | Use Filtered Omega for PID Error Calculations         | False   |
| SCV_FILTERED_OMEGA   | boolean | Use Filtered Omega for SCV Model                      | False   |
| CURRENT_OMEGA_FILTER | boolean | Use Filterd Omega for Current Multiplier Compensation | False   |

{% endcapture %}

{% capture tab10 %}

__ADC Config__

```yaml
ADC_SYNC: True  
ADC_MULTIPHASE: True  
ADC_BIPHASE: False  
```

| Parameter      | Type    | Description                                          | Default |
|----------------|---------|------------------------------------------------------|---------|
| ADC_SYNC       | boolean | ADC Syncronized with PWM. Required for Cascaded Mode | False   |
| ADC_MULTIPHASE | boolean | Multi-Phase ADC Measurement. BEMF compensated Mode   | False   |
| ADC_BIPHASE    | boolean | Bi-Phase ADC Measurement                             | False   |

__ADC Bias Calibration__

```yaml
CS_LEFT_OFFSET: 0  
CS_RIGHT_OFFSET: 0  
```

| Parameter       | Type  | Description                           | Default |
|-----------------|-------|---------------------------------------|---------|
| CS_LEFT_OFFSET  | int16 | Current Sense Calibration Value Left  | 0       |
| CS_RIGHT_OFFSET | int16 | Current Sense Calibration Value Right | 0       |

{% endcapture %}

{% capture tab11 %}

__Enable Features__

```yaml
AUTO_SYNC: True  
AUTO_BIAS: True  
AUTO_BRAKE: False  
CASCADED: True  
```

| Parameter  | Type    | Description                      | Default |
|------------|---------|----------------------------------|---------|
| AUTO_SYNC  | boolean | Automatic Syncronization Enabled | True    |
| AUTO_BIAS  | boolean | Auto Bias Enabled                | True    |
| AUTO_BRAKE | boolean | Auto Brake Enabled               | False   |
| CASCADED   | boolean | Cascaded Inner Loop Enabled      | False   |

{% endcapture %}

{% capture tab12 %}

__Synchronization__

```yaml
SYNC_KP: 256  
SYNC_KI: 4  
SYNC_LIMIT: 4096  
SYNC_INTERVAL: 8  
DT_I2C: 32  
DT_THRESHOLD: 2  
```

| Parameter     | Type   | Description                                        | Default |
|---------------|--------|----------------------------------------------------|---------|
| SYNC_KP       | uint16 | Phase Error Proportional Coefficient               | 256     |
| SYNC_KI       | uint16 | Phase Error Integral Coefficient                   | 4       |
| SYNC_LIMIT    | uint16 | Timer Adjustment Limit                             | 4096    |
| SYNC_INTERVAL | uint8  | Timer Adjustment Interval                          | 8       |
| DT_I2C        | uint16 | Desired Phase Error                                | 32      |
| DT_THRESHOLD  | uint16 | If below this threshold Timer will not be adjusted | 2       |

{% endcapture %}

{% capture tab13 %}

__ROS Parameters__

```yaml
ODOM_FRAME_ID: 'odom'  
BASE_FRAME_ID: 'base_footprint'  
CMD_VEL_TOPIC: 'cmd_vel_nav'  
BROADCAST_TF2: True  
PUB_ODOMETRY: True  
PUB_JOINTS: True  
PUB_DIAGNOSTICS: True  
```

| Parameter       | Type   | Description                                     | Default          |
|-----------------|--------|-------------------------------------------------|------------------|
| ODOM_FRAME_ID   | string | Sets the frame ID for the odometry data         | `odom`           |
| BASE_FRAME_ID   | string | Sets the frame ID for the robot's base frame    | `base_footprint` |
| CMD_VEL_TOPIC   | string | Command Topic Name                              | `cmd_vel`        |
| BROADCAST_TF2   | bool   | Enables or disables TF2 broadcast               | True             |
| PUB_ODOMETRY    | bool   | Enables or disables odometry data publication   | True             |
| PUB_JOINTS      | bool   | Enables or disables joint state publication     | True             |
| PUB_DIAGNOSTICS | bool   | Enables or disables diagnostic data publication | True             |

{% endcapture %}

{% capture tab14 %}

__Electrical Limits__

```yaml
MAIN_AMP_LIMIT: 3.6  
BAT_VOLTS_HIGH: 15.0  
BAT_VOLTS_LOW: 6.0  
LEFT_AMP_LIMIT: 2.4  
RIGHT_AMP_LIMIT: 2.4  
INA219_CAL: 8192  
```

| Parameter       | Type   | Description                                    | Default |
|-----------------|--------|------------------------------------------------|---------|
| MAIN_AMP_LIMIT  | float  | Maximum current draw for the main power supply | 3.6     |
| BAT_VOLTS_HIGH  | float  | Maximum battery voltage                        | 15.0    |
| BAT_VOLTS_LOW   | float  | Minimum battery voltage                        | 6.0     |
| LEFT_AMP_LIMIT  | float  | Maximum current limit for the left motor       | 1.6     |
| RIGHT_AMP_LIMIT | float  | Maximum current limit for the right motor      | 1.6     |
| INA219_CAL      | uint16 | INA219 Calibration Value                       | 8192    |

{% endcapture %}

{% capture tab15 %}

__Experimental Features__

```yaml
CROSS_COUPLED_CONTROL: True  
CROSS_KP: 4.0  
CROSS_K_LEFT: 1.0  
CROSS_K_RIGHT: 1.0  
```

| Parameter             | Type  | Description                      | Default |
|-----------------------|-------|----------------------------------|---------|
| CROSS_COUPLED_CONTROL | bool  | Enable Cross Coupled Control     | False   |
| CROSS_KP              | float | Cross Proportional Coefficient   | 4.0     |
| CROSS_K_LEFT          | float | Cross Feedback Left Coefficient  | 1.0     |
| CROSS_K_RIGHT         | float | Cross Feedback Right Coefficient | 1.0     |


{% endcapture %}

{% capture tab16 %}

__System Configuration__

```yaml
ROS2RPI_CONFIG: 0x33 # 0x00 # 0x0F # 0x33  
I2C_ADDRESS: 0x3c  
I2C_ENABLED: True  
DEBUG: False  
RTC_TRIM: 0x7FFF  
ALLOWED_SKIP: 3  
MONITOR_RATE: 100  
MAX_IDLE_SECONDS: 1800  
```

| Parameter        | Type   | Description                                                  | Default |
|------------------|--------|--------------------------------------------------------------|---------|
| ROS2RPI_CONFIG   | uint8  | Configuration for the ROS2RPI board (if used)                | 0x00    |
| I2C_ADDRESS      | uint8  | I2C address of the card                                      | 0x3C    |
| I2C_ENABLED      | bool   | Enables or disables I2C communication. For development only. | True    |
| DEBUG            | bool   | Enables or disables debug mode                               | False   |
| RTC_TRIM         | uint32 | Real-Time Clock trim value                                   | 0x7FFF  |
| ALLOWED_SKIP     | uint8  | Command timeout in units of 1 / UPDATE_RATE.                 | 3       |
| MONITOR_RATE     | uint8  | Rate at which current sensor data is monitored               | 100     |
| MAX_IDLE_SECONDS | uint16 | Maximum idle seconds before entering hibernate mode          | 3600    |

__ALLOWED_SKIP__

The ROSRider employs a command timeout mechanism to ensure safe operation and prevent unintended movement. This mechanism monitors the frequency of incoming commands from the host computer. If the system fails to receive a command within a specified time frame, it enters a state that restrains movement.
The `ALLOWED_SKIP` parameter in the ROSRider configuration determines the maximum number of consecutive command cycles that can be skipped before triggering the timeout. This value, when multiplied by the inverse of the `UPDATE_RATE` (measured in milliseconds), sets the overall timeout duration. For instance, if `ALLOWED_SKIP` is set to 3 and the `UPDATE_RATE` is 20Hz, the timeout duration would be 150 milliseconds.

__ROS2RPI_CONFIG__

| Hat Command | Description                                   |
|-------------|-----------------------------------------------|
| 0x00        | No Command                                    |
| 0x0F        | ROSRider ON, Serial Routed to DEBUG           |
| 0x33        | ROSRider ON, LIDAR ON, Serial Routed to LIDAR |


{% endcapture %}

{% include tabs.html 
   tab1_title="Driver" 
   tab1_content=tab1
   tab2_title="PWM" 
   tab2_content=tab2
   tab3_title="Motor" 
   tab3_content=tab3
   tab4_title="PID" 
   tab4_content=tab4
   tab5_title="Cascade" 
   tab5_content=tab5
   tab6_title="FF" 
   tab6_content=tab6
   tab7_title="Friction" 
   tab7_content=tab7
   tab8_title="Trim" 
   tab8_content=tab8
   tab9_title="Filters" 
   tab9_content=tab9
   tab10_title="ADC" 
   tab10_content=tab10
   tab11_title="Features" 
   tab11_content=tab11
   tab12_title="Sync" 
   tab12_content=tab12
   tab13_title="ROS" 
   tab13_content=tab13
   tab14_title="Safety" 
   tab14_content=tab14
   tab15_title="Experimental" 
   tab15_content=tab15
   tab16_title="System" 
   tab16_content=tab16
%}

__ROS2RPI_CONFIG Parameter__

When operating the ROSRider card in conjunction with the ROS2RPI card on a Raspberry Pi platform, the driver provides the capability to transmit commands to the ROS2RPI card. 
This functionality is particularly valuable for controlling peripheral devices, such as lidar units, during the driver initialization sequence.  

If the ROSRider card is deployed independently (standalone configuration), set this parameter to 0. For configurations involving the ROS2RPI card, refer to the [ROS2RPI documentation](https://docs.acada.dev/ros2rpi_doc) for appropriate parameter selection.


Stay tuned for our next chapter, where we'll guide you through the process of updating your ROSRider's firmware.

__Next Chapter:__ [Updating Firmware](../06_FIRMWARE/README.md)
