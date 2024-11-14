---
layout: default
title_url: /07_HARDWARE/README.html
title: "Hardware Specifications"
description: "ROSRider Control Card Hardware Specifications"
---

__Features__

|---|---|
| MCU | 32-bit ARM Cortex: TM4C123GH6PM |
| Motor Drivers | Two channel up to 2.5 amps, with current measurement |
| Hardware QEI | Can use any encoder gear motor, single or double phase |
| Servo Control | Two channel standard servos |
| Connections | Uses standard JST connectors, and commodity JST cables |
| RTC | On board RTC, wake on alarm possible |
| Firmware Update | Device firmware is updatable by end user over USB |

__Power__

|---|---|
| Power Input | 6V to 15V wide input voltage |
| Software Controllable Switch | MOSFET switch, reverse current protection |
| Hibernation | Auto sleep, Device hibernates if not used certain time |
| Current Protection | PTC Resetable Fuse, and also software current protection |
| Power Input Connector | XT30 or Klemens Type Connector |
| Power Output | Software Controllable Auxillary Power Output 5V, 400mA |
| Monitoring | Measures and reports bus voltage, bus current, independent motor currents |

__Communications__

|---|---|
| I2C | Dual QWIC ports, also available on header. Can be used for cascading with multiple units |
| SPI | Serial peripheral interface connector |
| USB | CDC-USB Serial, 921600 Bauds |
| Serial | Standard Serial Port, available as JST connector or header |

__Software__

|---|---|
| ROS2 Drivers | C++ Drivers for ROS2 Humble and ROS2 Jazzy |
| Parametric | ROSRider is configurable using yaml files |
| Innovation | Autosyncs with host to generate low latency data |

__Next Chapter:__ [Simulations](../09_SIMULATIONS/README.md)
