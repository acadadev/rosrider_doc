---
layout: default
title_url: /09_STATUS/README.html
title: "Status Registers"
description: "Explanation of Status Registers
---

**Status Registers**

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

__Next Chapter:__ [Troubleshooting](../10_DEBUG/README.md)