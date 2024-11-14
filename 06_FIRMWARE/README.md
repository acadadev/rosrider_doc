---
layout: default
title_url: /06_FIRMWARE/README.html
title: "Updating Firmware"
description: "Updating Firmware of ROSRider control card"
---

__Updating Your ROSRider's Firmware__

***Important Note:*** Before proceeding with the firmware update, ensure you have the correct firmware file for your specific ROSRider model. Incorrect firmware can lead to device malfunction.

To initiate the firmware update process, follow these steps:

Enter Bootloader Mode:
- Press and Hold the Boot Button: Keep the boot button pressed.
- Press and Release the Reset Button: While still holding the boot button, press and quickly release the reset button.
- Release the Boot Button: Once the reset button is released, you can also release the boot button.

The ROSRider should now enter bootloader mode. This is indicated by the green LED lighting up constantly.

Run this command to view system logs in real-time. 

```console
sudo tail -f /var/log/syslog
```

You should see a message indicating successful entry into bootloader mode, which typically reads:

<div style="display: flex; justify-content: space-around; margin: 25px 0;">
   <img src="../images/bootloader_mode.png" alt="Bootloader mode output" style="box-shadow: 0px 4px 8px rgba(0, 0, 0, 0.2);">
</div>

***Before Starting Update: Important Considerations***

- Power Supply: Ensure a stable power supply during the update process.
- USB Connection: A reliable USB connection is crucial.
- Firmware Compatibility: Always use the firmware provided by ACADA Robotics for your specific model.
- Patience: The firmware update process may take some time. Avoid interrupting the process.

By following these steps and adhering to the specific instructions provided by ACADA Robotics, you can successfully update your ROSRider's firmware and enhance its capabilities.

__Updating ROSRider Firmware with I2CRider.dfu__

__Prerequisites:__

- Linux Computer: You'll need a Linux-based computer (e.g., Ubuntu, Debian) to perform the firmware update.
- `dfu-util`: Ensure you have `dfu-util` installed on your system. It's a common tool for device firmware updates.
- `I2CRider.dfu`: Obtain this file from Acada Robotics. It contains the firmware update for your ROSRider.

__Steps:__

1. Set Device Suffix and Prefix:

	- Open a terminal on your Linux computer.
	- Execute the following commands to set the suffix and prefix for the DFU device:

	```console
	dfu-suffix -a I2CRider.dfu -v 0x1cbe -p 0x00ff
	dfu-prefix -s 0x2800 -a I2CRider.dfu
	```

2. Start the Firmware Update:

	- Run the following command to initiate the firmware update process:

	```console
	dfu-util -D I2CRider.dfu
	```

	```console
	$ dfu-util -D I2CRider.dfu
	dfu-util 0.9

	Copyright 2005-2009 Weston Schmidt, Harald Welte and OpenMoko Inc.
	Copyright 2010-2016 Tormod Volden and Stefan Schmidt
	This program is Free Software and has ABSOLUTELY NO WARRANTY
	Please report bugs to http://sourceforge.net/p/dfu-util/tickets/

	Match vendor ID from file: 1cbe
	Match product ID from file: 00ff
	Opening DFU capable USB device...
	ID 1cbe:00ff
	Run-time device DFU version 0110
	Claiming USB DFU Interface...
	Setting Alternate Setting #0 ...
	Determining device status: state = dfuIDLE, status = 0
	dfuIDLE, continuing
	DFU mode device DFU version 0110
	Device returned transfer size 1024
	Copying data from PC to DFU device
	Download	[=========================] 100%        26272 bytes
	Download done.
	state(2) = dfuIDLE, status(0) = No error condition is present
	Done!
	$ 
	```
[TODO: output]

__Next Chapter:__ [Troubleshooting](../10_DEBUG/README.md)