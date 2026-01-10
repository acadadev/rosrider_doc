---
layout: default
title_url: /06_FIRMWARE/README.html
title: "Firmware Update"
description: "Updating Firmware of ROSRider control card"
---

<div class="sl">
    <div class="sl1">
        > Updating Your ROSRider's Firmware
    </div>
</div>

To ensure long-term usability and adaptability, the ROSRider card supports firmware updates via USB.
This allows for continuous improvement and the addition of new features, without requiring specialized programming tools or hardware modifications.  

__Enter Bootloader Mode:__
- Press and Hold the Reset Button: Keep the reset button pressed.
- Press and Hold the Boot Button: While still holding the reset button, press the boot button.
- Release the Reset Button: Once the reset button is released, you can also release the boot button.

The ROSRider should now enter bootloader mode. This is indicated by the green LED lighting up constantly.

Run this command to view system logs in real-time. 

<div class="highlight notranslate position-relative">
  <div class="highlight">
    <pre id="command-view-syslog"><span></span>sudo tail -f /var/log/syslog</pre>
  </div>
  <clipboard-copy style="position:absolute; right:8px; top:8px;" for="command-view-syslog">
    <svg aria-hidden="true" height="16" viewBox="0 0 16 16" version="1.1" width="16" data-view-component="true" class="octicon octicon-copy js-clipboard-copy-icon">
    <path d="M0 6.75C0 5.784.784 5 1.75 5h1.5a.75.75 0 0 1 0 1.5h-1.5a.25.25 0 0 0-.25.25v7.5c0 .138.112.25.25.25h7.5a.25.25 0 0 0 .25-.25v-1.5a.75.75 0 0 1 1.5 0v1.5A1.75 1.75 0 0 1 9.25 16h-7.5A1.75 1.75 0 0 1 0 14.25Z"></path><path d="M5 1.75C5 .784 5.784 0 6.75 0h7.5C15.216 0 16 .784 16 1.75v7.5A1.75 1.75 0 0 1 14.25 11h-7.5A1.75 1.75 0 0 1 5 9.25Zm1.75-.25a.25.25 0 0 0-.25.25v7.5c0 .138.112.25.25.25h7.5a.25.25 0 0 0 .25-.25v-7.5a.25.25 0 0 0-.25-.25Z"></path>
    </svg>
    <svg aria-hidden="true" height="16" viewBox="0 0 16 16" version="1.1" width="16" data-view-component="true" class="octicon octicon-check js-clipboard-check-icon color-fg-success d-none">
    <path d="M13.78 4.22a.75.75 0 0 1 0 1.06l-7.25 7.25a.75.75 0 0 1-1.06 0L2.22 9.28a.751.751 0 0 1 .018-1.042.751.751 0 0 1 1.042-.018L6 10.94l6.72-6.72a.75.75 0 0 1 1.06 0Z"></path>
    </svg>
  </clipboard-copy>
</div>

You should see a message indicating successful entry into bootloader mode, which typically reads:

<div class="img_dv">
   <img class="replay" src="../images/rosrider/tailing_syslog.gif" alt="Bootloader Mode Output">
</div>

__Before Starting Update: Important Considerations__

 - ***Power Supply:*** Ensure a stable power supply during the update process.
 - ***USB Connection:*** A reliable USB connection is crucial.
 - ***Firmware Compatibility*** Always use the firmware provided by ACADA Robotics for your specific model.

By following these steps and adhering to the specific instructions provided by ACADA Robotics, you can successfully update your ROSRider's firmware and enhance its capabilities.

<div class="ck">
    <div class="ck1">
        ⚠️&nbsp;Checkpoint
    </div>
    <div class="ck2">
        Before proceeding with the firmware update, ensure you have the correct firmware file for your specific ROSRider model. Incorrect firmware can lead to device malfunction.
    </div>
</div>

<div class="sl">
    <div class="sl1">
        > Updating ROSRider Firmware with I2CRider.dfu
    </div>
</div>

__Prerequisites:__

- Linux Computer: You'll need a Linux-based computer (e.g., Ubuntu, Debian) to perform the firmware update.
- `dfu-util`: Ensure you have `dfu-util` installed on your system. It's a common tool for device firmware updates.
- `I2CRider.dfu`: Obtain this file from Acada Robotics. It contains the firmware update for your ROSRider.

__Steps:__

1. Execute the following commands to set the suffix and prefix for the DFU device:

<div class="highlight notranslate position-relative">
  <div class="highlight">
    <pre id="command-dfu-suffix"><span></span>dfu-suffix -a I2CRider.dfu -v 0x1cbe -p 0x00ff  
dfu-prefix -s 0x2800 -a I2CRider.dfu</pre>
  </div>
  <clipboard-copy style="position:absolute; right:8px; top:8px;" for="command-dfu-suffix">
    <svg aria-hidden="true" height="16" viewBox="0 0 16 16" version="1.1" width="16" data-view-component="true" class="octicon octicon-copy js-clipboard-copy-icon">
    <path d="M0 6.75C0 5.784.784 5 1.75 5h1.5a.75.75 0 0 1 0 1.5h-1.5a.25.25 0 0 0-.25.25v7.5c0 .138.112.25.25.25h7.5a.25.25 0 0 0 .25-.25v-1.5a.75.75 0 0 1 1.5 0v1.5A1.75 1.75 0 0 1 9.25 16h-7.5A1.75 1.75 0 0 1 0 14.25Z"></path><path d="M5 1.75C5 .784 5.784 0 6.75 0h7.5C15.216 0 16 .784 16 1.75v7.5A1.75 1.75 0 0 1 14.25 11h-7.5A1.75 1.75 0 0 1 5 9.25Zm1.75-.25a.25.25 0 0 0-.25.25v7.5c0 .138.112.25.25.25h7.5a.25.25 0 0 0 .25-.25v-7.5a.25.25 0 0 0-.25-.25Z"></path>
    </svg>
    <svg aria-hidden="true" height="16" viewBox="0 0 16 16" version="1.1" width="16" data-view-component="true" class="octicon octicon-check js-clipboard-check-icon color-fg-success d-none">
    <path d="M13.78 4.22a.75.75 0 0 1 0 1.06l-7.25 7.25a.75.75 0 0 1-1.06 0L2.22 9.28a.751.751 0 0 1 .018-1.042.751.751 0 0 1 1.042-.018L6 10.94l6.72-6.72a.75.75 0 0 1 1.06 0Z"></path>
    </svg>
  </clipboard-copy>
</div>

2. Run the following command to initiate the firmware update process:

<div class="highlight notranslate position-relative">
  <div class="highlight">
    <pre id="command-dfu"><span></span>dfu-util -D I2CRider.dfu</pre>
  </div>
  <clipboard-copy style="position:absolute; right:8px; top:8px;" for="command-dfu">
    <svg aria-hidden="true" height="16" viewBox="0 0 16 16" version="1.1" width="16" data-view-component="true" class="octicon octicon-copy js-clipboard-copy-icon">
    <path d="M0 6.75C0 5.784.784 5 1.75 5h1.5a.75.75 0 0 1 0 1.5h-1.5a.25.25 0 0 0-.25.25v7.5c0 .138.112.25.25.25h7.5a.25.25 0 0 0 .25-.25v-1.5a.75.75 0 0 1 1.5 0v1.5A1.75 1.75 0 0 1 9.25 16h-7.5A1.75 1.75 0 0 1 0 14.25Z"></path><path d="M5 1.75C5 .784 5.784 0 6.75 0h7.5C15.216 0 16 .784 16 1.75v7.5A1.75 1.75 0 0 1 14.25 11h-7.5A1.75 1.75 0 0 1 5 9.25Zm1.75-.25a.25.25 0 0 0-.25.25v7.5c0 .138.112.25.25.25h7.5a.25.25 0 0 0 .25-.25v-7.5a.25.25 0 0 0-.25-.25Z"></path>
    </svg>
    <svg aria-hidden="true" height="16" viewBox="0 0 16 16" version="1.1" width="16" data-view-component="true" class="octicon octicon-check js-clipboard-check-icon color-fg-success d-none">
    <path d="M13.78 4.22a.75.75 0 0 1 0 1.06l-7.25 7.25a.75.75 0 0 1-1.06 0L2.22 9.28a.751.751 0 0 1 .018-1.042.751.751 0 0 1 1.042-.018L6 10.94l6.72-6.72a.75.75 0 0 1 1.06 0Z"></path>
    </svg>
  </clipboard-copy>
</div>

If the update is successful, you will see the message `No error condition is present` This indicates that the firmware has been successfully written to the device. Remember to reset the board by pressing the reset button to exit bootloader mode and enter normal operation.

<div class="img_dv">
   <img class="replay" id="target_image" src="../images/rosrider/dfu_firmware_update.gif" alt="DFU util update output">
</div>

__Next Chapter:__ [Troubleshooting](../10_DEBUG/README.md)