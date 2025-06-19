## Project Title
A Robot Arm System for RB3 Gen2 Utilizing Depth Images


## Overview
This project develops a robot arm system for RB3 Gen2 using depth images. RB3 Gen2 serves as the core hardware platform, offering advanced motion control and sensor capabilities. Depth images enable the robot arm to perceive 3D environments for tasks like object recognition and grasping. The Qualcomm Intelligent Robotics Product SDK is utilized for efficient image processing and seamless integration with RB3 Gen2, enhancing the system's performance and reliability.
## Quick Start with QualComm RB3 gen2
Download the precompiled package for RB3 Gen2：

wget https://artifacts.codelinaro.org/artifactory/qli-ci/flashable-binaries/qirpsdk/qcs6490-rb3gen2-vision-kit/arm-qcom-6.6.65-QLI.1.4-Ver.1.1_robotics-product-sdk-1.1.zip

Use the following command to unzip the package:

unzip arm-qcom-6.6.65-QLI.1.4-Ver.1.1_robotics-product-sdk-1.1.zip

Configure prerequisites 

Ensure that the device is in emergency download (EDL) mode, and then proceed to update the udev rules.

Move to EDL mode

Using F_DL button

1. Connect the device to the host computer through the USB Type-C connector.

2. Hold down the F_DL button.

   ![image](https://github.com/user-attachments/assets/6ab908fe-d246-4eb4-aad0-28681d32597a)
3. Connect the device to a 12-V wall power supply.

4. Release the F_DL button. The device should now be in EDL mode.

5. To verify whether the device has entered EDL mode, run the following command on your host computer:
   lsusb
   Sample output  

Bus 002 Device 014: ID 05c6:9008 Qualcomm, Inc. Gobi Wireless Modem (QDL mode)

Update udev rules

Flash using the QDL tool

1. Download the QDL tool and unzip the contents of the downloaded folder. Qualcomm Linux 1.4 requires QDL version 2.3.1 or higher.

2. To provide executable permission, run the following command:
   chmod +x ./qdl
3. To flash the images, run the following commands:
   cd <extracted zip directory path>/target/qcs6490-rb3gen2-vision-kit/qcom-multimedia-image
   <qdl_tool_path>/qdl_2.3.1/QDL_Linux_x64/qdl prog_firehose_ddr.elf rawprogram*.xml patch*.xml

Run the lsusb command after the device successfully reboots following a flashing operation to view the device information in the terminal window, as shown in line 4 of the following message:

# Sample output for QCS6490
Bus 002 Device 003: ID 05c6:9135 Qualcomm, Inc. qcs6490-rb3gen2-vision-kit
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub


The specific content is as follows:[QualComm Intelligent Robotics Product SDK Quick Start]([QIRP User Guide - Qualcomm® Linux Documentation](https://docs.qualcomm.com/bundle/publicresource/topics/80-70018-265/quick-start_3.html?vproduct=1601111740013072&version=1.4&facet=Qualcomm Intelligent Robotics Product (QIRP) SDK)

...

## Contributing
https://github.com/freezingskrill

https://github.com/AIRHEADbot

...


## Authors

...

## License

...

## Reference

- [Qualcomm Linux](https://www.qualcomm.com/developer/software/qualcomm-linux)

- [QualComm Intelligent Robotics Product SDK](https://docs.qualcomm.com/bundle/publicresource/topics/80-70018-265/introduction_1.html?vproduct=1601111740013072&version=1.4&facet=Qualcomm Intelligent Robotics Product (QIRP) SDK)
