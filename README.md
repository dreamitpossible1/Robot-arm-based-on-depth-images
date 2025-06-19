## Project Title

**A Robot Arm System for RB3 Gen2 Utilizing Depth Images**

---

## Overview

This project develops a robot arm system for RB3 Gen2 using depth images. RB3 Gen2 serves as the core hardware platform, offering advanced motion control and sensor capabilities. Depth images enable the robot arm to perceive 3D environments for tasks like object recognition and grasping. The Qualcomm Intelligent Robotics Product SDK is utilized for efficient image processing and seamless integration with RB3 Gen2, enhancing the system's performance and reliability..


---

## Quick Start with Qualcomm RB3 Gen2

### 1. Download Precompiled Package

Download the precompiled Robotics Product SDK image for RB3 Gen2:

```bash
wget https://artifacts.codelinaro.org/artifactory/qli-ci/flashable-binaries/qirpsdk/qcs6490-rb3gen2-vision-kit/arm-qcom-6.6.65-QLI.1.4-Ver.1.1_robotics-product-sdk-1.1.zip
```

Unzip the downloaded package:

```bash
unzip arm-qcom-6.6.65-QLI.1.4-Ver.1.1_robotics-product-sdk-1.1.zip
```

---

### 2. Prepare the Device

#### Enter Emergency Download (EDL) Mode

**Steps:**

1. Connect the device to host computer via USB Type-C
2. Press and hold the **F\_DL** button
   <p align="center"> <img src="https://github.com/user-attachments/assets/edad3a81-028d-4929-a623-2e8469f661d5" alt="F_DL Button and USB Type-C port" /> </p> <p align="center"><i>Figure: Location of the USB Type-C port and F_DL button</i></p>

4. While holding the button, connect to 12V wall power
5. Release the **F\_DL** button
6. Verify EDL mode on host:

   ```bash
   lsusb
   ```

   **Expected Output:**

   ```
   Bus 002 Device 014: ID 05c6:9008 Qualcomm, Inc. Gobi Wireless Modem (QDL mode)
   ```

#### Update `udev` Rules (Host-side Setup)

Ensure your Linux system has proper `udev` rules to detect the RB3 device in EDL mode.

---

### 3. Flash the Board Using QDL Tool

#### a. Install and Prepare QDL Tool

1. Download [QDL](https://softwarecenter.qualcomm.com/catalog/item/Qualcomm_Device_Loader) Tool (version 2.3.1 or later required)
2. Make `qdl` binary executable:

   ```bash
   chmod +x ./qdl
   ```

#### b. Run Flashing Procedure


```bash
cd <extracted zip directory>/target/qcs6490-rb3gen2-vision-kit/qcom-multimedia-image
<qdl_tool_path>/qdl_2.3.1/QDL_Linux_x64/qdl prog_firehose_ddr.elf rawprogram*.xml patch*.xml
```

#### c. Post-Flash Verification

After the device reboots, verify it's recognized:

```bash
lsusb
```

**Expected Output:**

```
Bus 002 Device 003: ID 05c6:9135 Qualcomm, Inc. qcs6490-rb3gen2-vision-kit
```

---

## System Components

###  Hardware

* **Qualcomm RB3 Gen2 Vision Kit**
* **Robot Arm (3–6 DOF recommended)**
* **Depth Camera Module**

###  Software

* **QIRP SDK**
* **Linux Robotics Distro (ROS2 Jazzy)**
* **TFLite-based depth image perception pipeline**

---

## Run the Robot Arm System
Follow these steps to deploy and launch the robot arm system based on depth images:
1. Clone the Repository
```bash
git clone https://github.com/dreamitpossible1/Robot-arm-based-on-depth-images.git
```
3. Set Permissions and Run Script
```bash
cd Robot-arm-based-on-depth-images
chmod +x run_script.sh
./run_script.sh
```

https://github.com/dreamitpossible1/Robot-arm-based-on-depth-images/blob/main/Scipts_RB3/1.jpg
https://github.com/dreamitpossible1/Robot-arm-based-on-depth-images/blob/main/Scipts_RB3/2.jpg

4. Launch the ROS Node
```bash
roslaunch Robot-arm-based-on-depth-images sagittarius_main.launch
```

## Resources

### 📚 Official Documentation

* [QIRP SDK Introduction](https://docs.qualcomm.com/bundle/publicresource/topics/80-70018-265/introduction_1.html?vproduct=1601111740013072&version=1.4&facet=Qualcomm%20Intelligent%20Robotics%20Product%20%28QIRP%29%20SDK)
* [QIRP Quick Start Guide](https://docs.qualcomm.com/bundle/publicresource/topics/80-70018-265/quick-start_3.html?vproduct=1601111740013072&version=1.4)

### 🔗 External References

* [Qualcomm Linux](https://www.qualcomm.com/developer/software/qualcomm-linux)

---

## Contributing

We welcome contributions from the community!

* [@freezingskrill](https://github.com/freezingskrill)
* [@AIRHEADbot](https://github.com/AIRHEADbot)

Feel free to submit pull requests or open issues for enhancements and bug reports.

---

## Authors

*List your team members or contributors here*

---

## License


---

> 📌 *For further setup, demo videos, and integration tips, please refer to the official Qualcomm Developer Portal or contact the maintainers via GitHub issues.*

---

