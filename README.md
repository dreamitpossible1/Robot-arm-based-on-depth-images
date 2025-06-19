## Project Title

**A Robot Arm System for RB3 Gen2 Utilizing Depth Images**

---

## Overview

This project develops a robot arm system for **Qualcomm RB3 Gen2** using **depth images** to enable intelligent 3D perception. The RB3 Gen2 platform provides powerful motion control and sensor integration, and the **Qualcomm Intelligent Robotics Product SDK (QIRP SDK)** is used for image processing and system integration.

Key features include:

* Depth image–based object recognition and grasping
* Integration with RB3 Gen2 vision and motion stack
* Efficient, real-time perception using Qualcomm AI tools

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
3. While holding the button, connect to 12V wall power
4. Release the **F\_DL** button
5. Verify EDL mode on host:

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

Navigate to the extracted SDK path and run the flashing command:

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

### ✅ Hardware

* **Qualcomm RB3 Gen2 Vision Kit**
* **Robot Arm (3–6 DOF recommended)**
* **Depth Camera Module**

### ✅ Software

* **QIRP SDK 1.1**
* **Linux Robotics Distro (ROS2 Jazzy)**
* **TFLite-based depth image perception pipeline**

---

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

*Specify your license type here (e.g., MIT, BSD, Apache 2.0)*

---

> 📌 *For further setup, demo videos, and integration tips, please refer to the official Qualcomm Developer Portal or contact the maintainers via GitHub issues.*

---

