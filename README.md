<div align="center">

# 📡 DepthStreamer
### Turn Your Android Phone into a Real-Time 3D Laser / Depth Camera for Navigation, SLAM, and ROS 2

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Android](https://img.shields.io/badge/Android-API%2029%2B-green.svg)](https://developer.android.com)
[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.x-red.svg)](https://opencv.org)
[![NumPy](https://img.shields.io/badge/NumPy-1.22%2B-blue.svg)](https://numpy.org)

<br/>

[![Download APK](https://img.shields.io/badge/Download%20APK-DepthStreamer.apk-brightgreen.svg?style=for-the-badge&logo=android)](DepthStreamer.apk)

**Stream raw, uncompressed 3D Time-of-Flight (ToF) laser depth data from your Android phone directly to your PC, robot computer, or workstation over low-latency UDP at 30+ FPS.**

[Key Features](#-key-features) • [Navigation, SLAM & ROS 2](#-navigation-slam--robotics) • [Quickstart](#-quickstart-guide) • [Network Protocol](#-udp-binary-protocol) • [Supported Hardware](#-supported-hardware)

</div>

---

## 💡 Why DepthStreamer?

Commercial 3D LiDARs and depth sensors (such as Intel RealSense, Ouster, or Livox) cost between **$500 and $2,000+**. 

However, many modern Android smartphones already contain high-speed **Time-of-Flight (ToF) VCSEL infrared laser sensors** capable of measuring millimeter-accurate distances across thousands of points simultaneously.

**DepthStreamer** connects the two worlds:
- **Android App**: Captures raw `DEPTH16` buffers from the Android Camera2 API and broadcasts them over high-speed UDP.
- **`receiver.py`**: A lightweight, standalone Python receiver that reassembles packets into real-time NumPy depth matrices and displays a live night-vision heatmap with millimeter distance telemetry.

---

## ✨ Key Features

- ⚡ **Zero Cloud & Sub-10ms Latency**: Direct peer-to-peer UDP packet transmission over local Wi-Fi or USB tethering.
- 🎯 **Raw Metric Accuracy**: 16-bit depth values providing millimeter precision + 3-bit per-pixel confidence scores.
- 👁️ **Live Visualizer (`receiver.py`)**: Real-time normalized false-color depth stream with center distance telemetry and configurable range scaling.
- 🤖 **Extensible NumPy Buffer**: Incoming depth frames are stored in standard 2D NumPy arrays (`uint16`), making it trivial to pipe into **ROS 2**, **OpenCV**, or custom robotics pipelines.
- 🌙 **True Night Vision**: Operates in pitch-black darkness using the phone's active 850nm/940nm infrared laser emitter.

---

## 🚀 Navigation, SLAM & Robotics

`receiver.py` outputs a standard NumPy 2D array of distances in millimeters (`frame_buffer`) on every frame. You can directly plug this data into:

```
+------------------+         UDP Port 5005         +-----------------------+
|  Android Phone   | ============================> |      receiver.py      |
| (ToF Laser / AR) |  [16-byte Header + DEPTH16]   | (Live NumPy 2D Buffer)|
+------------------+                               +-----------+-----------+
                                                               |
                               +-------------------------------+-------------------------------+
                               |                               |                               |
                               v                               v                               v
                     +-------------------+           +-------------------+           +-------------------+
                     |   ROS 2 & Nav2    |           |    3D / 2D SLAM   |           |  Computer Vision  |
                     | Obstacle Avoidance|           |  (RTAB-Map, ICP)  |           | (Obstacle Detect) |
                     +-------------------+           +-------------------+           +-------------------+
```

### 1. 🤖 ROS 2 & Autonomous Navigation (Nav2)
Convert the 2D depth array into a `sensor_msgs/Image` (encoding `16UC1`) or project to a `sensor_msgs/PointCloud2`. Mount your phone to a robot or rover to act as a 3D obstacle avoidance sensor for **Nav2 costmaps** or use `depthimage_to_laserscan` to create a virtual 2D laser scan.

### 2. 🗺️ 3D SLAM (Simultaneous Localization & Mapping)
Feed depth frames into **RTAB-Map**, **KISS-ICP**, or **ORB-SLAM3** (RGB-D mode) for real-time indoor 3D room mapping and robot localization.

### 3. 🚗 Obstacle Avoidance & Autonomous Rover Control
Compute distances to bounding boxes or minimum forward clearance for collision prevention on DIY Arduino / Raspberry Pi rovers.

---

## 📱 Supported Hardware

Your Android device must have hardware depth sensing capability exposing Camera2 `REQUEST_AVAILABLE_CAPABILITIES_DEPTH_OUTPUT` (rear ToF / 3D Laser sensor or ARCore Depth support):

| Device Family | Models with ToF / Laser Sensor | Typical Resolution |
| :--- | :--- | :--- |
| **Samsung Galaxy** | S20 Ultra, S10 5G, Note 10+, Note 20 Ultra, A80 | 240x180 DEPTH16 |
| **Huawei** | P30 Pro, Mate 30 Pro, P40 Pro, Mate 40 Pro | Full ToF raw stream |
| **Honor** | View 20, 30 Pro+ | Full ToF raw stream |
| **Google Pixel** | Pixel 4 (front), Pixel 7/8/9 (ARCore Depth16) | Standard Camera2 Depth |
| **Other Android** | Any phone supporting Android Camera2 DEPTH16 API | Device-dependent |

---

## 📦 Repository Structure

```
DepthStreamer/
├── app/                              # Android Studio application source
│   ├── src/main/AndroidManifest.xml  # Permissions (Camera, Internet)
│   └── src/main/java/com/example/depthstreamer/
│       ├── MainActivity.kt           # Camera2 capture session & Jetpack Compose UI
│       └── UdpSender.kt              # Chunked UDP packet sender
├── receiver.py                       # 2D OpenCV colorized depth stream visualizer
├── requirements.txt                  # Minimal Python dependencies (numpy, opencv-python)
├── build.gradle.kts                  # Root Gradle build script
├── settings.gradle.kts               # Gradle settings
├── gradlew / gradlew.bat             # Gradle wrapper scripts
├── LICENSE                           # MIT License
└── README.md                         # Documentation
```

---

## ⚡ Quickstart Guide

### 1. Clone the Repository
```bash
git clone https://github.com/tomy11027/DepthStreamer.git
cd DepthStreamer
```

### 2. Install Python Dependencies
```bash
pip install -r requirements.txt
```

### 3. Install the Android App
- **Option A (Direct Download APK)**:
  Download **[DepthStreamer.apk](https://github.com/tomy11027/DepthStreamer/raw/main/DepthStreamer.apk)** directly from this repository and install it on your phone.
- **Option B (Build from Source)**:
  Open this project folder in **Android Studio** and click **Run**, or build via terminal:
  ```powershell
  .\gradlew.bat assembleRelease
  ```

### 4. Connect Device & PC
1. Connect both your phone and PC to the **same Wi-Fi network** (5 GHz Wi-Fi or USB Tethering is recommended for lowest latency).
2. Find your PC's local IP address (`ipconfig` on Windows or `ip addr` on Linux).
3. Open the **DepthStreamer** app on your phone:
   - Enter your PC's IP address.
   - Enter UDP Port: `5005` (default).
   - Tap **"Start Streaming"**.

### 5. Run the Receiver
Start the Python receiver on your PC:
```bash
python receiver.py
```

*Command-Line Options:*
```bash
python receiver.py --port 5005 --max-depth 3000 --scale 4
```
- `--ip`: UDP IP to bind (default: `0.0.0.0`)
- `--port`: UDP port to listen on (default: `5005`)
- `--max-depth`: Maximum distance in millimeters for colormap scaling (default: `3000`)
- `--scale`: Window upscale factor (default: `4`)
- `--quiet`: Mute terminal distance matrix printing

Press **`q`** in the window or **Ctrl+C** in the terminal to exit.

---

## 📡 UDP Binary Protocol

Data is transmitted as raw binary UDP packets chunked to fit under standard MTU limits (<65,535 bytes).

### Packet Structure
Each UDP packet starts with a **16-byte Little-Endian binary header**:

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                      Width (uint32_t)                         |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                      Height (uint32_t)                        |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                     Y-Offset (uint32_t)                       |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                      Y-Count (uint32_t)                       |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                 DEPTH16 Pixels (2 bytes each) ...             |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

### DEPTH16 Pixel Bit Breakdown
Each pixel is a 16-bit unsigned integer conforming to Android `ImageFormat.DEPTH16`:
- **Bits 0–12 (13 bits)**: Radial distance in millimeters ($0$ to $8,191\text{ mm}$). $0$ indicates invalid/no return.
- **Bits 13–15 (3 bits)**: Measurement confidence ($0 = 0\%$, $7 = 100\%$).

---

## 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.
