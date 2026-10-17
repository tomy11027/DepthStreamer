<div align="center">

# 📡 DepthStreamer
### Turn Your Android Phone into a Real-Time 3D LiDAR & Depth Camera for Robotics, SLAM, and Computer Vision

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Android](https://img.shields.io/badge/Android-API%2029%2B-green.svg)](https://developer.android.com)
[![Python](https://img.shields.io/badge/Python-3.9%2B-blue.svg)](https://www.python.org)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Iron%20%7C%20Jazzy-orange.svg)](https://docs.ros.org)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.x-red.svg)](https://opencv.org)
[![Open3D](https://img.shields.io/badge/Open3D-0.17%2B-purple.svg)](http://www.open3d.org)

**Stream raw, uncompressed 3D Time-of-Flight (ToF) laser depth data from Android directly to PC, ROS 2, or embedded computers over low-latency UDP at 30+ FPS.**

[Key Features](#-key-features) • [Robotics & SLAM Activities](#-robotics-slam--advanced-activities) • [Quickstart](#-quickstart-guide) • [ROS 2 Setup](#-ros-2-integration) • [Protocol](#-binary-network-protocol) • [Hardware Compatibility](#-supported-hardware)

</div>

---

## 💡 Why DepthStreamer?

Commercial 3D LiDARs and RGB-D depth sensors (such as Intel RealSense, Ouster, or Livox) cost between **$500 and $2,000+**. 

However, many modern Android smartphones already contain high-speed **Time-of-Flight (ToF) VCSEL infrared laser sensors** capable of measuring millimeter-accurate distances across thousands of points simultaneously.

**DepthStreamer** unlocks this hardware:
- Accesses raw Android Camera2 `DEPTH16` buffers directly.
- Streams real-time distance and confidence arrays over UDP with sub-10ms network latency.
- Provides ready-to-run pipelines for **ROS 2 Nav2, 3D SLAM, Point Cloud visualization, and ICP-based 3D object scanning**.

---

## ✨ Key Features

- ⚡ **Zero Cloud & Zero Latency**: Direct device-to-host peer-to-peer UDP transmission.
- 🎯 **Raw Metric Accuracy**: 16-bit depth values providing millimeter precision + 3-bit per-pixel confidence scores.
- 🤖 **Native ROS 2 Publisher**: Instantly broadcasts standard `sensor_msgs/Image`, `sensor_msgs/CameraInfo`, and `sensor_msgs/PointCloud2`.
- 🗺️ **SLAM & Nav2 Ready**: Drop-in sensor replacement for autonomous mobile robots, obstacle avoidance costmaps, and RTAB-Map.
- 👁️ **2D Jet Heatmap Visualizer** (`receiver.py`): Real-time normalized false-color depth stream with center distance telemetry.
- 🌐 **Interactive 3D Point Cloud Viewer** (`rviz_viewer.py`): Real-time Open3D hardware-accelerated 3D point cloud renderer.
- 📦 **Handheld 3D Scanner** (`object_scanner.py`): Multi-frame ICP alignment, voxel downsampling, statistical noise filtering, and `.ply` point cloud export.
- 🌙 **True Night Vision**: Operates in pitch-black darkness using active infrared laser illumination.

---

## 🚀 Robotics, SLAM & Advanced Activities

```
+------------------+         Low-Latency UDP (Port 5005)         +--------------------------+
|  Android Device  | ==========================================> |    Host PC / Robot SBC   |
| (ToF Laser / AR) |   [16-byte Header + DEPTH16 Chunks]         | (Ubuntu / Windows / Pi)  |
+------------------+                                             +-------------+------------+
                                                                               |
              +-----------------------+-----------------------+----------------+
              |                       |                       |
              v                       v                       v
      +---------------+       +---------------+       +---------------+
      |  receiver.py  |       | rviz_viewer.py|       |  ROS 2 Node   |
      | (2D Heatmap)  |       | (3D Open3D)   |       |  (Image/PCL)  |
      +---------------+       +---------------+       +-------+-------+
                                                              |
                               +------------------------------+------------------------------+
                               |                              |                              |
                               v                              v                              v
                      +------------------+           +------------------+           +------------------+
                      |   Nav2 Costmap   |           |    RTAB-Map /    |           |     KISS-ICP     |
                      | 3D Obstacle Clr  |           |     ORB-SLAM3    |           |    Point SLAM    |
                      +------------------+           +------------------+           +------------------+
```

### 1. 🤖 ROS 2 & Autonomous Navigation (Nav2)
Mount your smartphone on an autonomous mobile robot (AMR), TurtleBot, or drone:
- Broadcasts to `/camera/depth/points` for **Nav2 3D Obstacle Layer** voxel costmaps.
- Convert depth to virtual 2D laser scans using `depthimage_to_laserscan` to replace or supplement physical 2D LiDARs.

### 2. 🗺️ 3D SLAM (Simultaneous Localization & Mapping)
- **RTAB-Map (Real-Time Appearance-Based Mapping)**: Combine phone depth stream with camera odometry for full 3D dense indoor mapping.
- **KISS-ICP / Fast-LIO / CT-ICP**: Feed raw point clouds into modern LiDAR odometry algorithms for real-time trajectory estimation.
- **ORB-SLAM3**: Utilize calibrated depth frames for robust tracking without scale drift.

### 3. 📦 Handheld 3D Object & Room Scanning
- Use `object_scanner.py` to walk around objects or rooms.
- Real-time **Iterative Closest Point (ICP)** registers incoming frames into a unified global point cloud.
- Exports to standard `.ply` files compatible with **Blender, MeshLab, CloudCompare, Unreal Engine, and 3D printing**.

### 4. 🚹 Human Tracking & Gesture Recognition
- Infrared laser reflection is unaffected by ambient light or shadows, making it ideal for skeleton tracking, presence detection, and gesture control in robotics.

---

## 📱 Supported Hardware

Your Android device must have hardware depth sensing capability exposing Camera2 `REQUEST_AVAILABLE_CAPABILITIES_DEPTH_OUTPUT` (typically rear ToF / 3D Laser sensor or ARCore Depth support):

| Device Family | ToF / Laser Sensor Support | Typical Resolution |
| :--- | :--- | :--- |
| **Samsung Galaxy** | S20 Ultra, S10 5G, Note 10+, Note 20 Ultra, A80 | 240x180 DEPTH16 |
| **Huawei** | P30 Pro, Mate 30 Pro, P40 Pro, Mate 40 Pro | Full ToF raw stream |
| **Honor** | View 20, 30 Pro+ | Full ToF raw stream |
| **Google Pixel** | Pixel 4 (front), Pixel 7/8/9 (ARCore Depth16) | Standard Camera2 Depth |
| **Other Android** | Any phone supporting Android Camera2 DEPTH16 API | Device-dependent |

> **Note:** Devices that only use single-camera software stereo depth without hardware ToF can still run if the manufacturer exposes standard Android `DEPTH16` through Camera2.

---

## 📦 Repository Structure

```
DepthStreamer/
├── app/                              # Android Studio application source
│   ├── src/main/AndroidManifest.xml  # Camera & Internet permissions
│   └── src/main/java/com/example/depthstreamer/
│       ├── MainActivity.kt           # Camera2 capture session & UI
│       └── UdpSender.kt              # Chunked UDP socket packet sender
├── receiver.py                       # 2D OpenCV colorized depth visualizer
├── rviz_viewer.py                    # 3D Open3D real-time point cloud viewer
├── ros2_depth_publisher.py           # Native ROS 2 Image, CameraInfo & PointCloud2 node
├── object_scanner.py                 # Handheld 3D scanner GUI with ICP alignment & PLY export
├── opencv_object_scanner.py          # Lightweight OpenCV 3D point cloud scanner
├── requirements.txt                  # Python dependencies
├── build.gradle.kts                  # Android Gradle build script
├── settings.gradle.kts               # Android project settings
├── LICENSE                           # MIT License
└── CONTRIBUTING.md                   # Community contribution guide
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
# Create and activate virtual environment (recommended)
python -m venv .venv
source .venv/bin/activate    # On Linux/macOS
# or: .venv\Scripts\activate # On Windows

pip install -r requirements.txt
```

### 3. Build & Install Android App
1. Open the `DepthStreamer` root folder in **Android Studio**.
2. Connect your Android phone with **USB Debugging** enabled.
3. Click **Run** (`Shift + F10`) or build the APK via `Build > Build Bundle(s) / APK(s) > Build APK(s)`.
4. Grant the camera permission when prompted.

### 4. Connect Device & PC
1. Connect both your phone and PC to the **same Wi-Fi network** (5 GHz Wi-Fi or USB Tethering is recommended for lowest latency).
2. Find your PC's local IP address:
   - **Linux / macOS**: `ip addr` or `ifconfig`
   - **Windows**: `ipconfig` (look for IPv4 Address, e.g., `192.168.1.50`)
3. Open the **DepthStreamer** app on your phone:
   - Enter your PC's IP address.
   - Enter UDP Port: `5005` (default).
   - Tap **"Start Streaming"**.

---

## 🖥️ Running the Python Tools

### Option A: 2D Depth Stream Visualizer
Shows a night-vision color heatmap with center distance in millimeters:
```bash
python receiver.py
```
*Options:*
```bash
python receiver.py --port 5005 --max-depth 4000 --scale 4
```

### Option B: 3D Point Cloud Visualizer (RViz Style)
Launches an interactive 3D window displaying live depth points in real space:
```bash
python rviz_viewer.py
```
*Controls:*
- **Left Click + Drag**: Rotate 3D view
- **Right Click + Drag**: Pan
- **Scroll Wheel**: Zoom

### Option C: 3D Object Scanner with ICP Alignment
Capture multiple frames, align them using point-to-point ICP, filter noise, and save clean `.ply` 3D models:
```bash
python object_scanner.py
```

---

## 🤖 ROS 2 Integration

DepthStreamer includes a complete, production-grade ROS 2 node compatible with **ROS 2 Humble, Iron, and Jazzy**.

### 1. Launch the Publisher
Ensure your ROS 2 environment is sourced:
```bash
source /opt/ros/humble/setup.bash
python3 ros2_depth_publisher.py
```

### 2. Published ROS 2 Topics
| Topic Name | Type | Description |
| :--- | :--- | :--- |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | Raw 16UC1 depth frame in millimeters |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera intrinsics matrix ($f_x, f_y, c_x, c_y$) |
| `/camera/depth/points` | `sensor_msgs/msg/PointCloud2` | 3D $(X, Y, Z)$ point cloud + intensity confidence |

### 3. Visualize in RViz2
In another terminal:
```bash
rviz2
```
1. Set **Fixed Frame** to `camera_depth_optical_frame`.
2. Click **Add** -> By topic -> `/camera/depth/points` (`PointCloud2`).
3. Set **Color Transformer** to `AxisColor` or `Intensity`.

### 4. Nav2 Obstacle Avoidance Configuration
To use the phone's laser depth stream with **Nav2 Costmap2D**, add the following to your `nav2_params.yaml`:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: depth_pointcloud
        depth_pointcloud:
          topic: /camera/depth/points
          max_obstacle_height: 2.0
          min_obstacle_height: 0.05
          obstacle_max_range: 3.5
          obstacle_min_range: 0.1
          raytrace_max_range: 4.0
          raytrace_min_range: 0.1
          clearing: True
          marking: True
          data_type: "PointCloud2"
```

---

## 📡 Binary Network Protocol

To achieve ultra-low latency without CPU video-encoding overhead, data is transmitted as raw binary UDP packets chunked to fit under the standard MTU limit (<65,535 bytes).

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

### Default Camera Pinhole Intrinsics
When held in portrait orientation (rotated 90° clockwise):
$$\text{Resolution} = 180 \times 240$$
$$f_x = 203.1, \quad f_y = 203.7$$
$$c_x = 90.0, \quad c_y = 120.0$$

3D coordinate reconstruction:
$$Z = \frac{\text{depth\_mm}}{1000.0} \quad [\text{meters}]$$
$$X = \frac{(u - c_x) \cdot Z}{f_x}, \quad Y = \frac{(v - c_y) \cdot Z}{f_y}$$

---

## 🛠️ Troubleshooting & Tips

- **No depth camera found error**: Your device does not expose a hardware depth sensor via Camera2. Check with the "Camera2 API Probe" app on Google Play to verify if `DEPTH_OUTPUT` is listed in your rear camera capabilities.
- **Packets not received on PC**:
  - Verify your PC firewall is allowing incoming UDP traffic on port `5005`.
  - Windows PowerShell:
    ```powershell
    New-NetFirewallRule -DisplayName "DepthStreamer UDP" -Direction Inbound -LocalPort 5005 -Protocol UDP -Action Allow
    ```
  - Linux `ufw`:
    ```bash
    sudo ufw allow 5005/udp
    ```
- **Packet loss / high jitter**:
  - Connect your PC and phone to 5 GHz Wi-Fi instead of 2.4 GHz.
  - Or enable **USB Ethernet / USB Tethering** between your phone and PC for the lowest possible latency (< 2ms).

---

## 🤝 Contributing

Contributions are welcome! Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on submitting pull requests, reporting issues, and suggesting enhancements.

---

## 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.
