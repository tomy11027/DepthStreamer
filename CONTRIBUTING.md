# Contributing to DepthStreamer

Thank you for your interest in contributing to **DepthStreamer**! We welcome contributions from developers, roboticists, computer vision researchers, and enthusiasts.

Whether you're adding support for new Android devices, optimizing UDP throughput, integrating new SLAM pipelines, or improving documentation, your help is appreciated.

---

## Getting Started

1. **Fork the repository** on GitHub.
2. **Clone your fork** locally:
   ```bash
   git clone https://github.com/tomy11027/DepthStreamer.git
   cd DepthStreamer
   ```
3. **Create a topic branch**:
   ```bash
   git checkout -b feature/my-cool-feature
   ```

---

## Development Setup

### 1. Android Application (`app/`)
- Open the root `DepthStreamer` directory in **Android Studio** (Ladybug / Hedgehog or newer).
- Ensure Android SDK 35/36 is installed with Java 17+.
- Build the APK:
  - Inside Android Studio: `Build > Build Bundle(s) / APK(s) > Build APK(s)`
  - Or via command line with Gradle.
- Install on a physical device with Camera2 DEPTH16 / ToF support.

### 2. Python Tools
Create and activate a virtual environment:
```bash
python -m venv .venv
source .venv/bin/activate    # Linux / macOS
# or
.venv\Scripts\activate       # Windows

pip install -r requirements.txt
```

---

## Contribution Areas

We are particularly excited about contributions in:
- **ROS 2 & Robotics**:
  - Launch files for Nav2 and RTAB-Map
  - Odometry fusion (combining phone IMU with ToF depth)
  - `depthimage_to_laserscan` pre-configured parameters
- **Android App**:
  - Exposing IMU (accelerometer / gyroscope) synchronization alongside depth
  - Dual stream: RGB + ToF depth alignment
  - Direct point cloud generation on-device (NDK / C++)
  - Multi-resolution selection
- **Computer Vision & SLAM**:
  - Real-time mesh reconstruction
  - KISS-ICP and ORB-SLAM3 integration tutorials
  - Hand / body gesture detection models using raw ToF

---

## Submitting Pull Requests

1. **Keep changes focused**: A PR should ideally do one thing well.
2. **Test your code**:
   - Verify Python syntax: `python -m py_compile receiver.py`
   - Test UDP streaming locally using loopback or on a local Wi-Fi network.
3. **Write descriptive commit messages**:
   - Good: `feat(app): add support for 60fps depth streaming`
   - Bad: `updates`
4. **Open a Pull Request** against the `main` branch with a clear description of what changed and how it was tested.

---

## Code of Conduct

Please treat everyone with respect, kindness, and empathy. Be collaborative, constructive, and open to feedback.
