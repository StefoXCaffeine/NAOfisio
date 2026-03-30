# NAOfisio

> AI-powered physiotherapy assistant using the NAO humanoid robot with real-time pose estimation

![Python](https://img.shields.io/badge/Python-3.10-blue)
![ROS2](https://img.shields.io/badge/ROS2-Humble-orange)
![MediaPipe](https://img.shields.io/badge/MediaPipe-Pose-red)
![License](https://img.shields.io/badge/License-MIT-green)

---

## Overview

NAOfisio is a **ROS2-based project** that transforms the NAO humanoid robot into an interactive physiotherapy assistant. The system uses **MediaPipe Pose** for real-time human pose estimation to guide patients through therapeutic exercises and provides **AI-generated vocal feedback** via Google Gemini to correct posture in real-time.

The robot:
- Captures images from both NAO cameras (top and bottom)
- Detects and tracks patient pose landmarks
- Guides patients through predefined exercise poses
- Generates natural language corrections using Gemini AI
- Speaks feedback to the patient in Italian

---

## Features

| Feature | Description |
|---------|-------------|
| **Dual Camera Processing** | Combines top and bottom NAO cameras for complete pose coverage |
| **Real-time Pose Estimation** | MediaPipe Holistic for body, hand, and face landmark detection |
| **Orientation Detection** | Automatically detects if patient is facing front, sideways, or back |
| **Joint Angle Calculation** | Computes shoulder roll/pitch and elbow angles in real-time |
| **Voice Guidance** | Italian text-to-speech feedback for patient instructions |
| **AI Posture Correction** | Gemini-powered natural language suggestions for posture correction |
| **6 Physiotherapy Poses** | Predefined poses including T-Pose, Candelabro, Mani in alto, and more |

---

## Project Architecture

```
NAO Robot
    │
    ├── Top Camera (Front View)
    │       │
    │       ▼
    └── Bottom Camera
            │
            ▼
    ┌───────────────────┐
    │  Camera Pose Node │ ◄── nao_camera_viewer.py
    │  - Dual frame merge
    │  - MediaPipe Holistic
    │  - Landmark extraction
    │  - Angle calculation
    │  - Orientation detection
    └─────────┬─────────┘
              │ /nao_pose_info (Float32MultiArray)
              ▼
    ┌───────────────────┐
    │   NAO Routine Node│ ◄── nao_routine.py
    │  - Pose comparison
    │  - Tolerance check
    │  - Gemini AI query
    │  - TTS feedback
    └───────────────────┘
```

---

## Tech Stack

| Category | Technology |
|----------|------------|
| **Framework** | ROS2 (Humble) |
| **Robot SDK** | NAOqi (qi) |
| **Pose Estimation** | MediaPipe Holistic |
| **AI Generation** | Google Gemini API |
| **Computer Vision** | OpenCV |
| **TTS** | ALTextToSpeech (NAO) |
| **Motion Control** | ALMotion, ALRobotPosture |

---

## Installation

### Prerequisites

- ROS2 Humble (or compatible)
- Python 3.10+
- NAO Robot (physical or simulated)

### 1. Clone the repository

```bash
cd ~/nao_ws/src
git clone https://github.com/StefoXCaffeine/NAOfisio.git
```

### 2. Install dependencies

```bash
pip install rclpy opencv-python numpy mediapipe google-generativeai
```

### 3. Build the package

```bash
cd ~/nao_ws
colcon build --packages-select user_input
source install/setup.bash
```

---

## Usage

### Launch Camera Pose Node

This node connects to NAO, captures camera feeds, and publishes pose data:

```bash
ros2 run user_input nao_camera_viewer --ros-args -p ip:=192.168.0.102 -p port:=9559
```

### Launch Routine Node

This node subscribes to pose data, checks patient alignment, and provides voice feedback:

```bash
ros2 run user_input nao_routine --ros-args -p ip:=192.168.0.102 -p port:=9559
```

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ip` | `192.168.0.102` | NAO robot IP address |
| `port` | `9559` | NAOqi service port |

---

## Physiotherapy Poses

The system includes 6 predefined poses:

| ID | Name | Orientation | Description |
|----|------|-------------|-------------|
| 0 | In piedi | Front | Standing neutral position |
| 1 | T-Pos | Front | Arms extended horizontally |
| 2 | Candelabro | Front | Arms up with bent elbows |
| 3 | Mani in alto | Front | Both arms raised overhead |
| 4 | Zombi | Side | Side profile with arms hanging |
| 5 | Mani dietro la testa | Back | Arms behind head |

---

## Published Topics

### `/nao_pose_info` (Float32MultiArray)

Contains 7 values:
- `data[0]`: Orientation (0=front, 1=side, 2=back)
- `data[1]`: Left Shoulder Roll
- `data[2]`: Left Shoulder Pitch
- `data[3]`: Left Elbow Roll
- `data[4]`: Right Shoulder Roll
- `data[5]`: Right Shoulder Pitch
- `data[6]`: Right Elbow Roll

---

## Project Structure

```
NAOfisio/
├── Code/
│   └── user_input/              # ROS2 package
│       ├── package.xml          # Package manifest
│       ├── setup.py             # Build configuration
│       └── user_input/
│           ├── __init__.py
│           ├── nao_camera_viewer.py   # Camera + pose estimation node
│           ├── nao_routine.py         # Routine + AI feedback node
│           └── poses.json             # Physiotherapy poses definition
└── Documents/
    ├── naofisioterapia.pdf       # Project documentation
    └── NAOFISIOTERAPIA.pptx     # Presentation slides
```

---

## How It Works

1. **Camera Capture**: Both NAO cameras capture frames at 30 FPS
2. **Frame Combination**: Top and bottom frames are vertically concatenated
3. **Pose Detection**: MediaPipe extracts 33 body landmarks + hand landmarks
4. **Angle Calculation**: Shoulder roll/pitch and elbow angles are computed
5. **Orientation Check**: System detects if patient is facing front/side/back
6. **Comparison**: Current angles are compared against target pose angles
7. **AI Feedback**: If deviations exceed tolerance (0.3 rad), Gemini generates correction advice
8. **Voice Output**: NAO speaks the correction in Italian

---

## Future Improvements

- [ ] Transfer learning with custom physiotherapy pose dataset
- [ ] K-Fold Cross-Validation for pose classification
- [ ] Historical session tracking and progress reports
- [ ] Integration with NAO's emotional recognition
- [ ] Web dashboard for physiotherapists

---

## License

MIT License - see `LICENSE` file for details.

---

## Author

**Stefano Corrao**

- GitHub: [@StefoXCaffeine](https://github.com/StefoXCaffeine)
- LinkedIn: [linkedin.com/in/stefano-corrao](https://www.linkedin.com/in/stefano-corrao)

---

*If you find this project useful, leave a star!* ⭐
