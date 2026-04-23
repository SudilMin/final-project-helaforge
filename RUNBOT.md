# 🤖 Running the Physical Robot (Kobuki + Kinect)

This guide explains how to run the semantic navigation system on the **physical Kobuki robot** using a **Kinect camera** — no simulation required.

---

## 📦 Prerequisites

### 1. ROS 2 & Workspace
Make sure ROS 2 Jazzy and the workspace are built:
```bash
cd ~/Music/final-project-helaforge
source /opt/ros/jazzy/setup.bash
colcon build --packages-select diff_drive_robot --symlink-install
source install/setup.bash
```

### 2. Python Dependencies
```bash
pip3 install pyserial ultralytics vosk sounddevice --break-system-packages
```

### 3. Kinect Python Wrapper (freenect)
> Follow the full setup guide in the project Wiki.
> Once done, verify with:
```bash
python3 -c 'import freenect; print("Kinect wrapper OK")'
```

### 4. Create the map output folder
```bash
mkdir -p ~/Documents/ROS_final_project
```

---

## 🔌 Hardware Setup

### Plug in devices
| Device | Port |
|---|---|
| Kobuki robot | USB → usually `/dev/ttyUSB0` |
| Kinect camera | USB (3 separate USB connections: Motor, Camera, Audio) |

### Verify hardware is detected
```bash
# Kobuki serial port
ls /dev/ttyUSB*

# Kinect USB devices
lsusb | grep "045e"
# Should show 3 devices: 02ae (Camera), 02b0 (Motor), 02ad (Audio)

# Grant serial access without sudo
sudo chmod 666 /dev/ttyUSB0
```

### Test Kinect is working
```bash
# Visual test (opens a window)
freenect-glview

# Camera test only
freenect-camtest
```

---

## 🚀 Launch Sequence

Open **4 terminals** and run in this exact order.

---

### Terminal 1 — Kobuki Driver
Connects to the physical robot over serial. Subscribes to `/cmd_vel` to drive wheels and publishes `/odom` from wheel encoders.

```bash
source /opt/ros/jazzy/setup.bash
source ~/Music/final-project-helaforge/install/setup.bash

ros2 run diff_drive_robot kobuki_driver --ros-args -p serial_port:=/dev/ttyUSB0
```

✅ Expected output:
```
[kobuki_driver]: Kobuki connected on /dev/ttyUSB0
[kobuki_driver]: KobukiDriver ready.
```

---

### Terminal 2 — Kinect Bridge
Reads RGB and Depth frames from the physical Kinect and publishes them as ROS 2 image topics.

```bash
source /opt/ros/jazzy/setup.bash
source ~/Music/final-project-helaforge/install/setup.bash

ros2 run diff_drive_robot kinect_bridge
```

✅ Expected output:
```
[kinect_bridge]: KinectBridge started — publishing RGB + Depth at 15 Hz
```

> ⚠️ If you see `Error: Can't open device` — make sure the Kinect is plugged in and you have run:
> ```bash
> sudo udevadm control --reload-rules && sudo udevadm trigger
> ```

---

### Terminal 3 — Semantic Navigator (Brain)
Runs YOLO object detection, records object positions, and drives the robot autonomously. **Type commands directly here.**

```bash
source /opt/ros/jazzy/setup.bash
source ~/Music/final-project-helaforge/install/setup.bash

ros2 run diff_drive_robot semantic_navigator
```

✅ Expected output:
```
[semantic_navigator]: YOLO loaded ✓
[semantic_navigator]: Ready. Commands: scan | stop scan | list | <object_id>

=== Semantic Navigator CLI ===
Commands: scan | stop scan | list | <object_id>
```

---

### Terminal 4 — Input Method (choose one)

**Option A: Voice Commands (microphone)**
```bash
source /opt/ros/jazzy/setup.bash
source ~/Music/final-project-helaforge/install/setup.bash

ros2 run diff_drive_robot voice_commander
```

**Option B: Manual Keyboard Driving**
```bash
source /opt/ros/jazzy/setup.bash
source ~/Music/final-project-helaforge/install/setup.bash

ros2 run diff_drive_robot arrow_teleop
```
> Use `↑ ↓ ← →` arrow keys to drive. Press `Q` to quit.

---

## 🎮 Full Workflow

### Step 1 — Start Scanning
In **Terminal 3**, type:
```
scan
```
Or say **"scan"** into the microphone (Terminal 4 voice mode).

The robot will start logging any objects it sees through the Kinect.

---

### Step 2 — Drive Around the Room
Use **Terminal 4** (arrow teleop) to manually drive the robot around the environment.

Point the Kinect at objects like chairs, bottles, people, etc.

You will see detections appear in Terminal 3:
```
[semantic_navigator]: NEW OBJECT: chair_1   dist=1.2m  pos=(2.31, 0.88)
[semantic_navigator]: NEW OBJECT: bottle_1  dist=0.8m  pos=(1.10, -0.45)
```

---

### Step 3 — List Detected Objects
In **Terminal 3**, type:
```
list
```
Output:
```
  1. chair_1      → (+2.31, +0.88)
  2. bottle_1     → (+1.10, -0.45)
```

---

### Step 4 — Navigate to an Object
In **Terminal 3**, type the object ID:
```
chair_1
```
Or say **"go to chair 1"** (voice mode).

The robot will autonomously drive to 1 metre in front of `chair_1` and stop.

```
[semantic_navigator]: Navigating to chair_1 …
[semantic_navigator]: Arrived at chair_1
```

---

### Step 5 — Return Home
In **Terminal 3**, type:
```
stop scan
```
Or say **"stop scan"**.

The robot will retrace its path back to where it started.

---

## ✅ Topic Verification

You can verify everything is connected correctly in a new terminal:

```bash
source /opt/ros/jazzy/setup.bash

# Check all topics are publishing
ros2 topic list

# Verify odometry rate (~10 Hz)
ros2 topic hz /odom

# Verify Kinect camera rate (~15 Hz)
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/depth/image_raw

# Monitor velocity commands when navigating
ros2 topic echo /cmd_vel

# Monitor detected object commands
ros2 topic echo /semantic_nav/command
```

---

## 🛠️ Troubleshooting

| Problem | Fix |
|---|---|
| `Cannot open /dev/ttyUSB0` | Run `sudo chmod 666 /dev/ttyUSB0` |
| `Error: Can't open device` (Kinect) | Unplug and replug Kinect; run `sudo udevadm trigger` |
| `freenect not found` | Run `sudo python3 ~/libfreenect/wrappers/python/setup.py install` |
| Robot doesn't move | Check `/cmd_vel` is publishing; verify kobuki_driver is connected |
| YOLO not detecting objects | Ensure good lighting; objects must be 0.5–5.0 m from Kinect |
| `vosk model not found` | Download: `wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip && unzip *.zip && mv vosk-model-small-en-us-0.15 ~/vosk-model` |

---

## 📁 Map Persistence

Detected objects are automatically saved to:
```
~/Documents/ROS_final_project/yolo_semantic_map.json
```

This map is reloaded automatically the next time you run `semantic_navigator`, so previously detected objects are remembered across sessions.

---

## 🏗️ System Architecture

```
Physical Kinect
      │ (freenect Python wrapper)
      ▼
kinect_bridge.py ──► /camera/image_raw
                 ──► /camera/depth/image_raw
                 ──► /camera/camera_info

Physical Kobuki
      │ (serial 115200 baud)
      ▼
kobuki_driver.py ──► /odom  (wheel encoder odometry)
                 ◄── /cmd_vel (velocity commands)

voice_commander.py ──► /semantic_nav/command
  OR
stdin CLI          ──► /semantic_nav/command

                         ▼
               semantic_navigator.py
               ├── YOLO detection on /camera/image_raw
               ├── depth estimation from /camera/depth/image_raw
               ├── position tracking from /odom
               ├── obstacle avoidance from /scan (optional LiDAR)
               ├── object map: ~/Documents/ROS_final_project/yolo_semantic_map.json
               └── PID navigation ──► /cmd_vel ──► Kobuki wheels
```
