# Flight Stack for KRAC 2026

---

## System Architecture

Mission Manager serves as the central coordinator, managing mission progression based on the status of each subsystem.

### 'stack_cpp'
ROS2 ('rclcpp') nodes:
- Mission Manager
- Flight
- Target Guidance
- Gripper

### 'stack_py'
ROS2 ('rclpy') nodes:
- Vision
- Marker Detection
- YOLO
- Mission GUI

### 'mission_launch'
Launch files and configuration(s):
- mission.launch.py
- config/trajectory.yaml

---

## Flight Stack Bring-Up

1. Launch the flight stack.

'''bash
$ ros2 launch mission_launch mission.launch.py
'''

2. Start the Mission GUI.

'''bash
$ ros2 run stack_py mission_gui
'''

3. Press **S** in the Mission GUI to start the mission

---

## Mission State Communication

Subsystem status is maintained through bit-packed state representation.

| State  |  IDLE   |  BUSY   | SUCCESS |  ABORT  |
|--------|---------|---------|---------|---------|
| Binary |   00    |   01    |   10    |   11    |


---

## Implemented Components

- Mission Manager
- Flight
- Vision
- Marker Detection
- Target Guidance
- Logger
- Mission GUI

---

## Planned Components

- YOLO
- Gripper

---

## Tech Stack

- ROS2 Humble
- PX4 Autopilot firmware v1.17.0
- C++, Python
