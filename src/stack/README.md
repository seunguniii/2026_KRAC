# Flight Stack for KRAC 2026

---

## System Architecture
Mission Manager coordinates, determines mission progression based on subsystems as the central cooridnator.

---

## Mission-State Communication

Subsystem status is maintained through bit-packed state representation.

| State  |  IDLE   |  BUSY   |COMPLETED|  ERROR  |
|--------|---------|---------|---------|---------|
| Binary |   00    |   01    |   10    |   11    |


---

## Implemented Componenets

- Mission Manager
- MIssion GUI
- Logger

---

## Planned Components

- Waypoint Nav
- Vision
- Target Guidance
- Gripper

---

## Tech Stack

- ROS2 Humble
- PX4 Autopilot firmware v1.17.0
- C++, Python
