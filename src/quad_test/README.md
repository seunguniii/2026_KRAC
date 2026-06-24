# Flight Test Package for Quadcopter

---

## Evaluation Elements

- Euler Angles (Pitch, Roll, Yaw)
- Setpoint Accuracy

---

## Origin

- Sets takeoff coordinate as local origin

---

## Expected Behavior

- Takeoff @ altitude 2m with yaw = 0.0 (North)
- Go forwards 2m, backwards 2m (pitch)
- Go right 2m, left 2m (roll)
- Ascend @ altitude 3m
- Land

![flight_test_demo.gif](materials/quad_flight_test_demo.gif)

---

## Tech Stack

- ROS2 Humble
- PX4 Autopilot firmware v1.17.0
- C++
