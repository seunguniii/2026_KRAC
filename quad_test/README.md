# Flight Test Package for Quadcopter

---

## flight_test

###Evaluation Elements

- Euler Angles (Pitch, Roll)
- Setpoint Accuracy
- Sets takeoff coordinate as local origin

### Expected Behavior

- Takeoff @ altitude 2m with yaw = 0.0 (North)
- Go forwards 2m, backwards 2m (pitch)
- Go right 2m, left 2m (roll)
- Ascend @ altitude 3m
- Land

![flight_test_demo.gif](materials/quad_flight_test_demo.gif)

---

##gps_test

- LLA coordinate based flight
- !IMPORTANT!
-- Set waypoints to desired LLA coordinates before flight.

---

## Tech Stack

- ROS2 Humble
- PX4 Autopilot firmware v1.17.0
- C++
