# 🚗 Autonomous Path-Tracking Vehicle

**Autonomous Systems (MCTR 1002) · German University in Cairo · Spring 2026 · Team 18 (6 students)**

<p align="center">
  <img src="images/car.jpg" height="300">
  <img src="images/city-track-results.jpg" height="300">
</p>

📄 [Read the report (PDF)](1-%20Project%20Report/Report.pdf)

## Summary

A small self-driving car that follows a path, changes lanes to avoid obstacles and drives around a city-style track on its own. The software was first built and tested in a Gazebo simulation with ROS 2, then moved to a real car controlled by a Raspberry Pi 5 and an Arduino.

## Results on the real car

| Track | Result |
|---|---|
| Straight track (10 m) | Stayed within **4.5 cm** of the center line |
| Two-lane track with obstacles | Changed lanes around 2 obstacles with a maximum error of **3.4 cm** |
| City track with curves | Completed a **full lap** and returned to the start |

## How it works

**Simulation (Gazebo + ROS 2).** The team built a Kalman filter that cleans up noisy position data, path planners for three different tracks, steering controllers (Pure Pursuit, plus a Stanley controller for the city track) and a speed controller.

**Real car.** The Raspberry Pi 5 runs ROS 2: it plans the path and calculates the steering angle with Pure Pursuit, looking further ahead the faster the car drives. The Arduino Uno reads the wheel encoder and the MPU6050 gyroscope, estimates where the car is, filters out sensor noise, and controls the drive motor and the steering servo. The two boards talk over a USB serial cable.

## My role

✏️ *[Replace this line with 1–2 sentences about what you personally did in this project.]*

## What's in this folder

| Folder | Contents |
|---|---|
| [1- Project Report](1-%20Project%20Report/) | Final report (PDF) |
| [2- Presentation PPT](2-%20Presentation%20PPT/) | Presentation and poster |
| [3- Source Code & Other Relevent Data](3-%20Source%20Code%20%26%20Other%20Relevent%20Data/) | `ROS2` = simulation code (planners, controllers, Kalman filter, Gazebo tracks) · `Pi` = code for the real car · `Arduino` = motor and sensor code · `Test Data` = result plots |
| [4- Videos](4-%20Videos/) | Test-drive videos (zip) |

**Team:** Youssef Mohamed, Bassam Walid, Styven Hany, Somaya Magdy, Yassin Hesham, Mostafa Shakweer<br>
**Tools:** ROS 2 · Python · Gazebo · Raspberry Pi 5 · Arduino (C++) · MPU6050 IMU · Kalman filter · Pure Pursuit · Stanley controller
