# 🤖 4-DOF Pick-and-Place Robotic Arm

**Robotics course project · German University in Cairo · Winter 2025 · Team 07 (6 students)**

<p align="center">
  <img src="images/arm-1.jpg" width="32%">
  <img src="images/arm-2.jpg" width="32%">
</p>

## Summary

A robotic arm with four joints (4 degrees of freedom) that picks up objects and places them at target positions. We modeled how the arm moves in MATLAB, tested it in simulation and built the real arm.

## How it works

**Kinematics.** The arm was modeled with the Denavit–Hartenberg (DH) method. Forward and inverse kinematics give the joint angles needed to reach each pick and place point, and the Jacobian links the joint speeds to the speed of the gripper.

**Trajectories.** The gripper follows straight-line and spiral paths, and the final position error was checked.

**Simulation.** The SolidWorks model was imported into Simscape Multibody, and the arm was also simulated in Gazebo. The kinematics functions were converted to C++ with MATLAB Coder.

**Hardware.** The real arm uses MG996R servo motors controlled by an Arduino.

## My role

✏️ *[Replace this line with 1–2 sentences about what you personally did in this project.]*

## What's in this folder

| Folder | Contents |
|---|---|
| [1- Presentation PPT](1-%20Presentation%20PPT/) | Final presentation |
| [2- Source Code](2-%20Source%20Code/) | MATLAB code, Simscape model and CAD files (zip) |
| [3- Pictures](3-%20Pictures/) | Photos of the arm |
| [4- Videos](4-%20Videos/) | Demo videos |

**Team:** Bassam Walid, Mostafa Shakweer, Somaya Magdy, Styven Hany, Yassin Hesham, Youssef Mohamed<br>
**Tools:** MATLAB · Simscape Multibody · Gazebo · SolidWorks · Arduino
