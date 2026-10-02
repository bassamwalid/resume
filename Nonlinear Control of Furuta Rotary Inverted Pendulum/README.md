# 🔄 Nonlinear Control of a Furuta Pendulum

**Course project · German University in Cairo · Winter 2025 · Team 29 (6 students)**

<p align="center"><img src="images/rig.jpg" height="340"></p>

📄 [Read the report (PDF)](1-%20Project%20Report/Advanced%20Team29.pdf)

## Summary

The Furuta pendulum is a classic control challenge: a motor turns a horizontal arm, and a free-swinging pendulum on the end of the arm has to be balanced upside down. We modeled the system mathematically and designed a nonlinear controller that balances it.

## Result

In simulation, the controller brings the pendulum upright in about **0.5 seconds** and keeps it there from different starting angles, while respecting the real motor's limits (maximum torque ±1 N·m and its top speed).

<p align="center"><img src="images/pendulum-response.jpg" width="70%"></p>

## How it works

**Model.** The equations of motion were derived with the Euler–Lagrange method (including friction) and built in MATLAB/Simulink.

**Controller.** A backstepping controller based on Lyapunov stability theory. A Stateflow chart switches between start-up, stabilizing and balanced modes. A MATLAB script found the controller gains automatically by testing combinations until the pendulum stayed within 0.1 rad of upright for 5 seconds.

**Hardware.** A test rig was built with a JGB37-520 DC motor, an MD10C motor driver, a 2000-pulse encoder, an ACS712 current sensor, an Arduino Uno and a Raspberry Pi 5.

## My role

✏️ *[Replace this line with 1–2 sentences about what you personally did in this project.]*

## What's in this folder

| Folder | Contents |
|---|---|
| [1- Project Report](1-%20Project%20Report/) | Report written as a research paper (PDF) |
| [2- Source Code](2-%20Source%20Code/) | Simulink model, controller and gain-tuning scripts |
| [3- Pictures](3-%20Pictures/) | Photo of the test rig |
| [4- Videos](4-%20Videos/) | Project video |

**Team:** Bassam Walid, Styven Hany, Youssef Mohamed Abodeb, Yassin Hesham, Mostafa Shakweer, Somaya Magdy<br>
**Tools:** MATLAB · Simulink · Stateflow · Arduino · Raspberry Pi 5
