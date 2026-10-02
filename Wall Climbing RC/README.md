# 🧗 Wall-Climbing RC Car

**Mechatronics Engineering (MCTR 601) · German University in Cairo · Spring 2024 · Team of 3**

<p align="center">
  <img src="images/prototype.jpg" height="260">
  <img src="images/cad.jpg" height="260">
</p>

📄 [Read the report (PDF)](1-%20Project%20Report/Final%20Report.pdf)

## Summary

A remote-controlled car that can drive up walls. A propeller pushes the car against the wall so the wheels can grip, and a gyroscope-based controller tilts the propeller to keep the car stable. It is driven from a phone over Bluetooth.

## How it works

**Mechanical design.** The first prototype was made of balsa wood — it could hold itself on a wall, but the axles bent under the propeller's force. The final chassis was designed in SolidWorks and 3D-printed in PLA, with supports for the axles, the heavy parts placed low, and 3D-printed wheels with bicycle-tire rubber for more grip. The propeller's thrust was measured on a scale to make sure it was strong enough.

**Electronics.** An STM32 "Blue Pill" microcontroller, an A2212 brushless motor with a 12-inch propeller and a 30 A speed controller (3S LiPo battery), an MPU6050 gyroscope, servo motors, an L298N driver for the four wheel motors, and an HC-05 Bluetooth module.

**Software.** FreeRTOS runs two tasks at the same time: one reads the gyroscope and adjusts the propeller angle through a servo, and the other receives commands from the phone. The servo controller was tuned in MATLAB/Simulink.

<p align="center"><img src="images/wiring.jpg" width="70%"></p>

## My role

✏️ *[Replace this line with 1–2 sentences about what you personally did in this project.]*

## What's in this folder

| Folder | Contents |
|---|---|
| [1- Project Report](1-%20Project%20Report/) | Final report (PDF) |
| [2- Source Code](2-%20Source%20Code/) | STM32 code (Arduino IDE): the final version plus test programs for the motor, gyroscope and Bluetooth |
| [3- Solid Works](3-%20Solid%20Works/) | 3D models (zip) |
| [4- Pictures](4-%20Pictures/) | Wiring diagram |
| [5- Videos](5-%20Videos/) | Demo videos |

**Team:** Bassam Walid, Styven Hany, Youssef Mohamed<br>
**Tools:** STM32 · FreeRTOS · C/C++ · SolidWorks · 3D printing · MATLAB/Simulink
