# FPGA Traffic-Light Controller

**Digital System Design (ELCT 501) · German University in Cairo · Winter 2023 · Team project (11 members)**

<p align="center"><img src="images/scale-model.jpg" width="55%"></p>

**Documentation:** [Project report (PDF)](1-%20Project%20Report/DSD%20Report.pdf)

## Overview

A traffic-light controller for a four-way intersection, implemented in VHDL on a Basys 3 FPGA board and demonstrated on a scale model with 220 V lamps.

## Technical Approach

A seven-state finite state machine sequences the vehicle and pedestrian signals (60 s green, 3 s yellow, 66 s red). Infrared sensors detect red-light violations and trigger a buzzer alarm, a custom VHDL driver displays status messages on a 16×2 LCD, and relays switch the 220 V lamps on the model. The design was verified with a behavioral testbench in Vivado. Temperature readout from an LM35 sensor was handled by an Arduino, as the FPGA's analog input was not suitable for the sensor.

## My Role

Group representative for the 11-member team.

<!-- Optional: add one sentence about the part of the design you built. -->

## Repository Contents

| Folder | Contents |
|---|---|
| [1- Project Report](1-%20Project%20Report/) | Report with code and simulation results (PDF) |
| [2- Source Code](2-%20Source%20Code/) | `Working System Code`: final Vivado project · `Code for Testing the system`: version with shortened timings for simulation |
| [3- Pictures](3-%20Pictures/) | Scale model |
| [4- Videos](4-%20Videos/) | Video walkthrough |

**Team:** Mohammed Osama, Styven Hany, Bassam Walid, Kareem Hassan, Ahmed Amr, Yousef Nader Greiss, Somaya Magdy, Sara Hany, Yousef Mohammed, Ali Fathy, Mostafa Shakweer<br>
**Tools:** VHDL · Vivado · Basys 3 FPGA · Arduino
