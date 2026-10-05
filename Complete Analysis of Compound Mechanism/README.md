# Kinematic Analysis of a Compound Mechanism

**Machine Design (EDPT 903) · German University in Cairo · 2024 · Team project (2 members)**<br>
Supervisor: Prof. Imam Morgan

<p align="center">
  <img src="images/linkage.jpg" width="48%">
  <img src="images/matlab-plots.png" width="48%">
</p>

**Documentation:** [Project report (PDF)](1-%20Project%20Report/Project%20Report.pdf)

## Overview

Complete kinematic analysis of a two-loop compound mechanism (a four-bar linkage driving a sliding link), determining the position, velocity and acceleration of every link over a full revolution of the input crank.

## Technical Approach

Loop-closure equations for position, velocity and acceleration were derived analytically and solved numerically in MATLAB using `fsolve` at 1° increments over 0–360° of crank rotation, with the crank driven at 10 rad/s. The results were plotted and cross-checked against a model built in the Linkage mechanism simulator.

<!--
## My Role

Replace this paragraph with one or two sentences about your personal contribution,
then delete the first and last lines of this block so the section becomes visible.
-->

## Repository Contents

| Folder | Contents |
|---|---|
| [1- Project Report](1-%20Project%20Report/) | Analytical derivations, MATLAB code and plots (PDF) |
| [2- Source Code](2-%20Source%20Code/) | MATLAB scripts; `MD.m` is the main script |

**Team:** Bassam Walid Fawzy, Styven Hany Nabil<br>
**Tools:** MATLAB · Linkage
