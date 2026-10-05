# Electro-Pneumatic Pick-and-Place Station

**Mechatronics Lab (MCTR 704) · German University in Cairo · Winter 2024 · Team project (2 members)**

<p align="center">
  <img src="images/cad.png" height="280">
  <img src="images/station.jpg" height="280">
</p>

**Documentation:** [Project documentation (PDF)](1-%20Project%20Report/Report.PDF)

## Overview

An automated electro-pneumatic station (100 × 40 × 80 cm) that feeds parts from a magazine, lifts them and transfers them to an elevated platform using three pneumatic cylinders under relay control.

## Technical Approach

Cylinder A dispenses a part from the magazine, cylinder B raises it on a moving table while A retracts, and cylinder C transfers the part to its final position before B and C retract. The resulting motion sequence is **A+ (B+ A−) C+ (B− C−)**. Six reed switches provide cylinder end-position feedback, a sensor confirms part presence, and three 5/2 solenoid valves actuate the cylinders; the cycle is started by a push button. The station was designed in SolidWorks, simulated in Festo FluidSIM, and then built and commissioned.

<!--
## My Role

Replace this paragraph with one or two sentences about your personal contribution,
then delete the first and last lines of this block so the section becomes visible.
-->

## Repository Contents

| Folder | Contents |
|---|---|
| [1- Project Report](1-%20Project%20Report/) | Project documentation (PDF) |
| [2- FluidSim](2-%20FluidSim/) | FluidSIM circuit simulation files |
| [3- Solid Works](3-%20Solid%20Works/) | CAD model (zip) |
| [4- Pictures](4-%20Pictures/) | CAD rendering and control-panel photo |
| [5- Videos](5-%20Videos/) | Station in operation |

**Team:** Bassam Walid, Euginia Kamal Anwar<br>
**Tools:** Festo FluidSIM · SolidWorks · Electro-pneumatics · Relay control
