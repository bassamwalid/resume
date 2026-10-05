# Vision-Based Lane Keeping for a Scale Vehicle

**Image Processing for Mechatronics (MCTR 1010) · German University in Cairo · Spring 2026 · Team project (6 members)**<br>
Supervisor: Assoc. Prof. Omar M. Shehata

| 1. Camera input | 2. Thresholding | 3. Morphological cleanup | 4. Spline fitting | 5. Final output |
|:-:|:-:|:-:|:-:|:-:|
| <img src="images/1-camera.jpg" width="160"> | <img src="images/2-threshold.jpg" width="160"> | <img src="images/3-cleanup.jpg" width="160"> | <img src="images/4-lane-fit.jpg" width="160"> | <img src="images/5-result.jpg" width="160"> |

**Documentation:** [Project report (PDF)](1-%20Project%20Report/M4_Team06.pdf)

## Overview

A real-time, camera-based lane-keeping system for a 3D-printed scale vehicle. The perception and control pipeline runs on a Raspberry Pi using ROS 2 and OpenCV, while an Arduino actuates the drive motor and the steering servo.

## Key Results

| Metric | Result |
|---|---|
| Processing rate | **20.8 frames per second** (38.6 ms per frame) on a Raspberry Pi |
| Operating modes | Dual-lane, single-lane and temporary lane-loss scenarios |

## Technical Approach

Each frame is cropped to a region of interest, smoothed with a Gaussian blur, binarized by inverse thresholding and cleaned with morphological opening. Lane pixels are fitted with cubic Hermite splines and projected from image coordinates to ground coordinates in meters using a four-point homography. The lane centerline defines a lookahead target that is stabilized by a Kalman-style filter and converted into a steering command by a Pure Pursuit controller. When only one lane boundary is visible, the centerline is reconstructed with a 0.15 m lateral offset.

<!--
## My Role

Replace this paragraph with one or two sentences about your personal contribution,
then delete the first and last lines of this block so the section becomes visible.
-->

## Repository Contents

| Folder | Contents |
|---|---|
| [1- Project Report](1-%20Project%20Report/) | Final report (PDF) |
| [2- Presentation PPT](2-%20Presentation%20PPT/) | Presentation and poster |
| [3- Source Code](3-%20Source%20Code/) | ROS 2 package (zip): camera node, lane-detection node and serial bridge to the Arduino |
| [4- Pictures](4-%20Pictures/) | Circuit diagram and processed camera frames (zip) |
| [5- Videos](5-%20Videos/) | Two test runs on the track |

**Team:** Youssef Mohamed Abodeb, Somaya Magdy, Bassam Walid, Styven Hany, Mostafa Shakweer, Hassan Yassar<br>
**Tools:** Python · OpenCV · ROS 2 · Raspberry Pi · Arduino
