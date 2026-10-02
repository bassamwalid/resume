# 🦾 sEMG-Controlled Tendon-Driven Prosthetic Hand

**Bachelor Thesis · Mechatronics Engineering · German University in Cairo · 2025**<br>
Official title: *Brain-Computer Interface for Prosthetic Device Using EMG* · Supervisor: Assoc. Prof. Dr. Eng. Amir Roushdy Ali

🏆 Selected for presentation at ✏️ *[conference name, city, year]*, Germany.

<p align="center">
  <img src="images/hand.jpg" width="24%">
  <img src="images/grip-bottle.jpg" width="24%">
  <img src="images/grip-phone.jpg" width="24%">
  <img src="images/grip-scissors.jpg" width="24%">
</p>

📄 [Read the full thesis (PDF)](1-%20Thesis/Thesis%202.0.pdf)

## Summary

A low-cost bionic hand for people with a below-elbow amputation. It reads the electrical signals of the forearm muscles (surface EMG, or sEMG) and turns them into finger movements. The goal was a useful prosthetic hand that people in Egypt can actually afford.

## Results

| Test | Result |
|---|---|
| Detecting a full grip or a wrist bend | **100 %** correct (10 users) |
| Detecting single-finger movements | **98.25 %** correct (7 mistakes in 400 movements) |
| Load it can carry | **5 kg** |
| Time to close a finger | **about 0.84 s** |
| Battery charging (USB-C) | **full in 27 minutes** |
| Total cost | **about 5,000 EGP** |

## How it works

**Mechanical design.** Each finger works like a real tendon: a strong braided fishing line pulls the finger closed, and an elastic band pulls it open again. Small rubber pads between the joints absorb shocks like cartilage. The hand went through 11 design versions in SolidWorks; the final version has 151 parts and is 3D-printed in PLA.

**Motors and electronics.** Four small N20 gear motors (modified to run twice as fast) move the fingers through TB6612FNG motor drivers, and an MG90S servo rotates the thumb. Everything is controlled by an STM32F401 microcontroller. A custom 2-cell LiPo battery with a protection board charges from any USB-C charger, and the hand automatically stops moving while it charges.

**Reading the muscle signals.** Electrodes on the forearm feed a muscle sensor that filters and amplifies the signal. When the hand is switched on, it records 5 seconds of the relaxed muscle, smooths the signal and calculates its own on/off threshold for that person. This lets it work for different users and muscles without any machine learning.

**Testing.** Ten people tested the hand. It held a mug, a phone, scissors, a pencil and a full 0.6 L water bottle, and lifted a 5 kg dumbbell. The *flexor digitorum superficialis* muscle gave the most reliable signal.

## Follow-up project: bond-graph model (2026)

For the course *Mechatronics Programming for Real-Time Systems (MCTR 1015)*, the whole hand — battery, motors, gearboxes, tendons and finger joints — was modeled as a bond graph in 20-sim. The model gives the system's state-space equations, simulates finger and thumb motion at different battery voltages, and was compared with the real hand's measured finger-closing time. Team: Bassam Walid, Youssef Mohamed. Files: [8- Bond Graph Analysis](8-%20Bond%20Graph%20Analysis/).

## What's in this folder

| Folder | Contents |
|---|---|
| [1- Thesis](1-%20Thesis/) | Full thesis (PDF) |
| [2- Presentation PPT](2-%20Presentation%20PPT/) | Thesis presentation |
| [3- Source Arduino Code](3-%20Source%20Arduino%20Code/) | Microcontroller code — the final version is in `V6_FinalVersion` |
| [4- Solid Works](4-%20Solid%20Works/) | Final 3D model (zip) |
| [5- Pictures](5-%20Pictures/) | Photos of the hand and a step-by-step user guide (charging, electrode placement) |
| [6- Videos](6-%20Videos/) | Grip tests, finger movement and EMG control demos |
| [7- Previous Design Iterations](7-%20Previous%20Design%20Iterations/) | Earlier 3D-model versions |
| [8- Bond Graph Analysis](8-%20Bond%20Graph%20Analysis/) | 20-sim models, report and slides |

**Tools:** SolidWorks · 3D printing · STM32 · C/C++ (Arduino IDE) · EMG sensors · 20-sim
