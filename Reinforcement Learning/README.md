# Adaptive Traffic-Signal Control with Reinforcement Learning

**Reinforcement Learning & Optimal Control (MCTR 1024) · German University in Cairo · Spring 2026 · Team project (5 members)**

<p align="center"><img src="images/dqn-demo.gif" width="320"></p>
<p align="center"><em>Trained DQN agent controlling the simulated intersection</em></p>

**Documentation:** [Project report (PDF)](1-%20Project%20Report/Team21_Final_Report.pdf)

## Overview

Fixed-time traffic signals cannot adapt to changing demand. This project formulates signal control at a four-way intersection as a Markov Decision Process and compares a fixed-time baseline with three reinforcement-learning controllers: tabular Q-learning, tabular SARSA and a Deep Q-Network (DQN).

## Key Results

Averaged over five traffic scenarios (low, medium and high uniform demand, uneven demand and rush hour), relative to the fixed-time controller:

| Controller | Average waiting time | Average queue length |
|---|---|---|
| Q-learning | −29% | −30% |
| SARSA | −32% | −34% |
| **DQN** | **−47%** | **−49%** |

DQN achieved the best overall performance. Among the tabular methods, Q-learning achieved the higher overall score because it switched phases less frequently than SARSA.

<p align="center"><img src="images/waiting-time.png" width="85%"></p>

## Technical Approach

**Simulation environment.** A Python queue-based simulator of a single four-way intersection with stochastic Poisson arrivals, yellow-light transitions and a finite capacity of 40 vehicles per approach.

**Reward design.** Several reward formulations were evaluated in a staged experiment; the selected normalized composite reward balances queue reduction, waiting time and throughput, with a penalty on phase switching.

**Deep Q-Network.** Implemented in PyTorch with experience replay and a target network, using a 19-dimensional continuous observation of the intersection state.

<!--
## My Role

Replace this paragraph with one or two sentences about your personal contribution,
then delete the first and last lines of this block so the section becomes visible.
-->

## Repository Contents

| Folder | Contents |
|---|---|
| [1- Project Report](1-%20Project%20Report/) | Final report (PDF) |
| [2- Source Code](2-%20Source%20Code/Code/) | Python implementation: environment, agents, training and evaluation ([setup and usage](2-%20Source%20Code/Code/README.md)) |
| [results](2-%20Source%20Code/Code/results/) | Trained models, logs, figures and animations |

**Team:** Styven Hany, Bassam Walid, Somaya Magdy, Ali Khaled, Youssef Mohamed<br>
**Tools:** Python · PyTorch · NumPy · pandas · Matplotlib
