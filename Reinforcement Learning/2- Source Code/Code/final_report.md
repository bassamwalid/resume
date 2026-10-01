# Final Report: RL Traffic Signal Control

This report summarizes the completed reinforcement-learning traffic signal project. It is intended to stand alone after the earlier planning notes are removed.

## 1. Project Goal

The project studies adaptive traffic signal control for a single four-way intersection. The objective is to reduce congestion and waiting time while maintaining high throughput and avoiding unrealistic phase switching.

Controllers compared:

| Controller | Type | Environment |
|---|---|---|
| Fixed-time | Baseline | Rule-based fixed green duration |
| Q-learning | Tabular RL | Discrete state environment |
| SARSA | Tabular RL | Discrete state environment |
| DQN | Deep RL | Continuous-observation environment |

The final completed run is stored in:

```text
results/final_mixed_r11_gpu/
```

Important implementation files:

| Path | Purpose |
|---|---|
| `configs/final_r11_mixed.yaml` | Final experiment configuration and hyperparameters |
| `src/env/traffic_env.py` | Queue-based traffic simulation and reward logic |
| `src/env/state_encoder.py` | Discrete state encoder for tabular methods |
| `src/env/continuous_traffic_env.py` | Continuous observation wrapper for DQN |
| `src/agents/q_learning.py` | Q-learning agent |
| `src/agents/sarsa.py` | SARSA agent |
| `src/extensions/dqn_agent.py` | PyTorch DQN agent |
| `src/run_final_experiments.py` | Final automation script |
| `src/visualize.py` | Improved animation and trace visualizer |

## 2. Environment Formulation

The environment models one four-way intersection with four possible green phases:

| Action | Served Direction |
|---:|---|
| 0 | North |
| 1 | South |
| 2 | East |
| 3 | West |

At every timestep:

1. Vehicles arrive according to a Poisson process.
2. The chosen phase serves up to `service_rate = 2` vehicles.
3. If the action changes phase, a yellow-light transition of `yellow_duration = 2` timesteps is applied.
4. The episode ends at `max_timesteps = 500` or when any queue reaches `max_queue = 40`.

Core environment parameters from `configs/final_r11_mixed.yaml`:

| Setting | Value |
|---|---:|
| Max timesteps | 500 |
| Max queue per direction | 40 |
| Yellow duration | 2 |
| Service rate | 2 vehicles/timestep |
| Fixed-time green duration | 20 |

The environment state contains four queues, one for each direction:

```text
queues = [north_queue, south_queue, east_queue, west_queue]
```

At each timestep, the environment calculates:

| Quantity | Meaning |
|---|---|
| `arrivals` | New vehicles sampled from the scenario arrival rates |
| `departed` / `throughput` | Vehicles served by the current green phase |
| `total_queue` | Sum of all four queue lengths |
| `queue_delta` | Change in total queue from the previous timestep |
| `waiting_time` | Current total queue used as timestep waiting-time cost |
| `max_red_queue` | Largest queue among directions not currently green |
| `switched` | 1 when a phase change is initiated, otherwise 0 |
| `overflow` | True if any queue reaches `max_queue` |

The episode termination condition is:

```text
done = timestep >= max_timesteps or max(queue_lengths) >= max_queue
```

### MDP Formulation

The traffic-control problem is formulated as a Markov Decision Process:

| Component | Definition in this project |
|---|---|
| State | Encoded traffic queues, signal phase, timing, and for DQN, continuous traffic features |
| Action | Select one of four green directions: North, South, East, or West |
| Transition | Queue update from Poisson arrivals, service, yellow transitions, and selected phase |
| Reward | Final normalized composite reward with switch penalty |
| Episode | Up to 500 timesteps or until queue overflow |
| Policy | Mapping from observed traffic state to selected signal phase |

### Tabular State

Q-learning and SARSA use a discrete encoded state:

```text
(north_queue_bin, south_queue_bin, east_queue_bin, west_queue_bin,
 current_phase, phase_timer_bin)
```

Queue bins:

| Queue Length | Bin |
|---:|---:|
| 0 to 3 | 0 |
| 4 to 7 | 1 |
| 8 or more | 2 |

Phase timer bins:

| Phase Timer | Bin |
|---:|---:|
| 0 to 5 | 0 |
| 6 to 15 | 1 |
| 16 or more | 2 |

The tabular state is intentionally compact. It sacrifices exact queue values, but it keeps Q-learning and SARSA interpretable and feasible with a dictionary-based Q-table.

### Tabular Action Selection and Updates

Both tabular agents use epsilon-greedy exploration during training. During evaluation, epsilon is set to `0.0`, so the agents act greedily from the learned Q-table.

Q-learning is off-policy and updates toward the best next action:

```text
Q(s,a) <- Q(s,a) + lr * [r + gamma * max_a Q(s',a) - Q(s,a)]
```

SARSA is on-policy and updates toward the action actually selected next:

```text
Q(s,a) <- Q(s,a) + lr * [r + gamma * Q(s',a') - Q(s,a)]
```

### DQN State

DQN uses a 19-dimensional continuous observation:

```text
4 normalized queue lengths
4 one-hot current phase values
4 one-hot pending phase values
1 normalized phase timer
1 normalized yellow timer
1 normalized episode progress
4 normalized arrival-rate values
```

This lets DQN see a richer continuous representation while Q-learning and SARSA remain simple and explainable.

The continuous environment keeps the same dynamics, actions, rewards, arrivals, yellow transitions, and termination rules as the tabular environment. Only the observation representation changes. This makes the DQN comparison fair because DQN does not use a different simulator; it only receives a different state encoding.

### DQN Model Formulation

DQN approximates action values with a neural network:

```text
input 19 -> hidden 128 -> hidden 128 -> output 4
```

The four outputs correspond to the estimated Q-values for the four traffic phases. The agent uses:

| Component | Role |
|---|---|
| Replay buffer | Stores previous transitions and samples mini-batches |
| Target network | Stabilizes Q-learning targets |
| Adam optimizer | Updates the neural-network parameters |
| Smooth L1 loss | Trains predicted Q-values toward target Q-values |
| Gradient clipping | Prevents unstable large updates |

## 3. Traffic Scenarios

Final training uses `mixed_scenarios`, meaning each training episode randomly selects one scenario using the fixed base seed.

| Scenario | Arrival Pattern |
|---|---|
| `low_uniform` | N=0.10, S=0.10, E=0.10, W=0.10 |
| `uniform_medium` | N=0.30, S=0.30, E=0.30, W=0.30 |
| `high_uniform` | N=0.55, S=0.55, E=0.55, W=0.55 |
| `uneven` | N=0.50, S=0.50, E=0.20, W=0.20 |
| `rush_hour` | First third favors N/S, middle is balanced, final third favors E/W |

Rush-hour pattern:

| Episode Segment | North | South | East | West |
|---|---:|---:|---:|---:|
| First third | 0.55 | 0.55 | 0.20 | 0.20 |
| Middle third | 0.30 | 0.30 | 0.30 | 0.30 |
| Final third | 0.20 | 0.20 | 0.55 | 0.55 |

During mixed-scenario training, the selected scenario changes episode by episode. The episode seed is:

```text
episode_seed = training_base_seed + episode_index
```

This keeps the training run reproducible while still exposing each agent to different traffic patterns.

## 4. Reward Experiment

Before final training, reward functions were compared in two stages.

### Score Formula

All reward functions and final controllers were scored relative to fixed-time:

```text
queue_improvement      = (fixed_avg_queue - controller_avg_queue) / fixed_avg_queue
wait_improvement       = (fixed_avg_wait - controller_avg_wait) / fixed_avg_wait
throughput_improvement = (controller_throughput - fixed_throughput) / fixed_throughput
switch_penalty_score   = (controller_switches - fixed_switches) / fixed_switches

raw_score =
    0.40 * queue_improvement
  + 0.40 * wait_improvement
  + 0.20 * throughput_improvement
  - 0.10 * switch_penalty_score

score_percent = raw_score * 100
```

Negative scores were not capped. A negative value means the controller or reward performed worse than fixed-time under the combined score.

### Normalized Reward Weights

The composite reward weights were computed from component magnitudes, not from performance scores.

The target normalized reward was:

```text
R7 = -w1 * queue_delta
     -w2 * total_waiting_time
     +w3 * throughput
```

For component scales:

```text
queue_scale      = mean(abs(queue_delta))
wait_scale       = mean(abs(total_waiting_time))
throughput_scale = mean(abs(throughput))
```

Weights were calculated using inverse scales:

```text
inv_q = 1 / (queue_scale + eps)
inv_w = 1 / (wait_scale + eps)
inv_t = 1 / (throughput_scale + eps)

w1 = inv_q / (inv_q + inv_w + inv_t)
w2 = inv_w / (inv_q + inv_w + inv_t)
w3 = inv_t / (inv_q + inv_w + inv_t)
```

The reward experiment log reported approximately:

| Component | Scale | Final Weight |
|---|---:|---:|
| queue_delta | 1.0950 | 0.4850 |
| total_waiting_time | 31.5117 | 0.0169 |
| throughput | 1.0661 | 0.4981 |

The final frozen config used:

```text
w1 = 0.48428457611594394
w2 = 0.01686242633086334
w3 = 0.49885299755319273
w4 = 0.75
```

### Stage A Results

Stage A compared main reward designs R1 to R8.

| Reward | Meaning | Score Percent |
|---|---|---:|
| R1 | Queue delta only | 16.97 |
| R2 | Waiting time only | -23.15 |
| R3 | Throughput only | -22.93 |
| R4 | Queue delta + waiting time | 18.34 |
| R5 | Queue delta + throughput | 13.77 |
| R6 | Unnormalized composite | -15.65 |
| R7 | Normalized composite | 20.38 |
| R8 | Queue-pressure/fairness reward | 6.34 |

Stage A winner: `R7`, the normalized composite reward.

### Stage B Results

Stage B added a switch penalty to the Stage A winner.

| Reward | Switch Penalty | Score Percent |
|---|---:|---:|
| B0 | 0.00 | 20.38 |
| R9 | 0.10 | 20.62 |
| R10 | 0.25 | 21.85 |
| R11 | 0.50 | 22.70 |
| R12 | 1.00 | 23.06 |
| R13 | 1.25 | 21.20 |
| R14 | 2.00 | 14.65 |

The strongest score in the sweep was around `w4 = 1.00`, while `w4 = 0.50` was close. The final reward was set to the midpoint:

```text
R_final = R7 - 0.75 * switched
```

Expanded:

```text
R_final =
    -0.48428457611594394 * queue_delta
    -0.01686242633086334 * total_waiting_time
    +0.49885299755319273 * throughput
    -0.75 * switched
```

## 5. Final Training Setup

Final training used the frozen reward above and mixed-scenario training.

### Shared Training Settings

| Setting | Value |
|---|---:|
| Training scenario mode | `mixed_scenarios` |
| Training scenarios | `low_uniform`, `uniform_medium`, `high_uniform`, `uneven`, `rush_hour` |
| Training base seed | 42 |
| Q-learning episodes | 12000 |
| SARSA episodes | 12000 |
| DQN episodes | 8000 |

### Tabular Training Parameters

Q-learning and SARSA used the shared `training` block from `configs/final_r11_mixed.yaml`:

| Parameter | Value |
|---|---:|
| Episodes | 12000 |
| Learning rate | 0.1 |
| Discount factor | 0.95 |
| Epsilon start | 1.0 |
| Epsilon decay | 0.999 |
| Epsilon min | 0.01 |
| Seed | 42 |

### DQN Training Parameters

DQN used the separate `dqn` block from `configs/final_r11_mixed.yaml`:

| Parameter | Value |
|---|---:|
| Episodes | 8000 |
| Learning rate | 0.0005 |
| Discount factor | 0.97 |
| Epsilon start | 1.0 |
| Epsilon decay | 0.9995 |
| Epsilon min | 0.05 |
| Replay capacity | 100000 |
| Batch size | 128 |
| Hidden size | 128 |
| Target update steps | 500 |
| Warmup steps | 2000 |
| Gradient clip | 5.0 |

### Fixed-Time Baseline

Fixed-time does not learn. It cycles through North, South, East, and West with:

```text
green_duration = 20 timesteps
```

### Hardware and CUDA

DQN was run with CUDA:

```text
PyTorch: 2.11.0+cu128
CUDA build: 12.8
GPU available: True
GPU: NVIDIA GeForce RTX 4060 Laptop GPU
```

Q-learning and SARSA are tabular NumPy/Python methods, so they run on CPU. DQN uses GPU for neural-network updates when CUDA is available, while the traffic environment simulation still runs in Python on CPU.

## 6. Final Evaluation Setup

Controllers evaluated:

```text
fixed_time
q_learning
sarsa
dqn
```

Scenarios evaluated:

```text
low_uniform
uniform_medium
high_uniform
uneven
rush_hour
```

Evaluation seeds:

```text
10042, 20042, 30042
```

For each scenario and seed, 100 evaluation episodes were run:

```text
5 scenarios * 3 seeds * 100 episodes = 1500 evaluation episodes per controller
```

Evaluation was greedy for learned controllers:

```text
evaluation epsilon = 0.0
```

The three evaluation base seeds produced independent test sets:

```text
Eval set 1: 10042, 10043, ..., 10141
Eval set 2: 20042, 20043, ..., 20141
Eval set 3: 30042, 30043, ..., 30141
```

For the final comparison, each controller was therefore evaluated on the same scenario/seed structure, which makes the comparison fair and reproducible.

## 7. Final Results

Source files:

```text
results/final_mixed_r11_gpu/logs/final_overall_controller_summary.csv
results/final_mixed_r11_gpu/logs/final_scenario_summary.csv
results/final_mixed_r11_gpu/logs/final_verdict_q_learning_vs_sarsa.csv
```

### Overall Controller Summary

| Controller | Overall Score | Score Std | Avg Queue | Avg Wait | Throughput | Switches | Overflow Episodes |
|---|---:|---:|---:|---:|---:|---:|---:|
| Fixed-time | 0.00 | 0.00 | 35.07 | 36.01 | 246.96 | 12.47 | 893 |
| Q-learning | 18.19 | 9.49 | 24.53 | 25.67 | 458.76 | 38.24 | 307 |
| SARSA | 15.94 | 8.87 | 23.27 | 24.42 | 459.63 | 44.10 | 303 |
| DQN | 19.39 | 12.45 | 17.91 | 19.08 | 465.01 | 59.59 | 300 |

### Scenario Score Summary

| Controller | Low | Medium | High | Uneven | Rush Hour | Overall |
|---|---:|---:|---:|---:|---:|---:|
| Fixed-time | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| Q-learning | 15.47 | 23.68 | 2.71 | 25.67 | 23.41 | 18.19 |
| SARSA | 15.08 | 23.67 | 1.26 | 21.89 | 17.79 | 15.94 |
| DQN | 37.26 | 24.81 | 5.23 | 18.61 | 11.04 | 19.39 |

### Final Tabular Verdict

Q-learning is the better tabular controller in the final GPU run.

| Controller | Overall Mean Score | Score Std | Avg Wait | Avg Queue | Switches | Comment |
|---|---:|---:|---:|---:|---:|---|
| Q-learning | 18.19 | 9.49 | 25.67 | 24.53 | 38.24 | Winner by higher overall score |
| SARSA | 15.94 | 8.87 | 24.42 | 23.27 | 44.10 | Lower queue/wait, but lower score |

SARSA had lower average queue and waiting time, but Q-learning had the higher scenario-averaged score and fewer phase switches.

### Best Overall Controller

DQN achieved the best overall score:

```text
DQN overall score = 19.39
Q-learning score  = 18.19
SARSA score       = 15.94
Fixed-time score  = 0.00
```

DQN also had the lowest overall average queue and waiting time, but it switched phases the most and had the largest score standard deviation. This means DQN was strongest on average, but less stable across scenarios than the tabular methods.

## 8. Interpretation

Main findings:

1. All learned controllers improved strongly over fixed-time on the combined score.
2. DQN was best overall, especially for low traffic and medium traffic.
3. Q-learning was the best tabular method in the final run.
4. SARSA produced lower average queue and waiting time than Q-learning overall, but its score was reduced by lower scenario scores and more switching.
5. High-uniform traffic remained difficult for all controllers. Every controller had many overflow episodes under high uniform demand.
6. Fixed-time was simple and stable, but it failed badly in high, uneven, and rush-hour demand.

## 9. Important Graphs

Use these graphs in the final report or presentation.

### Core Final Comparison Graphs

```text
results/final_mixed_r11_gpu/figures/final_score_by_scenario.png
results/final_mixed_r11_gpu/figures/final_overall_score_comparison.png
results/final_mixed_r11_gpu/figures/final_average_queue_by_scenario.png
results/final_mixed_r11_gpu/figures/final_average_wait_by_scenario.png
results/final_mixed_r11_gpu/figures/final_throughput_by_scenario.png
results/final_mixed_r11_gpu/figures/final_phase_switches_by_scenario.png
results/final_mixed_r11_gpu/figures/final_overflow_by_scenario.png
```

Most important two:

```text
final_score_by_scenario.png
final_overall_score_comparison.png
```

In `final_overall_score_comparison.png`, the black error bars are the standard deviation of each controller's scenario scores. A long error bar means the controller is less consistent across scenarios.

### Training Graphs

Training plots use rolling averages. The raw CSV data is unchanged.

```text
Q-learning and SARSA rolling window: 500 episodes
DQN rolling window: 200 episodes
```

Important training plots:

```text
results/final_mixed_r11_gpu/figures/q_sarsa_training_reward.png
results/final_mixed_r11_gpu/figures/q_sarsa_training_queue.png
results/final_mixed_r11_gpu/figures/q_sarsa_training_wait.png
results/final_mixed_r11_gpu/figures/q_sarsa_training_throughput.png
results/final_mixed_r11_gpu/figures/q_sarsa_training_switches.png
results/final_mixed_r11_gpu/figures/dqn_training_curves.png
```

### Policy-Behavior Graphs

There are many behavior plots, so only include a small subset in the main report. Recommended scenarios:

```text
uniform_medium
rush_hour
```

Recommended behavior plots for each selected scenario:

```text
*_total_queue_by_controller.png
*_phase_by_controller.png
*_cumulative_throughput.png
*_phase_duration_histogram.png
```

Individual queue-length plots are useful for appendix material, but they are too detailed for the main report.

Improved animations can also be generated for presentation:

```text
results/final_mixed_r11_gpu/animations/fixed_time_better_animation.gif
results/final_mixed_r11_gpu/animations/q_learning_better_animation.gif
results/final_mixed_r11_gpu/animations/sarsa_better_animation.gif
results/final_mixed_r11_gpu/animations/dqn_better_animation.gif
```

These animations draw road lanes, vehicle queues, traffic lights, active/yellow phases, queue bars, and metric overlays.

### Improved Visualization Description

The improved visualizer in `src/visualize.py` keeps the same environment and learned policies, but renders the rollout more clearly for presentation. It adds:

| Visual Element | Purpose |
|---|---|
| Road rectangles and lane markings | Shows the four-way intersection layout |
| Vehicle blocks | Shows queued vehicles in each incoming direction |
| Traffic-light circles | Shows red, green, and yellow phases |
| Active stop-line highlight | Makes the currently served direction obvious |
| Queue bars | Gives a compact quantitative view of congestion |
| Metrics overlay | Shows controller, scenario, timestep, phase, total queue, waiting time, reward, throughput, and switches |
| Better GIF filenames | Separates final presentation animations from older simple animations |

Generated improved animation files:

| Controller | Improved GIF | Snapshot |
|---|---|---|
| Fixed-time | `results/final_mixed_r11_gpu/animations/fixed_time_better_animation.gif` | `results/final_mixed_r11_gpu/figures/fixed_time_better_snapshot.png` |
| Q-learning | `results/final_mixed_r11_gpu/animations/q_learning_better_animation.gif` | `results/final_mixed_r11_gpu/figures/q_learning_better_snapshot.png` |
| SARSA | `results/final_mixed_r11_gpu/animations/sarsa_better_animation.gif` | `results/final_mixed_r11_gpu/figures/sarsa_better_snapshot.png` |
| DQN | `results/final_mixed_r11_gpu/animations/dqn_better_animation.gif` | `results/final_mixed_r11_gpu/figures/dqn_better_snapshot.png` |

The visualizer attempts to save MP4 as well, but in the current environment the required video writer was unavailable, so GIF was used as the final presentation format.

Command used to generate the improved animations:

```text
python src/visualize.py --all-agents --config configs/final_r11_mixed.yaml --output-dir results/final_mixed_r11_gpu --scenario uniform_medium --max-steps 160 --fps 6
```

### Reward Experiment Graphs

Use these for the reward-selection section:

```text
results/reward_experiment/figures/reward_stage_a_score_percent_comparison.png
results/reward_experiment/figures/reward_stage_b_score_percent_comparison.png
results/reward_experiment/figures/reward_stage_a_metrics.png
results/reward_experiment/figures/reward_stage_b_switch_penalty_metrics.png
```

## 10. Conclusion

The final project successfully compares fixed-time control, tabular RL, and deep RL for traffic signal control under multiple traffic scenarios.

The reward experiment showed that a normalized composite reward was better than using queue, wait, or throughput alone. Adding a moderate switch penalty improved behavior by discouraging unnecessary phase changes. The final reward was therefore frozen as:

```text
R_final = R7 - 0.75 * switched
```

After final mixed-scenario training and multi-seed evaluation, DQN achieved the highest overall score and the lowest average queue/waiting time. However, it also switched phases more often and had the highest variability across scenarios. Among the tabular methods, Q-learning had the higher final score and is the final tabular winner, while SARSA had slightly smoother congestion metrics but lower score.

Final ranking by overall score:

| Rank | Controller | Overall Score |
|---:|---|---:|
| 1 | DQN | 19.39 |
| 2 | Q-learning | 18.19 |
| 3 | SARSA | 15.94 |
| 4 | Fixed-time | 0.00 |

The strongest final conclusion is that learned controllers clearly outperform fixed-time control, and DQN gives the best overall performance when enough training is used and GPU acceleration is available.

## 11. Limitations and Future Work

The project is complete for the current queue-based simulator, but several limitations remain:

1. The simulator is queue-based and simplified. It does not model detailed vehicle acceleration, lane changing, turning movements, or real road geometry.
2. The action space selects one served direction at a time. Real intersections may use protected turns, combined phases, pedestrian timing, and safety constraints.
3. DQN achieves the best overall score but switches more often than the tabular controllers, so realism and signal smoothness remain important tradeoffs.
4. High-uniform traffic remains difficult for every controller, with many overflow episodes.
5. The final results use one fixed set of training seeds. More training seeds would strengthen statistical confidence, but would require more compute time.

Possible future work:

| Extension | Purpose |
|---|---|
| Longest-queue-first baseline | Add a stronger non-RL adaptive baseline |
| Fairness metrics | Track longest red time and max directional queue |
| More random training seeds | Measure training stability |
| Larger DQN or Double DQN | Improve deep RL stability |
| PPO/SAC-style algorithms | Explore policy-gradient or continuous-control approaches |
| SUMO + TraCI | Replace the queue simulator with a microscopic traffic simulator |

SUMO would be the most realistic future extension. It could model lanes, vehicle routes, turning movements, and multi-intersection traffic networks. The RL loop would connect Python to SUMO through TraCI: read traffic state, choose a signal action, apply it to SUMO, advance the simulation, and compute reward from queue length, waiting time, throughput, or time loss.
