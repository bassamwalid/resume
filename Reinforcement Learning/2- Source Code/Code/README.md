# RL Traffic Signal Control

Adaptive traffic signal control for a simulated four-way intersection.

Implemented controllers:

- Fixed-time baseline
- Tabular Q-learning
- Tabular SARSA
- Deep Q-Network (DQN) with a continuous-observation environment

The final experiment uses the frozen reward:

```text
R_final = R7 - 0.75 * switched
```

with mixed-scenario training and evaluation across five traffic scenarios.

## Setup

Install the regular Python dependencies:

```powershell
python -m pip install -r requirements.txt
```

For NVIDIA GPU DQN training, install the CUDA PyTorch build instead of the CPU build:

```powershell
python -m pip uninstall -y torch torchvision torchaudio
python -m pip install --upgrade torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128
```

Verify GPU availability:

```powershell
python -c "import torch; print(torch.__version__); print(torch.version.cuda); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU only')"
```

Expected GPU output on the final machine:

```text
2.11.0+cu128
12.8
True
NVIDIA GeForce RTX 4060 Laptop GPU
```

## Final Full Experiment

Run the complete final pipeline:

```powershell
python src/run_final_experiments.py --config configs/final_r11_mixed.yaml --output-dir results/final_mixed_r11_gpu
```

This trains:

- Q-learning for `training.episodes` from the YAML
- SARSA for `training.episodes` from the YAML
- DQN for `dqn.episodes` from the YAML

Current final settings:

```text
Q-learning: 12000 episodes
SARSA: 12000 episodes
DQN: 8000 episodes
Training seed: 42
Evaluation seeds: 10042, 20042, 30042
Evaluation episodes: 100 per seed per scenario
```

Optional overrides:

```powershell
python src/run_final_experiments.py --config configs/final_r11_mixed.yaml --output-dir results/final_mixed_r11_gpu --q-episodes 12000 --sarsa-episodes 12000 --dqn-episodes 8000
```

Quick smoke test:

```powershell
python src/run_final_experiments.py --config configs/final_r11_mixed.yaml --output-dir results/final_smoke --q-episodes 2 --sarsa-episodes 2 --dqn-episodes 2 --scenarios low_uniform --eval-seeds 10042 --eval-episodes 2 --skip-policy-plots
```

Rerun final evaluation only using already trained models:

```powershell
python src/run_final_experiments.py --config configs/final_r11_mixed.yaml --output-dir results/final_mixed_r11_gpu --skip-training --skip-policy-plots
```

Regenerate policy-behavior plots only using already trained models:

```powershell
python src/run_final_experiments.py --config configs/final_r11_mixed.yaml --output-dir results/final_mixed_r11_gpu --skip-training --skip-evaluation
```

Regenerate the smoothed Q-learning/SARSA training plots only:

```powershell
python src/run_final_experiments.py --config configs/final_r11_mixed.yaml --output-dir results/final_mixed_r11_gpu --skip-training --skip-evaluation --skip-policy-plots
```

## Individual Training

Train one agent manually:

```powershell
python src/train.py --agent q_learning --config configs/final_r11_mixed.yaml --scenario mixed_scenarios --output-dir results/q_learning_final
python src/train.py --agent sarsa --config configs/final_r11_mixed.yaml --scenario mixed_scenarios --output-dir results/sarsa_final
python src/train.py --agent dqn --config configs/final_r11_mixed.yaml --scenario mixed_scenarios --output-dir results/dqn_final
```

Override episode counts manually:

```powershell
python src/train.py --agent q_learning --config configs/final_r11_mixed.yaml --scenario mixed_scenarios --episodes 12000 --output-dir results/q_learning_final
python src/train.py --agent sarsa --config configs/final_r11_mixed.yaml --scenario mixed_scenarios --episodes 12000 --output-dir results/sarsa_final
python src/train.py --agent dqn --config configs/final_r11_mixed.yaml --scenario mixed_scenarios --episodes 8000 --output-dir results/dqn_final
```

## Individual Evaluation

Evaluate a trained controller on one scenario:

```powershell
python src/evaluate.py --agent fixed_time --config configs/final_r11_mixed.yaml --scenario uniform_medium --seed 10042 --episodes 100 --output-dir results/final_mixed_r11_gpu
python src/evaluate.py --agent q_learning --config configs/final_r11_mixed.yaml --scenario uniform_medium --seed 10042 --episodes 100 --output-dir results/final_mixed_r11_gpu
python src/evaluate.py --agent sarsa --config configs/final_r11_mixed.yaml --scenario uniform_medium --seed 10042 --episodes 100 --output-dir results/final_mixed_r11_gpu
python src/evaluate.py --agent dqn --config configs/final_r11_mixed.yaml --scenario uniform_medium --seed 10042 --episodes 100 --output-dir results/final_mixed_r11_gpu
```

Available scenarios:

```text
low_uniform
uniform_medium
high_uniform
uneven
rush_hour
mixed_scenarios
```

## Compare Controllers

Compare controllers on one scenario:

```powershell
python src/compare.py --config configs/final_r11_mixed.yaml --input-dir results/final_mixed_r11_gpu --output-dir results/final_mixed_r11_gpu --scenario uniform_medium --episodes 100 --seed 10042
```

## Reward Experiment

The completed reward-selection outputs are in:

```text
results/reward_experiment/
```

To rerun the reward sweep:

```powershell
python src/reward_experiment.py --config configs/default.yaml --output-dir results/reward_experiment_new --episodes 3000 --eval-episodes 100 --eval-seed-offsets 10000,20000,30000
```

The reward experiment was designed around the `uniform_medium` demand setting. If rerunning it for exact comparability, keep the traffic rates at `0.30` for all directions.

## Visualization

Generate improved traffic animations for all trained controllers:

```powershell
python src/visualize.py --all-agents --config configs/final_r11_mixed.yaml --output-dir results/final_mixed_r11_gpu --scenario uniform_medium --max-steps 160 --fps 6
```

Generate one controller only:

```powershell
python src/visualize.py --agent dqn --config configs/final_r11_mixed.yaml --output-dir results/final_mixed_r11_gpu --scenario rush_hour --max-steps 160 --fps 6
```

Improved animation outputs:

```text
results/final_mixed_r11_gpu/animations/fixed_time_better_animation.gif
results/final_mixed_r11_gpu/animations/q_learning_better_animation.gif
results/final_mixed_r11_gpu/animations/sarsa_better_animation.gif
results/final_mixed_r11_gpu/animations/dqn_better_animation.gif
```

## Important Outputs

Final result tables:

```text
results/final_mixed_r11_gpu/logs/final_scenario_summary.csv
results/final_mixed_r11_gpu/logs/final_overall_controller_summary.csv
results/final_mixed_r11_gpu/logs/final_verdict_q_learning_vs_sarsa.csv
results/final_mixed_r11_gpu/logs/final_evaluation_seed_summary.csv
```

Important final figures:

```text
results/final_mixed_r11_gpu/figures/final_score_by_scenario.png
results/final_mixed_r11_gpu/figures/final_overall_score_comparison.png
results/final_mixed_r11_gpu/figures/final_average_queue_by_scenario.png
results/final_mixed_r11_gpu/figures/final_average_wait_by_scenario.png
results/final_mixed_r11_gpu/figures/final_throughput_by_scenario.png
results/final_mixed_r11_gpu/figures/final_phase_switches_by_scenario.png
results/final_mixed_r11_gpu/figures/final_overflow_by_scenario.png
```

Important training figures:

```text
results/final_mixed_r11_gpu/figures/q_sarsa_training_reward.png
results/final_mixed_r11_gpu/figures/q_sarsa_training_queue.png
results/final_mixed_r11_gpu/figures/q_sarsa_training_wait.png
results/final_mixed_r11_gpu/figures/q_sarsa_training_throughput.png
results/final_mixed_r11_gpu/figures/q_sarsa_training_switches.png
results/final_mixed_r11_gpu/figures/dqn_training_curves.png
```

Final written report:

```text
final_report.md
```
