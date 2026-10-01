"""Run reward-function comparison experiments for Q-learning and SARSA."""

from __future__ import annotations

import argparse
from copy import deepcopy
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

from agents.fixed_time import FixedTimeAgent
from agents.q_learning import QLearningAgent
from agents.sarsa import SarsaAgent
from env.traffic_env import TrafficEnv
from utils.metrics import aggregate_episode_metrics, mean, summarize_episode
from utils.saving import ensure_output_dirs, load_config, save_csv, save_json

try:
    from tqdm import trange
except ImportError:  # pragma: no cover
    def trange(*args, **kwargs):
        return range(*args)


AGENTS = ("q_learning", "sarsa")
STAGE_A_REWARDS = ("R1", "R2", "R3", "R4", "R5", "R6", "R7", "R8")
SINGLE_OBJECTIVE_REWARDS = ("R1", "R2", "R3")
SWITCH_REWARDS = ("B0", "R9", "R10", "R11", "R12", "R13", "R14")
SWITCH_PENALTIES = (0.0, 0.1, 0.25, 0.5, 1.0, 1.25, 2.0)
DEFAULT_EVAL_SEED_OFFSETS = (10000, 20000, 30000)
EPSILON = 1e-8
REWARD_ORDER = {
    "B0": 0,
    **{f"R{index}": index for index in range(1, 15)},
}

REWARD_FORMULAS = {
    "R1": "-queue_delta",
    "R2": "-total_waiting_time",
    "R3": "throughput",
    "R4": "-queue_delta - w2 * total_waiting_time",
    "R5": "-queue_delta + w3 * throughput",
    "R6": "-queue_delta - total_waiting_time + throughput",
    "R7": "-w1 * queue_delta - w2 * total_waiting_time + w3 * throughput",
    "R8": "-0.5 * total_queue + 2.0 * throughput - max_red_queue - switched",
    "B0": "winner_reward - 0.00 * switched",
    "R9": "winner_reward - 0.10 * switched",
    "R10": "winner_reward - 0.25 * switched",
    "R11": "winner_reward - 0.50 * switched",
    "R12": "winner_reward - 1.00 * switched",
    "R13": "winner_reward - 1.25 * switched",
    "R14": "winner_reward - 2.00 * switched",
}

REWARD_COMMENTS = {
    "R1": "Queue-growth control only.",
    "R2": "Waiting-time minimization only.",
    "R3": "Throughput maximization only.",
    "R4": "Queue control with computed waiting-time weight.",
    "R5": "Queue control with computed throughput weight.",
    "R6": "Unweighted full composite reward.",
    "R7": "Normalized full composite reward.",
    "R8": "Queue-pressure reward with fairness and switching terms.",
    "B0": "Stage A winner with no added switch penalty.",
    "R9": "Winner reward with very small switch penalty.",
    "R10": "Winner reward with small switch penalty.",
    "R11": "Winner reward with medium switch penalty.",
    "R12": "Winner reward with strong switch penalty.",
    "R13": "Winner reward with extra strong switch penalty.",
    "R14": "Winner reward with very strong switch penalty.",
}


def build_agent(agent_name, config, seed=None):
    training = config.get("training", {})
    kwargs = {
        "n_actions": TrafficEnv.n_actions,
        "learning_rate": training.get("learning_rate", 0.1),
        "discount_factor": training.get("discount_factor", 0.95),
        "epsilon_start": training.get("epsilon_start", 1.0),
        "epsilon_decay": training.get("epsilon_decay", 0.995),
        "epsilon_min": training.get("epsilon_min", 0.01),
        "seed": seed,
    }
    if agent_name == "q_learning":
        return QLearningAgent(**kwargs)
    if agent_name == "sarsa":
        return SarsaAgent(**kwargs)
    raise ValueError(f"Unsupported reward experiment agent: {agent_name}")


def train_agent(agent_name, config, episodes, seed, quiet=False):
    env = TrafficEnv.from_config(config, seed=seed)
    agent = build_agent(agent_name, config, seed=seed)
    rows = []

    iterator = trange(episodes, desc=f"{agent_name} {config['reward']['type']}", disable=quiet)
    for episode_index in iterator:
        state = env.reset(seed=seed + episode_index)
        done = False
        cumulative_reward = 0.0
        step_infos = []

        if agent_name == "sarsa":
            action = agent.select_action(state)

        while not done:
            if agent_name == "q_learning":
                action = agent.select_action(state)
                next_state, reward, done, info = env.step(action)
                agent.update(state, action, reward, next_state, done)
            else:
                next_state, reward, done, info = env.step(action)
                next_action = agent.select_action(next_state) if not done else 0
                agent.update(state, action, reward, next_state, next_action, done)
                action = next_action

            cumulative_reward += reward
            step_infos.append(info)
            state = next_state

        rows.append(
            summarize_episode(
                step_infos,
                cumulative_reward,
                episode=episode_index + 1,
                epsilon=agent.epsilon,
            )
        )
        agent.decay_epsilon()

    return agent, rows


def collect_policy_evaluation(
    agent_name,
    agent,
    config,
    episodes,
    seed,
    quiet=False,
    eval_set=None,
    episode_offset=0,
):
    env = TrafficEnv.from_config(config, seed=seed)
    rows = []
    component_infos = []
    reward_type = config["reward"]["type"]
    suffix = f" set {eval_set}" if eval_set is not None else ""
    iterator = trange(episodes, desc=f"eval {agent_name} {reward_type}{suffix}", disable=quiet)

    for episode_index in iterator:
        episode_seed = seed + episode_index
        state = env.reset(seed=episode_seed)
        if hasattr(agent, "reset"):
            agent.reset()

        done = False
        cumulative_reward = 0.0
        step_infos = []
        info = env.get_info()

        while not done:
            if agent_name == "fixed_time":
                action = agent.select_action(state, timestep=env.timestep, info=info)
            else:
                action = agent.select_action(state, epsilon=0.0)

            next_state, reward, done, info = env.step(action)
            cumulative_reward += reward
            step_infos.append(info)
            component_infos.append(info)
            state = next_state

        row = summarize_episode(
            step_infos,
            cumulative_reward,
            episode=episode_offset + episode_index + 1,
            epsilon=None if agent_name == "fixed_time" else 0.0,
        )
        if eval_set is not None:
            row["eval_set"] = int(eval_set)
            row["eval_base_seed"] = int(seed)
            row["episode_seed"] = int(episode_seed)
        rows.append(row)

    return rows, component_infos


def evaluate_policy(agent_name, agent, config, episodes, seed, quiet=False):
    rows, component_infos = collect_policy_evaluation(
        agent_name,
        agent,
        config,
        episodes=episodes,
        seed=seed,
        quiet=quiet,
    )

    summary = aggregate_episode_metrics(rows)
    components = summarize_reward_components(component_infos)
    return rows, summary, components


def evaluation_base_seeds(training_seed, eval_seed_offsets):
    return [int(training_seed) + int(offset) for offset in eval_seed_offsets]


def parse_eval_seed_offsets(value):
    if value is None:
        return DEFAULT_EVAL_SEED_OFFSETS
    if isinstance(value, int):
        return (int(value),)
    if isinstance(value, (list, tuple)):
        offsets = tuple(int(offset) for offset in value)
    else:
        offsets = tuple(
            int(part.strip())
            for part in str(value).split(",")
            if part.strip()
        )
    if not offsets:
        raise ValueError("at least one evaluation seed offset is required")
    return offsets


def evaluate_policy_seed_sets(
    agent_name,
    agent,
    config,
    episodes_per_seed,
    training_seed,
    eval_seed_offsets,
    quiet=False,
):
    all_rows = []
    all_component_infos = []
    seed_summary_rows = []

    for eval_set, eval_seed in enumerate(
        evaluation_base_seeds(training_seed, eval_seed_offsets),
        start=1,
    ):
        rows, component_infos = collect_policy_evaluation(
            agent_name,
            agent,
            config,
            episodes=episodes_per_seed,
            seed=eval_seed,
            quiet=quiet,
            eval_set=eval_set,
            episode_offset=len(all_rows),
        )
        summary = aggregate_episode_metrics(rows)
        components = summarize_reward_components(component_infos)
        seed_summary_rows.append(
            {
                "eval_set": eval_set,
                "eval_base_seed": eval_seed,
                "first_episode_seed": eval_seed,
                "last_episode_seed": eval_seed + episodes_per_seed - 1,
                **summary,
                **components,
            }
        )
        all_rows.extend(rows)
        all_component_infos.extend(component_infos)

    summary = aggregate_episode_metrics(all_rows)
    components = summarize_reward_components(all_component_infos)
    return all_rows, summary, components, seed_summary_rows


def summarize_reward_components(step_infos):
    return {
        "mean_abs_queue_delta": mean(abs(info["queue_delta"]) for info in step_infos),
        "mean_total_waiting_time": mean(info["waiting_time"] for info in step_infos),
        "mean_throughput_component": mean(info["throughput"] for info in step_infos),
        "mean_total_queue_component": mean(info["total_queue"] for info in step_infos),
        "mean_max_red_queue_component": mean(info["max_red_queue"] for info in step_infos),
        "mean_switch_component": mean(info["switched"] for info in step_infos),
    }


def reward_config_for(
    base_config,
    reward_id,
    w1=1.0,
    w2=0.1,
    w3=1.0,
    w4=0.0,
    base_reward_type="R1",
    episodes=None,
    eval_episodes=None,
):
    config = deepcopy(base_config)
    config["reward"] = {
        "type": reward_id,
        "w1": float(w1),
        "w2": float(w2),
        "w3": float(w3),
        "w4": float(w4),
        "base_reward_type": base_reward_type,
    }
    config.setdefault("training", {})
    config.setdefault("evaluation", {})
    if episodes is not None:
        config["training"]["episodes"] = int(episodes)
    if eval_episodes is not None:
        config["evaluation"]["episodes"] = int(eval_episodes)
    return config


def score_against_fixed(summary, fixed_summary):
    fixed_queue = max(float(fixed_summary["mean_average_queue_length"]), EPSILON)
    fixed_wait = max(float(fixed_summary["mean_average_waiting_time"]), EPSILON)
    fixed_throughput = max(float(fixed_summary["mean_total_throughput"]), EPSILON)
    fixed_switches = max(float(fixed_summary["mean_phase_switches"]), 1.0)

    queue_improvement = (fixed_summary["mean_average_queue_length"] - summary["mean_average_queue_length"]) / fixed_queue
    wait_improvement = (fixed_summary["mean_average_waiting_time"] - summary["mean_average_waiting_time"]) / fixed_wait
    throughput_improvement = (summary["mean_total_throughput"] - fixed_summary["mean_total_throughput"]) / fixed_throughput
    switch_penalty_score = (summary["mean_phase_switches"] - fixed_summary["mean_phase_switches"]) / fixed_switches

    raw_score = (
        0.40 * queue_improvement
        + 0.40 * wait_improvement
        + 0.20 * throughput_improvement
        - 0.10 * switch_penalty_score
    )
    raw_score = float(raw_score)
    score_percent = raw_score * 100.0
    return {
        "queue_improvement": float(queue_improvement),
        "wait_improvement": float(wait_improvement),
        "throughput_improvement": float(throughput_improvement),
        "switch_penalty_score": float(switch_penalty_score),
        "raw_score": raw_score,
        "score_percent": score_percent,
    }


def make_result_row(stage, reward_id, agent_name, config, summary, components, fixed_summary):
    score = score_against_fixed(summary, fixed_summary)
    reward = config["reward"]
    return {
        "stage": stage,
        "reward_id": reward_id,
        "agent": agent_name,
        "formula": REWARD_FORMULAS[reward_id],
        "comment": REWARD_COMMENTS[reward_id],
        "base_reward_type": reward.get("base_reward_type", ""),
        "w1": reward.get("w1", 1.0),
        "w2": reward.get("w2", 0.1),
        "w3": reward.get("w3", 1.0),
        "w4": reward.get("w4", 0.0),
        "episodes": summary["episodes"],
        "mean_cumulative_reward": summary["mean_cumulative_reward"],
        "mean_average_queue_length": summary["mean_average_queue_length"],
        "mean_average_waiting_time": summary["mean_average_waiting_time"],
        "mean_total_throughput": summary["mean_total_throughput"],
        "mean_phase_switches": summary["mean_phase_switches"],
        "avg_queue": summary["mean_average_queue_length"],
        "avg_wait": summary["mean_average_waiting_time"],
        "throughput": summary["mean_total_throughput"],
        "phase_switches": summary["mean_phase_switches"],
        "overflow_episodes": summary["overflow_episodes"],
        **components,
        **score,
    }


def average_result_rows(rows):
    metric_fields = [
        "mean_cumulative_reward",
        "mean_average_queue_length",
        "mean_average_waiting_time",
        "mean_total_throughput",
        "mean_phase_switches",
        "mean_abs_queue_delta",
        "mean_total_waiting_time",
        "mean_throughput_component",
        "mean_total_queue_component",
        "mean_max_red_queue_component",
        "mean_switch_component",
        "queue_improvement",
        "wait_improvement",
        "throughput_improvement",
        "switch_penalty_score",
        "raw_score",
        "score_percent",
    ]
    averaged = {}
    for field in metric_fields:
        averaged[field] = mean(float(row[field]) for row in rows)
    averaged["overflow_episodes"] = sum(int(row["overflow_episodes"]) for row in rows)
    return averaged


def calculate_normalized_reward_weights(stage_a_rows, fixed_components):
    """Calculate w1, w2, and w3 from component magnitudes, not scores."""
    component_rows = [
        row for row in stage_a_rows if row["reward_id"] in SINGLE_OBJECTIVE_REWARDS
    ]

    queue_scale = mean(
        [row["mean_abs_queue_delta"] for row in component_rows]
        + [fixed_components["mean_abs_queue_delta"]]
    )
    wait_scale = mean(
        [row["mean_total_waiting_time"] for row in component_rows]
        + [fixed_components["mean_total_waiting_time"]]
    )
    throughput_scale = mean(
        [row["mean_throughput_component"] for row in component_rows]
        + [fixed_components["mean_throughput_component"]]
    )

    inv_q = 1.0 / (queue_scale + EPSILON)
    inv_w = 1.0 / (wait_scale + EPSILON)
    inv_t = 1.0 / (throughput_scale + EPSILON)
    normalizer = inv_q + inv_w + inv_t

    weights = {
        "queue_scale": queue_scale,
        "wait_scale": wait_scale,
        "throughput_scale": throughput_scale,
        "w1": inv_q / normalizer,
        "w2": inv_w / normalizer,
        "w3": inv_t / normalizer,
    }

    weight_rows = [
        {
            "component": "queue_delta",
            "used_by_rewards": "R7",
            "mean_component_magnitude": queue_scale,
            "inverse_scale": inv_q,
            "normalizer": normalizer,
            "final_weight": weights["w1"],
        },
        {
            "component": "total_waiting_time",
            "used_by_rewards": "R4,R7",
            "mean_component_magnitude": wait_scale,
            "inverse_scale": inv_w,
            "normalizer": normalizer,
            "final_weight": weights["w2"],
        },
        {
            "component": "throughput",
            "used_by_rewards": "R5,R7",
            "mean_component_magnitude": throughput_scale,
            "inverse_scale": inv_t,
            "normalizer": normalizer,
            "final_weight": weights["w3"],
        },
    ]
    return weights, weight_rows


def run_reward_agent(
    stage,
    reward_id,
    agent_name,
    config,
    episodes,
    eval_episodes,
    seed,
    eval_seed_offsets,
    output_dir,
    quiet,
):
    logs_dir = output_dir / "logs"
    agent, training_rows = train_agent(agent_name, config, episodes=episodes, seed=seed, quiet=quiet)
    evaluation_rows, summary, components, seed_summary_rows = evaluate_policy_seed_sets(
        agent_name,
        agent,
        config,
        episodes_per_seed=eval_episodes,
        training_seed=seed,
        eval_seed_offsets=eval_seed_offsets,
        quiet=quiet,
    )
    save_csv(training_rows, logs_dir / f"{stage}_{reward_id}_{agent_name}_training_metrics.csv")
    save_csv(evaluation_rows, logs_dir / f"{stage}_{reward_id}_{agent_name}_evaluation_metrics.csv")
    save_csv(seed_summary_rows, logs_dir / f"{stage}_{reward_id}_{agent_name}_evaluation_seed_summary.csv")
    return summary, components


def run_fixed_baseline(base_config, eval_episodes, seed, eval_seed_offsets, output_dir, quiet):
    fixed_config = reward_config_for(base_config, "R1", episodes=1, eval_episodes=eval_episodes)
    fixed_agent = FixedTimeAgent(
        green_duration=fixed_config.get("fixed_time", {}).get("green_duration", 20),
        n_actions=TrafficEnv.n_actions,
    )
    rows, summary, components, seed_summary_rows = evaluate_policy_seed_sets(
        "fixed_time",
        fixed_agent,
        fixed_config,
        episodes_per_seed=eval_episodes,
        training_seed=seed,
        eval_seed_offsets=eval_seed_offsets,
        quiet=quiet,
    )
    save_csv(rows, output_dir / "logs" / "fixed_time_baseline_evaluation_metrics.csv")
    save_csv(seed_summary_rows, output_dir / "logs" / "fixed_time_baseline_evaluation_seed_summary.csv")
    save_json(
        {
            "summary": summary,
            "components": components,
            "evaluation_seed_offsets": list(eval_seed_offsets),
            "evaluation_base_seeds": evaluation_base_seeds(seed, eval_seed_offsets),
            "evaluation_episodes_per_seed": eval_episodes,
            "total_evaluation_episodes": eval_episodes * len(eval_seed_offsets),
        },
        output_dir / "logs" / "fixed_time_baseline_summary.json",
    )
    return summary, components


def write_experiment_config(config, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as file:
        file.write(format_yaml_mapping(config))


def format_yaml_mapping(mapping, indent=0):
    lines = []
    prefix = " " * indent
    for key, value in mapping.items():
        if isinstance(value, dict):
            lines.append(f"{prefix}{key}:")
            lines.append(format_yaml_mapping(value, indent + 2))
        else:
            lines.append(f"{prefix}{key}: {format_yaml_scalar(value)}")
    return "\n".join(lines) + ("\n" if indent == 0 else "")


def format_yaml_scalar(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None:
        return "null"
    return str(value)


def build_final_table(rows):
    table_rows = []
    for reward_id in sorted({row["reward_id"] for row in rows}, key=reward_sort_key):
        reward_rows = [row for row in rows if row["reward_id"] == reward_id]
        averaged = average_result_rows(reward_rows)
        first = reward_rows[0]
        table_rows.append(
            {
                "stage": first["stage"],
                "reward_id": reward_id,
                "formula": first["formula"],
                "base_reward_type": first["base_reward_type"],
                "w1": first["w1"],
                "w2": first["w2"],
                "w3": first["w3"],
                "w4": first["w4"],
                "avg_queue": averaged["mean_average_queue_length"],
                "avg_wait": averaged["mean_average_waiting_time"],
                "throughput": averaged["mean_total_throughput"],
                "phase_switches": averaged["mean_phase_switches"],
                "switches": averaged["mean_phase_switches"],
                "queue_improvement": averaged["queue_improvement"],
                "wait_improvement": averaged["wait_improvement"],
                "throughput_improvement": averaged["throughput_improvement"],
                "switch_penalty_score": averaged["switch_penalty_score"],
                "raw_score": averaged["raw_score"],
                "score_percent": averaged["score_percent"],
                "comment": first["comment"],
            }
        )
    return table_rows


def reward_sort_key(reward_id):
    return REWARD_ORDER.get(str(reward_id), 999)


def select_winner(stage_a_rows):
    table_rows = build_final_table(stage_a_rows)
    return max(table_rows, key=lambda row: (row["score_percent"], -row["phase_switches"]))


def plot_metric_grid(rows, output_dir, filename, title, fixed_summary):
    if not rows:
        return None

    output_dir.mkdir(parents=True, exist_ok=True)
    reward_ids = sorted({row["reward_id"] for row in rows}, key=reward_sort_key)
    metrics = [
        (
            "mean_average_queue_length",
            "Average queue length (vehicles)",
            fixed_summary["mean_average_queue_length"],
        ),
        (
            "mean_average_waiting_time",
            "Average waiting time (vehicle-timesteps)",
            fixed_summary["mean_average_waiting_time"],
        ),
        (
            "mean_total_throughput",
            "Total throughput (vehicles cleared per episode)",
            fixed_summary["mean_total_throughput"],
        ),
        (
            "mean_phase_switches",
            "Number of phase switches per episode",
            fixed_summary["mean_phase_switches"],
        ),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    for axis, (metric, label, fixed_value) in zip(axes.ravel(), metrics):
        for agent_name in AGENTS:
            y_values = []
            for reward_id in reward_ids:
                row = next(
                    row for row in rows if row["reward_id"] == reward_id and row["agent"] == agent_name
                )
                y_values.append(row[metric])
            axis.plot(reward_ids, y_values, marker="o", linewidth=1.8, label=agent_name)
        axis.axhline(
            fixed_value,
            linestyle="--",
            linewidth=1,
            color="black",
            alpha=0.7,
            label="Fixed-time baseline",
        )
        axis.set_title(label)
        axis.set_ylabel(label)
        axis.grid(True, alpha=0.3)
        axis.legend()

    fig.suptitle(title)
    fig.tight_layout()
    path = output_dir / filename
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def plot_score_bars(table_rows, output_dir, filename, title):
    output_dir.mkdir(parents=True, exist_ok=True)
    labels = [row["reward_id"] for row in table_rows]
    scores = [row["score_percent"] for row in table_rows]
    colors = ["#2e7d32" if score >= 0 else "#b71c1c" for score in scores]

    fig, axis = plt.subplots(figsize=(11, 5))
    axis.bar(labels, scores, color=colors)
    axis.axhline(0, linewidth=1, color="black")
    axis.set_title(title)
    axis.set_xlabel("Reward function")
    axis.set_ylabel("Performance score relative to fixed-time (%)")
    axis.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    path = output_dir / filename
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def plot_improvement_grid(table_rows, output_dir, filename, title):
    output_dir.mkdir(parents=True, exist_ok=True)
    labels = [row["reward_id"] for row in table_rows]
    metrics = [
        ("queue_improvement", "Queue improvement relative to fixed-time (%)"),
        ("wait_improvement", "Waiting-time improvement relative to fixed-time (%)"),
        ("throughput_improvement", "Throughput improvement relative to fixed-time (%)"),
        ("switch_penalty_score", "Extra switching relative to fixed-time (%)"),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    for axis, (metric, ylabel) in zip(axes.ravel(), metrics):
        values = [row[metric] * 100.0 for row in table_rows]
        colors = ["#2e7d32" if value >= 0 else "#b71c1c" for value in values]
        axis.bar(labels, values, color=colors)
        axis.axhline(0, linewidth=1, color="black")
        axis.set_title(ylabel)
        axis.set_ylabel(ylabel)
        axis.grid(True, axis="y", alpha=0.3)

    fig.suptitle(title)
    fig.tight_layout()
    path = output_dir / filename
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def run_experiment(
    base_config,
    output_dir,
    episodes,
    eval_episodes,
    seed,
    eval_seed_offsets=DEFAULT_EVAL_SEED_OFFSETS,
    quiet=False,
):
    eval_seed_offsets = tuple(int(offset) for offset in eval_seed_offsets)
    output_dir = ensure_output_dirs(output_dir)
    output_dir = Path(output_dir)
    logs_dir = output_dir / "logs"
    figures_dir = output_dir / "figures"
    generated_config_dir = output_dir / "generated_configs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    figures_dir.mkdir(parents=True, exist_ok=True)
    generated_config_dir.mkdir(parents=True, exist_ok=True)

    fixed_summary, fixed_components = run_fixed_baseline(
        base_config,
        eval_episodes=eval_episodes,
        seed=seed,
        eval_seed_offsets=eval_seed_offsets,
        output_dir=output_dir,
        quiet=quiet,
    )

    default_w2 = float(base_config.get("reward", {}).get("w2", 0.1))
    default_w3 = float(base_config.get("reward", {}).get("w3", 1.0))

    stage_a_rows = []
    reward_configs = {}

    for reward_id in SINGLE_OBJECTIVE_REWARDS:
        reward_configs[reward_id] = reward_config_for(
            base_config,
            reward_id,
            w1=1.0,
            w2=default_w2,
            w3=default_w3,
            episodes=episodes,
            eval_episodes=eval_episodes,
        )
        write_experiment_config(reward_configs[reward_id], generated_config_dir / f"reward_{reward_id}.yaml")
        for agent_name in AGENTS:
            summary, components = run_reward_agent(
                "stage_a",
                reward_id,
                agent_name,
                reward_configs[reward_id],
                episodes,
                eval_episodes,
                seed,
                eval_seed_offsets,
                output_dir,
                quiet,
            )
            stage_a_rows.append(
                make_result_row("A", reward_id, agent_name, reward_configs[reward_id], summary, components, fixed_summary)
            )

    normalized_weights, weight_rows = calculate_normalized_reward_weights(stage_a_rows, fixed_components)
    save_csv(weight_rows, logs_dir / "reward_weight_calculation.csv")
    computed_w2 = normalized_weights["w2"]
    computed_w3 = normalized_weights["w3"]

    remaining_configs = {
        "R4": reward_config_for(
            base_config,
            "R4",
            w1=normalized_weights["w1"],
            w2=computed_w2,
            w3=default_w3,
            episodes=episodes,
            eval_episodes=eval_episodes,
        ),
        "R5": reward_config_for(
            base_config,
            "R5",
            w1=normalized_weights["w1"],
            w2=default_w2,
            w3=computed_w3,
            episodes=episodes,
            eval_episodes=eval_episodes,
        ),
        "R6": reward_config_for(
            base_config,
            "R6",
            w1=1.0,
            w2=1.0,
            w3=1.0,
            episodes=episodes,
            eval_episodes=eval_episodes,
        ),
        "R7": reward_config_for(
            base_config,
            "R7",
            w1=normalized_weights["w1"],
            w2=normalized_weights["w2"],
            w3=normalized_weights["w3"],
            episodes=episodes,
            eval_episodes=eval_episodes,
        ),
        "R8": reward_config_for(
            base_config,
            "R8",
            w1=normalized_weights["w1"],
            w2=normalized_weights["w2"],
            w3=normalized_weights["w3"],
            episodes=episodes,
            eval_episodes=eval_episodes,
        ),
    }
    reward_configs.update(remaining_configs)

    for reward_id in ("R4", "R5", "R6", "R7", "R8"):
        write_experiment_config(reward_configs[reward_id], generated_config_dir / f"reward_{reward_id}.yaml")
        for agent_name in AGENTS:
            summary, components = run_reward_agent(
                "stage_a",
                reward_id,
                agent_name,
                reward_configs[reward_id],
                episodes,
                eval_episodes,
                seed,
                eval_seed_offsets,
                output_dir,
                quiet,
            )
            stage_a_rows.append(
                make_result_row("A", reward_id, agent_name, reward_configs[reward_id], summary, components, fixed_summary)
            )

    save_csv(stage_a_rows, logs_dir / "reward_stage_a_results.csv")
    stage_a_table = build_final_table(stage_a_rows)
    save_csv(stage_a_table, logs_dir / "reward_stage_a_final_table.csv")

    winner = select_winner(stage_a_rows)
    winner_id = winner["reward_id"]
    winner_config = reward_configs[winner_id]
    save_json({"winner": winner}, logs_dir / "reward_stage_a_winner.json")

    stage_b_rows = []
    for reward_id, penalty in zip(SWITCH_REWARDS, SWITCH_PENALTIES):
        reward_configs[reward_id] = reward_config_for(
            base_config,
            reward_id,
            w1=winner_config["reward"].get("w1", 1.0),
            w2=winner_config["reward"].get("w2", default_w2),
            w3=winner_config["reward"].get("w3", default_w3),
            w4=penalty,
            base_reward_type=winner_id,
            episodes=episodes,
            eval_episodes=eval_episodes,
        )
        write_experiment_config(reward_configs[reward_id], generated_config_dir / f"reward_{reward_id}.yaml")
        for agent_name in AGENTS:
            summary, components = run_reward_agent(
                "stage_b",
                reward_id,
                agent_name,
                reward_configs[reward_id],
                episodes,
                eval_episodes,
                seed,
                eval_seed_offsets,
                output_dir,
                quiet,
            )
            stage_b_rows.append(
                make_result_row("B", reward_id, agent_name, reward_configs[reward_id], summary, components, fixed_summary)
            )

    save_csv(stage_b_rows, logs_dir / "reward_stage_b_results.csv")
    stage_b_table = build_final_table(stage_b_rows)
    save_csv(stage_b_table, logs_dir / "reward_stage_b_final_table.csv")

    all_rows = stage_a_rows + stage_b_rows
    final_table = build_final_table(all_rows)
    save_csv(all_rows, logs_dir / "reward_all_agent_results.csv")
    save_csv(final_table, logs_dir / "reward_final_table.csv")

    plot_metric_grid(
        stage_a_rows,
        figures_dir,
        "reward_stage_a_metrics.png",
        "Reward comparison R1-R8",
        fixed_summary,
    )
    plot_metric_grid(
        stage_b_rows,
        figures_dir,
        "reward_stage_b_switch_penalty_metrics.png",
        "Switch penalty comparison",
        fixed_summary,
    )
    plot_score_bars(
        stage_a_table,
        figures_dir,
        "reward_stage_a_score_percent_comparison.png",
        "Reward Function Score Comparison R1-R8",
    )
    plot_score_bars(
        stage_b_table,
        figures_dir,
        "reward_stage_b_score_percent_comparison.png",
        "Switch Penalty Score Comparison",
    )
    plot_improvement_grid(
        stage_a_table,
        figures_dir,
        "reward_stage_a_improvements.png",
        "Stage A improvement metrics relative to fixed-time",
    )
    plot_improvement_grid(
        stage_b_table,
        figures_dir,
        "reward_stage_b_improvements.png",
        "Stage B improvement metrics relative to fixed-time",
    )

    summary = {
        "episodes_per_training_run": episodes,
        "evaluation_episodes": eval_episodes,
        "evaluation_episodes_per_seed": eval_episodes,
        "evaluation_seed_offsets": list(eval_seed_offsets),
        "evaluation_base_seeds": evaluation_base_seeds(seed, eval_seed_offsets),
        "total_evaluation_episodes_per_policy": eval_episodes * len(eval_seed_offsets),
        "seed": seed,
        "fixed_time_baseline": fixed_summary,
        "fixed_time_components": fixed_components,
        "normalized_reward_weights": normalized_weights,
        "r7_weights": normalized_weights,
        "stage_a_winner": winner,
        "outputs": {
            "stage_a_results": str(logs_dir / "reward_stage_a_results.csv"),
            "stage_b_results": str(logs_dir / "reward_stage_b_results.csv"),
            "final_table": str(logs_dir / "reward_final_table.csv"),
            "stage_a_metrics_plot": str(figures_dir / "reward_stage_a_metrics.png"),
            "stage_b_metrics_plot": str(figures_dir / "reward_stage_b_switch_penalty_metrics.png"),
            "stage_a_score_plot": str(figures_dir / "reward_stage_a_score_percent_comparison.png"),
            "stage_b_score_plot": str(figures_dir / "reward_stage_b_score_percent_comparison.png"),
        },
    }
    save_json(summary, logs_dir / "reward_experiment_summary.json")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/default.yaml")
    parser.add_argument("--output-dir", default="results/reward_experiment")
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument("--eval-episodes", type=int, default=None)
    parser.add_argument(
        "--eval-seed-offsets",
        default=None,
        help="Comma-separated offsets added to the training seed, e.g. 10000,20000,30000.",
    )
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    config = load_config(args.config)
    seed = int(args.seed if args.seed is not None else config.get("training", {}).get("seed", 42))
    episodes = int(args.episodes if args.episodes is not None else config.get("training", {}).get("episodes", 3000))
    eval_episodes = int(
        args.eval_episodes
        if args.eval_episodes is not None
        else config.get("evaluation", {}).get("episodes", 100)
    )
    eval_seed_offsets = parse_eval_seed_offsets(
        args.eval_seed_offsets
        if args.eval_seed_offsets is not None
        else config.get("evaluation", {}).get("seed_offsets", DEFAULT_EVAL_SEED_OFFSETS)
    )
    summary = run_experiment(
        config,
        output_dir=args.output_dir,
        episodes=episodes,
        eval_episodes=eval_episodes,
        seed=seed,
        eval_seed_offsets=eval_seed_offsets,
        quiet=args.quiet,
    )

    print("Reward experiment complete.")
    print(f"Training episodes per run: {summary['episodes_per_training_run']}")
    print(
        "Evaluation seeds: "
        + ", ".join(str(seed) for seed in summary["evaluation_base_seeds"])
    )
    print(f"Evaluation episodes per seed: {summary['evaluation_episodes_per_seed']}")
    print(f"Winner from R1-R8: {summary['stage_a_winner']['reward_id']}")
    print(f"Final table: {summary['outputs']['final_table']}")
    print(f"Stage A plot: {summary['outputs']['stage_a_metrics_plot']}")
    print(f"Stage B plot: {summary['outputs']['stage_b_metrics_plot']}")


if __name__ == "__main__":
    main()
