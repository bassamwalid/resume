"""Run the final mixed-scenario RL traffic-signal experiments.

This script trains the final Q-learning, SARSA, and DQN controllers with the
frozen R11 reward, evaluates fixed-time and learned controllers on every final
scenario, and saves the comparison tables and plots used in the final report.
"""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

from evaluate import build_evaluation_agent, evaluate_agent
from train import train_agent
from env.continuous_traffic_env import ContinuousTrafficEnv
from env.state_encoder import DIRECTIONS
from env.traffic_env import (
    MIXED_SCENARIOS,
    TrafficEnv,
    config_with_traffic_scenario,
    parse_scenario_names,
)
from utils.metrics import aggregate_episode_metrics, mean, summarize_episode
from utils.plotting import (
    plot_training_curves,
    rolling_mean_values,
    training_smoothing_window,
)
from utils.saving import ensure_output_dirs, load_config, save_csv, save_json, save_q_table


FINAL_CONFIG = "configs/final_r11_mixed.yaml"
OUTPUT_DIR = "results/final_mixed_r11"

CONTROLLERS = (
    "fixed_time",
    "q_learning",
    "sarsa",
    "dqn",
)

FINAL_SCENARIOS = (
    "low_uniform",
    "uniform_medium",
    "high_uniform",
    "uneven",
    "rush_hour",
)

EVAL_BASE_SEEDS = (10042, 20042, 30042)
EVAL_EPISODES = 100
TRAIN_BASE_SEED = 42
EPSILON = 1e-8


def parse_int_list(value):
    if value is None:
        return list(EVAL_BASE_SEEDS)
    if isinstance(value, str):
        return [int(part.strip()) for part in value.split(",") if part.strip()]
    return [int(item) for item in value]


def training_episode_counts(config, q_episodes=None, sarsa_episodes=None, dqn_episodes=None):
    """Resolve final training episodes from CLI overrides or the YAML config."""
    training = config.get("training", {})
    dqn = config.get("dqn", {})
    tabular_default = int(training.get("episodes", 12000))
    return {
        "q_learning": int(q_episodes if q_episodes is not None else tabular_default),
        "sarsa": int(sarsa_episodes if sarsa_episodes is not None else tabular_default),
        "dqn": int(dqn_episodes if dqn_episodes is not None else dqn.get("episodes", 3000)),
    }


def read_csv_rows(path):
    with open(path, newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def to_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def std(values):
    values = [float(value) for value in values]
    if len(values) <= 1:
        return 0.0
    average = mean(values)
    variance = sum((value - average) ** 2 for value in values) / (len(values) - 1)
    return float(variance ** 0.5)


def score_against_fixed(summary, fixed_summary):
    fixed_queue = max(float(fixed_summary["mean_average_queue_length"]), EPSILON)
    fixed_wait = max(float(fixed_summary["mean_average_waiting_time"]), EPSILON)
    fixed_throughput = max(float(fixed_summary["mean_total_throughput"]), EPSILON)
    fixed_switches = max(float(fixed_summary["mean_phase_switches"]), 1.0)

    queue_improvement = (
        fixed_summary["mean_average_queue_length"] - summary["mean_average_queue_length"]
    ) / fixed_queue
    wait_improvement = (
        fixed_summary["mean_average_waiting_time"] - summary["mean_average_waiting_time"]
    ) / fixed_wait
    throughput_improvement = (
        summary["mean_total_throughput"] - fixed_summary["mean_total_throughput"]
    ) / fixed_throughput
    switch_penalty_score = (
        summary["mean_phase_switches"] - fixed_summary["mean_phase_switches"]
    ) / fixed_switches

    raw_score = (
        0.40 * queue_improvement
        + 0.40 * wait_improvement
        + 0.20 * throughput_improvement
        - 0.10 * switch_penalty_score
    )
    return {
        "queue_improvement": float(queue_improvement),
        "wait_improvement": float(wait_improvement),
        "throughput_improvement": float(throughput_improvement),
        "switch_penalty_score": float(switch_penalty_score),
        "raw_score": float(raw_score),
        "score_percent": float(raw_score * 100.0),
    }


def train_final_agents(config, output_dir, training_scenario, train_seed, episode_counts):
    logs_dir = output_dir / "logs"
    q_table_dir = output_dir / "q_tables"
    model_dir = output_dir / "models"

    training_rows_by_agent = {}
    for agent_name, episodes in episode_counts.items():
        agent, rows = train_agent(
            agent_name,
            config,
            episodes=int(episodes),
            seed=train_seed,
            scenario=training_scenario,
        )
        training_rows_by_agent[agent_name] = rows
        save_csv(rows, logs_dir / f"{agent_name}_training_metrics.csv")
        save_json(config, logs_dir / f"{agent_name}_training_config.json")
        if agent_name == "dqn":
            agent.save(model_dir / "dqn_model.pt")
        else:
            save_q_table(agent.get_q_table(), q_table_dir / f"{agent_name}_q_table.pkl")
        plot_training_curves(rows, output_dir, agent_name)

    plot_tabular_training_comparison(training_rows_by_agent, output_dir)
    return training_rows_by_agent


def load_training_rows(output_dir):
    rows_by_agent = {}
    for agent_name in ("q_learning", "sarsa"):
        path = output_dir / "logs" / f"{agent_name}_training_metrics.csv"
        if path.exists():
            rows_by_agent[agent_name] = read_csv_rows(path)
    return rows_by_agent


def evaluate_final_controllers(config, output_dir, scenarios, eval_seeds, eval_episodes):
    episode_rows = []
    seed_summary_rows = []
    seed_summaries = {}

    for controller in CONTROLLERS:
        for scenario in scenarios:
            for eval_seed in eval_seeds:
                rows, summary = evaluate_agent(
                    controller,
                    config,
                    output_dir=output_dir,
                    episodes=eval_episodes,
                    seed=eval_seed,
                    save_outputs=False,
                    scenario=scenario,
                )
                for row in rows:
                    row["controller"] = controller
                    row["agent"] = controller
                    row["scenario"] = scenario
                    row["eval_base_seed"] = int(eval_seed)
                    episode_rows.append(row)

                summary_row = {
                    "controller": controller,
                    "agent": controller,
                    "scenario": scenario,
                    "eval_base_seed": int(eval_seed),
                    **summary,
                }
                seed_summary_rows.append(summary_row)
                seed_summaries[(controller, scenario, eval_seed)] = summary

    for row in seed_summary_rows:
        fixed_summary = seed_summaries[("fixed_time", row["scenario"], row["eval_base_seed"])]
        row.update(score_against_fixed(row, fixed_summary))

    scenario_summary_rows = build_scenario_summary_rows(episode_rows, seed_summary_rows)
    verdict_rows = build_tabular_verdict_rows(scenario_summary_rows, scenarios)
    overall_rows = build_overall_controller_rows(scenario_summary_rows)

    logs_dir = output_dir / "logs"
    save_csv(episode_rows, logs_dir / "final_evaluation_episode_metrics.csv")
    save_csv(seed_summary_rows, logs_dir / "final_evaluation_seed_summary.csv")
    save_csv(scenario_summary_rows, logs_dir / "final_scenario_summary.csv")
    save_csv(verdict_rows, logs_dir / "final_verdict_q_learning_vs_sarsa.csv")
    save_csv(overall_rows, logs_dir / "final_overall_controller_summary.csv")
    save_json(
        {
            "scenarios": list(scenarios),
            "controllers": list(CONTROLLERS),
            "eval_base_seeds": list(eval_seeds),
            "eval_episodes_per_seed_per_scenario": int(eval_episodes),
            "total_eval_episodes_per_controller": len(scenarios)
            * len(eval_seeds)
            * int(eval_episodes),
            "verdict": verdict_rows,
        },
        logs_dir / "final_experiment_summary.json",
    )

    plot_final_comparisons(scenario_summary_rows, overall_rows, output_dir, scenarios)
    return scenario_summary_rows, verdict_rows


def build_scenario_summary_rows(episode_rows, seed_summary_rows):
    rows = []
    grouped_episodes = defaultdict(list)
    grouped_seed_summaries = defaultdict(list)

    for row in episode_rows:
        grouped_episodes[(row["controller"], row["scenario"])].append(row)
    for row in seed_summary_rows:
        grouped_seed_summaries[(row["controller"], row["scenario"])].append(row)

    for (controller, scenario), rows_for_group in sorted(grouped_episodes.items()):
        seed_rows = grouped_seed_summaries[(controller, scenario)]
        queue_values = [to_float(row["average_queue_length"]) for row in rows_for_group]
        wait_values = [to_float(row["average_waiting_time"]) for row in rows_for_group]
        throughput_values = [to_float(row["total_throughput"]) for row in rows_for_group]
        switch_values = [to_float(row["phase_switches"]) for row in rows_for_group]
        overflow_values = [1 if to_float(row["overflow_count"]) > 0 else 0 for row in rows_for_group]
        score_values = [to_float(row["score_percent"]) for row in seed_rows]

        rows.append(
            {
                "controller": controller,
                "agent": controller,
                "scenario": scenario,
                "episodes": len(rows_for_group),
                "avg_queue_mean": mean(queue_values),
                "avg_queue_std": std(queue_values),
                "avg_wait_mean": mean(wait_values),
                "avg_wait_std": std(wait_values),
                "throughput_mean": mean(throughput_values),
                "throughput_std": std(throughput_values),
                "phase_switches_mean": mean(switch_values),
                "phase_switches_std": std(switch_values),
                "overflow_episodes": int(sum(overflow_values)),
                "overflow_episode_rate": mean(overflow_values),
                "score_percent_mean": mean(score_values),
                "score_percent_std": std(score_values),
                "score_percent": mean(score_values),
            }
        )
    return rows


def build_tabular_verdict_rows(scenario_summary_rows, scenarios):
    rows = []
    for controller in ("q_learning", "sarsa"):
        scenario_scores = []
        row = {"controller": controller}
        controller_rows = [
            item for item in scenario_summary_rows if item["controller"] == controller
        ]
        for scenario in scenarios:
            score = next(
                item["score_percent_mean"]
                for item in controller_rows
                if item["scenario"] == scenario
            )
            row[scenario] = score
            scenario_scores.append(score)

        row["overall_mean_score"] = mean(scenario_scores)
        row["score_std"] = std(scenario_scores)
        row["overall_avg_wait"] = mean(item["avg_wait_mean"] for item in controller_rows)
        row["overall_avg_queue"] = mean(item["avg_queue_mean"] for item in controller_rows)
        row["overall_phase_switches"] = mean(
            item["phase_switches_mean"] for item in controller_rows
        )
        rows.append(row)

    if len(rows) == 2:
        q_row, sarsa_row = rows
        difference = q_row["overall_mean_score"] - sarsa_row["overall_mean_score"]
        if abs(difference) > 2.0:
            winner = "q_learning" if difference > 0 else "sarsa"
            reason = "higher overall mean score"
        else:
            winner = select_close_tabular_winner(q_row, sarsa_row)
            reason = "close score, selected by secondary criteria"
        for row in rows:
            row["final_comment"] = "winner: " + reason if row["controller"] == winner else ""
    return rows


def select_close_tabular_winner(q_row, sarsa_row):
    criteria = (
        ("overall_avg_wait", False),
        ("overall_avg_queue", False),
        ("overall_phase_switches", False),
        ("score_std", False),
    )
    for metric, higher_is_better in criteria:
        q_value = q_row[metric]
        sarsa_value = sarsa_row[metric]
        if abs(q_value - sarsa_value) <= EPSILON:
            continue
        if higher_is_better:
            return "q_learning" if q_value > sarsa_value else "sarsa"
        return "q_learning" if q_value < sarsa_value else "sarsa"
    return "q_learning"


def build_overall_controller_rows(scenario_summary_rows):
    rows = []
    for controller in CONTROLLERS:
        controller_rows = [
            row for row in scenario_summary_rows if row["controller"] == controller
        ]
        if not controller_rows:
            continue
        rows.append(
            {
                "controller": controller,
                "overall_score_mean": mean(row["score_percent_mean"] for row in controller_rows),
                "overall_score_std": std(row["score_percent_mean"] for row in controller_rows),
                "overall_avg_queue": mean(row["avg_queue_mean"] for row in controller_rows),
                "overall_avg_wait": mean(row["avg_wait_mean"] for row in controller_rows),
                "overall_throughput": mean(row["throughput_mean"] for row in controller_rows),
                "overall_phase_switches": mean(
                    row["phase_switches_mean"] for row in controller_rows
                ),
                "total_overflow_episodes": int(
                    sum(row["overflow_episodes"] for row in controller_rows)
                ),
            }
        )
    return rows


def plot_tabular_training_comparison(training_rows_by_agent, output_dir):
    if not {"q_learning", "sarsa"}.issubset(training_rows_by_agent):
        return

    figures_dir = output_dir / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)
    plots = [
        ("cumulative_reward", "Episode reward", "q_sarsa_training_reward.png"),
        ("average_queue_length", "Average queue length (vehicles)", "q_sarsa_training_queue.png"),
        ("average_waiting_time", "Average waiting time (vehicle-timesteps)", "q_sarsa_training_wait.png"),
        ("total_throughput", "Total throughput (vehicles cleared per episode)", "q_sarsa_training_throughput.png"),
        ("phase_switches", "Number of phase switches per episode", "q_sarsa_training_switches.png"),
    ]

    for metric, ylabel, filename in plots:
        fig, axis = plt.subplots(figsize=(10, 5))
        for agent_name in ("q_learning", "sarsa"):
            rows = training_rows_by_agent[agent_name]
            episodes = [int(row["episode"]) for row in rows]
            values = rolling_mean_values(rows, metric, agent_name)
            axis.plot(episodes, values, linewidth=1.0, alpha=0.75, label=agent_name)
        axis.set_xlabel("Episode")
        axis.set_ylabel(ylabel)
        axis.set_title("Q-learning vs SARSA rolling training average")
        axis.grid(True, alpha=0.3)
        axis.legend()
        axis.text(
            0.01,
            0.02,
            f"rolling window={training_smoothing_window('q_learning')}",
            transform=axis.transAxes,
            fontsize=9,
            alpha=0.75,
        )
        fig.tight_layout()
        fig.savefig(figures_dir / filename, dpi=160)
        plt.close(fig)


def plot_final_comparisons(scenario_summary_rows, overall_rows, output_dir, scenarios):
    figures_dir = output_dir / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)
    scenarios = list(scenarios)
    controllers = list(CONTROLLERS)

    plot_grouped_metric(
        scenario_summary_rows,
        scenarios,
        controllers,
        metric="score_percent_mean",
        ylabel="Performance score relative to fixed-time (%)",
        title="Final score percentage per controller per scenario",
        path=figures_dir / "final_score_by_scenario.png",
    )
    plot_grouped_metric(
        scenario_summary_rows,
        scenarios,
        controllers,
        metric="avg_queue_mean",
        ylabel="Average queue length (vehicles)",
        title="Average queue by scenario",
        path=figures_dir / "final_average_queue_by_scenario.png",
    )
    plot_grouped_metric(
        scenario_summary_rows,
        scenarios,
        controllers,
        metric="avg_wait_mean",
        ylabel="Average waiting time (vehicle-timesteps)",
        title="Average waiting time by scenario",
        path=figures_dir / "final_average_wait_by_scenario.png",
    )
    plot_grouped_metric(
        scenario_summary_rows,
        scenarios,
        controllers,
        metric="throughput_mean",
        ylabel="Total throughput (vehicles cleared per episode)",
        title="Throughput by scenario",
        path=figures_dir / "final_throughput_by_scenario.png",
    )
    plot_grouped_metric(
        scenario_summary_rows,
        scenarios,
        controllers,
        metric="phase_switches_mean",
        ylabel="Average number of phase switches per episode",
        title="Phase switches by scenario",
        path=figures_dir / "final_phase_switches_by_scenario.png",
    )
    plot_grouped_metric(
        scenario_summary_rows,
        scenarios,
        controllers,
        metric="overflow_episodes",
        ylabel="Overflow episodes out of total evaluation episodes",
        title="Overflow episodes by scenario",
        path=figures_dir / "final_overflow_by_scenario.png",
    )
    plot_overall_scores(overall_rows, figures_dir / "final_overall_score_comparison.png")


def plot_grouped_metric(rows, scenarios, controllers, metric, ylabel, title, path):
    lookup = {
        (row["controller"], row["scenario"]): row
        for row in rows
    }
    width = 0.18
    x_positions = list(range(len(scenarios)))
    fig, axis = plt.subplots(figsize=(12, 6))

    for index, controller in enumerate(controllers):
        offset = (index - (len(controllers) - 1) / 2.0) * width
        values = [
            lookup.get((controller, scenario), {}).get(metric, 0.0)
            for scenario in scenarios
        ]
        axis.bar(
            [position + offset for position in x_positions],
            values,
            width=width,
            label=controller,
        )

    if metric == "score_percent_mean":
        axis.axhline(0, linewidth=1, color="black")
    axis.set_xticks(x_positions)
    axis.set_xticklabels(scenarios, rotation=20)
    axis.set_ylabel(ylabel)
    axis.set_title(title)
    axis.grid(True, axis="y", alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_overall_scores(rows, path):
    fig, axis = plt.subplots(figsize=(8, 5))
    controllers = [row["controller"] for row in rows]
    scores = [row["overall_score_mean"] for row in rows]
    errors = [row["overall_score_std"] for row in rows]
    axis.bar(controllers, scores, yerr=errors, capsize=4)
    axis.axhline(0, linewidth=1, color="black")
    axis.set_ylabel("Performance score relative to fixed-time (%)")
    axis.set_title("Overall score comparison across scenarios")
    axis.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def generate_policy_behavior_plots(config, output_dir, scenarios, eval_seed):
    behavior_dir = output_dir / "figures" / "policy_behavior"
    behavior_dir.mkdir(parents=True, exist_ok=True)

    for scenario in scenarios:
        traces = {}
        for controller in CONTROLLERS:
            traces[controller] = run_policy_trace(
                controller,
                config,
                output_dir,
                scenario=scenario,
                seed=eval_seed,
            )
            plot_direction_queues(
                traces[controller],
                behavior_dir / f"{scenario}_{controller}_queue_lengths.png",
                title=f"{controller} queue lengths on {scenario}",
            )

        plot_trace_metric(
            traces,
            metric="total_queue",
            ylabel="Total queue length",
            title=f"Total queue over time on {scenario}",
            path=behavior_dir / f"{scenario}_total_queue_by_controller.png",
        )
        plot_trace_metric(
            traces,
            metric="phase",
            ylabel="Selected phase",
            title=f"Selected phase over time on {scenario}",
            path=behavior_dir / f"{scenario}_phase_by_controller.png",
        )
        plot_phase_duration_histogram(
            traces,
            behavior_dir / f"{scenario}_phase_duration_histogram.png",
            title=f"Phase duration histogram on {scenario}",
        )
        plot_trace_metric(
            traces,
            metric="cumulative_throughput",
            ylabel="Cumulative throughput",
            title=f"Cumulative throughput on {scenario}",
            path=behavior_dir / f"{scenario}_cumulative_throughput.png",
        )
        plot_trace_metric(
            traces,
            metric="waiting_time",
            ylabel="Waiting time per timestep",
            title=f"Waiting time over time on {scenario}",
            path=behavior_dir / f"{scenario}_waiting_time.png",
        )


def run_policy_trace(controller, config, output_dir, scenario, seed):
    scenario_config = config_with_traffic_scenario(config, scenario)
    env_class = ContinuousTrafficEnv if controller == "dqn" else TrafficEnv
    env = env_class.from_config(scenario_config, seed=seed)
    agent = build_evaluation_agent(
        controller,
        scenario_config,
        output_dir,
        seed=seed,
        state_dim=getattr(env, "state_dim", None),
    )

    state = env.reset(seed=seed)
    if hasattr(agent, "reset"):
        agent.reset()

    rows = []
    step_infos = []
    cumulative_reward = 0.0
    cumulative_throughput = 0
    done = False
    info = env.get_info()

    while not done:
        if controller == "fixed_time":
            action = agent.select_action(state, timestep=env.timestep, info=info)
        else:
            action = agent.select_action(state, epsilon=0.0)
        next_state, reward, done, info = env.step(action)
        cumulative_reward += reward
        cumulative_throughput += int(info["throughput"])
        step_infos.append(info)
        row = {
            "timestep": info["timestep"],
            "phase": info["current_phase"],
            "waiting_time": info["waiting_time"],
            "total_queue": info["total_queue"],
            "cumulative_throughput": cumulative_throughput,
        }
        for direction in DIRECTIONS:
            row[direction] = info["queue_lengths"][direction]
        rows.append(row)
        state = next_state

    summary = summarize_episode(step_infos, cumulative_reward, episode=1, epsilon=None)
    return {
        "controller": controller,
        "scenario": scenario,
        "rows": rows,
        "phase_durations": phase_durations([row["phase"] for row in rows]),
        "summary": summary,
    }


def phase_durations(phases):
    if not phases:
        return []
    durations = []
    current_phase = phases[0]
    current_duration = 1
    for phase in phases[1:]:
        if phase == current_phase:
            current_duration += 1
        else:
            durations.append(current_duration)
            current_phase = phase
            current_duration = 1
    durations.append(current_duration)
    return durations


def plot_direction_queues(trace, path, title):
    rows = trace["rows"]
    if not rows:
        return
    fig, axis = plt.subplots(figsize=(10, 5))
    timesteps = [row["timestep"] for row in rows]
    for direction in DIRECTIONS:
        axis.plot(timesteps, [row[direction] for row in rows], label=direction)
    axis.set_xlabel("Timestep")
    axis.set_ylabel("Queue length")
    axis.set_title(title)
    axis.grid(True, alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_trace_metric(traces, metric, ylabel, title, path):
    fig, axis = plt.subplots(figsize=(10, 5))
    for controller, trace in traces.items():
        rows = trace["rows"]
        axis.plot(
            [row["timestep"] for row in rows],
            [row[metric] for row in rows],
            label=controller,
            linewidth=1.4,
        )
    axis.set_xlabel("Timestep")
    axis.set_ylabel(ylabel)
    axis.set_title(title)
    axis.grid(True, alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_phase_duration_histogram(traces, path, title):
    fig, axis = plt.subplots(figsize=(10, 5))
    for controller, trace in traces.items():
        durations = trace["phase_durations"]
        if durations:
            axis.hist(durations, bins=20, alpha=0.45, label=controller)
    axis.set_xlabel("Phase duration")
    axis.set_ylabel("Frequency")
    axis.set_title(title)
    axis.grid(True, axis="y", alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=FINAL_CONFIG)
    parser.add_argument("--output-dir", default=OUTPUT_DIR)
    parser.add_argument("--train-seed", type=int, default=TRAIN_BASE_SEED)
    parser.add_argument("--eval-seeds", default=",".join(str(seed) for seed in EVAL_BASE_SEEDS))
    parser.add_argument("--eval-episodes", type=int, default=EVAL_EPISODES)
    parser.add_argument("--q-episodes", type=int, default=None)
    parser.add_argument("--sarsa-episodes", type=int, default=None)
    parser.add_argument("--dqn-episodes", type=int, default=None)
    parser.add_argument("--training-scenario", default="mixed_scenarios")
    parser.add_argument("--scenarios", default=",".join(FINAL_SCENARIOS))
    parser.add_argument("--skip-training", action="store_true")
    parser.add_argument("--skip-evaluation", action="store_true")
    parser.add_argument("--skip-policy-plots", action="store_true")
    args = parser.parse_args()

    config = load_config(args.config)
    output_dir = ensure_output_dirs(args.output_dir)
    scenarios = parse_scenario_names(args.scenarios)
    eval_seeds = parse_int_list(args.eval_seeds)
    episode_counts = training_episode_counts(
        config,
        q_episodes=args.q_episodes,
        sarsa_episodes=args.sarsa_episodes,
        dqn_episodes=args.dqn_episodes,
    )

    if not args.skip_training:
        train_final_agents(
            config,
            output_dir,
            training_scenario=args.training_scenario,
            train_seed=args.train_seed,
            episode_counts=episode_counts,
        )
    else:
        plot_tabular_training_comparison(load_training_rows(output_dir), output_dir)

    scenario_summary_rows = []
    verdict_rows = []
    if not args.skip_evaluation:
        scenario_summary_rows, verdict_rows = evaluate_final_controllers(
            config,
            output_dir,
            scenarios=scenarios,
            eval_seeds=eval_seeds,
            eval_episodes=args.eval_episodes,
        )

    if not args.skip_policy_plots:
        behavior_scenarios = scenarios
        generate_policy_behavior_plots(
            config,
            output_dir,
            scenarios=behavior_scenarios,
            eval_seed=eval_seeds[0],
        )

    print("Final experiments completed.")
    print(f"Output directory: {output_dir}")
    if verdict_rows:
        winner = next(
            (row["controller"] for row in verdict_rows if row.get("final_comment")),
            None,
        )
        if winner:
            print(f"Tabular verdict winner: {winner}")
    if scenario_summary_rows:
        print(f"Scenario summary: {output_dir / 'logs' / 'final_scenario_summary.csv'}")


if __name__ == "__main__":
    main()
