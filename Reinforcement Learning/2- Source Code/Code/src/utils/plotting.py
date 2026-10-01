"""Plotting utilities for experiment outputs."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import pandas as pd


TRAINING_PLOT_METRICS = [
    (
        "cumulative_reward",
        "Episode reward",
        "Episode reward",
    ),
    (
        "average_queue_length",
        "Average queue length",
        "Average queue length (vehicles)",
    ),
    (
        "average_waiting_time",
        "Average waiting time",
        "Average waiting time (vehicle-timesteps)",
    ),
    (
        "total_throughput",
        "Total throughput",
        "Total throughput (vehicles cleared per episode)",
    ),
    (
        "phase_switches",
        "Phase switches",
        "Number of phase switches per episode",
    ),
]


def training_smoothing_window(agent_name):
    """Return the rolling-average window for a training plot."""
    return 200 if str(agent_name).lower() == "dqn" else 500


def numeric_column(rows, metric):
    dataframe = pd.DataFrame(rows)
    return pd.to_numeric(dataframe[metric], errors="coerce")


def rolling_mean_values(rows, metric, agent_name):
    """Return a pandas rolling mean for one plotted training metric."""
    window = training_smoothing_window(agent_name)
    return (
        numeric_column(rows, metric)
        .rolling(window=window, min_periods=1)
        .mean()
        .tolist()
    )


def plot_training_curves(rows, output_dir, agent_name):
    """Save smoothed training curves without changing the raw metric rows."""
    if not rows:
        return None

    output_path = Path(output_dir) / "figures"
    output_path.mkdir(parents=True, exist_ok=True)

    episodes = numeric_column(rows, "episode").tolist()
    window = training_smoothing_window(agent_name)

    fig, axes = plt.subplots(3, 2, figsize=(13, 10))
    flat_axes = axes.ravel()
    for axis, (metric, title, ylabel) in zip(flat_axes, TRAINING_PLOT_METRICS):
        axis.plot(
            episodes,
            rolling_mean_values(rows, metric, agent_name),
            linewidth=1.6,
        )
        axis.set_title(f"{title} rolling mean")
        axis.set_xlabel("Episode")
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.3)
    flat_axes[-1].axis("off")

    fig.suptitle(f"{agent_name} training metrics (rolling mean, window={window})")
    fig.tight_layout()
    path = output_path / f"{agent_name}_training_curves.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def plot_comparison(rows, output_dir):
    """Save bar charts comparing controllers."""
    if not rows:
        return None

    output_path = Path(output_dir) / "figures"
    output_path.mkdir(parents=True, exist_ok=True)

    agents = [row["agent"] for row in rows]
    plots = [
        ("mean_average_queue_length", "Average queue length"),
        ("mean_average_waiting_time", "Average waiting time"),
        ("mean_total_throughput", "Total throughput"),
        ("mean_phase_switches", "Phase switches"),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for axis, (metric, title) in zip(axes.ravel(), plots):
        axis.bar(agents, [row[metric] for row in rows])
        axis.set_title(title)
        axis.tick_params(axis="x", labelrotation=20)
        axis.grid(True, axis="y", alpha=0.3)

    fig.suptitle("Controller comparison")
    fig.tight_layout()
    path = output_path / "comparison_bars.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path
