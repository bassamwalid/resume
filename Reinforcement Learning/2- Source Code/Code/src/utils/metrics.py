"""Metric helpers for training and evaluation."""

METRIC_FIELDS = [
    "episode",
    "steps",
    "cumulative_reward",
    "average_queue_length",
    "average_waiting_time",
    "total_throughput",
    "phase_switches",
    "overflow_count",
    "epsilon",
]


def summarize_episode(step_infos, cumulative_reward, episode=None, epsilon=None):
    """Convert per-step environment info into one episode-level metric row."""
    steps = len(step_infos)
    if steps == 0:
        return {
            "episode": episode,
            "steps": 0,
            "cumulative_reward": float(cumulative_reward),
            "average_queue_length": 0.0,
            "average_waiting_time": 0.0,
            "total_throughput": 0,
            "phase_switches": 0,
            "overflow_count": 0,
            "epsilon": epsilon,
        }

    return {
        "episode": episode,
        "steps": steps,
        "cumulative_reward": float(cumulative_reward),
        "average_queue_length": mean(info["total_queue"] for info in step_infos),
        "average_waiting_time": mean(info["waiting_time"] for info in step_infos),
        "total_throughput": int(sum(info["throughput"] for info in step_infos)),
        "phase_switches": int(sum(1 for info in step_infos if info["phase_switch"])),
        "overflow_count": int(sum(1 for info in step_infos if info["overflow"])),
        "epsilon": epsilon,
    }


def aggregate_episode_metrics(rows):
    """Average a list of episode metric rows for comparison."""
    if not rows:
        return {
            "episodes": 0,
            "mean_cumulative_reward": 0.0,
            "mean_average_queue_length": 0.0,
            "mean_average_waiting_time": 0.0,
            "mean_total_throughput": 0.0,
            "mean_phase_switches": 0.0,
            "overflow_episodes": 0,
        }

    return {
        "episodes": len(rows),
        "mean_cumulative_reward": mean(row["cumulative_reward"] for row in rows),
        "mean_average_queue_length": mean(row["average_queue_length"] for row in rows),
        "mean_average_waiting_time": mean(row["average_waiting_time"] for row in rows),
        "mean_total_throughput": mean(row["total_throughput"] for row in rows),
        "mean_phase_switches": mean(row["phase_switches"] for row in rows),
        "overflow_episodes": int(sum(1 for row in rows if row["overflow_count"] > 0)),
    }


def mean(values):
    values = list(values)
    if not values:
        return 0.0
    return float(sum(values) / len(values))

