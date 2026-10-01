"""Create a simple traffic signal animation for a trained/evaluated policy."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np

from env.continuous_traffic_env import ContinuousTrafficEnv
from env.state_encoder import DIRECTIONS
from env.traffic_env import TrafficEnv, config_with_traffic_scenario
from evaluate import build_evaluation_agent
from utils.saving import ensure_output_dirs, load_config, save_csv

try:
    import imageio.v2 as imageio
except ImportError:  # pragma: no cover
    imageio = None

try:
    from PIL import Image
except ImportError:  # pragma: no cover
    Image = None


PHASE_COLORS = {
    "green": "#2e7d32",
    "yellow": "#f9a825",
    "red": "#b71c1c",
}

ROAD_COLOR = "#3f454a"
ROAD_EDGE = "#2f3438"
LANE_MARKING = "#e7ecef"
BACKGROUND = "#dfe9df"
CENTER_COLOR = "#596168"
VEHICLE_COLORS = {
    "north": "#1f77b4",
    "south": "#ff7f0e",
    "east": "#2ca02c",
    "west": "#9467bd",
}
QUEUE_BAR_COLOR = "#1565c0"
QUEUE_BAR_BG = "#d8dee3"
TEXT_COLOR = "#1f2933"
MAX_VISIBLE_VEHICLES = 12


def run_policy_trace(agent_name, config, output_dir, max_steps=None, seed=None, scenario=None):
    """Run one episode and collect per-step data for visualization."""
    visualization = config.get("visualization", {})
    training = config.get("training", {})
    max_steps = int(max_steps if max_steps is not None else visualization.get("max_steps", 160))
    seed = int(seed if seed is not None else training.get("seed", 42) + 20000)
    if scenario is not None:
        config = config_with_traffic_scenario(config, scenario)

    env_class = ContinuousTrafficEnv if agent_name == "dqn" else TrafficEnv
    env = env_class.from_config(config, seed=seed)
    agent = build_evaluation_agent(
        agent_name,
        config,
        output_dir,
        seed=seed,
        state_dim=getattr(env, "state_dim", None),
    )

    state = env.reset(seed=seed)
    if hasattr(agent, "reset"):
        agent.reset()

    records = []
    info = env.get_info()
    done = False
    cumulative_reward = 0.0
    cumulative_throughput = 0

    while not done and len(records) < max_steps:
        if agent_name == "fixed_time":
            action = agent.select_action(state, timestep=env.timestep, info=info)
        else:
            action = agent.select_action(state, epsilon=0.0)

        next_state, reward, done, info = env.step(action)
        cumulative_reward += float(reward)
        cumulative_throughput += int(info["throughput"])
        row = {
            "timestep": info["timestep"],
            "agent": agent_name,
            "scenario": config.get("traffic", {}).get("scenario", "custom"),
            "action": int(action),
            "reward": float(reward),
            "cumulative_reward": float(cumulative_reward),
            "current_phase": info["current_phase"],
            "current_direction": info["current_direction"],
            "phase_timer": info["phase_timer"],
            "is_yellow": info["is_yellow"],
            "departed": info["departed"],
            "total_queue": info["total_queue"],
            "throughput": info["throughput"],
            "cumulative_throughput": int(cumulative_throughput),
            "waiting_time": info["waiting_time"],
            "phase_switch": info["phase_switch"],
            "switch_count": info["switch_count"],
            "overflow": info["overflow"],
        }
        for direction in DIRECTIONS:
            row[f"{direction}_queue"] = info["queue_lengths"][direction]
        records.append(row)
        state = next_state

    return records, env.max_queue


def render_frame(record, max_queue, agent_name):
    """Render one traffic state into an RGB image array."""
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-11, 11)
    ax.set_ylim(-11, 11)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_facecolor(BACKGROUND)

    draw_intersection(ax, record)
    draw_vehicles(ax, record)
    draw_queue_bars(ax, record, max_queue)
    draw_signal_lights(ax, record)
    draw_metrics_overlay(ax, record, agent_name)

    fig.canvas.draw()
    frame = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
    plt.close(fig)
    return frame


def draw_intersection(ax, record):
    """Draw road lanes, stop lines, and active-phase highlights."""
    current_phase = int(record["current_phase"])
    is_yellow = bool(record["is_yellow"])
    active_color = PHASE_COLORS["yellow"] if is_yellow else PHASE_COLORS["green"]

    ax.add_patch(Rectangle((-2.7, -11), 5.4, 22, color=ROAD_COLOR, ec=ROAD_EDGE, zorder=0))
    ax.add_patch(Rectangle((-11, -2.7), 22, 5.4, color=ROAD_COLOR, ec=ROAD_EDGE, zorder=0))
    ax.add_patch(Rectangle((-2.7, -2.7), 5.4, 5.4, color=CENTER_COLOR, ec=ROAD_EDGE, zorder=1))

    for start in (-10, -6, -2, 2, 6):
        ax.plot([0, 0], [start, start + 2.0], color=LANE_MARKING, lw=1.0, alpha=0.7, zorder=2)
        ax.plot([start, start + 2.0], [0, 0], color=LANE_MARKING, lw=1.0, alpha=0.7, zorder=2)

    stop_lines = {
        0: ((-2.45, 2.9), (2.45, 2.9)),
        1: ((-2.45, -2.9), (2.45, -2.9)),
        2: ((2.9, -2.45), (2.9, 2.45)),
        3: ((-2.9, -2.45), (-2.9, 2.45)),
    }
    for phase_id, ((x1, y1), (x2, y2)) in stop_lines.items():
        color = active_color if phase_id == current_phase else "#f5f7fa"
        width = 4.0 if phase_id == current_phase else 2.0
        ax.plot([x1, x2], [y1, y2], color=color, lw=width, solid_capstyle="round", zorder=5)

    ax.text(0, 10.25, "N", ha="center", va="center", fontsize=12, weight="bold", color=TEXT_COLOR)
    ax.text(0, -10.25, "S", ha="center", va="center", fontsize=12, weight="bold", color=TEXT_COLOR)
    ax.text(10.25, 0, "E", ha="center", va="center", fontsize=12, weight="bold", color=TEXT_COLOR)
    ax.text(-10.25, 0, "W", ha="center", va="center", fontsize=12, weight="bold", color=TEXT_COLOR)


def draw_vehicles(ax, record):
    """Draw queued vehicle blocks in each incoming lane."""
    for direction in DIRECTIONS:
        queue = int(record[f"{direction}_queue"])
        visible = min(queue, MAX_VISIBLE_VEHICLES)
        for index in range(visible):
            x_pos, y_pos, width, height = vehicle_geometry(direction, index)
            ax.add_patch(
                Rectangle(
                    (x_pos, y_pos),
                    width,
                    height,
                    color=VEHICLE_COLORS[direction],
                    ec="#17202a",
                    linewidth=0.45,
                    zorder=4,
                )
            )
        if queue > MAX_VISIBLE_VEHICLES:
            label_x, label_y = overflow_label_position(direction)
            ax.text(
                label_x,
                label_y,
                f"+{queue - MAX_VISIBLE_VEHICLES}",
                ha="center",
                va="center",
                fontsize=9,
                weight="bold",
                color=TEXT_COLOR,
                bbox={"boxstyle": "round,pad=0.18", "fc": "white", "ec": "none", "alpha": 0.85},
                zorder=6,
            )


def vehicle_geometry(direction, index):
    gap = 0.12
    if direction == "north":
        return (-1.8, 3.25 + index * (0.48 + gap), 0.75, 0.48)
    if direction == "south":
        return (1.05, -3.73 - index * (0.48 + gap), 0.75, 0.48)
    if direction == "east":
        return (3.25 + index * (0.48 + gap), 1.05, 0.48, 0.75)
    return (-3.73 - index * (0.48 + gap), -1.8, 0.48, 0.75)


def overflow_label_position(direction):
    positions = {
        "north": (-1.42, 10.55),
        "south": (1.42, -10.55),
        "east": (10.55, 1.42),
        "west": (-10.55, -1.42),
    }
    return positions[direction]


def draw_queue_bars(ax, record, max_queue):
    max_queue = max(1, int(max_queue))
    scale = 5.8
    edge_color = "#0d47a1"
    label_color = TEXT_COLOR

    queues = {direction: record[f"{direction}_queue"] for direction in DIRECTIONS}
    lengths = {direction: min(scale, scale * queues[direction] / max_queue) for direction in DIRECTIONS}

    ax.add_patch(Rectangle((3.5, 3.1), 0.45, scale, color=QUEUE_BAR_BG, ec="#95a0aa", zorder=2))
    ax.add_patch(Rectangle((3.5, 3.1), 0.45, lengths["north"], color=QUEUE_BAR_COLOR, ec=edge_color, zorder=3))
    ax.text(4.3, 8.9, f"N {queues['north']}", color=label_color, fontsize=9, va="center")

    ax.add_patch(
        Rectangle((-4.0, -8.9), 0.45, scale, color=QUEUE_BAR_BG, ec="#95a0aa", zorder=2)
    )
    ax.add_patch(
        Rectangle((-4.0, -3.1 - lengths["south"]), 0.45, lengths["south"], color=QUEUE_BAR_COLOR, ec=edge_color, zorder=3)
    )
    ax.text(-5.0, -8.9, f"S {queues['south']}", color=label_color, fontsize=9, va="center")

    ax.add_patch(Rectangle((3.1, -4.0), scale, 0.45, color=QUEUE_BAR_BG, ec="#95a0aa", zorder=2))
    ax.add_patch(Rectangle((3.1, -4.0), lengths["east"], 0.45, color=QUEUE_BAR_COLOR, ec=edge_color, zorder=3))
    ax.text(8.9, -5.0, f"E {queues['east']}", color=label_color, fontsize=9, ha="center")

    ax.add_patch(
        Rectangle((-8.9, 3.55), scale, 0.45, color=QUEUE_BAR_BG, ec="#95a0aa", zorder=2)
    )
    ax.add_patch(
        Rectangle((-3.1 - lengths["west"], 3.55), lengths["west"], 0.45, color=QUEUE_BAR_COLOR, ec=edge_color, zorder=3)
    )
    ax.text(-8.9, 4.6, f"W {queues['west']}", color=label_color, fontsize=9, ha="center")


def draw_signal_lights(ax, record):
    phase = int(record["current_phase"])
    is_yellow = bool(record["is_yellow"])
    signal_positions = {
        0: (2.25, 3.45),
        1: (-2.25, -3.45),
        2: (3.45, -2.25),
        3: (-3.45, 2.25),
    }

    for phase_id, (x_pos, y_pos) in signal_positions.items():
        if phase_id == phase:
            color = PHASE_COLORS["yellow"] if is_yellow else PHASE_COLORS["green"]
        else:
            color = PHASE_COLORS["red"]
        ax.add_patch(Circle((x_pos, y_pos), 0.42, color=color, ec="#202124", linewidth=0.9, zorder=6))
        ax.add_patch(Circle((x_pos, y_pos), 0.58, fill=False, ec="white", linewidth=1.0, zorder=5))


def draw_metrics_overlay(ax, record, agent_name):
    scenario = record.get("scenario", "custom")
    status = "YELLOW" if record["is_yellow"] else "GREEN"
    text = "\n".join(
        [
            f"Controller: {agent_name}",
            f"Scenario: {scenario}",
            f"Timestep: {record['timestep']}",
            f"Phase: {record['current_direction']} ({status})",
            f"Total queue: {record['total_queue']}",
            f"Waiting time: {record['waiting_time']}",
            f"Reward: {record['reward']:.2f}",
            f"Throughput: {record['cumulative_throughput']}",
            f"Switches: {record['switch_count']}",
        ]
    )
    ax.text(
        -10.6,
        10.6,
        text,
        ha="left",
        va="top",
        fontsize=9.5,
        color=TEXT_COLOR,
        bbox={
            "boxstyle": "round,pad=0.45",
            "fc": "white",
            "ec": "#c7d0d8",
            "alpha": 0.92,
        },
        zorder=10,
    )


def save_visualization(records, max_queue, output_dir, agent_name, fps=6, save_gif=True, save_mp4=True):
    """Save trace CSV, final snapshot, GIF, and MP4 when supported."""
    output_dir = ensure_output_dirs(output_dir)
    trace_path = output_dir / "logs" / f"{agent_name}_visualization_trace.csv"
    save_csv(records, trace_path)

    if not records:
        return {
            "trace": trace_path,
            "snapshot": None,
            "gif": None,
            "mp4": None,
        }

    frames = [render_frame(record, max_queue, agent_name) for record in records]
    snapshot_path = output_dir / "figures" / f"{agent_name}_better_snapshot.png"
    plt.imsave(snapshot_path, frames[-1])

    gif_path = None
    if save_gif and imageio is not None:
        gif_path = output_dir / "animations" / f"{agent_name}_better_animation.gif"
        imageio.mimsave(gif_path, frames, duration=1.0 / max(1, int(fps)))
    elif save_gif and Image is not None:
        gif_path = output_dir / "animations" / f"{agent_name}_better_animation.gif"
        images = [Image.fromarray(frame) for frame in frames]
        images[0].save(
            gif_path,
            save_all=True,
            append_images=images[1:],
            duration=int(1000 / max(1, int(fps))),
            loop=0,
        )

    mp4_path = None
    if save_mp4 and imageio is not None:
        try:
            mp4_path = output_dir / "animations" / f"{agent_name}_better_animation.mp4"
            imageio.mimsave(mp4_path, frames, fps=max(1, int(fps)))
        except Exception:
            mp4_path = None

    return {
        "trace": trace_path,
        "snapshot": snapshot_path,
        "gif": gif_path,
        "mp4": mp4_path,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--agent", choices=["fixed_time", "q_learning", "sarsa", "dqn"], default=None)
    parser.add_argument("--all-agents", action="store_true")
    parser.add_argument("--config", default="configs/default.yaml")
    parser.add_argument("--output-dir", default="results")
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--fps", type=int, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--scenario", default=None)
    parser.add_argument("--no-gif", action="store_true")
    parser.add_argument("--no-mp4", action="store_true")
    args = parser.parse_args()

    config = load_config(args.config)
    visualization = config.get("visualization", {})
    agent_name = args.agent or visualization.get("agent", "fixed_time")
    fps = int(args.fps if args.fps is not None else visualization.get("fps", 6))
    output_dir = ensure_output_dirs(args.output_dir)

    agent_names = ["fixed_time", "q_learning", "sarsa", "dqn"] if args.all_agents else [agent_name]
    for selected_agent in agent_names:
        records, max_queue = run_policy_trace(
            selected_agent,
            config,
            output_dir=output_dir,
            max_steps=args.max_steps,
            seed=args.seed,
            scenario=args.scenario,
        )
        paths = save_visualization(
            records,
            max_queue,
            output_dir=output_dir,
            agent_name=selected_agent,
            fps=fps,
            save_gif=not args.no_gif,
            save_mp4=not args.no_mp4,
        )

        print(f"Saved visualization trace: {paths['trace']}")
        if paths["snapshot"]:
            print(f"Saved final snapshot: {paths['snapshot']}")
        if paths["gif"]:
            print(f"Saved GIF animation: {paths['gif']}")
        elif not args.no_gif:
            print("GIF was not saved because neither imageio nor Pillow is installed.")
        if paths["mp4"]:
            print(f"Saved MP4 animation: {paths['mp4']}")
        elif not args.no_mp4:
            print("MP4 was not saved because the required video writer is unavailable.")


if __name__ == "__main__":
    main()
