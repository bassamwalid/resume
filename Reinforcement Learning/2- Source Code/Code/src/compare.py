"""Compare fixed-time, Q-Learning, SARSA, and DQN controllers."""

from __future__ import annotations

import argparse
from pathlib import Path

from evaluate import evaluate_agent
from utils.plotting import plot_comparison
from utils.saving import ensure_output_dirs, load_config, save_csv, save_json


def comparison_rows(config, output_dir, episodes=None, seed=None, scenario=None):
    rows = []
    for agent_name in ("fixed_time", "q_learning", "sarsa", "dqn"):
        if agent_name in ("q_learning", "sarsa"):
            q_table_path = Path(output_dir) / "q_tables" / f"{agent_name}_q_table.pkl"
            if not q_table_path.exists():
                print(f"Skipping {agent_name}: missing Q-table at {q_table_path}")
                continue
        if agent_name == "dqn":
            model_path = Path(output_dir) / "models" / "dqn_model.pt"
            if not model_path.exists():
                print(f"Skipping {agent_name}: missing DQN model at {model_path}")
                continue

        _, summary = evaluate_agent(
            agent_name,
            config,
            output_dir=output_dir,
            episodes=episodes,
            seed=seed,
            save_outputs=False,
            scenario=scenario,
        )
        rows.append({"agent": agent_name, **summary})
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/default.yaml")
    parser.add_argument("--input-dir", default=None)
    parser.add_argument("--output-dir", default="results")
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--scenario", default=None)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()

    config = load_config(args.config)
    output_dir = ensure_output_dirs(args.output_dir)
    input_dir = Path(args.input_dir) if args.input_dir is not None else output_dir
    rows = comparison_rows(
        config,
        input_dir,
        episodes=args.episodes,
        seed=args.seed,
        scenario=args.scenario,
    )

    if not rows:
        raise RuntimeError("no agents were available to compare")

    comparison_csv = output_dir / "logs" / "comparison.csv"
    comparison_json = output_dir / "logs" / "comparison_summary.json"
    save_csv(rows, comparison_csv)
    save_json(rows, comparison_json)

    figure_path = None
    if not args.no_plots:
        figure_path = plot_comparison(rows, output_dir)

    print(f"Saved comparison table: {comparison_csv}")
    if figure_path:
        print(f"Saved comparison plot: {figure_path}")
    for row in rows:
        print(
            f"{row['agent']}: "
            f"avg_queue={row['mean_average_queue_length']:.2f}, "
            f"avg_wait={row['mean_average_waiting_time']:.2f}, "
            f"throughput={row['mean_total_throughput']:.2f}, "
            f"switches={row['mean_phase_switches']:.2f}"
        )


if __name__ == "__main__":
    main()
