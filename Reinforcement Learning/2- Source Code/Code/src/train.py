"""Train traffic signal control agents."""

from __future__ import annotations

import argparse

import numpy as np

from agents.q_learning import QLearningAgent
from agents.sarsa import SarsaAgent
from env.continuous_traffic_env import ContinuousTrafficEnv
from env.traffic_env import (
    MIXED_SCENARIOS,
    TrafficEnv,
    config_with_traffic_scenario,
    parse_scenario_names,
)
from utils.metrics import summarize_episode
from utils.plotting import plot_training_curves
from utils.saving import ensure_output_dirs, load_config, save_csv, save_json, save_q_table

try:
    from tqdm import trange
except ImportError:  # pragma: no cover
    def trange(*args, **kwargs):
        return range(*args)


def build_agent(agent_name, config, seed=None):
    training = config.get("training", {})
    if agent_name == "dqn":
        from extensions.dqn_agent import DQNAgent

        return DQNAgent.from_config(
            state_dim=ContinuousTrafficEnv.state_dim,
            n_actions=TrafficEnv.n_actions,
            config=config,
            seed=seed,
        )

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
    raise ValueError(f"unsupported training agent: {agent_name}")


def train_agent(agent_name, config, episodes=None, seed=None, scenario=None):
    training = config.get("training", {})
    dqn_config = config.get("dqn", {})
    default_episodes = dqn_config.get("episodes", training.get("episodes", 1000)) if agent_name == "dqn" else training.get("episodes", 1000)
    episodes = int(episodes if episodes is not None else default_episodes)
    seed = int(seed if seed is not None else training.get("seed", 42))
    traffic = config.get("traffic", {})
    scenario = str(scenario or traffic.get("scenario", "uniform_medium"))
    config = config_with_traffic_scenario(config, scenario)
    training_scenarios = parse_scenario_names(
        traffic.get("training_scenarios", ",".join(MIXED_SCENARIOS))
    )
    scenario_rng = np.random.default_rng(seed)

    env_class = ContinuousTrafficEnv if agent_name == "dqn" else TrafficEnv
    env = env_class.from_config(config, seed=seed)
    agent = build_agent(agent_name, config, seed=seed)
    rows = []

    iterator = trange(episodes, desc=f"Training {agent_name}")
    for episode_index in iterator:
        episode_seed = seed + episode_index
        episode_scenario = scenario
        if scenario == "mixed_scenarios":
            episode_scenario = str(scenario_rng.choice(training_scenarios))
        env.set_traffic_scenario(episode_scenario)
        state = env.reset(seed=episode_seed)
        done = False
        cumulative_reward = 0.0
        step_infos = []

        losses = []

        if agent_name == "sarsa":
            action = agent.select_action(state)

        while not done:
            if agent_name == "q_learning":
                action = agent.select_action(state)
                next_state, reward, done, info = env.step(action)
                agent.update(state, action, reward, next_state, done)
            elif agent_name == "sarsa":
                next_state, reward, done, info = env.step(action)
                next_action = agent.select_action(next_state) if not done else 0
                agent.update(state, action, reward, next_state, next_action, done)
                action = next_action
            else:
                action = agent.select_action(state)
                next_state, reward, done, info = env.step(action)
                loss = agent.update(state, action, reward, next_state, done)
                if loss is not None:
                    losses.append(loss)

            cumulative_reward += reward
            step_infos.append(info)
            state = next_state

        row = summarize_episode(
            step_infos,
            cumulative_reward,
            episode=episode_index + 1,
            epsilon=agent.epsilon,
        )
        if agent_name == "dqn":
            row["mean_loss"] = float(sum(losses) / len(losses)) if losses else 0.0
            row["replay_size"] = len(agent.replay_buffer)
        row["scenario"] = episode_scenario
        row["episode_seed"] = int(episode_seed)
        rows.append(row)
        agent.decay_epsilon()

    return agent, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--agent", choices=["q_learning", "sarsa", "dqn"], required=True)
    parser.add_argument("--config", default="configs/default.yaml")
    parser.add_argument("--output-dir", default="results")
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument(
        "--scenario",
        default=None,
        help="Traffic scenario to train on, or mixed_scenarios for per-episode sampling.",
    )
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()

    config = load_config(args.config)
    output_dir = ensure_output_dirs(args.output_dir)
    agent, rows = train_agent(
        args.agent,
        config,
        episodes=args.episodes,
        seed=args.seed,
        scenario=args.scenario,
    )

    metrics_path = output_dir / "logs" / f"{args.agent}_training_metrics.csv"
    q_table_path = output_dir / "q_tables" / f"{args.agent}_q_table.pkl"
    model_path = output_dir / "models" / f"{args.agent}_model.pt"
    config_path = output_dir / "logs" / f"{args.agent}_training_config.json"

    save_csv(rows, metrics_path)
    if args.agent == "dqn":
        agent.save(model_path)
    else:
        save_q_table(agent.get_q_table(), q_table_path)
    save_json(config, config_path)

    figure_path = None
    if not args.no_plots:
        figure_path = plot_training_curves(rows, output_dir, args.agent)

    final = rows[-1] if rows else {}
    print(f"Saved metrics: {metrics_path}")
    if args.agent == "dqn":
        print(f"Saved model: {model_path}")
    else:
        print(f"Saved Q-table: {q_table_path}")
    if figure_path:
        print(f"Saved training plot: {figure_path}")
    if final:
        print(
            "Final episode: "
            f"reward={final['cumulative_reward']:.2f}, "
            f"avg_queue={final['average_queue_length']:.2f}, "
            f"throughput={final['total_throughput']}"
        )


if __name__ == "__main__":
    main()
