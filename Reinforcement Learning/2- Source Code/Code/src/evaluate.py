"""Evaluate trained traffic signal controllers."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from agents.fixed_time import FixedTimeAgent
from agents.q_learning import QLearningAgent
from agents.sarsa import SarsaAgent
from env.continuous_traffic_env import ContinuousTrafficEnv
from env.traffic_env import (
    MIXED_SCENARIOS,
    TrafficEnv,
    config_with_traffic_scenario,
    parse_scenario_names,
)
from utils.metrics import aggregate_episode_metrics, summarize_episode
from utils.saving import ensure_output_dirs, load_config, load_q_table, save_csv, save_json

try:
    from tqdm import trange
except ImportError:  # pragma: no cover
    def trange(*args, **kwargs):
        return range(*args)


def build_evaluation_agent(agent_name, config, output_dir, seed=None, state_dim=None):
    if agent_name == "fixed_time":
        fixed_time = config.get("fixed_time", {})
        return FixedTimeAgent(
            green_duration=fixed_time.get("green_duration", 20),
            n_actions=TrafficEnv.n_actions,
        )

    if agent_name == "dqn":
        from extensions.dqn_agent import DQNAgent

        agent = DQNAgent.from_config(
            state_dim=state_dim or ContinuousTrafficEnv.state_dim,
            n_actions=TrafficEnv.n_actions,
            config=config,
            seed=seed,
        )
        model_path = Path(output_dir) / "models" / "dqn_model.pt"
        if not model_path.exists():
            raise FileNotFoundError(
                f"missing DQN model: {model_path}. Train DQN before evaluation."
            )
        agent.load(model_path)
        return agent

    training = config.get("training", {})
    evaluation = config.get("evaluation", {})
    kwargs = {
        "n_actions": TrafficEnv.n_actions,
        "learning_rate": training.get("learning_rate", 0.1),
        "discount_factor": training.get("discount_factor", 0.95),
        "epsilon_start": evaluation.get("epsilon", 0.0),
        "epsilon_decay": 1.0,
        "epsilon_min": evaluation.get("epsilon", 0.0),
        "seed": seed,
    }

    if agent_name == "q_learning":
        agent = QLearningAgent(**kwargs)
    elif agent_name == "sarsa":
        agent = SarsaAgent(**kwargs)
    else:
        raise ValueError(f"unsupported evaluation agent: {agent_name}")

    q_table_path = Path(output_dir) / "q_tables" / f"{agent_name}_q_table.pkl"
    if not q_table_path.exists():
        raise FileNotFoundError(
            f"missing Q-table: {q_table_path}. Train the agent before evaluation."
        )
    agent.set_q_table(load_q_table(q_table_path))
    return agent


def evaluate_agent(
    agent_name,
    config,
    output_dir="results",
    episodes=None,
    seed=None,
    save_outputs=True,
    scenario=None,
    output_label=None,
):
    output_dir = ensure_output_dirs(output_dir)
    requested_seed = seed
    requested_scenario = scenario
    training = config.get("training", {})
    evaluation = config.get("evaluation", {})
    episodes = int(episodes if episodes is not None else evaluation.get("episodes", 50))
    seed = int(seed if seed is not None else training.get("seed", 42) + 10000)
    epsilon = float(evaluation.get("epsilon", 0.0))
    traffic = config.get("traffic", {})
    scenario = str(scenario or traffic.get("scenario", "uniform_medium"))
    config = config_with_traffic_scenario(config, scenario)
    evaluation_scenarios = parse_scenario_names(
        traffic.get("training_scenarios", ",".join(MIXED_SCENARIOS))
    )
    scenario_rng = np.random.default_rng(seed)

    env_class = ContinuousTrafficEnv if agent_name == "dqn" else TrafficEnv
    env = env_class.from_config(config, seed=seed)
    agent = build_evaluation_agent(
        agent_name,
        config,
        output_dir,
        seed=seed,
        state_dim=getattr(env, "state_dim", None),
    )
    rows = []

    iterator = trange(episodes, desc=f"Evaluating {agent_name}")
    for episode_index in iterator:
        episode_seed = seed + episode_index
        episode_scenario = scenario
        if scenario == "mixed_scenarios":
            episode_scenario = str(scenario_rng.choice(evaluation_scenarios))
        env.set_traffic_scenario(episode_scenario)
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
                action = agent.select_action(state, epsilon=epsilon)

            next_state, reward, done, info = env.step(action)
            cumulative_reward += reward
            step_infos.append(info)
            state = next_state

        row = summarize_episode(
            step_infos,
            cumulative_reward,
            episode=episode_index + 1,
            epsilon=epsilon if agent_name != "fixed_time" else None,
        )
        row["scenario"] = episode_scenario
        row["eval_base_seed"] = int(seed)
        row["episode_seed"] = int(episode_seed)
        rows.append(row)

    summary = aggregate_episode_metrics(rows)

    if save_outputs:
        suffix = ""
        if output_label:
            suffix = f"_{output_label}"
        elif requested_scenario is not None or requested_seed is not None:
            suffix = f"_{scenario}_seed{seed}"
        save_csv(rows, output_dir / "logs" / f"{agent_name}{suffix}_evaluation_metrics.csv")
        save_json(summary, output_dir / "logs" / f"{agent_name}{suffix}_evaluation_summary.json")

    return rows, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--agent", choices=["fixed_time", "q_learning", "sarsa", "dqn"], required=True)
    parser.add_argument("--config", default="configs/default.yaml")
    parser.add_argument("--output-dir", default="results")
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--scenario", default=None, help="Traffic scenario to evaluate.")
    args = parser.parse_args()

    config = load_config(args.config)
    _, summary = evaluate_agent(
        args.agent,
        config,
        output_dir=args.output_dir,
        episodes=args.episodes,
        seed=args.seed,
        save_outputs=True,
        scenario=args.scenario,
    )

    print(f"Evaluation summary for {args.agent}:")
    for key, value in summary.items():
        print(f"  {key}: {value}")


if __name__ == "__main__":
    main()
