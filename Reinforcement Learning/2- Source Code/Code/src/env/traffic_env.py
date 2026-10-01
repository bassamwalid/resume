"""Single-intersection traffic simulation environment."""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict, Optional

import numpy as np

from .state_encoder import DIRECTIONS, encode_state, phase_name


SCENARIO_ARRIVAL_RATES = {
    "low_uniform": {
        "north": 0.10,
        "south": 0.10,
        "east": 0.10,
        "west": 0.10,
    },
    "uniform_medium": {
        "north": 0.30,
        "south": 0.30,
        "east": 0.30,
        "west": 0.30,
    },
    "high_uniform": {
        "north": 0.55,
        "south": 0.55,
        "east": 0.55,
        "west": 0.55,
    },
    "uneven": {
        "north": 0.50,
        "south": 0.50,
        "east": 0.20,
        "west": 0.20,
    },
}

MIXED_SCENARIOS = (
    "low_uniform",
    "uniform_medium",
    "high_uniform",
    "uneven",
    "rush_hour",
)


def parse_scenario_names(value):
    """Parse scenario lists from YAML values or comma-separated CLI strings."""
    if value is None:
        return list(MIXED_SCENARIOS)
    if isinstance(value, str):
        names = [part.strip() for part in value.split(",") if part.strip()]
    else:
        names = [str(part).strip() for part in value if str(part).strip()]
    return names or list(MIXED_SCENARIOS)


def scenario_arrival_rates(scenario, fallback=None):
    """Return base arrival rates for a named scenario."""
    scenario = str(scenario or "custom")
    if scenario in SCENARIO_ARRIVAL_RATES:
        return dict(SCENARIO_ARRIVAL_RATES[scenario])
    if scenario in {"rush_hour", "mixed_scenarios"}:
        return dict(SCENARIO_ARRIVAL_RATES["uniform_medium"])
    if fallback is not None:
        return {direction: float(fallback[direction]) for direction in DIRECTIONS}
    return dict(SCENARIO_ARRIVAL_RATES["uniform_medium"])


def config_with_traffic_scenario(config, scenario):
    """Copy a config and replace its traffic scenario and base arrival rates."""
    updated = deepcopy(config)
    traffic = updated.setdefault("traffic", {})
    traffic["scenario"] = str(scenario)
    traffic["arrival_rates"] = scenario_arrival_rates(
        scenario,
        fallback=traffic.get("arrival_rates"),
    )
    return updated


def compute_base_reward(
    reward_type,
    queue_delta,
    total_waiting_time,
    throughput,
    total_queue=0,
    max_red_queue=0,
    switched=0,
    w1=1.0,
    w2=0.1,
    w3=1.0,
):
    """Compute one of the Stage A base reward functions."""
    reward_type = str(reward_type).upper()
    if reward_type == "R1":
        return -queue_delta
    if reward_type == "R2":
        return -total_waiting_time
    if reward_type == "R3":
        return throughput
    if reward_type == "R4":
        return -queue_delta - w2 * total_waiting_time
    if reward_type == "R5":
        return -queue_delta + w3 * throughput
    if reward_type == "R6":
        return -queue_delta - total_waiting_time + throughput
    if reward_type == "R7":
        return -w1 * queue_delta - w2 * total_waiting_time + w3 * throughput
    if reward_type == "R8":
        return -0.5 * total_queue + 2.0 * throughput - max_red_queue - int(bool(switched))
    raise ValueError(f"Unknown base reward type: {reward_type}")


class TrafficEnv:
    """Discrete-time traffic environment for one four-way intersection."""

    n_actions = 4

    def __init__(
        self,
        max_timesteps: int = 500,
        max_queue: int = 40,
        yellow_duration: int = 2,
        service_rate: int = 2,
        arrival_rates: Optional[Dict[str, float]] = None,
        reward_config: Optional[Dict[str, Any]] = None,
        traffic_scenario: str = "custom",
        seed: Optional[int] = None,
    ):
        self.max_timesteps = int(max_timesteps)
        self.max_queue = int(max_queue)
        self.yellow_duration = int(yellow_duration)
        self.service_rate = int(service_rate)
        self.traffic_scenario = traffic_scenario

        self.set_traffic_scenario(traffic_scenario, arrival_rates=arrival_rates)

        reward_config = reward_config or {}
        self.reward_config = reward_config
        self.reward_type = reward_config.get("type")
        self.reward_w1 = float(reward_config.get("w1", 1.0))
        self.reward_w2 = float(reward_config.get("w2", 0.1))
        self.reward_w3 = float(reward_config.get("w3", 1.0))
        self.reward_w4 = float(reward_config.get("w4", 0.0))
        self.base_reward_type = reward_config.get("base_reward_type", "R1")
        self.alpha_reward = float(reward_config.get("alpha_reward", 1.0))
        self.beta_reward = float(reward_config.get("beta_reward", 1.0))
        self.throughput_weight = float(reward_config.get("throughput_weight", 2.0))
        self.use_composite_reward = bool(reward_config.get("use_composite_reward", False))

        self.rng = np.random.default_rng(seed)
        self.reset()

    @classmethod
    def from_config(cls, config: Dict[str, Any], seed: Optional[int] = None) -> "TrafficEnv":
        """Create an environment from the project YAML configuration."""
        environment = config.get("environment", {})
        traffic = config.get("traffic", {})
        reward = config.get("reward", {})
        return cls(
            max_timesteps=environment.get("max_timesteps", 500),
            max_queue=environment.get("max_queue", 40),
            yellow_duration=environment.get("yellow_duration", 2),
            service_rate=environment.get("service_rate", 2),
            arrival_rates=traffic.get("arrival_rates"),
            reward_config=reward,
            traffic_scenario=traffic.get("scenario", "custom"),
            seed=seed,
        )

    def reset(self, seed: Optional[int] = None):
        """Reset the environment and return the initial encoded state."""
        if seed is not None:
            self.rng = np.random.default_rng(seed)

        self.queues = np.zeros(self.n_actions, dtype=int)
        self.current_phase = 0
        self.pending_phase = None
        self.yellow_remaining = 0
        self.phase_timer = 0
        self.timestep = 0

        self.switch_count = 0
        self.total_throughput = 0
        self.total_waiting_time = 0.0
        self.overflow_count = 0

        return self._get_state()

    def step(self, action: int):
        """Advance the simulation by one timestep."""
        action = int(action)
        if action < 0 or action >= self.n_actions:
            raise ValueError(f"action must be in [0, {self.n_actions - 1}], got {action}")

        previous_total_queue = int(self.queues.sum())
        arrivals = self.rng.poisson(self._arrival_rates_for_timestep()).astype(int)
        self.queues += arrivals

        waiting_time = int(self.queues.sum())
        departed = 0
        switched_this_step = False
        yellow_active_this_step = False

        if self.yellow_remaining > 0:
            yellow_active_this_step = True
            self._advance_yellow()
        elif action != self.current_phase:
            switched_this_step = True
            self.switch_count += 1
            self.pending_phase = action

            if self.yellow_duration > 0:
                yellow_active_this_step = True
                self.yellow_remaining = self.yellow_duration
                self._advance_yellow()
            else:
                self.current_phase = action
                self.phase_timer = 0
                departed = self._serve_current_phase()
                self.phase_timer += 1
        else:
            departed = self._serve_current_phase()
            self.phase_timer += 1

        total_queue = int(self.queues.sum())
        queue_delta = total_queue - previous_total_queue
        max_red_queue = self._max_red_queue()
        reward = self._calculate_reward(
            queue_delta,
            waiting_time,
            departed,
            switched_this_step,
            total_queue=total_queue,
            max_red_queue=max_red_queue,
        )

        self.timestep += 1
        self.total_throughput += int(departed)
        self.total_waiting_time += float(waiting_time)

        overflow = bool(self.queues.max(initial=0) >= self.max_queue)
        if overflow:
            self.overflow_count += 1
        done = self.timestep >= self.max_timesteps or overflow

        info = {
            "timestep": self.timestep,
            "queues": self.queues.copy(),
            "queue_lengths": self._queue_dict(),
            "arrivals": dict(zip(DIRECTIONS, arrivals.astype(int).tolist())),
            "departed": int(departed),
            "throughput": int(departed),
            "waiting_time": int(waiting_time),
            "total_queue": total_queue,
            "max_red_queue": int(max_red_queue),
            "queue_delta": int(queue_delta),
            "abs_queue_delta": int(abs(queue_delta)),
            "abs_waiting_time": int(abs(waiting_time)),
            "abs_throughput": int(abs(departed)),
            "reward": float(reward),
            "current_phase": int(self.current_phase),
            "current_direction": phase_name(self.current_phase),
            "pending_phase": None if self.pending_phase is None else int(self.pending_phase),
            "phase_timer": int(self.phase_timer),
            "phase_switch": switched_this_step,
            "switched": int(switched_this_step),
            "switch_count": int(self.switch_count),
            "is_yellow": yellow_active_this_step,
            "overflow": overflow,
        }

        return self._get_state(), reward, done, info

    def get_info(self):
        """Return a lightweight snapshot of the current environment status."""
        return {
            "timestep": self.timestep,
            "queue_lengths": self._queue_dict(),
            "current_phase": int(self.current_phase),
            "current_direction": phase_name(self.current_phase),
            "pending_phase": None if self.pending_phase is None else int(self.pending_phase),
            "phase_timer": int(self.phase_timer),
            "switch_count": int(self.switch_count),
            "is_yellow": self.yellow_remaining > 0,
            "overflow": bool(self.queues.max(initial=0) >= self.max_queue),
        }

    def _get_state(self):
        return encode_state(self.queues, self.current_phase, self.phase_timer)

    def _queue_dict(self):
        return {direction: int(self.queues[index]) for index, direction in enumerate(DIRECTIONS)}

    def set_traffic_scenario(self, traffic_scenario, arrival_rates=None):
        """Switch the traffic demand scenario used by future steps."""
        self.traffic_scenario = str(traffic_scenario or "custom")
        rates = scenario_arrival_rates(self.traffic_scenario, fallback=arrival_rates)
        self.arrival_rates = np.array(
            [float(rates[direction]) for direction in DIRECTIONS],
            dtype=float,
        )

    def _arrival_rates_for_timestep(self):
        if self.traffic_scenario == "rush_hour":
            first_cutoff = self.max_timesteps / 3.0
            second_cutoff = 2.0 * self.max_timesteps / 3.0
            if self.timestep < first_cutoff:
                rates = (0.55, 0.55, 0.20, 0.20)
            elif self.timestep < second_cutoff:
                rates = (0.30, 0.30, 0.30, 0.30)
            else:
                rates = (0.20, 0.20, 0.55, 0.55)
            return np.array(rates, dtype=float)
        return self.arrival_rates

    def _advance_yellow(self):
        self.yellow_remaining -= 1
        if self.yellow_remaining <= 0 and self.pending_phase is not None:
            self.current_phase = int(self.pending_phase)
            self.pending_phase = None
            self.phase_timer = 0

    def _serve_current_phase(self):
        departed = min(self.service_rate, int(self.queues[self.current_phase]))
        self.queues[self.current_phase] -= departed
        return departed

    def _max_red_queue(self):
        red_queues = [
            int(self.queues[index])
            for index in range(self.n_actions)
            if index != int(self.current_phase)
        ]
        return max(red_queues) if red_queues else 0

    def _calculate_reward(
        self,
        queue_delta,
        waiting_time,
        throughput,
        switched=False,
        total_queue=0,
        max_red_queue=0,
    ):
        if self.reward_type:
            reward_type = str(self.reward_type).upper()
            if reward_type in {"R1", "R2", "R3", "R4", "R5", "R6", "R7", "R8"}:
                return compute_base_reward(
                    reward_type,
                    queue_delta,
                    waiting_time,
                    throughput,
                    total_queue=total_queue,
                    max_red_queue=max_red_queue,
                    switched=switched,
                    w1=self.reward_w1,
                    w2=self.reward_w2,
                    w3=self.reward_w3,
                )

            if reward_type in {"B0", "R9", "R10", "R11", "R12", "R13", "R14"}:
                base_reward = compute_base_reward(
                    self.base_reward_type,
                    queue_delta,
                    waiting_time,
                    throughput,
                    total_queue=total_queue,
                    max_red_queue=max_red_queue,
                    switched=switched,
                    w1=self.reward_w1,
                    w2=self.reward_w2,
                    w3=self.reward_w3,
                )
                return base_reward - self.reward_w4 * int(bool(switched))

            raise ValueError(f"Unknown reward type: {reward_type}")

        if self.use_composite_reward:
            return (
                -self.alpha_reward * queue_delta
                - self.beta_reward * waiting_time
                + self.throughput_weight * throughput
            )
        return -self.beta_reward * waiting_time + self.throughput_weight * throughput
