"""Continuous-observation traffic environment for DQN experiments."""

from __future__ import annotations

from typing import Any, Dict, Optional

import numpy as np

from .traffic_env import TrafficEnv


class ContinuousTrafficEnv(TrafficEnv):
    """TrafficEnv variant that returns normalized continuous state vectors.

    Observation layout:
    - 4 normalized queue lengths
    - 4 one-hot current phase values
    - 4 one-hot pending phase values, all zeros when not switching
    - normalized phase timer
    - normalized yellow timer
    - normalized episode progress
    - 4 normalized arrival-rate values
    """

    state_dim = 19

    @classmethod
    def from_config(
        cls,
        config: Dict[str, Any],
        seed: Optional[int] = None,
    ) -> "ContinuousTrafficEnv":
        """Create a continuous-observation environment from YAML config."""
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

    def _get_state(self):
        queue_scale = max(1.0, float(self.max_queue))
        timer_scale = max(1.0, float(self.max_timesteps))
        yellow_scale = max(1.0, float(self.yellow_duration))
        current_arrival_rates = self._arrival_rates_for_timestep()
        arrival_scale = max(1.0, float(np.max(current_arrival_rates)))

        queues = np.clip(self.queues.astype(np.float32) / queue_scale, 0.0, 1.0)

        current_phase = np.zeros(self.n_actions, dtype=np.float32)
        current_phase[int(self.current_phase)] = 1.0

        pending_phase = np.zeros(self.n_actions, dtype=np.float32)
        if self.pending_phase is not None:
            pending_phase[int(self.pending_phase)] = 1.0

        phase_timer = np.array(
            [min(float(self.phase_timer) / timer_scale, 1.0)],
            dtype=np.float32,
        )
        yellow_timer = np.array(
            [min(float(self.yellow_remaining) / yellow_scale, 1.0)],
            dtype=np.float32,
        )
        progress = np.array(
            [min(float(self.timestep) / timer_scale, 1.0)],
            dtype=np.float32,
        )
        arrivals = np.clip(
            current_arrival_rates.astype(np.float32) / arrival_scale,
            0.0,
            1.0,
        )

        return np.concatenate(
            [
                queues,
                current_phase,
                pending_phase,
                phase_timer,
                yellow_timer,
                progress,
                arrivals,
            ]
        ).astype(np.float32)
