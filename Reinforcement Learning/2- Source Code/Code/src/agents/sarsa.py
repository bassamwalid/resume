"""Tabular SARSA agent."""

from collections import defaultdict

import numpy as np


class SarsaAgent:
    """On-policy tabular SARSA with epsilon-greedy exploration."""

    def __init__(
        self,
        n_actions=4,
        learning_rate=0.1,
        discount_factor=0.95,
        epsilon_start=1.0,
        epsilon_decay=0.995,
        epsilon_min=0.01,
        seed=None,
    ):
        self.n_actions = int(n_actions)
        self.learning_rate = float(learning_rate)
        self.discount_factor = float(discount_factor)
        self.epsilon = float(epsilon_start)
        self.epsilon_decay = float(epsilon_decay)
        self.epsilon_min = float(epsilon_min)
        self.rng = np.random.default_rng(seed)
        self.q_table = defaultdict(self._zero_action_values)

    def _zero_action_values(self):
        return np.zeros(self.n_actions, dtype=float)

    def select_action(self, state, epsilon=None):
        epsilon = self.epsilon if epsilon is None else float(epsilon)
        if self.rng.random() < epsilon:
            return int(self.rng.integers(self.n_actions))
        return self.greedy_action(state)

    def greedy_action(self, state):
        values = self.q_table[tuple(state)]
        best_value = np.max(values)
        best_actions = np.flatnonzero(np.isclose(values, best_value))
        return int(self.rng.choice(best_actions))

    def update(self, state, action, reward, next_state, next_action, done):
        state = tuple(state)
        next_state = tuple(next_state)
        action = int(action)
        next_action = int(next_action)

        current_value = self.q_table[state][action]
        next_value = 0.0 if done else float(self.q_table[next_state][next_action])
        target = reward + self.discount_factor * next_value
        self.q_table[state][action] = current_value + self.learning_rate * (target - current_value)

    def decay_epsilon(self):
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)

    def set_q_table(self, q_table):
        self.q_table = defaultdict(self._zero_action_values)
        for state, values in q_table.items():
            self.q_table[tuple(state)] = np.asarray(values, dtype=float)

    def get_q_table(self):
        return {tuple(state): np.asarray(values, dtype=float) for state, values in self.q_table.items()}

