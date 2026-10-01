"""Deep Q-Network agent for continuous traffic observations."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import random

import numpy as np

try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    import torch.optim as optim
except ImportError:  # pragma: no cover
    torch = None
    nn = None
    F = None
    optim = None


def require_torch():
    if torch is None:
        raise ImportError(
            "DQN requires PyTorch. Install project dependencies with "
            "`pip install -r requirements.txt`."
        )


class QNetwork(nn.Module if nn is not None else object):
    """Small fully connected Q-network."""

    def __init__(self, state_dim, n_actions, hidden_size=128):
        super().__init__()
        self.layers = nn.Sequential(
            nn.Linear(state_dim, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, n_actions),
        )

    def forward(self, state):
        return self.layers(state)


@dataclass
class ReplayBatch:
    states: object
    actions: object
    rewards: object
    next_states: object
    dones: object


class ReplayBuffer:
    """Fixed-size experience replay buffer."""

    def __init__(self, capacity, seed=None):
        self.capacity = int(capacity)
        self.memory = deque(maxlen=self.capacity)
        self.rng = random.Random(seed)

    def push(self, state, action, reward, next_state, done):
        self.memory.append(
            (
                np.asarray(state, dtype=np.float32),
                int(action),
                float(reward),
                np.asarray(next_state, dtype=np.float32),
                bool(done),
            )
        )

    def sample(self, batch_size, device):
        transitions = self.rng.sample(self.memory, int(batch_size))
        states, actions, rewards, next_states, dones = zip(*transitions)
        return ReplayBatch(
            states=torch.as_tensor(np.asarray(states), dtype=torch.float32, device=device),
            actions=torch.as_tensor(actions, dtype=torch.long, device=device).unsqueeze(1),
            rewards=torch.as_tensor(rewards, dtype=torch.float32, device=device).unsqueeze(1),
            next_states=torch.as_tensor(
                np.asarray(next_states),
                dtype=torch.float32,
                device=device,
            ),
            dones=torch.as_tensor(dones, dtype=torch.float32, device=device).unsqueeze(1),
        )

    def __len__(self):
        return len(self.memory)


class DQNAgent:
    """DQN with replay memory, target network, and epsilon-greedy actions."""

    def __init__(
        self,
        state_dim,
        n_actions=4,
        learning_rate=0.001,
        discount_factor=0.95,
        epsilon_start=1.0,
        epsilon_decay=0.995,
        epsilon_min=0.01,
        replay_capacity=50000,
        batch_size=64,
        hidden_size=128,
        target_update_steps=250,
        warmup_steps=200,
        gradient_clip=5.0,
        seed=None,
        device=None,
    ):
        require_torch()
        self.state_dim = int(state_dim)
        self.n_actions = int(n_actions)
        self.learning_rate = float(learning_rate)
        self.discount_factor = float(discount_factor)
        self.epsilon = float(epsilon_start)
        self.epsilon_decay = float(epsilon_decay)
        self.epsilon_min = float(epsilon_min)
        self.batch_size = int(batch_size)
        self.target_update_steps = int(target_update_steps)
        self.warmup_steps = int(warmup_steps)
        self.gradient_clip = float(gradient_clip)
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))

        self.rng = np.random.default_rng(seed)
        random.seed(seed)
        torch.manual_seed(0 if seed is None else int(seed))

        self.policy_net = QNetwork(self.state_dim, self.n_actions, int(hidden_size)).to(self.device)
        self.target_net = QNetwork(self.state_dim, self.n_actions, int(hidden_size)).to(self.device)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.target_net.eval()

        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=self.learning_rate)
        self.replay_buffer = ReplayBuffer(replay_capacity, seed=seed)
        self.training_steps = 0

    @classmethod
    def from_config(cls, state_dim, n_actions, config, seed=None):
        dqn = config.get("dqn", {})
        training = config.get("training", {})
        return cls(
            state_dim=state_dim,
            n_actions=n_actions,
            learning_rate=dqn.get("learning_rate", 0.001),
            discount_factor=dqn.get("discount_factor", training.get("discount_factor", 0.95)),
            epsilon_start=dqn.get("epsilon_start", training.get("epsilon_start", 1.0)),
            epsilon_decay=dqn.get("epsilon_decay", training.get("epsilon_decay", 0.995)),
            epsilon_min=dqn.get("epsilon_min", training.get("epsilon_min", 0.01)),
            replay_capacity=dqn.get("replay_capacity", 50000),
            batch_size=dqn.get("batch_size", 64),
            hidden_size=dqn.get("hidden_size", 128),
            target_update_steps=dqn.get("target_update_steps", 250),
            warmup_steps=dqn.get("warmup_steps", 200),
            gradient_clip=dqn.get("gradient_clip", 5.0),
            seed=seed,
            device=dqn.get("device", None),
        )

    def select_action(self, state, epsilon=None):
        epsilon = self.epsilon if epsilon is None else float(epsilon)
        if self.rng.random() < epsilon:
            return int(self.rng.integers(self.n_actions))

        self.policy_net.eval()
        with torch.no_grad():
            state_tensor = torch.as_tensor(
                np.asarray(state, dtype=np.float32),
                dtype=torch.float32,
                device=self.device,
            ).unsqueeze(0)
            q_values = self.policy_net(state_tensor)
        self.policy_net.train()
        return int(torch.argmax(q_values, dim=1).item())

    def update(self, state, action, reward, next_state, done):
        self.replay_buffer.push(state, action, reward, next_state, done)
        if len(self.replay_buffer) < max(self.batch_size, self.warmup_steps):
            return None

        batch = self.replay_buffer.sample(self.batch_size, self.device)
        q_values = self.policy_net(batch.states).gather(1, batch.actions)

        with torch.no_grad():
            next_q_values = self.target_net(batch.next_states).max(dim=1, keepdim=True)[0]
            targets = batch.rewards + self.discount_factor * next_q_values * (1.0 - batch.dones)

        loss = F.smooth_l1_loss(q_values, targets)

        self.optimizer.zero_grad()
        loss.backward()
        if self.gradient_clip > 0:
            nn.utils.clip_grad_norm_(self.policy_net.parameters(), self.gradient_clip)
        self.optimizer.step()

        self.training_steps += 1
        if self.training_steps % self.target_update_steps == 0:
            self.sync_target_network()

        return float(loss.item())

    def sync_target_network(self):
        self.target_net.load_state_dict(self.policy_net.state_dict())

    def decay_epsilon(self):
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)

    def save(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        torch.save(
            {
                "state_dim": self.state_dim,
                "n_actions": self.n_actions,
                "policy_state_dict": self.policy_net.state_dict(),
                "target_state_dict": self.target_net.state_dict(),
                "optimizer_state_dict": self.optimizer.state_dict(),
                "epsilon": self.epsilon,
                "training_steps": self.training_steps,
            },
            path,
        )

    def load(self, path, load_optimizer=False):
        checkpoint = torch.load(path, map_location=self.device)
        self.policy_net.load_state_dict(checkpoint["policy_state_dict"])
        self.target_net.load_state_dict(checkpoint.get("target_state_dict", checkpoint["policy_state_dict"]))
        if load_optimizer and "optimizer_state_dict" in checkpoint:
            self.optimizer.load_state_dict(checkpoint["optimizer_state_dict"])
        self.epsilon = float(checkpoint.get("epsilon", self.epsilon))
        self.training_steps = int(checkpoint.get("training_steps", self.training_steps))
        self.policy_net.eval()
        self.target_net.eval()
