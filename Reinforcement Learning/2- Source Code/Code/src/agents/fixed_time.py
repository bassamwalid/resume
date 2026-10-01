"""Fixed-time baseline traffic signal controller."""


class FixedTimeAgent:
    """Cycle through north, south, east, and west with fixed green duration."""

    def __init__(self, green_duration=20, n_actions=4):
        self.green_duration = int(green_duration)
        self.n_actions = int(n_actions)
        self.current_action = 0

    def reset(self):
        self.current_action = 0

    def select_action(self, state=None, timestep=None, info=None):
        if info and not info.get("is_yellow", False):
            if int(info.get("phase_timer", 0)) >= self.green_duration:
                current_phase = int(info.get("current_phase", self.current_action))
                self.current_action = (current_phase + 1) % self.n_actions
        return self.current_action

