"""Discrete state encoder for the traffic signal MDP."""

DIRECTIONS = ("north", "south", "east", "west")
PHASE_TO_DIRECTION = dict(enumerate(DIRECTIONS))
DIRECTION_TO_PHASE = {direction: phase for phase, direction in PHASE_TO_DIRECTION.items()}


def bin_queue(queue_length):
    """Map a raw queue length to low, medium, or high congestion bins."""
    if queue_length <= 3:
        return 0
    if queue_length <= 7:
        return 1
    return 2


def bin_phase_timer(phase_timer):
    """Map the current green duration to short, medium, or long bins."""
    if phase_timer <= 5:
        return 0
    if phase_timer <= 15:
        return 1
    return 2


def ordered_queue_lengths(queue_lengths):
    """Return queue lengths in north, south, east, west order."""
    if isinstance(queue_lengths, dict):
        return [queue_lengths[direction] for direction in DIRECTIONS]
    return list(queue_lengths)


def encode_state(queue_lengths, current_phase, phase_timer):
    """Encode queues, current phase, and phase timer as a hashable tuple."""
    queue_bins = tuple(bin_queue(q) for q in ordered_queue_lengths(queue_lengths))
    return queue_bins + (int(current_phase), bin_phase_timer(phase_timer))


def phase_name(phase):
    """Return the direction name for a phase id."""
    return PHASE_TO_DIRECTION[int(phase)]

