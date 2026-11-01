"""Track lifecycle management and state transitions.

States:
    TENTATIVE: Initial track acquisition; accumulating squitter hits.
    CONFIRMED: Validated multi-hit track with verified position.
    COASTING: Missing squitter updates (> 5.0 s); kinematics predicted via filter.
    DELETED: Dropped track due to prolonged absence (> 30.0 s).

Authoritative reference: docs/05_TRACKING_AND_STATE_ESTIMATION.md Section 4.
"""

from __future__ import annotations

from enum import StrEnum


class TrackState(StrEnum):
    """4-state track lifecycle enumeration."""

    TENTATIVE = "TENTATIVE"
    CONFIRMED = "CONFIRMED"
    COASTING = "COASTING"
    DELETED = "DELETED"


class TrackLifecycleManager:
    """Manages hit counts and time-driven lifecycle transitions for a single track."""

    def __init__(
        self,
        confirmation_m: int = 3,
        confirmation_window_s: float = 5.0,
        coasting_timeout_s: float = 5.0,
        deletion_timeout_s: float = 30.0,
    ) -> None:
        self.confirmation_m = confirmation_m
        self.confirmation_window_s = confirmation_window_s
        self.coasting_timeout_s = coasting_timeout_s
        self.deletion_timeout_s = deletion_timeout_s

        self._state: TrackState = TrackState.TENTATIVE
        self.hit_timestamps: list[float] = []
        self.first_seen_s: float = 0.0
        self.last_seen_s: float = 0.0

    @property
    def state(self) -> TrackState:
        """Current lifecycle state of the track."""
        return self._state

    @state.setter
    def state(self, value: TrackState) -> None:
        self._state = value

    def register_hit(self, timestamp_s: float, has_valid_position: bool) -> TrackState:
        """Record a valid frame reception and evaluate state promotion."""
        if not self.hit_timestamps:
            self.first_seen_s = timestamp_s
        self.last_seen_s = timestamp_s
        self.hit_timestamps.append(timestamp_s)

        # Prune hits outside confirmation window
        cutoff = timestamp_s - self.confirmation_window_s
        self.hit_timestamps = [t for t in self.hit_timestamps if t >= cutoff]

        if self.state == TrackState.TENTATIVE:
            # Promote to CONFIRMED if M hits within N seconds and position is established
            if len(self.hit_timestamps) >= self.confirmation_m and has_valid_position:
                self.state = TrackState.CONFIRMED

        elif self.state == TrackState.COASTING:
            # Re-acquisition returns track to CONFIRMED
            self.state = TrackState.CONFIRMED

        return self.state

    def evaluate_timeouts(self, current_time_s: float) -> TrackState:
        """Evaluate inactivity timeouts and transition track state accordingly."""
        dt = current_time_s - self.last_seen_s
        if dt > self.deletion_timeout_s:
            self.state = TrackState.DELETED
        elif self.state == TrackState.CONFIRMED and dt > self.coasting_timeout_s:
            self.state = TrackState.COASTING
        return self.state


__all__ = [
    "TrackLifecycleManager",
    "TrackState",
]
