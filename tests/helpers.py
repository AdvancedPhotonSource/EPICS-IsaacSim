"""Test helpers shared across modules."""

import math

from isaac_epics.plant import PlantState


class FakePlant:
    """A deterministic plant for driving :class:`MotorAxis` in tests.

    Models a first-order lag toward the commanded target, because that is what
    a PhysX articulation drive does: it converges asymptotically rather than
    snapping. A perfect-follower plant would hide every bug that only appears
    when the readback lags the setpoint, which is most of the interesting ones.
    """

    def __init__(self, joint="x_axis", tau=0.0, dt=1 / 60.0, start=0.0):
        self.joint = joint
        self.tau = tau
        self.dt = dt
        self.position = start      # simulation units
        self.t_sim = 0.0
        self.playing = True
        self.connected = True

    def state(self):
        return PlantState(
            positions={self.joint: self.position},
            t_sim=self.t_sim,
            playing=self.playing and self.connected,
            connected=self.connected,
            age=0.0 if self.connected else 99.0,
        )

    def advance(self, target_sim):
        if not (self.playing and self.connected):
            return
        self.t_sim += self.dt
        if target_sim is None:
            return
        if self.tau <= 0:
            self.position = target_sim
        else:
            alpha = 1.0 - math.exp(-self.dt / self.tau)
            self.position += (target_sim - self.position) * alpha


def run(axis, plant, seconds=5.0, stop_when_done=True):
    """Drive ``axis`` against ``plant`` and return the last outputs."""
    steps = int(seconds / plant.dt)
    outputs = None
    for index in range(steps):
        outputs = axis.tick(plant.state())
        plant.advance(outputs.target_sim)
        if stop_when_done and index > 2 and outputs.done_moving == 1 and not axis.in_motion:
            break
    return outputs


