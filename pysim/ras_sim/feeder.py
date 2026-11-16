"""The cradle feeder for the RUN cycle: puts the next cube in the cradle once the last one has left
and the arm is back home (a person or a chute on the real bench)."""
from __future__ import annotations

import random

from .app import Simulation, wait, wait_until


def feeder(sim: Simulation, n: int, seed: int | None = 0, gap_ms: float = 600, colours=None):
    rng = random.Random(seed)
    seq = list(colours) if colours else [rng.choice(["RED", "BLUE"]) for _ in range(n)]
    for colour in seq[:n]:
        yield from wait_until(sim, lambda: sim.world.cradle_cube() is None and sim.fw.state in ("IDLE", "TEACH"),
                              timeout_ms=10 * 60_000, what="an empty cradle")
        yield from wait(sim, gap_ms)
        sim.world.place_in_cradle(colour)
    return seq[:n]
