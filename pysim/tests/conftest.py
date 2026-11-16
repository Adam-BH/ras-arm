import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ras_sim.app import Simulation  # noqa: E402


@pytest.fixture
def sim():
    """a simulation that has booted and homed from its unknown power-up pose"""
    s = Simulation(seed=3)
    assert s.run_until(lambda: s.fw.state == "IDLE", 30000), f"did not home: {s.events()}"
    yield s
    s.close()
