"""The firmware twin on the physical world: protocol, homing, the RUN cycle and the fault paths."""
import pytest

from ras_sim.app import Simulation
from ras_sim.config import FW
from ras_sim.protocol import parse_telemetry


def cycle(sim, colour, timeout=40000):
    b = sim.world.place_in_cradle(colour)
    n = sim.fw.cycles
    if not sim.fw.running:
        assert sim.send("RUN").ok
    assert sim.run_until(lambda: sim.fw.cycles > n or sim.fw.state == "ERROR", timeout), sim.events()
    return b


def test_console_replies(sim):
    assert sim.send("PING").text == "ras-arm 1.0.0"
    assert sim.send("STATUS").text == "IDLE NONE homed=1 running=0"
    assert sim.send("FOO").text == "unknown"
    assert sim.send("MOVE 1 2").text == "args"
    assert sim.send("MOVE 900 0 0").text == "unreachable"
    assert sim.send("JOG 0 0 5").text == "state"                   # TEACH only
    assert sim.send("SET nope 1").text == "key"
    assert sim.send("RESET").text == "state"
    assert sim.send("MOVE 200 50 100").ok
    assert sim.send("MOVE 200 0 100").text == "busy"
    assert sim.run_until(lambda: any(ln.line == "DONE MOVE" for ln in sim.log), 5000)
    tip = sim.world.tip()
    assert tip == pytest.approx((200, 50, 100), abs=0.5)           # the physical arm got there


def test_telemetry_is_parseable(sim):
    lines = []
    sim.tel_sinks.append(lines.append)
    sim.run_for(200)
    t = parse_telemetry(lines[-1])
    assert len(lines) == 4 and t.state == "IDLE" and t.tip == (160, 0, 170) and t.joints[6] == FW.GRIP_OPEN_MM


def test_homing_finds_the_real_pose():
    s = Simulation(power_up_pose=(-40, 70, 45, 0, 0, 0))
    assert s.run_until(lambda: s.fw.state == "IDLE", 40000), s.events()
    assert max(abs(o) for o in s.world.offset[:3]) < 0.1             # step counters match the arm
    assert s.world.tip() == pytest.approx((160, 0, 170), abs=0.5)
    s.close()


def test_red_to_bin_a_blue_to_bin_b(sim):
    r = cycle(sim, "RED")
    b = cycle(sim, "BLUE")
    assert (sim.world.where(r), sim.world.where(b)) == ("bin_A", "bin_B")
    assert "EV PLACED RED" in sim.events() and "EV PLACED BLUE" in sim.events()
    assert sim.fw.state == "IDLE" and sim.fw.cycles == 2


def test_slipped_grasp_is_retried(sim):
    sim.world.fail_next_pick()
    b = cycle(sim, "RED", 60000)
    ev = sim.events()
    assert "EV PICK_RETRY 1" in ev and "EV PICKED 1" in ev
    assert sim.world.where(b) == "bin_A"


def test_two_slips_then_pick_failed(sim):
    sim.send("SET pick_retries 0")
    sim.world.fail_next_pick()
    b = cycle(sim, "BLUE")
    assert sim.fw.state == "ERROR" and "EV ERROR PICK_FAILED" in sim.events()
    assert sim.world.where(b) == "cradle"                           # the cube never left


def test_small_step_loss_is_caught_by_rehoming(sim):
    sim.send("SET rehome_every 1")
    sim.world.lose_steps(0, 0.8)                    # J1 skipped 0.8°: 3 mm off at the cradle, still grasps
    b = cycle(sim, "RED", 60000)
    assert sim.world.where(b) == "bin_A"
    assert sim.run_until(lambda: sim.fw.state == "IDLE" and sim.fw.homed, 40000)
    assert "EV REHOME" in sim.events()
    assert abs(sim.world.offset[0]) < 0.1


def test_large_step_loss_fails_the_pick_and_reset_recovers(sim):
    sim.world.lose_steps(0, 15.0)                   # 57 mm off at the cradle: the fingers miss the cube
    b = cycle(sim, "RED", 60000)
    assert sim.fw.state == "ERROR" and "EV ERROR PICK_FAILED" in sim.events()
    assert sim.send("RESET").ok                     # re-homes: the step counters match the arm again
    assert sim.run_until(lambda: sim.fw.state == "IDLE", 40000)
    assert abs(sim.world.offset[0]) < 0.1
    assert sim.send("RUN").ok
    assert sim.run_until(lambda: sim.world.where(b) == "bin_A", 40000), sim.events()


def test_broken_switch_wire_fails_homing():
    s = Simulation()
    s.world.break_switch(1)
    assert s.run_until(lambda: s.fw.state == "ERROR", 20000)
    assert "EV HOME_FAIL broken_wire 2" in s.events()
    s.close()


def test_estop_cuts_power_and_reset_rehomes(sim):
    assert sim.send("MOVE 230 -60 140").ok
    sim.run_for(300)
    sim.world.set_power(False)
    sim.run_for(1500)
    assert sim.fw.state == "ESTOP" and not sim.fw.homed
    assert sim.world.tip()[2] < 100                                  # the limp arm sagged
    assert sim.send("RESET").text == "estop"                         # rail still off
    sim.world.set_power(True)
    sim.run_for(20)
    assert sim.send("RESET").ok
    assert sim.run_until(lambda: sim.fw.state == "IDLE", 40000), sim.events()
    assert sim.world.tip() == pytest.approx((160, 0, 170), abs=0.5)


def test_teach_save_and_calibrate(sim):
    assert sim.send("TEACH").ok
    sim.world.place_in_cradle("RED")
    for line in ("JOG 0 0 -20", "CAL RED"):
        assert sim.send(line).ok, line
        assert sim.run_until(lambda: not sim.fw.fsm_busy(), 5000)
    assert sim.send("SAVE HOME").ok
    assert sim.fw.poses["HOME"].z == pytest.approx(150, abs=0.01)
    assert "EV CAL RED" in sim.events()
    assert sim.fw.cal["red"][0] > 0.55                                # red chroma from the physical cube
    assert sim.send("DONE").ok and sim.fw.state == "IDLE"
