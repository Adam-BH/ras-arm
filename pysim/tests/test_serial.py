"""The twin's serial port speaks exactly like the UNO's USB serial (docs/serial_protocol.md), tested with a
real pyserial client on socket:// (the same code a Python script or `pio device monitor` would use)."""
import pytest

serial = pytest.importorskip("serial")      # pyserial (requirements.txt)

from ras_sim.app import BOOTLOADER_MS, Simulation
from ras_sim.config import CELL
from ras_sim.protocol import parse_reply, parse_telemetry
from ras_sim.serial_bridge import SerialBridge


class Host:
    """a program on the PC with the port open; the simulation advances while it waits"""
    def __init__(self, sim, url):
        self.sim, self.port, self.buf = sim, serial.serial_for_url(url, baudrate=115200, timeout=0), b""

    def pump(self, ms):
        self.sim.run_for(ms)
        self.buf += self.port.read(1 << 20)

    def lines(self):
        *done, rest = self.buf.decode("ascii").split("\n")
        self.buf = rest.encode()
        return done

    def ask(self, text: bytes, ms=20):
        self.lines()
        self.port.write(text)
        self.pump(ms)
        return [ln for ln in self.lines() if not ln.startswith("T,")]


@pytest.fixture
def bench():
    sim = Simulation(seed=5)
    assert sim.run_until(lambda: sim.fw.state == "IDLE", 30000)
    bridge = SerialBridge(sim, "tcp:0")
    host = Host(sim, bridge.url)
    yield sim, bridge, host
    host.port.close()
    bridge.close()
    sim.close()


def test_opening_the_port_resets_the_board(bench):
    sim, bridge, host = bench
    host.pump(20)
    assert bridge.connected and sim.booting and sim.resets == 1
    assert host.ask(b"PING\n", 200) == []                           # the bootloader eats it
    host.pump(BOOTLOADER_MS)
    assert "EV BOOT" in host.lines()
    assert sim.run_until(lambda: sim.fw.state == "IDLE", 30000)     # homes again from where it was


def test_line_framing_like_the_firmware(bench):
    sim, bridge, host = bench
    host.pump(BOOTLOADER_MS + 50)
    assert sim.run_until(lambda: sim.fw.state == "IDLE", 30000)
    host.port.read(1 << 20)
    assert host.ask(b"ping\r\n") == ["OK PING ras-arm 1.0.0"]          # \r ignored, case-insensitive
    assert host.ask(b"\n") == []                                       # empty line: no reply
    assert host.ask(b"STA") == [] and host.ask(b"TUS\n") == ["OK STATUS IDLE NONE homed=1 running=0"]
    got = host.ask(b"PING\nPOSES\n")                                   # two lines in one write
    assert got[0] == "OK PING ras-arm 1.0.0" and got[-1] == "OK POSES"
    c = CELL["CUBE"]
    assert f"POSE CUBE {c.x:.1f} {c.y:.1f} {c.z:.1f} {c.roll:.1f}" in got
    assert host.ask(b"MOVE " + b"1" * 90 + b"\n") == ["ERR MOVE args"]  # cut at 71 characters, like LINE_MAX
    assert host.ask(b"FLY\n") == ["ERR FLY unknown"]


def test_moves_done_and_telemetry_over_the_port(bench):
    sim, bridge, host = bench
    host.pump(BOOTLOADER_MS + 50)
    assert sim.run_until(lambda: sim.fw.state == "IDLE", 30000)
    host.port.read(1 << 20)
    assert host.ask(b"MOVE 200 50 120\n") == ["OK MOVE"]
    host.pump(3000)
    got = host.lines()
    assert "DONE MOVE" in got
    tel = [parse_telemetry(ln) for ln in got if ln.startswith("T,")]
    assert 58 <= len(tel) <= 62                                        # 20 Hz
    assert all(t is not None for t in tel) and tel[-1].tip == (200, 50, 120)
    assert all(ln.endswith("\r") is False for ln in got)              # '\n' only, like console.c


def test_one_host_at_a_time_and_reconnect_resets(bench):
    sim, bridge, host = bench
    host.pump(BOOTLOADER_MS + 50)
    other = serial.serial_for_url(bridge.url, baudrate=115200, timeout=0)
    sim.run_for(50)
    with pytest.raises(serial.SerialException):                        # busy, like a COM port in use
        other.read(100)
    assert sim.resets == 1                                             # and no reset
    other.close()
    host.port.close()
    sim.run_for(50)
    assert not bridge.connected
    again = Host(sim, bridge.url)
    again.pump(BOOTLOADER_MS + 50)
    assert sim.resets == 2 and "EV BOOT" in again.lines()
    again.port.close()


def test_no_reset_option():
    sim = Simulation(seed=5)
    assert sim.run_until(lambda: sim.fw.state == "IDLE", 30000)
    bridge = SerialBridge(sim, "tcp:0", reset_on_open=False)
    host = Host(sim, bridge.url)
    host.pump(20)
    assert sim.resets == 0 and parse_reply(host.ask(b"STATUS\n")[0]).text.startswith("IDLE")
    host.port.close()
    bridge.close()
    sim.close()
