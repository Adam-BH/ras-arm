"""The twin on a serial port: the same bytes as the UNO's USB serial (docs/serial_protocol.md).

    --serial tcp:5005   a TCP port speaking the raw serial stream. Any pyserial program opens it as
                        serial.serial_for_url("socket://localhost:5005", 115200); PlatformIO too:
                        pio device monitor --port socket://localhost:5005 --baud 115200
    --serial COM20      a real (virtual) COM port, e.g. one end of a com0com pair: a serial monitor,
                        the Arduino serial monitor or PuTTY open the other end (COM21) as if it were the UNO.

Like the board (firmware/src/2_devices/serial_link.c, 6_app/console.c):
- input: ASCII lines ending in \\n, \\r ignored, characters past LINE_MAX − 1 (71) dropped, one reply per line;
- output: every line ends in \\n, telemetry included (20 Hz, all the time);
- opening the port resets the UNO (DTR → auto-reset): ~1.6 s of silence in the bootloader, whatever the host
  sends meanwhile is lost, then EV BOOT and homing. --no-reset turns that off.
"""
from __future__ import annotations

import select
import socket

from .config import FW

LINE_MAX = 72                     # firmware/src/2_devices/serial_link.h
TX_KEEP = 64 * 1024               # bytes kept for a host that does not read (the UNO never waits for the host)


class SerialBridge:
    def __init__(self, sim, spec: str, reset_on_open: bool = True, host: str = "127.0.0.1"):
        self.sim, self.spec, self.reset_on_open = sim, spec, reset_on_open
        self.rx = bytearray()
        self.tx = bytearray()
        self.lines_in = self.lines_out = 0
        self.client: socket.socket | None = None
        self.peer = ""
        self.ser = None
        if spec.lower().startswith("tcp:"):
            self.kind = "tcp"
            port = int(spec.split(":")[-1])
            self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server.bind((host, port))
            self.server.listen(1)
            self.server.setblocking(False)
            self.host, self.port = self.server.getsockname()
        else:
            import serial                                    # pyserial, only needed for a real COM port
            self.kind = "com"
            self.ser = serial.Serial(spec, baudrate=FW.SERIAL_BAUD, timeout=0, write_timeout=0.05)
            self._dsr = bool(self.ser.dsr)
        sim.line_sinks.append(self._out)
        sim.on_tick.append(self.poll)

    # ───────────── status ─────────────
    @property
    def url(self) -> str:
        return f"socket://{self.host}:{self.port}" if self.kind == "tcp" else self.spec

    @property
    def connected(self) -> bool:
        return self.client is not None if self.kind == "tcp" else self._dsr

    def describe(self) -> str:
        who = f"host {self.peer} connected" if self.connected and self.kind == "tcp" else (
            "host connected" if self.connected else "waiting for a host")
        return f"serial {self.url} | {FW.SERIAL_BAUD} baud | {who}"

    # ───────────── every tick ─────────────
    def poll(self):
        if self.kind == "tcp":
            self._poll_tcp()
        else:
            self._poll_com()

    def _opened(self):
        """a host opened the port: DTR pulses the UNO's reset line"""
        self.rx.clear()
        self.tx.clear()
        if self.reset_on_open:
            self.sim.reboot()

    def _poll_tcp(self):
        ready, _, _ = select.select([self.server], [], [], 0)
        if ready:
            conn, addr = self.server.accept()
            if self.client is not None:                        # a serial port has one owner: busy
                conn.close()
            else:
                conn.setblocking(False)
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.client, self.peer = conn, f"{addr[0]}:{addr[1]}"
                self._opened()
        if self.client is None:
            return
        try:
            while True:
                data = self.client.recv(4096)
                if not data:                                   # the host closed the port
                    self._drop_client()
                    return
                self._feed(data)
        except (BlockingIOError, InterruptedError):
            pass
        except OSError:
            self._drop_client()
            return
        self._flush_tcp()

    def _flush_tcp(self):
        if self.client is None or not self.tx:
            return
        try:
            n = self.client.send(self.tx)
            del self.tx[:n]
        except (BlockingIOError, InterruptedError):
            pass
        except OSError:
            self._drop_client()

    def _drop_client(self):
        try:
            self.client.close()
        except OSError:
            pass
        self.client, self.peer = None, ""
        self.rx.clear()
        self.tx.clear()

    def _poll_com(self):
        dsr = bool(self.ser.dsr)                               # com0com wires the other end's DTR to our DSR
        if dsr and not self._dsr:
            self._opened()
        self._dsr = dsr
        n = self.ser.in_waiting
        if n:
            self._feed(self.ser.read(n))

    # ───────────── bytes in / lines out ─────────────
    def _feed(self, data: bytes):
        if self.sim.booting:                                   # the bootloader eats what arrives during reset
            self.rx.clear()
            return
        for ch in data:
            if ch == 0x0D:                                     # \r ignored
                continue
            if ch == 0x0A:                                     # \n ends a line
                line = self.rx.decode("ascii", errors="replace")
                self.rx.clear()
                self.lines_in += 1
                self.sim.send(line)                            # the reply goes out through _out
            elif len(self.rx) < LINE_MAX - 1:                  # too long: extra characters dropped
                self.rx.append(ch)

    def _out(self, line: str):
        data = (line + "\n").encode("ascii", errors="replace")
        self.lines_out += 1
        if self.kind == "tcp":
            if self.client is None:
                return
            self.tx += data
            if len(self.tx) > TX_KEEP:
                del self.tx[:len(self.tx) - TX_KEEP]
            self._flush_tcp()
        else:
            try:
                self.ser.write(data)
            except Exception:                                  # nobody reading the other end: dropped
                pass

    def close(self):
        if self.kind == "tcp":
            if self.client is not None:
                self._drop_client()
            self.server.close()
        elif self.ser is not None:
            self.ser.close()
        if self._out in self.sim.line_sinks:
            self.sim.line_sinks.remove(self._out)
        if self.poll in self.sim.on_tick:
            self.sim.on_tick.remove(self.poll)
