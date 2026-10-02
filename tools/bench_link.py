"""Bench link: drive the first STM32 image from the Mac, over the Nucleo's ST-LINK serial port.

A stand-in for the operator app during bring-up (docs/bringup/stm32-first-image.md, steps
4-7). It sends Heartbeat, DriveCommand, SafetyStateRequest and EstopRequest, and prints every
Fault and Nack the robot sends back, in words.

    python tools/bench_link.py /dev/tty.usbmodemXXXX      # ls /dev/tty.usbmodem* to find it

Frames are built and parsed by protocol/codec.py and the generated protocol/generated/messages.py,
the same code the operator app will use, so nothing here re-implements the protocol.

Safety: the drive command starts at zero and only changes when typed. Quitting centres the
stick and asks to disarm, so the robot never keeps acting on a command after the tool is gone.
The first image has no motor drivers, but the tool behaves as if it did.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Protocol

# Run as `python tools/bench_link.py` from anywhere: find the codec and the generated bindings
# the same way pyproject.toml's pythonpath does for the tests.
_REPO = Path(__file__).resolve().parent.parent
for _sub in ("protocol", "protocol/generated"):
    if str(_REPO / _sub) not in sys.path:
        sys.path.insert(0, str(_REPO / _sub))

import codec  # noqa: E402
import messages as m  # noqa: E402

# ADR 0014: Fault(NONE) contexts 2-4 name the arm interlock that failed.
ARM_INTERLOCKS = {2: "fault latched", 3: "link stale", 4: "stick not centred"}
# ADR 0015 Q4: Fault(NONE) with this bit set is the boot report; the low byte is RCC_CSR >> 24.
BOOT_REPORT_MARKER = 0x80000000
# RCC_CSR reset-cause bits (stm32g474xx.h, cmsis-device-g4 v1.2.6), as bit n of that byte.
RESET_FLAGS = [(1, "OBL"), (2, "PIN"), (3, "BOR"), (4, "SFT"), (5, "IWDG"), (6, "WWDG"),
               (7, "LPWR")]
# firmware/core/safety/safety_supervisor.hpp: kLeftWheel, kRightWheel.
WHEEL_BITS = [(0x1, "left"), (0x2, "right")]
WHEEL_CONTEXT_CODES = {m.FaultCode.WHEEL_STALL, m.FaultCode.ENCODER_FAULT}

# CLAUDE.md: operator commands at ~50 Hz.
STREAM_PERIOD_S = 0.02
# docs/bringup/uart-link.md (owner, 2026-09-30).
BAUD = 460800
U32 = 0xFFFFFFFF


class BytePort(Protocol):
    """What BenchLink needs from a port. pyserial's Serial (with timeout=0) fits."""

    def write(self, data: bytes) -> int | None: ...
    def read(self, size: int) -> bytes: ...


def _host_clock_us() -> int:
    return time.monotonic_ns() // 1000


def _int32(diff: int) -> int:
    """A u32 difference read as signed: the MCU's rule for "newer" across a wrap."""
    diff &= U32
    return diff - (1 << 32) if diff >= 1 << 31 else diff


def drive_command(linear_m_s: float, angular_rad_s: float) -> m.DriveCommand:
    """A DriveCommand, or ValueError if either value is NaN or outside the schema range."""
    cmd = m.DriveCommand(cmd_linear_speed_m_s=linear_m_s, cmd_angular_rate_rad_s=angular_rad_s)
    if math.isnan(linear_m_s) or math.isnan(angular_rad_s) or not cmd.in_range():
        lin, ang = (m.DriveCommand.LIMITS[k] for k in
                    ("cmd_linear_speed_m_s", "cmd_angular_rate_rad_s"))
        raise ValueError(f"drive must be within linear {lin} m/s, angular {ang} rad/s")
    return cmd


class BenchLink:
    """Builds and parses frames over any byte port. No threads, no pyserial: testable."""

    def __init__(self, port: BytePort, clock_us: Callable[[], int] = _host_clock_us) -> None:
        self.port = port
        self.clock_us = clock_us
        self.decoder = codec.FrameDecoder()
        self._seq = 0
        self._last_ts: int | None = None

    # --- sending ----------------------------------------------------------------------
    def send_heartbeat(self) -> None:
        self._send(m.Heartbeat())

    def send_drive(self, linear_m_s: float, angular_rad_s: float) -> None:
        """Raises ValueError, sending nothing, if either value is outside the schema range."""
        self._send(drive_command(linear_m_s, angular_rad_s))

    def send_state_request(self, state: m.SafetyState) -> None:
        self._send(m.SafetyStateRequest(requested_state=state,
                                        magic=m.SafetyStateRequest.MAGICS["magic"]))

    def send_estop(self) -> None:
        self._send(m.EstopRequest(magic=m.EstopRequest.MAGICS["magic"]))

    def _send(self, msg) -> None:
        frame = codec.encode_frame(msg.MESSAGE_ID, self._seq, self._next_timestamp(), msg.encode())
        self._seq = (self._seq + 1) & 0xFF
        self.port.write(frame)

    def _next_timestamp(self) -> int:
        # The MCU drops a frame that is not newer than the last one of its type (ADR 0015 §3),
        # so never repeat a timestamp, even if the host clock hasn't moved.
        t = self.clock_us() & U32
        if self._last_ts is not None and _int32(t - self._last_ts) <= 0:
            t = (self._last_ts + 1) & U32
        self._last_ts = t
        return t

    # --- receiving --------------------------------------------------------------------
    def poll(self) -> list:
        """Every robot message that has fully arrived, decoded. Garbage is counted, not raised."""
        data = self.port.read(4096)
        out = []
        for frame in self.decoder.push(data) if data else []:
            cls = m.MESSAGE_BY_ID.get(frame.message_id)
            if cls is None:
                continue
            try:
                out.append(cls.decode(frame.payload))
            except ValueError:
                continue  # an out-of-range field; the decoder already vouched for the framing
        return out


def describe_reset_flags(flags: int) -> list[str]:
    return [name for bit, name in RESET_FLAGS if flags & (1 << bit)]


def describe(msg) -> str:
    """One line, in words, for a message from the robot."""
    if isinstance(msg, m.Fault):
        ctx = msg.context
        if msg.fault_code == m.FaultCode.NONE:
            if ctx & BOOT_REPORT_MARKER:
                causes = describe_reset_flags(ctx & 0xFF) or ["none set"]
                return f"boot: reset causes {', '.join(causes)} (raw 0x{ctx & 0xFF:02X})"
            reason = ARM_INTERLOCKS.get(ctx, f"unknown interlock {ctx}")
            return f"arm refused: {reason}"
        detail = f"context {ctx}"
        if msg.fault_code in WHEEL_CONTEXT_CODES:
            detail = " + ".join(name for bit, name in WHEEL_BITS if ctx & bit) or detail
        elif msg.fault_code in (m.FaultCode.COMMS_TIMEOUT, m.FaultCode.LOOP_OVERRUN):
            detail = f"{ctx / 1000:.1f} ms"
        return f"FAULT {msg.fault_code.name}: {detail}"
    if isinstance(msg, m.Nack):
        cls = m.MESSAGE_BY_ID.get(msg.rejected_message_id)
        what = cls.__name__ if cls else f"id 0x{msg.rejected_message_id:02X}"
        return f"NACK {what} seq {msg.rejected_seq}: {msg.reason.name}"
    return f"{type(msg).__name__}: {msg}"


HELP = """commands:
  stream on|off   send Heartbeat + DriveCommand at 50 Hz (on at start)
  drive V W       set the streamed command: V m/s forward, W rad/s left
  stop            centre the stick (drive 0 0)
  arm             request ARMED
  disarm          request DISARMED (also clears FAULT / ESTOP)
  estop           send EstopRequest
  quit            centre, disarm, exit"""


class Session:
    """The REPL's state and commands, separate from the threads so tests can drive it."""

    def __init__(self, link: BenchLink) -> None:
        self.link = link
        self.command = (0.0, 0.0)
        self.streaming = True
        self._lock = threading.Lock()  # the stream thread and the REPL share the port

    def handle(self, line: str) -> str:
        words = line.split()
        if not words:
            return ""
        verb, args = words[0].lower(), words[1:]
        with self._lock:
            if verb == "arm":
                self.link.send_state_request(m.SafetyState.ARMED)
                return "sent: arm request"
            if verb in ("disarm", "clear"):
                self.link.send_state_request(m.SafetyState.DISARMED)
                return "sent: disarm request"
            if verb == "estop":
                self.link.send_estop()
                return "sent: e-stop"
            if verb == "stop":
                self.command = (0.0, 0.0)
                return "stick centred"
            if verb == "drive" and len(args) == 2:
                try:
                    linear, angular = float(args[0]), float(args[1])
                    drive_command(linear, angular)
                except ValueError as exc:
                    return f"refused: {exc}"
                self.command = (linear, angular)
                return f"streaming drive {linear} m/s, {angular} rad/s"
            if verb == "stream" and args in (["on"], ["off"]):
                self.streaming = args[0] == "on"
                return f"stream {args[0]}"
        return HELP

    def stream_tick(self) -> None:
        with self._lock:
            if self.streaming:
                self.link.send_heartbeat()
                self.link.send_drive(*self.command)

    def shutdown(self) -> None:
        with self._lock:
            self.command = (0.0, 0.0)
            self.streaming = False
            self.link.send_drive(0.0, 0.0)
            self.link.send_state_request(m.SafetyState.DISARMED)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("port", help="serial device, e.g. /dev/tty.usbmodem1103")
    parser.add_argument("--baud", type=int, default=BAUD)
    args = parser.parse_args(argv)

    try:
        import serial  # pyserial; only needed for a real port
    except ImportError:
        print("pyserial is missing: python -m pip install -r requirements-dev.txt", file=sys.stderr)
        return 1

    port = serial.Serial(args.port, args.baud, timeout=0)
    link = BenchLink(port)
    session = Session(link)
    done = threading.Event()

    def stream() -> None:
        while not done.wait(STREAM_PERIOD_S):
            session.stream_tick()

    def listen() -> None:
        while not done.is_set():
            with session._lock:
                replies = link.poll()
            for msg in replies:
                print(f"\r<- {describe(msg)}\n> ", end="", flush=True)
            time.sleep(0.005)

    threads = [threading.Thread(target=f, daemon=True) for f in (stream, listen)]
    for t in threads:
        t.start()
    print(f"connected to {args.port} at {args.baud} baud. Streaming a centred stick.\n{HELP}")
    try:
        while True:
            line = input("> ")
            if line.strip().lower() in ("quit", "exit"):
                break
            reply = session.handle(line)
            if reply:
                print(reply)
    except (EOFError, KeyboardInterrupt):
        print()
    finally:
        done.set()
        for t in threads:
            t.join(timeout=1.0)
        session.shutdown()
        port.flush()
        port.close()
        print("stick centred, disarm requested, port closed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
