"""Tests for tools/bench_link.py: the Mac-side bench tool for the first STM32 image.

Everything runs against a fake port, so neither pyserial nor a board is needed. Frames are
checked by decoding them with protocol/codec.py, the same decoder the operator app will use.
"""

from __future__ import annotations

import pytest

import bench_link
import codec
import messages as m
from bench_link import BenchLink, describe, describe_reset_flags


class FakePort:
    """Records writes; `read` returns whatever the test queued as the robot's replies."""

    def __init__(self) -> None:
        self.written = bytearray()
        self.incoming = bytearray()

    def write(self, data: bytes) -> int:
        self.written += data
        return len(data)

    def read(self, size: int) -> bytes:
        out = bytes(self.incoming[:size])
        del self.incoming[:size]
        return out


def sent_frames(port: FakePort) -> list[codec.DecodedFrame]:
    return codec.FrameDecoder().push(bytes(port.written))


def robot_says(port: FakePort, msg, seq: int = 0, t_us: int = 0) -> None:
    port.incoming += codec.encode_frame(msg.MESSAGE_ID, seq, t_us, msg.encode())


def make(clock_us: int = 1000) -> tuple[BenchLink, FakePort]:
    port = FakePort()
    return BenchLink(port, clock_us=lambda: clock_us), port


# ------------------------------------------------------------------------------- sending

def test_heartbeat_is_a_valid_empty_frame() -> None:
    link, port = make()
    link.send_heartbeat()
    (frame,) = sent_frames(port)
    assert frame.message_id == m.Heartbeat.MESSAGE_ID
    assert frame.payload == b""


def test_drive_command_round_trips() -> None:
    link, port = make()
    link.send_drive(0.25, -1.0)
    (frame,) = sent_frames(port)
    cmd = m.DriveCommand.decode(frame.payload)
    assert cmd.cmd_linear_speed_m_s == pytest.approx(0.25)
    assert cmd.cmd_angular_rate_rad_s == pytest.approx(-1.0)


@pytest.mark.parametrize(("linear", "angular"), [(2.5, 0.0), (0.0, -7.0), (float("nan"), 0.0)])
def test_drive_values_outside_the_schema_are_refused_before_sending(linear, angular) -> None:
    # The MCU would Nack them (RANGE_REJECT); refusing here keeps a typo off the wire.
    link, port = make()
    with pytest.raises(ValueError):
        link.send_drive(linear, angular)
    assert port.written == b""


def test_state_requests_carry_the_schema_magic() -> None:
    link, port = make()
    link.send_state_request(m.SafetyState.ARMED)
    (frame,) = sent_frames(port)
    req = m.SafetyStateRequest.decode(frame.payload)  # decode() rejects a bad magic
    assert req.requested_state == m.SafetyState.ARMED
    assert req.magic_ok()


def test_estop_carries_the_schema_magic() -> None:
    link, port = make()
    link.send_estop()
    (frame,) = sent_frames(port)
    assert m.EstopRequest.decode(frame.payload).magic_ok()


def test_seq_increments_and_wraps_at_256() -> None:
    link, port = make()
    for _ in range(258):
        link.send_heartbeat()
    seqs = [f.seq for f in sent_frames(port)]
    assert seqs[:3] == [0, 1, 2]
    assert seqs[255:] == [255, 0, 1]


def test_timestamps_are_strictly_newer_so_the_stale_filter_accepts_them() -> None:
    # The MCU rejects a frame not newer than the last of its type (ADR 0015 section 3), so
    # two frames in the same microsecond must still carry increasing timestamps.
    port = FakePort()
    link = BenchLink(port, clock_us=lambda: 5000)  # a frozen clock
    link.send_heartbeat()
    link.send_heartbeat()
    a, b = sent_frames(port)
    assert b.timestamp_us > a.timestamp_us


def test_timestamps_wrap_at_two_to_the_32() -> None:
    now = [0xFFFFFFFF - 1]
    port = FakePort()
    link = BenchLink(port, clock_us=lambda: now[0])
    link.send_heartbeat()
    now[0] = 0xFFFFFFFF + 10  # the host clock passed the wrap
    link.send_heartbeat()
    a, b = sent_frames(port)
    assert a.timestamp_us == 0xFFFFFFFE
    assert b.timestamp_us == 9  # mod 2^32
    assert ((b.timestamp_us - a.timestamp_us) & 0xFFFFFFFF) < 0x80000000, "newer, as int32"


# ----------------------------------------------------------------------------- receiving

def test_poll_decodes_fault_and_nack_replies() -> None:
    link, port = make()
    robot_says(port, m.Fault(fault_code=m.FaultCode.WHEEL_STALL, context=1))
    robot_says(port, m.Nack(rejected_message_id=0x04, reason=m.NackReason.ARM_INTERLOCK,
                            rejected_seq=7), seq=1)
    replies = link.poll()
    assert [type(r) for r in replies] == [m.Fault, m.Nack]
    assert replies[0].fault_code == m.FaultCode.WHEEL_STALL


def test_poll_survives_garbage_and_counts_it() -> None:
    link, port = make()
    port.incoming += b"\x13\x37garbage\x00"
    robot_says(port, m.Fault(fault_code=m.FaultCode.COMMS_TIMEOUT, context=200000))
    replies = link.poll()
    assert len(replies) == 1
    assert link.decoder.stats.frames_ok == 1


def test_poll_returns_nothing_when_nothing_arrived() -> None:
    link, _ = make()
    assert link.poll() == []


# --------------------------------------------------------------------------- descriptions

def test_describes_a_wheel_stall_with_the_wheels_named() -> None:
    text = describe(m.Fault(fault_code=m.FaultCode.WHEEL_STALL, context=3))
    assert "WHEEL_STALL" in text and "left" in text and "right" in text


@pytest.mark.parametrize(("context", "words"), [
    (2, "fault latched"), (3, "link stale"), (4, "stick not centred")])
def test_describes_a_refused_arm_by_interlock(context, words) -> None:
    text = describe(m.Fault(fault_code=m.FaultCode.NONE, context=context))
    assert "arm refused" in text and words in text


def test_describes_the_boot_report_with_reset_causes() -> None:
    # Pin reset + brown-out: RCC_CSR bits 26 and 27, i.e. bits 2 and 3 of the reported byte.
    text = describe(m.Fault(fault_code=m.FaultCode.NONE, context=0x80000000 | 0x0C))
    assert "boot" in text and "PIN" in text and "BOR" in text and "IWDG" not in text


def test_reset_flag_names_follow_rcc_csr() -> None:
    # stm32g474xx.h v1.2.6: OBLRSTF 25, PINRSTF 26, BORRSTF 27, SFTRSTF 28, IWDGRSTF 29,
    # WWDGRSTF 30, LPWRRSTF 31. The firmware sends CSR >> 24, so bit n of the byte is bit n+24.
    assert describe_reset_flags(1 << (29 - 24)) == ["IWDG"]
    assert describe_reset_flags(0xFE) == ["OBL", "PIN", "BOR", "SFT", "IWDG", "WWDG", "LPWR"]
    assert describe_reset_flags(0) == []


def test_describes_a_nack_with_the_message_it_rejected() -> None:
    text = describe(m.Nack(rejected_message_id=m.SafetyStateRequest.MESSAGE_ID,
                           reason=m.NackReason.NOT_DISARMED, rejected_seq=9))
    assert "SafetyStateRequest" in text and "NOT_DISARMED" in text and "9" in text


def test_describes_comms_timeout_silence_in_ms() -> None:
    text = describe(m.Fault(fault_code=m.FaultCode.COMMS_TIMEOUT, context=200000))
    assert "200.0 ms" in text


# ------------------------------------------------------------------------------- the REPL

def test_repl_commands_send_the_right_frames() -> None:
    link, port = make()
    session = bench_link.Session(link)
    for line in ["arm", "disarm", "estop", "drive 0.1 0.5"]:
        session.handle(line)
    ids = [f.message_id for f in sent_frames(port)]
    assert ids == [m.SafetyStateRequest.MESSAGE_ID, m.SafetyStateRequest.MESSAGE_ID,
                   m.EstopRequest.MESSAGE_ID]
    assert session.command == (0.1, 0.5), "drive sets what the stream sends; it sends nothing itself"


def test_repl_rejects_an_out_of_range_drive_and_keeps_the_old_command() -> None:
    link, _ = make()
    session = bench_link.Session(link)
    session.handle("drive 0.1 0")
    reply = session.handle("drive 9 0")
    assert "refused" in reply
    assert session.command == (0.1, 0.0)


def test_repl_stop_centres_the_stick() -> None:
    link, _ = make()
    session = bench_link.Session(link)
    session.handle("drive 0.2 0")
    session.handle("stop")
    assert session.command == (0.0, 0.0)


def test_stream_tick_sends_heartbeat_then_drive() -> None:
    link, port = make()
    session = bench_link.Session(link)
    session.handle("drive 0.1 0")
    session.stream_tick()
    ids = [f.message_id for f in sent_frames(port)]
    assert ids == [m.Heartbeat.MESSAGE_ID, m.DriveCommand.MESSAGE_ID]


def test_shutdown_centres_the_stick_and_requests_disarm() -> None:
    # CLAUDE.md: never leave a motor-driving process running. Quitting must not leave the
    # last command in force until the comms watchdog notices.
    link, port = make()
    session = bench_link.Session(link)
    session.handle("drive 0.2 0")
    session.shutdown()
    frames = sent_frames(port)
    assert session.command == (0.0, 0.0)
    assert m.DriveCommand.decode(frames[-2].payload).cmd_linear_speed_m_s == 0.0
    req = m.SafetyStateRequest.decode(frames[-1].payload)
    assert req.requested_state == m.SafetyState.DISARMED


def test_unknown_repl_command_lists_help() -> None:
    link, _ = make()
    assert "arm" in bench_link.Session(link).handle("fly")
