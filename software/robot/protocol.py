"""STM32 protocol v3 codec (frames, CRC-8, little-endian helpers).

Keep byte-identical with ``Docs/03-i2c-protocol-spec.md`` and the firmware
(``Core/Src/robot_protocol.c``):

    Request  (16 B): | A5 | seq | cmd | len | payload[11] | crc8 |
    Response (32 B): | 5A | seq | status | len | payload[27] | crc8 |

CRC-8/ATM: polynomial 0x07, initial value 0x00, no reflection.
Test vector: CRC8(b"123456789") == 0xF4.

Version 3: speeds are 16-bit mm/s, the board reports a full error/event log
(:data:`EVENT_NAMES`) and its calibrated settings are readable/writable
through the GET/SET/SAVE/LOAD/RESET_CONFIG commands.

File:    software/robot/protocol.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import struct
from typing import Optional, Sequence, Tuple

REQ_SIZE = 16
RSP_SIZE = 32
REQ_PAYLOAD_SIZE = 11

#: SET_GYRO wire scale: the i16 payload counts 0.1 deg/s (covers +/-3276.7).
#: Stream frames are silent: the board applies them without publishing a
#: response (Docs/09 section 7.3) - never retried, never read back.
GYRO_RATE_SCALE = 0.1
#: first firmware version that supports the silent SET_GYRO stream command
GYRO_STREAM_FW_MAJOR = 3
GYRO_STREAM_FW_MINOR = 3
#: SET_VELOCITY wire scale: the i16 omega payload counts 0.1 deg/s; the
#: velocity payload counts whole mm/s (i16 covers +/-32767 mm/s). Silent
#: latest-wins frames: the board RAMPS towards the targets, stops the
#: streaming mode when no frame arrives for ROBOT_STREAM_STALE_MS (200 ms in
#: fw 3.4) and never publishes a response for a stream frame.
VELOCITY_OMEGA_SCALE = 0.1
#: first firmware version that supports the silent SET_VELOCITY stream
VELOCITY_STREAM_FW_MAJOR = 3
VELOCITY_STREAM_FW_MINOR = 4
RSP_PAYLOAD_SIZE = 27

SOF_REQUEST = 0xA5
SOF_RESPONSE = 0x5A

# Command codes --------------------------------------------------------------
CMD_STOP = 0x01
CMD_ESTOP = 0x02
CMD_CLEAR_FAULTS = 0x03
CMD_SET_POSE = 0x04
CMD_RESET_ODOMETRY = 0x05
CMD_RELEASE_MOTORS = 0x06
CMD_DRIVE = 0x10
CMD_TURN = 0x11
CMD_GOTO = 0x12
CMD_STRAFE = 0x13
CMD_SET_MOTION_LIMITS = 0x14
CMD_SET_DEADMAN = 0x15
CMD_ARC = 0x16
CMD_QUEUE_ARC = 0x17
CMD_SET_GYRO = 0x18
CMD_SET_VELOCITY = 0x19
CMD_QUERY_ODOMETRY = 0x20
CMD_QUERY_STATUS = 0x21
CMD_QUERY_TICKS = 0x22
CMD_QUERY_EVENTS = 0x23
CMD_DISPLAY_MODE = 0x30
CMD_DISPLAY_TEXT = 0x31
CMD_GET_CONFIG = 0x40
CMD_SET_CONFIG = 0x41
CMD_SAVE_CONFIG = 0x42
CMD_LOAD_CONFIG = 0x43
CMD_RESET_CONFIG = 0x44
CMD_PING = 0x7F

# Status codes ---------------------------------------------------------------
ST_OK = 0x00
ST_BAD_CRC = 0x01
ST_BAD_FRAME = 0x02
ST_UNKNOWN_CMD = 0x03
ST_BUSY = 0x04
ST_RANGE = 0x05
ST_FAULT = 0x06
ST_BAD_LEN = 0x07
ST_STORAGE = 0x08
ST_NODATA = 0x7F

STATUS_NAMES = {
    ST_OK: "OK",
    ST_BAD_CRC: "BAD_CRC",
    ST_BAD_FRAME: "BAD_FRAME",
    ST_UNKNOWN_CMD: "UNKNOWN_CMD",
    ST_BUSY: "BUSY",
    ST_RANGE: "RANGE",
    ST_FAULT: "FAULT",
    ST_BAD_LEN: "BAD_LEN",
    ST_STORAGE: "STORAGE",
    ST_NODATA: "NODATA",
}

CMD_NAMES = {
    CMD_STOP: "STOP",
    CMD_ESTOP: "ESTOP",
    CMD_CLEAR_FAULTS: "CLEAR_FAULTS",
    CMD_SET_POSE: "SET_POSE",
    CMD_RESET_ODOMETRY: "RESET_ODOM",
    CMD_RELEASE_MOTORS: "RELEASE_MOTORS",
    CMD_DRIVE: "DRIVE",
    CMD_TURN: "TURN",
    CMD_GOTO: "GOTO",
    CMD_STRAFE: "STRAFE",
    CMD_SET_MOTION_LIMITS: "SET_LIMITS",
    CMD_SET_DEADMAN: "SET_DEADMAN",
    CMD_ARC: "ARC",
    CMD_QUEUE_ARC: "QUEUE_ARC",
    CMD_SET_GYRO: "SET_GYRO",
    CMD_SET_VELOCITY: "SET_VELOCITY",
    CMD_QUERY_ODOMETRY: "Q_ODOMETRY",
    CMD_QUERY_STATUS: "Q_STATUS",
    CMD_QUERY_TICKS: "Q_TICKS",
    CMD_QUERY_EVENTS: "Q_EVENTS",
    CMD_DISPLAY_MODE: "DISP_MODE",
    CMD_DISPLAY_TEXT: "DISP_TEXT",
    CMD_GET_CONFIG: "GET_CONFIG",
    CMD_SET_CONFIG: "SET_CONFIG",
    CMD_SAVE_CONFIG: "SAVE_CONFIG",
    CMD_LOAD_CONFIG: "LOAD_CONFIG",
    CMD_RESET_CONFIG: "RESET_CONFIG",
    CMD_PING: "PING",
}

# Motion states ---------------------------------------------------------------
STATE_IDLE = 0
STATE_DRIVE = 1
STATE_TURN = 2
STATE_STRAFE = 3
STATE_GOTO = 4
STATE_FAULT = 5
STATE_ESTOP = 6
STATE_ARC = 7
STATE_STREAM = 8

STATE_NAMES = {
    STATE_IDLE: "IDLE",
    STATE_DRIVE: "DRIVE",
    STATE_TURN: "TURN",
    STATE_STRAFE: "STRAFE",
    STATE_GOTO: "GOTO",
    STATE_FAULT: "FAULT",
    STATE_ESTOP: "ESTOP",
    STATE_ARC: "ARC",
    STATE_STREAM: "STREAM",
}

# Fault bits (must match motion.h; 16-bit mask) -----------------------------
FAULT_ESTOP = 0x0001
FAULT_LIMIT = 0x0002
FAULT_STALL = 0x0004
FAULT_TIMEOUT = 0x0008
FAULT_DEADMAN = 0x0010

FAULT_NAMES = {
    FAULT_ESTOP: "ESTOP",
    FAULT_LIMIT: "LIMIT",
    FAULT_STALL: "STALL",
    FAULT_TIMEOUT: "TIMEOUT",
    FAULT_DEADMAN: "DEADMAN",
}

# Error/event codes (must match robot_events.h) ------------------------------
EVENT_NONE = 0x00
EVENT_ESTOP = 0x01
EVENT_LIMIT = 0x02
EVENT_STALL = 0x03
EVENT_TIMEOUT = 0x04
EVENT_DEADMAN = 0x05
EVENT_BAD_CRC = 0x06
EVENT_BAD_FRAME = 0x07
EVENT_UNKNOWN_CMD = 0x08
EVENT_BAD_PARAM = 0x09
EVENT_BUSY_REJECT = 0x0A
EVENT_I2C_ERROR = 0x0B
EVENT_FLASH_ERROR = 0x0C
EVENT_CONFIG_FALLBACK = 0x0D
EVENT_CONFIG_SAVED = 0x0E
EVENT_CONFIG_APPLIED = 0x0F
EVENT_CONFIG_DEFAULTS = 0x10
EVENT_RESET_IWDG = 0x11
EVENT_RESET_POR = 0x12
EVENT_RESET_SW = 0x13
EVENT_RESET_OTHER = 0x14
EVENT_BAD_LEN = 0x15
EVENT_I2C_RECOVER = 0x16

#: code -> (name, severity). Severity drives the console log level.
EVENT_NAMES = {
    EVENT_ESTOP: ("ESTOP", "error"),
    EVENT_LIMIT: ("LIMIT_SWITCH", "error"),
    EVENT_STALL: ("STALL", "error"),
    EVENT_TIMEOUT: ("TIMEOUT", "error"),
    EVENT_DEADMAN: ("DEADMAN", "error"),
    EVENT_BAD_CRC: ("BAD_CRC", "warn"),
    EVENT_BAD_FRAME: ("BAD_FRAME", "warn"),
    EVENT_UNKNOWN_CMD: ("UNKNOWN_CMD", "warn"),
    EVENT_BAD_PARAM: ("BAD_PARAM", "warn"),
    EVENT_BUSY_REJECT: ("BUSY_REJECT", "warn"),
    EVENT_I2C_ERROR: ("I2C_ERROR", "warn"),
    EVENT_FLASH_ERROR: ("FLASH_ERROR", "error"),
    EVENT_CONFIG_FALLBACK: ("CONFIG_FALLBACK", "warn"),
    EVENT_CONFIG_SAVED: ("CONFIG_SAVED", "info"),
    EVENT_CONFIG_APPLIED: ("CONFIG_APPLIED", "info"),
    EVENT_CONFIG_DEFAULTS: ("CONFIG_DEFAULTS", "info"),
    EVENT_RESET_IWDG: ("RESET_WATCHDOG", "warn"),
    EVENT_RESET_POR: ("RESET_POWER_ON", "info"),
    EVENT_RESET_SW: ("RESET_SOFTWARE", "info"),
    EVENT_RESET_OTHER: ("RESET_OTHER", "info"),
    EVENT_BAD_LEN: ("BAD_LEN", "warn"),
    EVENT_I2C_RECOVER: ("I2C_RECOVER", "warn"),
}


def event_name(code: int) -> str:
    """Human-readable name of an error/event code."""
    entry = EVENT_NAMES.get(code)
    return entry[0] if entry else f"EVENT_0x{code:02X}"


def event_severity(code: int) -> str:
    """Return "info", "warn" or "error" for an error/event code."""
    entry = EVENT_NAMES.get(code)
    return entry[1] if entry else "warn"


# Display modes --------------------------------------------------------------
DISPLAY_OFF = 0
DISPLAY_AUTO = 1
DISPLAY_MANUAL = 2


def crc8(data: bytes) -> int:
    """CRC-8/ATM over ``data`` (poly 0x07, init 0x00)."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_request(seq: int, cmd: int, payload: bytes = b"") -> bytes:
    """Assemble one 16-byte request frame (payload zero-padded)."""
    if len(payload) > REQ_PAYLOAD_SIZE:
        raise ValueError(f"payload too long: {len(payload)} > {REQ_PAYLOAD_SIZE}")
    body = bytes([SOF_REQUEST, seq & 0xFF, cmd & 0xFF, len(payload)]) + payload.ljust(REQ_PAYLOAD_SIZE, b"\x00")
    return body + bytes([crc8(body)])


def parse_response(frame: bytes) -> Tuple[int, bytes]:
    """Validate a 32-byte response; returns ``(status, payload)``.

    Raises ``ValueError`` when the frame is malformed or the CRC fails.
    """
    if len(frame) != RSP_SIZE:
        raise ValueError(f"response size {len(frame)} != {RSP_SIZE}")
    if frame[0] != SOF_RESPONSE:
        raise ValueError(f"bad SOF 0x{frame[0]:02X}")
    if crc8(frame[: RSP_SIZE - 1]) != frame[RSP_SIZE - 1]:
        raise ValueError("bad CRC")
    length = min(frame[3], RSP_PAYLOAD_SIZE)
    return frame[2], frame[4 : 4 + length]


# Little-endian payload helpers ----------------------------------------------
def put_i16(value: int) -> bytes:
    """Encode ``value`` as a little-endian signed 16-bit field."""
    return struct.pack("<h", int(value))


def put_u16(value: int) -> bytes:
    """Encode ``value`` as a little-endian unsigned 16-bit field."""
    return struct.pack("<H", int(value))


def put_u8(value: int) -> bytes:
    """Encode ``value`` as a single byte."""
    return struct.pack("<B", int(value))


def put_i32(value: int) -> bytes:
    """Encode ``value`` as a little-endian signed 32-bit field."""
    return struct.pack("<i", int(value))


def put_u32(value: int) -> bytes:
    """Encode ``value`` as a little-endian unsigned 32-bit field."""
    return struct.pack("<I", int(value))


def get_i16(payload: bytes, offset: int) -> int:
    """Read a little-endian signed 16-bit field."""
    return struct.unpack_from("<h", payload, offset)[0]


def get_u16(payload: bytes, offset: int) -> int:
    """Read a little-endian unsigned 16-bit field."""
    return struct.unpack_from("<H", payload, offset)[0]


def get_u8(payload: bytes, offset: int) -> int:
    """Read one byte."""
    return payload[offset]


def get_i32(payload: bytes, offset: int) -> int:
    """Read a little-endian signed 32-bit field."""
    return struct.unpack_from("<i", payload, offset)[0]


def get_u32(payload: bytes, offset: int) -> int:
    """Read a little-endian unsigned 32-bit field."""
    return struct.unpack_from("<I", payload, offset)[0]


def decode_faults(faults: int) -> Sequence[str]:
    """Human readable fault list for logging/UI."""
    return [name for bit, name in FAULT_NAMES.items() if faults & bit]


def decode_queue(payload: bytes) -> Optional[int]:
    """QUERY_STATUS queue depth (offset 9) or None if truncated."""
    return payload[9] if len(payload) > 9 else None
