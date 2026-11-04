"""Protocol codec tests: CRC vector, frame layout, LE helpers, parsing.

File:    tests/test_protocol.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from software.robot import protocol as p  # noqa: E402


class ProtocolTests(unittest.TestCase):
    def test_crc_reference_vector(self):
        # Same vector as the firmware (Core/Src/robot_protocol.c)
        """CRC reference vector."""
        self.assertEqual(p.crc8(b"123456789"), 0xF4)

    def test_build_request_layout(self):
        """Build request layout."""
        frame = p.build_request(7, p.CMD_DRIVE, p.put_i16(200) + p.put_u8(100))
        self.assertEqual(len(frame), p.REQ_SIZE)
        self.assertEqual(frame[0], p.SOF_REQUEST)
        self.assertEqual(frame[1], 7)
        self.assertEqual(frame[2], p.CMD_DRIVE)
        self.assertEqual(frame[3], 3)
        self.assertEqual(frame[4], 200 & 0xFF)
        self.assertEqual(frame[5], 200 >> 8)
        self.assertEqual(frame[6], 100)
        self.assertEqual(frame[15], p.crc8(frame[:15]))

    def test_build_request_rejects_long_payload(self):
        """Build request rejects long payload."""
        with self.assertRaises(ValueError):
            p.build_request(1, p.CMD_PING, bytes(12))

    def test_parse_response_ok(self):
        """Parse response ok."""
        payload = p.put_i32(-123456) + p.put_i32(789)
        body = bytes([p.SOF_RESPONSE, 9, p.ST_OK, len(payload)]) + payload.ljust(p.RSP_PAYLOAD_SIZE, b"\x00")
        frame = body + bytes([p.crc8(body)])
        status, decoded = p.parse_response(frame)
        self.assertEqual(status, p.ST_OK)
        self.assertEqual(p.get_i32(decoded, 0), -123456)
        self.assertEqual(p.get_i32(decoded, 4), 789)

    def test_parse_response_rejects_bad_crc(self):
        """Parse response rejects bad CRC."""
        body = bytes([p.SOF_RESPONSE, 1, 0, 0]) + bytes(p.RSP_PAYLOAD_SIZE)
        frame = bytearray(body + bytes([p.crc8(body)]))
        frame[5] ^= 0xFF
        with self.assertRaises(ValueError):
            p.parse_response(bytes(frame))

    def test_parse_response_rejects_bad_sof(self):
        """Parse response rejects bad sof."""
        body = bytes([0x00, 1, 0, 0]) + bytes(p.RSP_PAYLOAD_SIZE)
        frame = body + bytes([p.crc8(body)])
        with self.assertRaises(ValueError):
            p.parse_response(frame)

    def test_arc_frame_layout(self):
        # native arc: radius [mm i16], sweep [0.1 deg i16], speed [u8]
        """Arc frame layout."""
        frame = p.build_request(9, p.CMD_ARC, p.put_i16(-300) + p.put_i16(-900) + p.put_u8(120))
        self.assertEqual(len(frame), p.REQ_SIZE)
        self.assertEqual(frame[2], p.CMD_ARC)
        self.assertEqual(frame[3], 5)
        self.assertEqual(p.get_i16(frame, 4), -300)
        self.assertEqual(p.get_i16(frame, 6), -900)
        self.assertEqual(frame[8], 120)
        self.assertEqual(p.CMD_NAMES[p.CMD_ARC], "ARC")
        self.assertEqual(p.STATE_NAMES[p.STATE_ARC], "ARC")

    def test_signed_helpers(self):
        """Signed helpers."""
        self.assertEqual(p.get_i16(p.put_i16(-1500), 0), -1500)
        self.assertEqual(p.get_u16(p.put_u16(40000), 0), 40000)
        self.assertEqual(p.get_u8(p.put_u8(255), 0), 255)

    def test_release_motors_frame_has_no_payload(self):
        """RELEASE_MOTORS is a bare idle-only command (0x06)."""
        frame = p.build_request(11, p.CMD_RELEASE_MOTORS)
        self.assertEqual(frame[2], 0x06)
        self.assertEqual(frame[3], 0)
        self.assertEqual(p.CMD_NAMES[p.CMD_RELEASE_MOTORS], "RELEASE_MOTORS")


#: Frozen wire contract (issue #7): one GOLDEN request frame per command,
#: built with sequence 7. A change to a payload offset, a field width, a
#: command code or the CRC must fail this table - the firmware author and the
#: RPi side validate against the same frames (Docs/STM_docs/03 section 4).
GOLDEN_REQUESTS = {
    "PING": (p.put_u32(0x11223344), "a5077f04443322110000000000000032"),
    "STOP": (b"", "a507010000000000000000000000008d"),
    "ESTOP": (b"", "a5070200000000000000000000000036"),
    "CLEAR_FAULTS": (b"", "a50703000000000000000000000000a2"),
    "SET_POSE": (
        p.put_i16(12345) + p.put_i16(23456) + p.put_i16(-900) + p.put_u16(350),
        "a50704083930a05b7cfc5e0100000054",
    ),
    "RESET_ODOM": (b"", "a50705000000000000000000000000d3"),
    "RELEASE_MOTORS": (b"", "a5070600000000000000000000000068"),
    "DRIVE": (p.put_i16(200) + p.put_u16(350), "a5071004c8005e0100000000000000c9"),
    "TURN": (p.put_i16(900) + p.put_u16(250), "a50711048403fa00000000000000007d"),
    "GOTO": (
        p.put_i16(12345) + p.put_i16(23456) + p.put_i16(-900) + p.put_u16(350),
        "a50712083930a05b7cfc5e010000005a",
    ),
    "STRAFE": (p.put_i16(150) + p.put_u16(200), "a50713049600c8000000000000000059"),
    "SET_LIMITS": (
        p.put_u16(800) + p.put_u16(1000) + p.put_u16(1200),
        "a50714062003e803b00400000000002b",
    ),
    "SET_DEADMAN": (p.put_u16(1500), "a5071502dc05000000000000000000d2"),
    "ARC": (
        p.put_i16(-300) + p.put_i16(-900) + p.put_u16(120),
        "a5071606d4fe7cfc78000000000000eb",
    ),
    "QUEUE_ARC": (
        p.put_i16(500) + p.put_i16(450) + p.put_u16(200),
        "a5071706f401c201c8000000000000d2",
    ),
    "SET_GYRO": (p.put_i16(123) + p.put_u8(5), "a50718037b00050000000000000000b5"),
    "SET_VELOCITY": (p.put_i16(250) + p.put_i16(-750), "a5071904fa0012fd0000000000000017"),
    "Q_ODOMETRY": (b"", "a50720000000000000000000000000e7"),
    "Q_STATUS": (b"", "a5072100000000000000000000000073"),
    "Q_TICKS": (b"", "a50722000000000000000000000000c8"),
    "Q_EVENTS": (p.put_u16(42), "a50723022a00000000000000000000c9"),
    "DISP_MODE": (p.put_u8(1), "a50730010100000000000000000000da"),
    "DISP_TEXT": (p.put_u8(1) + p.put_u8(2) + b"HELLO", "a5073107010248454c4c4f00000000b0"),
    "GET_CONFIG": (p.put_u8(3) + p.put_u8(4), "a50740020304000000000000000000de"),
    "SET_CONFIG": (
        p.put_u8(3) + p.put_u8(2) + p.put_u16(100) + p.put_u16(200),
        "a507410603026400c80000000000009f",
    ),
    "SAVE_CONFIG": (b"", "a50742000000000000000000000000cd"),
    "LOAD_CONFIG": (b"", "a5074300000000000000000000000059"),
    "RESET_CONFIG": (b"", "a50744000000000000000000000000bc"),
}

#: Frozen command table (code -> name): adding/removing/renaming a command is
#: a protocol change and must update the spec AND this table on purpose.
GOLDEN_COMMANDS = {
    1: "STOP", 2: "ESTOP", 3: "CLEAR_FAULTS", 4: "SET_POSE", 5: "RESET_ODOM",
    6: "RELEASE_MOTORS", 16: "DRIVE", 17: "TURN", 18: "GOTO", 19: "STRAFE",
    20: "SET_LIMITS", 21: "SET_DEADMAN", 22: "ARC", 23: "QUEUE_ARC",
    24: "SET_GYRO", 25: "SET_VELOCITY", 32: "Q_ODOMETRY", 33: "Q_STATUS",
    34: "Q_TICKS", 35: "Q_EVENTS", 48: "DISP_MODE", 49: "DISP_TEXT",
    64: "GET_CONFIG", 65: "SET_CONFIG", 66: "SAVE_CONFIG", 67: "LOAD_CONFIG",
    68: "RESET_CONFIG", 127: "PING",
}

GOLDEN_STATUSES = {
    0: "OK", 1: "BAD_CRC", 2: "BAD_FRAME", 3: "UNKNOWN_CMD", 4: "BUSY",
    5: "RANGE", 6: "FAULT", 7: "BAD_LEN", 8: "STORAGE", 127: "NODATA",
}


class ProtocolFreezeTests(unittest.TestCase):
    """Issue #7: the v3 command/status field list is frozen and validated.

    The Python codec and the firmware implement the table in
    Docs/STM_docs/03 section 4. These tests fail on ANY wire change
    (command code, payload offset/width, CRC) so the two sides cannot drift
    silently: the golden hex below is the contract, not a snapshot of the
    current helpers.
    """

    def test_the_command_table_is_frozen(self):
        """Every command code and its name are pinned."""
        self.assertEqual(p.CMD_NAMES, GOLDEN_COMMANDS)
        self.assertEqual(
            sorted(GOLDEN_REQUESTS),
            sorted(GOLDEN_COMMANDS.values()),
            "every command needs one golden request frame",
        )

    def test_the_status_table_is_frozen(self):
        """Every status code and its name are pinned."""
        self.assertEqual(p.STATUS_NAMES, GOLDEN_STATUSES)

    def test_every_command_has_a_golden_frame(self):
        """Build one representative frame per command and match the wire bytes."""
        by_name = {name: code for code, name in p.CMD_NAMES.items()}
        for name, (payload, expected) in GOLDEN_REQUESTS.items():
            with self.subTest(command=name):
                frame = p.build_request(7, by_name[name], payload)
                self.assertEqual(len(frame), p.REQ_SIZE)
                self.assertEqual(frame.hex(), expected)
                self.assertEqual(frame[0], p.SOF_REQUEST)
                self.assertEqual(frame[1], 7)
                self.assertEqual(frame[2], by_name[name])
                self.assertEqual(frame[3], len(payload))
                self.assertEqual(frame[15], p.crc8(frame[:15]))


if __name__ == "__main__":
    unittest.main()
