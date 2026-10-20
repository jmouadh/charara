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


if __name__ == "__main__":
    unittest.main()
