"""Settings register map tests: mask bit metadata + per-bit writes.

Pins the contract the console Calibration tab relies on:

* ``bits`` metadata of the mask registers must match the firmware layout -
  ``steer_invert_mask`` is a bit per motor ID (FL = 2, FR = 1, RL = 3,
  RR = 0), ``encoder_invert_mask`` is bit0 left / bit1 right, and
  ``strafe_pattern`` is bit0..3 = FL, FR, RL, RR;
* writing a single bit must read back as exactly that bit, so one wheel or
  one encoder can be inverted on its own (no cross-talk, no off-by-one).

File:    tests/test_robot_config.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-30
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from software.robot import protocol as p  # noqa: E402
from software.robot import robot_config  # noqa: E402
from software.robot.link import MockLink  # noqa: E402


class MaskMetadataTests(unittest.TestCase):
    """The bit metadata (UI columns) must mirror the firmware register map."""

    def test_steer_invert_bits_match_firmware_motor_ids(self):
        """Steer invert bits match firmware motor ids."""
        register = robot_config.REGISTER_BY_KEY["steer_invert_mask"]
        self.assertEqual(
            [(spec.label, spec.bit) for spec in register.bits],
            [("FL", robot_config.MOTOR_ID_FL), ("FR", robot_config.MOTOR_ID_FR),
             ("RL", robot_config.MOTOR_ID_RL), ("RR", robot_config.MOTOR_ID_RR)],
        )
        # wiring contract (robot_configs.h, Docs/STM_docs/01): FL = M2, FR = M1,
        # RL = M3, RR = M0 - a wrong number here inverts the wrong wheel
        self.assertEqual(
            (robot_config.MOTOR_ID_FL, robot_config.MOTOR_ID_FR,
             robot_config.MOTOR_ID_RL, robot_config.MOTOR_ID_RR),
            (2, 1, 3, 0),
        )

    def test_encoder_invert_bits(self):
        """Encoder invert bits."""
        register = robot_config.REGISTER_BY_KEY["encoder_invert_mask"]
        self.assertEqual([(spec.label, spec.bit) for spec in register.bits], [("L", 0), ("R", 1)])

    def test_strafe_pattern_bits(self):
        """Strafe pattern bits."""
        register = robot_config.REGISTER_BY_KEY["strafe_pattern"]
        self.assertEqual(
            [(spec.label, spec.bit) for spec in register.bits],
            [("FL", 0), ("FR", 1), ("RL", 2), ("RR", 3)],
        )

    def test_as_meta_carries_bits_and_ranges(self):
        """As meta carries bits and ranges."""
        meta = {entry["key"]: entry for entry in robot_config.meta()}
        steer = meta["steer_invert_mask"]
        self.assertEqual((steer["min"], steer["max"]), (0, 15))
        self.assertEqual([bit["bit"] for bit in steer["bits"]], [2, 1, 3, 0])
        encoder = meta["encoder_invert_mask"]
        self.assertEqual((encoder["min"], encoder["max"]), (0, 3))
        # the strafe pattern is not an inversion: its caption says so
        self.assertIn("forward", meta["strafe_pattern"]["bits_hint"])
        # numeric registers keep an empty list: the UI renders a number field
        self.assertEqual(meta["wheel_diameter_mm"]["bits"], [])

    def test_mask_values_validate(self):
        """Mask values validate."""
        raw = robot_config.validate_engineering(
            {"steer_invert_mask": 15, "encoder_invert_mask": 3, "strafe_pattern": 15}
        )
        self.assertEqual(raw, {"steer_invert_mask": 15, "encoder_invert_mask": 3, "strafe_pattern": 15})
        with self.assertRaises(ValueError):
            robot_config.validate_engineering({"steer_invert_mask": 16})
        with self.assertRaises(ValueError):
            robot_config.validate_engineering({"encoder_invert_mask": 4})


class PerBitWriteTests(unittest.TestCase):
    """One bit at a time: every motor / encoder bit round-trips on its own."""

    def setUp(self):
        """Prepare the per-test fixtures."""
        self.link = MockLink()

    def _transact(self, cmd, payload=b""):
        """Transact."""
        return p.parse_response(self.link.transact(p.build_request(1, cmd, payload)))

    def _write_register(self, key, raw):
        """Write register."""
        register = robot_config.REGISTER_BY_KEY[key]
        status, _ = self._transact(
            p.CMD_SET_CONFIG, p.put_u8(register.index) + p.put_u8(1) + p.put_u16(raw)
        )
        self.assertEqual(status, p.ST_OK)
        return register

    def _read_register(self, key):
        """Read register."""
        register = robot_config.REGISTER_BY_KEY[key]
        status, payload = self._transact(p.CMD_GET_CONFIG, p.put_u8(register.index) + p.put_u8(1))
        self.assertEqual(status, p.ST_OK)
        return p.get_u16(payload, 2)

    def test_each_motor_bit_inverts_only_that_motor(self):
        """Each motor bit inverts only that motor."""
        register = robot_config.REGISTER_BY_KEY["steer_invert_mask"]
        for spec in register.bits:
            mask = 1 << spec.bit
            self._write_register("steer_invert_mask", mask)
            self.assertEqual(self._read_register("steer_invert_mask"), mask, spec.label)
        self._write_register("steer_invert_mask", 0)
        self.assertEqual(self._read_register("steer_invert_mask"), 0)

    def test_each_encoder_bit_inverts_only_that_encoder(self):
        """Each encoder bit inverts only that encoder."""
        register = robot_config.REGISTER_BY_KEY["encoder_invert_mask"]
        for spec in register.bits:
            mask = 1 << spec.bit
            self._write_register("encoder_invert_mask", mask)
            self.assertEqual(self._read_register("encoder_invert_mask"), mask, spec.label)

    def test_masks_written_together_survive_chunking(self):
        """Masks written together survive chunking."""
        raw = robot_config.validate_engineering(
            {"steer_invert_mask": 5, "encoder_invert_mask": 2, "strafe_pattern": 6}
        )
        for offset, chunk in robot_config.chunk_writes(raw):
            payload = p.put_u8(offset) + p.put_u8(len(chunk)) + b"".join(
                p.put_u16(value) for value in chunk
            )
            status, _ = self._transact(p.CMD_SET_CONFIG, payload)
            self.assertEqual(status, p.ST_OK)
        self.assertEqual(self._read_register("steer_invert_mask"), 5)
        self.assertEqual(self._read_register("encoder_invert_mask"), 2)
        self.assertEqual(self._read_register("strafe_pattern"), 6)

    def test_masks_survive_save_and_load(self):
        """Masks survive save and load."""
        self._write_register("steer_invert_mask", 0b1010)
        self._write_register("encoder_invert_mask", 0b11)
        self.assertEqual(self._transact(p.CMD_SAVE_CONFIG)[0], p.ST_OK)
        self.assertEqual(self._transact(p.CMD_RESET_CONFIG)[0], p.ST_OK)
        self.assertEqual(self._read_register("steer_invert_mask"), 0)
        self.assertEqual(self._transact(p.CMD_LOAD_CONFIG)[0], p.ST_OK)
        self.assertEqual(self._read_register("steer_invert_mask"), 0b1010)
        self.assertEqual(self._read_register("encoder_invert_mask"), 0b11)


if __name__ == "__main__":
    unittest.main()
