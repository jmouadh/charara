"""Robot settings register map (mirror of the firmware ``robot_settings.h``).

The STM32 keeps its calibrated configuration (wheel geometry, encoder data,
motion limits, gains, orientation masks) as an image of 16-bit registers in
the last flash sector. This module is the RPi-side contract:

* the register table (index, key, unit, range) used by the API and the UI,
* encode/decode between engineering units and raw register values,
* validation before a write,
* the compile-time defaults, so the mock link and the console can work
  without a board.

Keep in sync with ``App/Inc/robot_settings.h`` and ``Docs/03-i2c-protocol-spec.md``.

File:    software/robot/robot_config.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Mapping, Tuple

MAP_VERSION = 1

#: Register runs one SET_CONFIG frame can carry (firmware limit).
SET_CHUNK_MAX = 4
#: Register runs one GET_CONFIG frame can return (firmware limit).
GET_CHUNK_MAX = 12


@dataclass(frozen=True)
class BitSpec:
    """One bit of a mask register, shown as an independent 0/1 control.

    The UI renders one column per bit (loaded from the board, applied back as
    the register value), so the bit order is part of the calibration contract
    instead of an operator guess.
    """

    bit: int
    label: str
    hint: str = ""      # short caption under the column (wheel / motor id)
    tooltip: str = ""   # hover text explaining exactly what the bit does

    def as_meta(self) -> Dict:
        """Bit metadata as a JSON-friendly dict."""
        return {"bit": self.bit, "label": self.label, "hint": self.hint, "tooltip": self.tooltip}


@dataclass(frozen=True)
class Register:
    """One settings register: raw u16 value with a display scale."""

    index: int
    key: str
    label: str
    unit: str
    group: str
    min_value: int
    max_value: int
    scale: float = 1.0     # engineering value = raw / scale
    read_only: bool = False
    bits: Tuple[BitSpec, ...] = ()  # set for mask registers (one control per bit)
    bits_hint: str = ""    # caption above the bit columns (mask registers)

    def encode(self, value: float) -> int:
        """Engineering value -> raw register value (validated by the caller)."""
        return int(round(float(value) * self.scale))

    def decode(self, raw: int) -> float:
        """Raw register value -> engineering value (int scaling stays int)."""
        if self.scale == 1.0:
            return int(raw)
        return raw / self.scale

    def as_meta(self) -> Dict:
        """Register metadata as a JSON-friendly dict."""
        return {
            "index": self.index,
            "key": self.key,
            "label": self.label,
            "unit": self.unit,
            "group": self.group,
            "min": self.decode(self.min_value),
            "max": self.decode(self.max_value),
            "step": 1.0 / self.scale if self.scale >= 10 else 1.0,
            "read_only": self.read_only,
            "bits": [spec.as_meta() for spec in self.bits],
            "bits_hint": self.bits_hint,
        }


#: Stepper motor IDs as wired to the wheels (robot_configs.h, Docs/STM_docs/01):
#: FL = terminal M2, FR = M1, RL = M3, RR = M0. ``steer_invert_mask`` is a bit
#: per motor ID, so these constants are the ONLY place where wheel names are
#: bound to bit positions - the UI builds its columns from this metadata.
MOTOR_ID_FL = 2
MOTOR_ID_FR = 1
MOTOR_ID_RL = 3
MOTOR_ID_RR = 0


#: The full register map; index order must match the firmware.
REGISTERS: Tuple[Register, ...] = (
    Register(0, "map_version", "Register map version", "", "info", 1, 255, read_only=True),
    Register(1, "wheel_diameter_mm", "Stepper wheel diameter", "mm", "stepper_wheels", 100, 3000, 10.0),
    Register(2, "stepper_microsteps", "Stepper microsteps", "steps", "stepper_wheels", 1, 64),
    Register(3, "robot_track_mm", "Drive wheel track (left-right)", "mm", "geometry", 100, 4000, 10.0),
    Register(4, "wheelbase_mm", "Wheelbase (front-back)", "mm", "geometry", 100, 4000, 10.0),
    Register(5, "odom_wheel_circumference_mm", "Odometry wheel circumference", "mm", "odom_encoders", 200, 10000, 10.0),
    Register(6, "odom_ticks_per_rev", "Odometry ticks per revolution", "ticks", "odom_encoders", 100, 20000),
    Register(7, "odom_track_mm", "Odometry wheel track", "mm", "odom_encoders", 100, 4000, 10.0),
    Register(8, "max_speed_mm_s", "Max speed", "mm/s", "limits", 50, 900),
    Register(9, "min_speed_mm_s", "Min speed (creep)", "mm/s", "limits", 20, 150),
    Register(10, "accel_mm_s2", "Acceleration", "mm/s2", "limits", 50, 2000),
    Register(11, "decel_mm_s2", "Deceleration", "mm/s2", "limits", 50, 2000),
    Register(12, "heading_gain", "Heading hold gain", "1/s", "gains", 0, 500, 100.0),
    Register(13, "turn_gain", "Turn controller gain", "1/s", "gains", 0, 500, 100.0),
    Register(
        14, "strafe_pattern", "Strafe +Y wheel pattern (FL, FR, RL, RR)", "bits", "flags", 0, 15,
        bits=(
            BitSpec(0, "FL", "M2", "+Y strafe - front-left wheel: 1 = driven forward, 0 = backward"),
            BitSpec(1, "FR", "M1", "+Y strafe - front-right wheel: 1 = driven forward, 0 = backward"),
            BitSpec(2, "RL", "M3", "+Y strafe - rear-left wheel: 1 = driven forward, 0 = backward"),
            BitSpec(3, "RR", "M0", "+Y strafe - rear-right wheel: 1 = driven forward, 0 = backward"),
        ),
        bits_hint="1 = driven forward, 0 = backward (this is not an inversion)",
    ),
    Register(
        15, "steer_invert_mask", "Motor direction inversions (per wheel)", "bits", "flags", 0, 15,
        bits=(
            BitSpec(MOTOR_ID_FL, "FL", "M2", "Invert the front-left drive direction (motor ID 2)"),
            BitSpec(MOTOR_ID_FR, "FR", "M1", "Invert the front-right drive direction (motor ID 1)"),
            BitSpec(MOTOR_ID_RL, "RL", "M3", "Invert the rear-left drive direction (motor ID 3)"),
            BitSpec(MOTOR_ID_RR, "RR", "M0", "Invert the rear-right drive direction (motor ID 0)"),
        ),
    ),
    Register(
        16, "encoder_invert_mask", "Encoder direction inversions (L, R)", "bits", "flags", 0, 3,
        bits=(
            BitSpec(0, "L", "left", "Invert the LEFT odometry encoder counting direction"),
            BitSpec(1, "R", "right", "Invert the RIGHT odometry encoder counting direction"),
        ),
    ),
)

REGISTER_COUNT = len(REGISTERS)
REGISTER_BY_INDEX: Dict[int, Register] = {register.index: register for register in REGISTERS}
REGISTER_BY_KEY: Dict[str, Register] = {register.key: register for register in REGISTERS}

#: Compile-time defaults (raw register values), mirroring robot_settings.c -
#: the macros live in App/Inc/robot_configs.h and the same table (with units)
#: is in Docs/STM_docs/03-i2c-protocol-spec.md section 6.
DEFAULTS: Dict[str, int] = {
    "map_version": MAP_VERSION,
    "wheel_diameter_mm": 600,
    "stepper_microsteps": 32,
    "robot_track_mm": 1480,
    "wheelbase_mm": 1480,
    "odom_wheel_circumference_mm": 2000,
    "odom_ticks_per_rev": 4000,
    "odom_track_mm": 1700,
    "max_speed_mm_s": 800,
    "min_speed_mm_s": 100,
    "accel_mm_s2": 600,
    "decel_mm_s2": 500,
    "heading_gain": 120,
    "turn_gain": 150,
    "strafe_pattern": 0b1001,
    "steer_invert_mask": 0,
    "encoder_invert_mask": 0,
}

#: Absolute protocol speed ceiling in mm/s.
MAX_SPEED_LIMIT_MM_S = 900


def default_raw() -> List[int]:
    """Raw register image of the factory defaults (index order)."""
    return [DEFAULTS[register.key] for register in REGISTERS]


def registers_to_dict(raw_values: Mapping[int, int]) -> Dict[str, float]:
    """Raw index->value mapping -> engineering value dict."""
    out: Dict[str, float] = {}
    for index, raw in raw_values.items():
        register = REGISTER_BY_INDEX.get(index)
        if register is not None:
            out[register.key] = register.decode(raw)
    return out


def validate_engineering(values: Mapping[str, float]) -> Dict[str, int]:
    """Validate engineering values and return the raw register image.

    Raises ``ValueError`` with a readable message on unknown keys, read-only
    registers or out-of-range values.
    """
    raw: Dict[str, int] = {}
    for key, value in values.items():
        register = REGISTER_BY_KEY.get(key)
        if register is None:
            raise ValueError(f"unknown settings register '{key}'")
        if register.read_only:
            raise ValueError(f"register '{key}' is read-only")
        try:
            encoded = register.encode(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"register '{key}': {value!r} is not a number") from exc
        if encoded < register.min_value or encoded > register.max_value:
            raise ValueError(
                f"register '{key}': {value} outside [{register.decode(register.min_value)}, "
                f"{register.decode(register.max_value)}] {register.unit}".strip()
            )
        raw[key] = encoded
    return raw


def chunk_writes(raw: Mapping[str, int]) -> List[Tuple[int, List[int]]]:
    """Split a raw register edit into contiguous runs a SET_CONFIG frame takes.

    Returns ``(offset, values)`` tuples; runs longer than ``SET_CHUNK_MAX``
    are split further. Non-contiguous edits are simply sent as several frames.
    """
    runs: List[Tuple[int, List[int]]] = []
    keys = sorted(raw.keys(), key=lambda key: REGISTER_BY_KEY[key].index)
    current_offset = -1
    current: List[int] = []
    for key in keys:
        index = REGISTER_BY_KEY[key].index
        if current and index != current_offset + len(current):
            runs.append((current_offset, current))
            current = []
        if not current:
            current_offset = index
        current.append(raw[key] % 0x10000)
        while len(current) >= SET_CHUNK_MAX:
            runs.append((current_offset, current[:SET_CHUNK_MAX]))
            current_offset += SET_CHUNK_MAX
            current = current[SET_CHUNK_MAX:]
    if current:
        runs.append((current_offset, current))
    return runs


def meta() -> List[Dict]:
    """UI/API metadata for every register."""
    return [register.as_meta() for register in REGISTERS]
