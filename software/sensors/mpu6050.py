"""MPU6050 gyroscope driver (I2C; dedicated bus recommended, Docs/09 9.2).

Purpose: yaw-rate samples for the localization service and, through the
session, for the gyro forward stream to the STM32. The driver is deliberately
small and defensive: **every register access can raise** :class:`ImuError`,
and the caller decides how to degrade - a missing or interrupted IMU is a
normal operating state of this robot, never a fault.

Mounting: at the centre of the robot, axis-aligned (Docs/09 9.1); the robot
yaw rate is read from the gyro axis closest to the vertical. ``yaw_axis``
names that axis and its sign ("z", "-z", ...) - an IMU mounted upside down
(sensor facing down) is ``-z``. :meth:`MPU6050.detect_orientation` reads the
gravity vector once at boot and reports the axis pointing UP, so the session
configures the sign automatically and exposes the mounting tilt for the
health panel. A small tilt needs no correction: 1 deg scales the yaw rate
by cos(1 deg) = 0.9998 (Docs/09 9.1).

Registers used: WHO_AM_I(0x75), PWR_MGMT_1(0x6B), CONFIG(0x1A),
GYRO_CONFIG(0x1B), SMPLRT_DIV(0x19), GYRO_XOUT_H(0x43), ACCEL_XOUT_H(0x3B).

Gyro scaling: FS_SEL 0=+/-250, 1=+/-500, 2=+/-1000, 3=+/-2000 deg/s with
131.0 / 65.5 / 32.8 / 16.4 LSB per deg/s.

File:    software/sensors/mpu6050.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-05
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

try:  # smbus2 exists on the Pi; tests inject a fake bus instead
    from smbus2 import SMBus  # type: ignore
except ImportError:  # pragma: no cover - exercised only without smbus2
    SMBus = None


class ImuError(Exception):
    """Any IMU failure: bus error, missing device, invalid configuration."""


#: LSB per deg/s for each FS_SEL range (index = FS_SEL)
_SCALE_LSB_PER_DPS = (131.0, 65.5, 32.8, 16.4)

#: sensor axis names: the position of an axis inside a gyro/accel block
_AXES = "xyz"

#: acceleration LSB per g at AFS_SEL 0 (+/-2 g, the power-on default)
_ACCEL_LSB_PER_G = 16384.0


@dataclass(frozen=True)
class MountOrientation:
    """Gravity-based mount detection result (Docs/09 sections 9.1 and 9.3)."""

    axis: str         #: sensor axis closest to vertical ("x" | "y" | "z")
    sign: int         #: +1 when that axis points UP, -1 when it points down
    tilt_deg: float   #: angle between that axis and the measured gravity [deg]
    gravity_g: float  #: magnitude of the averaged gravity vector [g]

    @property
    def value(self) -> str:
        """The axis as a signed string for ``MPU6050.yaw_axis``."""
        return self.axis if self.sign > 0 else f"-{self.axis}"


class MPU6050:
    """Minimal MPU6050 driver: probe, configure, read the yaw rate.

    ``bus`` can be injected (any object with ``read_byte_data``,
    ``read_i2c_block_data``, ``write_byte_data`` and ``close``) so tests and
    simulations run without hardware; otherwise the driver opens the given
    I2C bus number on ``open()``.
    """

    ADDRESS = 0x68
    #: WHO_AM_I values seen on the MPU60x0 family (0x68 = MPU6050)
    KNOWN_IDS = (0x68, 0x70, 0x71, 0x73)

    REG_SMPLRT_DIV = 0x19
    REG_CONFIG = 0x1A
    REG_GYRO_CONFIG = 0x1B
    REG_ACCEL_XOUT_H = 0x3B
    REG_GYRO_XOUT_H = 0x43
    REG_PWR_MGMT_1 = 0x6B
    REG_WHO_AM_I = 0x75

    def __init__(
        self,
        bus_number: Optional[int] = None,
        address: int = ADDRESS,
        *,
        bus=None,
        gyro_range_dps: int = 500,
        sample_rate_hz: float = 100.0,
        dlpf_code: int = 0x03,
        yaw_axis: str = "z",
    ) -> None:
        """Create the driver; no bus traffic until :meth:`open` + probe."""
        self._bus_number = bus_number
        self._address = int(address)
        self._bus = bus
        self._owns_bus = bus is None
        self._dlpf_code = int(dlpf_code) & 0x07
        if gyro_range_dps not in (250, 500, 1000, 2000):
            gyro_range_dps = 500
        self._fs_sel = {250: 0, 500: 1, 1000: 2, 2000: 3}[gyro_range_dps]
        self._scale = _SCALE_LSB_PER_DPS[self._fs_sel]
        # with the DLPF enabled the gyro output is 1 kHz: divide it down
        divisor = round(1000.0 / max(4.0, float(sample_rate_hz))) - 1
        self._smplrt_div = min(255, max(0, divisor))
        self._yaw_axis = "z"
        self._yaw_sign = 1
        self.yaw_axis = yaw_axis  # validates; "-z" = sensor facing down

    # ------------------------------------------------------------ lifecycle
    def open(self) -> None:
        """Open the I2C bus (no-op when a bus object was injected)."""
        if self._bus is not None:
            return
        if SMBus is None:
            raise ImuError("smbus2 is not installed (pip install smbus2)")
        try:
            self._bus = SMBus(self._bus_number if self._bus_number is not None else 1)
        except OSError as exc:
            raise ImuError(f"cannot open i2c bus {self._bus_number}: {exc}") from exc

    def close(self) -> None:
        """Close the bus when this driver opened it (errors ignored)."""
        if self._bus is None or not self._owns_bus:
            return
        try:
            self._bus.close()
        except Exception:
            pass
        self._bus = None

    # --------------------------------------------------------------- setup
    def probe(self) -> int:
        """Return the WHO_AM_I chip id; raises :class:`ImuError` on bus error."""
        chip = self._read_u8(self.REG_WHO_AM_I)
        if chip == 0:
            raise ImuError("WHO_AM_I reads 0x00 (no device answering)")
        return chip

    def configure(self) -> None:
        """Wake the device and apply DLPF, gyro range and sample rate."""
        # PWR_MGMT_1: clear SLEEP, clock source = PLL with X gyro (1)
        self._write_u8(self.REG_PWR_MGMT_1, 0x01)
        # CONFIG: DLPF code (also switches the gyro output rate to 1 kHz)
        self._write_u8(self.REG_CONFIG, self._dlpf_code)
        # GYRO_CONFIG: FS_SEL in bits 4:3
        self._write_u8(self.REG_GYRO_CONFIG, self._fs_sel << 3)
        # SMPLRT_DIV: sample rate = 1 kHz / (1 + div)
        self._write_u8(self.REG_SMPLRT_DIV, self._smplrt_div)

    # -------------------------------------------------------------- reading
    @property
    def yaw_axis(self) -> str:
        """Signed yaw axis in use, e.g. ``"z"`` (up) or ``"-z"`` (facing down)."""
        return self._yaw_axis if self._yaw_sign > 0 else f"-{self._yaw_axis}"

    @yaw_axis.setter
    def yaw_axis(self, value: str) -> None:
        """Set the yaw axis from ``x``, ``y``, ``z``, ``-x``, ``-y`` or ``-z``."""
        text = str(value or "").strip().lower()
        sign = -1 if text.startswith("-") else 1
        axis = text.lstrip("+-")
        if axis not in _AXES:
            raise ImuError(f"invalid yaw axis '{value}' (use x, y, z, -x, -y or -z)")
        self._yaw_axis, self._yaw_sign = axis, sign

    def read_yaw_rate_dps(self) -> float:
        """Read the robot yaw rate in deg/s (the configured yaw axis, signed).

        Raises :class:`ImuError` on any bus failure so callers can degrade.
        """
        raw = self._read_block(self.REG_GYRO_XOUT_H, 6)
        index = _AXES.index(self._yaw_axis)
        value = int.from_bytes(bytes(raw[index * 2:index * 2 + 2]), "big", signed=True)
        return self._yaw_sign * value / self._scale

    def read_yaw_rate_raw(self) -> Tuple[int, int, int]:
        """Read the raw (x, y, z) gyro counts (diagnostics/calibration)."""
        block = self._read_block(self.REG_GYRO_XOUT_H, 6)
        return (
            int.from_bytes(bytes(block[0:2]), byteorder="big", signed=True),
            int.from_bytes(bytes(block[2:4]), byteorder="big", signed=True),
            int.from_bytes(bytes(block[4:6]), byteorder="big", signed=True),
        )

    def read_accel_g(self) -> Tuple[float, float, float]:
        """Read the acceleration in g (AFS_SEL 0 = +/-2 g, the reset value)."""
        block = self._read_block(self.REG_ACCEL_XOUT_H, 6)
        return (
            int.from_bytes(bytes(block[0:2]), byteorder="big", signed=True) / _ACCEL_LSB_PER_G,
            int.from_bytes(bytes(block[2:4]), byteorder="big", signed=True) / _ACCEL_LSB_PER_G,
            int.from_bytes(bytes(block[4:6]), byteorder="big", signed=True) / _ACCEL_LSB_PER_G,
        )

    def detect_orientation(self, samples: int = 8, delay_s: float = 0.003) -> MountOrientation:
        """Identify the sensor axis pointing UP, from the gravity vector.

        The MPU6050 reports +1 g on whichever axis points up, so an IMU
        mounted upside down (the sensor facing down) reads -1 g on its
        vertical axis: the result then has ``sign = -1`` and ``value`` is
        the ``"-z"``-style string to assign to :attr:`yaw_axis`. The tilt
        is how far the measured gravity is from the ideal axis - the
        mounting imperfection, reported for health, never corrected
        (Docs/09 9.1).

        Raises :class:`ImuError` when the gravity vector is unusable (no
        device answering, the robot being shaken, free fall).
        """
        count = max(1, int(samples))
        gx = gy = gz = 0.0
        for index in range(count):
            ax, ay, az = self.read_accel_g()
            gx += ax
            gy += ay
            gz += az
            if delay_s > 0.0 and index + 1 < count:
                time.sleep(delay_s)
        gx /= count
        gy /= count
        gz /= count
        norm = math.sqrt(gx * gx + gy * gy + gz * gz)
        if not math.isfinite(norm) or norm < 0.3:
            raise ImuError(f"gravity vector unusable ({norm:.2f} g) - cannot detect the mount")
        values = (gx, gy, gz)
        index = max(range(3), key=lambda i: abs(values[i]))
        axis = _AXES[index]
        sign = 1 if values[index] >= 0.0 else -1
        cos_angle = (values[index] * sign) / norm
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, cos_angle))))
        return MountOrientation(axis=axis, sign=sign, tilt_deg=tilt, gravity_g=norm)

    def read_gyro_bias_dps(self, samples: int = 100, delay_s: float = 0.002) -> float:
        """Average ``samples`` yaw-rate reads: the bias at standstill.

        Called at boot (robot known at rest). A device that stops answering
        mid-calibration raises :class:`ImuError` (the caller then treats the
        IMU as absent).
        """
        count = max(1, int(samples))
        total = 0.0
        for _ in range(count):
            total += self.read_yaw_rate_dps()
            if delay_s > 0.0:
                time.sleep(delay_s)
        return total / count

    # ------------------------------------------------------------- plumbing
    def _read_u8(self, register: int) -> int:
        """Read one byte; wraps bus errors into :class:`ImuError`."""
        try:
            return int(self._bus.read_byte_data(self._address, register))
        except OSError as exc:
            raise ImuError(f"i2c read failed (reg 0x{register:02X}): {exc}") from exc

    def _read_block(self, register: int, length: int) -> List[int]:
        """Read a register block; wraps bus errors into :class:`ImuError`."""
        try:
            block = self._bus.read_i2c_block_data(self._address, register, length)
        except OSError as exc:
            raise ImuError(f"i2c block read failed (reg 0x{register:02X}): {exc}") from exc
        if len(block) != length:
            raise ImuError(f"short i2c read: {len(block)} != {length}")
        return [int(value) & 0xFF for value in block]

    def _write_u8(self, register: int, value: int) -> None:
        """Write one byte; wraps bus errors into :class:`ImuError`."""
        try:
            self._bus.write_byte_data(self._address, register, int(value) & 0xFF)
        except OSError as exc:
            raise ImuError(f"i2c write failed (reg 0x{register:02X}): {exc}") from exc
