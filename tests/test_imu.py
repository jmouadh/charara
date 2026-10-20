"""IMU tests: MPU6050 driver on a fake bus + session sampler lifecycle.

Covered (Docs/09 sections 9-10): register setup and scaling, bus failures
becoming ImuError, the session never starting a reader for the mock
transport, an absent IMU being a normal logged state, the sampler feeding
the fusion, and a lost IMU exiting the task without touching anything else.

File:    tests/test_imu.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-05
"""

import asyncio
import math
import os
import sys
import time
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
os.environ.setdefault("ROBOT_TRANSPORT", "mock")

from software.config import settings  # noqa: E402
from software.sensors import mpu6050 as mpu_module  # noqa: E402
from software.sensors.mpu6050 import ImuError, MPU6050, MountOrientation  # noqa: E402
from software.session import RobotSession  # noqa: E402


class FakeBus:
    """Duck-typed smbus replacement: registers plus programmable failures."""

    def __init__(self, chip_id=0x68, gz_raw=0, gravity=(0.0, 0.0, 1.0), gyro_raw=None):
        """Create the fake bus: chip id, raw gyro Z, gravity [g], raw xyz."""
        self.chip_id = chip_id
        self.gz_raw = gz_raw
        self.gravity = gravity
        self.gyro_raw = gyro_raw
        self.fail = False
        self.writes = []

    def read_byte_data(self, address, register):
        """Read one register byte."""
        if self.fail:
            raise OSError("bus down")
        if register == MPU6050.REG_WHO_AM_I:
            return self.chip_id
        return 0

    def read_i2c_block_data(self, address, register, length):
        """Read a register block (the gyro and accel blocks are meaningful)."""
        if self.fail:
            raise OSError("bus down")
        if register == MPU6050.REG_ACCEL_XOUT_H:
            block = []
            for value in self.gravity:
                raw = int(round(float(value) * 16384.0)) & 0xFFFF
                block += [(raw >> 8) & 0xFF, raw & 0xFF]
            return block[:length]
        if self.gyro_raw is not None:
            block = []
            for value in self.gyro_raw:
                raw = int(value) & 0xFFFF
                block += [(raw >> 8) & 0xFF, raw & 0xFF]
            return block[:length]
        value = self.gz_raw & 0xFFFF
        return [0, 0, 0, 0, (value >> 8) & 0xFF, value & 0xFF][:length]

    def write_byte_data(self, address, register, value):
        """Record one register write."""
        if self.fail:
            raise OSError("bus down")
        self.writes.append((register, value))

    def close(self):
        """Close the fake bus (no-op)."""


class DriverTests(unittest.TestCase):
    """MPU6050 register-level behaviour."""

    def test_probe_and_configure_writes(self):
        """Probe reads the chip id; configure applies the expected registers."""
        bus = FakeBus()
        driver = MPU6050(bus=bus)
        self.assertEqual(driver.probe(), 0x68)
        driver.configure()
        written = dict(bus.writes)
        self.assertEqual(written[MPU6050.REG_PWR_MGMT_1], 0x01)  # wake + PLL
        self.assertEqual(written[MPU6050.REG_GYRO_CONFIG], 0x08)  # +/-500 dps
        self.assertEqual(written[MPU6050.REG_CONFIG], 0x03)  # DLPF code
        self.assertEqual(written[MPU6050.REG_SMPLRT_DIV], 9)  # 100 Hz

    def test_yaw_rate_scaling_and_sign(self):
        """Raw counts convert at the configured scale; a mirrored mount flips."""
        bus = FakeBus(gz_raw=int(10.0 * 65.5))
        self.assertAlmostEqual(MPU6050(bus=bus).read_yaw_rate_dps(), 10.0, places=1)
        mirrored = MPU6050(bus=bus, yaw_axis="-z")
        self.assertAlmostEqual(mirrored.read_yaw_rate_dps(), -10.0, places=1)

    def test_yaw_axis_validation_and_switching(self):
        """yaw_axis accepts x/y/z with an optional sign; anything else fails."""
        bus = FakeBus(gyro_raw=(int(7.0 * 65.5), 0, int(3.0 * 65.5)))
        driver = MPU6050(bus=bus, yaw_axis="-x")
        self.assertAlmostEqual(driver.read_yaw_rate_dps(), -7.0, places=1)
        driver.yaw_axis = "z"
        self.assertAlmostEqual(driver.read_yaw_rate_dps(), 3.0, places=1)
        with self.assertRaises(ImuError):
            driver.yaw_axis = "w"

    def test_accel_reads_gravity_in_g(self):
        """The accelerometer block converts at 16384 LSB/g (+/-2 g default)."""
        ax, ay, az = MPU6050(bus=FakeBus(gravity=(0.0, 0.0, 1.0))).read_accel_g()
        self.assertAlmostEqual(ax, 0.0, places=4)
        self.assertAlmostEqual(ay, 0.0, places=4)
        self.assertAlmostEqual(az, 1.0, places=4)

    def test_detect_orientation_upright_and_facing_down(self):
        """Gravity says which axis points up; facing down flips the sign."""
        upright = MPU6050(bus=FakeBus(gravity=(0.0, 0.0, 1.0))).detect_orientation(samples=2, delay_s=0.0)
        self.assertEqual((upright.axis, upright.sign), ("z", 1))
        self.assertEqual(upright.value, "z")
        self.assertAlmostEqual(upright.tilt_deg, 0.0, places=1)

        facing_down = MPU6050(bus=FakeBus(gravity=(0.0, 0.0, -1.0))).detect_orientation(samples=2, delay_s=0.0)
        self.assertEqual((facing_down.axis, facing_down.sign), ("z", -1))
        self.assertEqual(facing_down.value, "-z")

    def test_detect_orientation_reports_the_mounting_tilt(self):
        """A tilted mount is measured - reported, never corrected."""
        tilt = 2.0
        gravity = (0.0, math.sin(math.radians(tilt)), math.cos(math.radians(tilt)))
        result = MPU6050(bus=FakeBus(gravity=gravity)).detect_orientation(samples=2, delay_s=0.0)
        self.assertEqual(result.axis, "z")
        self.assertAlmostEqual(result.tilt_deg, tilt, places=1)

    def test_detect_orientation_on_a_side_mount(self):
        """The strongest gravity axis wins, whatever the bearing."""
        result = MPU6050(bus=FakeBus(gravity=(0.9, 0.1, 0.2))).detect_orientation(samples=2, delay_s=0.0)
        self.assertEqual((result.axis, result.sign), ("x", 1))

    def test_detect_orientation_needs_gravity(self):
        """An unusable gravity vector raises ImuError (degrade, never guess)."""
        driver = MPU6050(bus=FakeBus(gravity=(0.0, 0.0, 0.05)))
        with self.assertRaises(ImuError):
            driver.detect_orientation(samples=1, delay_s=0.0)

    def test_bias_average(self):
        """The calibration helper averages the samples."""
        bus = FakeBus(gz_raw=int(2.0 * 65.5))
        driver = MPU6050(bus=bus)
        self.assertAlmostEqual(driver.read_gyro_bias_dps(samples=3, delay_s=0.0), 2.0, places=1)

    def test_bus_errors_become_imu_error(self):
        """Every bus failure surfaces as ImuError, never OSError."""
        bus = FakeBus()
        bus.fail = True
        driver = MPU6050(bus=bus)
        with self.assertRaises(ImuError):
            driver.probe()
        with self.assertRaises(ImuError):
            driver.read_yaw_rate_dps()
        with self.assertRaises(ImuError):
            driver.configure()

    def test_silent_device_is_detected(self):
        """A WHO_AM_I of zero means nothing is answering."""
        driver = MPU6050(bus=FakeBus(chip_id=0))
        with self.assertRaises(ImuError):
            driver.probe()

    def test_missing_smbus2_raises_imu_error(self):
        """Without the smbus2 package the driver refuses to open a bus."""
        original = mpu_module.SMBus
        mpu_module.SMBus = None
        try:
            driver = MPU6050(bus_number=1)
            with self.assertRaises(ImuError):
                driver.open()
        finally:
            mpu_module.SMBus = original


class FakeDriver:
    """MPU6050 stand-in for the session tests."""

    def __init__(
        self,
        rate=12.0,
        bias=0.0,
        fail_open=False,
        fail_read=False,
        orientation=None,
        fail_detect=False,
        yaw_axis="z",
    ):
        """Configure the fake behaviour."""
        self.rate = rate
        self.bias = bias
        self.fail_open = fail_open
        self.fail_read = fail_read
        self.orientation = orientation  # MountOrientation, None = default z -
        self.fail_detect = fail_detect
        self.closed = False
        self.reads = 0
        self.detected = 0
        self.yaw_axis = yaw_axis  # mirrors the real driver's constructor

    def open(self):
        """Open the fake device."""
        if self.fail_open:
            raise ImuError("no device")

    def probe(self):
        """Return the standard chip id."""
        return 0x68

    def configure(self):
        """Configure the fake device (no-op)."""

    def detect_orientation(self, samples=8, delay_s=0.003):
        """Return the programmed mount detection (gravity read)."""
        self.detected += 1
        if self.fail_detect:
            raise ImuError("robot shaken at boot")
        if self.orientation is not None:
            return self.orientation
        return MountOrientation(axis="z", sign=-1, tilt_deg=1.5, gravity_g=1.0)

    def read_gyro_bias_dps(self):
        """Return the configured bias."""
        return self.bias

    def read_yaw_rate_dps(self):
        """Return the configured rate (or raise when programmed to fail)."""
        self.reads += 1
        if self.fail_read:
            raise ImuError("gone")
        return self.rate

    def close(self):
        """Close the fake device."""
        self.closed = True


class SlowDriver(FakeDriver):
    """FakeDriver whose reads really take time (coarse-clock proof).

    Windows ``time.monotonic`` advances in ~15.6 ms ticks, so a sampler
    loop that spins through reads instantly measures a zero interval; this
    driver blocks long enough that every interval crosses a clock tick.
    """

    def read_yaw_rate_dps(self):
        """Block briefly, then return the configured rate."""
        time.sleep(0.02)
        return super().read_yaw_rate_dps()


class SessionImuTests(unittest.IsolatedAsyncioTestCase):
    """Session IMU lifecycle: absent, present, lost - always optional."""

    def _session(self, **over):
        """Build a session with the IMU enabled and fast sampling in tests."""
        values = {"imu_enabled": True, "imu_rate_hz": 200.0}
        values.update(over)
        return RobotSession(replace(settings, **values))

    def _rest_odometry(self, x=0.0, y=0.0, theta=0.0, v=0.0):
        """A fake board odometry snapshot for the at-rest bias tests."""
        return SimpleNamespace(
            x_mm=x,
            y_mm=y,
            theta_deg=theta,
            v_left_mm_s=v,
            v_right_mm_s=v,
            is_idle=(v == 0.0),
        )

    async def test_mock_transport_never_starts_a_reader(self):
        """Mock mode must not touch any IMU code path."""
        session = self._session()
        session.transport = "mock"
        called = []
        session._make_imu_driver = lambda: called.append(1)
        await session.start_imu()
        self.assertEqual(called, [])
        self.assertEqual(session.imu_status()["state"], "off")
        self.assertFalse(session.imu_status()["present"])

    async def test_absent_imu_is_logged_once_and_inactive(self):
        """A missing IMU is one log line, then business as usual."""
        session = self._session()
        session.transport = "i2c"
        logs = []

        async def logger(level, message):
            """Capture one log line."""
            logs.append((level, message))

        session.logger = logger
        session._make_imu_driver = lambda: FakeDriver(fail_open=True)
        await session.start_imu()
        self.assertEqual(session.imu_status()["state"], "absent")
        self.assertFalse(session.imu_status()["present"])
        self.assertEqual(len(logs), 1)
        self.assertFalse(session.localization.gyro_active())

    async def test_sampler_feeds_the_fusion(self):
        """A present IMU feeds bias-corrected samples into the fusion."""
        session = self._session()
        session.transport = "i2c"
        driver = FakeDriver(rate=12.0)
        session._make_imu_driver = lambda: driver
        await session.start_imu()
        try:
            self.assertEqual(session.imu_status()["state"], "ready")
            for _ in range(50):
                if session.localization.gyro_active():
                    break
                await asyncio.sleep(0.01)
            self.assertTrue(session.localization.gyro_active())
            self.assertIn("+gyro", session.localization.status()["mode"])
            self.assertGreaterEqual(session.imu_status()["samples"], 1)
        finally:
            await session.stop_imu()
        self.assertTrue(driver.closed)

    async def test_gyro_scale_and_integral(self):
        """GYRO_SCALE scales the fused rate and the spin-check integral."""
        session = self._session(gyro_scale=2.0)
        session.transport = "i2c"
        session._make_imu_driver = lambda: SlowDriver(rate=10.0, bias=0.0)
        await session.start_imu()
        try:
            for _ in range(200):
                if session.imu_status()["samples"] >= 4:
                    break
                await asyncio.sleep(0.001)
            status = session.imu_status()
            self.assertGreaterEqual(status["samples"], 4)
            self.assertAlmostEqual(status["rate_deg_s"], 20.0, places=1)
            self.assertAlmostEqual(status["scale"], 2.0)
            # 20 deg/s over 3 sampled intervals of ~20 ms: roughly 1.2 deg
            self.assertGreater(status["integrated_deg"], 0.1)
        finally:
            await session.stop_imu()

    async def test_spin_scale_workflow(self):
        """Reset -> hand spin -> apply: the correction is relative and live."""
        session = self._session(gyro_scale=1.0)
        session.transport = "i2c"
        session._make_imu_driver = lambda: FakeDriver(rate=0.0)
        await session.start_imu()
        try:
            session.imu_reset_integral()
            self.assertEqual(session.imu_status()["integrated_deg"], 0.0)
            # pretend a 1-turn hand spin integrated 350 deg of yaw
            session.imu_integrated_deg = 350.0
            result = session.imu_spin_scale(1.0)
            self.assertAlmostEqual(result["applied"], 360.0 / 350.0, places=6)
            self.assertAlmostEqual(session.imu_status()["scale"], 360.0 / 350.0, places=6)
            # a second spin, now measured with the corrected scale, is stable
            session.imu_integrated_deg = 360.0
            stable = session.imu_spin_scale(1.0)
            self.assertAlmostEqual(stable["applied"], 360.0 / 350.0, places=6)
        finally:
            await session.stop_imu()

    def test_spin_scale_guards(self):
        """Bad turns and degenerate spins are refused; scale window is checked."""
        session = self._session()
        with self.assertRaises(ValueError):
            session.imu_spin_scale(1.0)  # nothing integrated yet
        session.imu_integrated_deg = 360.0
        with self.assertRaises(ValueError):
            session.imu_spin_scale(0.0)
        with self.assertRaises(ValueError):
            session.imu_apply_scale(5.0)

    async def test_recalibrate_bias_updates_live_value(self):
        """Bias re-averaging swaps the live value used by the sampler."""
        session = self._session()
        session.transport = "i2c"
        driver = FakeDriver(rate=0.0, bias=1.5)
        session._make_imu_driver = lambda: driver
        await session.start_imu()
        try:
            bias = await session.imu_recalibrate_bias()
            self.assertAlmostEqual(bias, 1.5, places=3)
            self.assertAlmostEqual(session.imu_status()["bias_deg_s"], 1.5, places=3)
        finally:
            await session.stop_imu()

    async def test_noise_measurement_reports_mean_and_sigma(self):
        """A still, noiseless fake IMU measures a zero mean and zero sigma."""
        session = self._session()
        session.transport = "i2c"
        driver = FakeDriver(rate=1.5, bias=1.5)  # (rate - bias) = 0 at rest
        session._make_imu_driver = lambda: driver
        await session.start_imu()
        try:
            for _ in range(100):
                if session.imu_status()["samples"] >= 2:
                    break
                await asyncio.sleep(0.01)
            result = await session.imu_measure_noise(seconds=0.5)
            self.assertGreaterEqual(result["samples"], 10)
            self.assertAlmostEqual(result["mean_deg_s"], 0.0, places=6)
            self.assertAlmostEqual(result["sigma_deg_s"], 0.0, places=6)
        finally:
            await session.stop_imu()

    def test_bias_refinement_needs_settled_rest(self):
        """The bias only moves after the pose held still for the settle time."""
        session = self._session()
        session.BIAS_REST_SETTLE_S = 1.0
        session.BIAS_REFINE_TAU_S = 0.01
        session.connected = True
        session.odometry = self._rest_odometry()
        session.imu_bias_deg_s = -1.5
        session._refine_bias_at_rest(0.0, 100.0)   # rest clock starts
        session._refine_bias_at_rest(0.0, 100.9)   # still settling: frozen
        self.assertAlmostEqual(session.imu_bias_deg_s, -1.5, places=6)
        session._refine_bias_at_rest(0.0, 101.1)   # settled: dt clock starts
        session._refine_bias_at_rest(0.0, 101.12)  # first real refinement
        self.assertGreater(session.imu_bias_deg_s, -1.5)

    def test_bias_refinement_freezes_on_any_motion(self):
        """Wheel speed, execution or a moving board pose freeze the bias."""
        session = self._session()
        session.connected = True
        session.BIAS_REST_SETTLE_S = 0.0
        session.BIAS_REFINE_TAU_S = 0.01
        session.imu_bias_deg_s = -1.5
        # wheels turning
        session.odometry = self._rest_odometry(v=50.0)
        for stamp in (100.0, 101.0, 102.0):
            session._refine_bias_at_rest(0.0, stamp)
        self.assertAlmostEqual(session.imu_bias_deg_s, -1.5, places=6)
        # the pose keeps changing (hand push, wheels still)
        for index, stamp in enumerate((200.0, 200.1, 200.2, 200.3)):
            session.odometry = self._rest_odometry(x=float(index * 40))
            session._refine_bias_at_rest(0.0, stamp)
        self.assertAlmostEqual(session.imu_bias_deg_s, -1.5, places=6)
        # a real rotation with still wheels (large raw rate): gated out
        session.odometry = self._rest_odometry()
        for stamp in (300.0, 300.1, 300.2, 300.3):
            session._refine_bias_at_rest(20.0, stamp)
        self.assertAlmostEqual(session.imu_bias_deg_s, -1.5, places=6)
        # an execution in progress (even between segments) freezes it
        session.executor = SimpleNamespace(is_running=lambda: True)
        for stamp in (400.0, 400.1, 400.2):
            session._refine_bias_at_rest(0.0, stamp)
        session.executor = None
        self.assertAlmostEqual(session.imu_bias_deg_s, -1.5, places=6)

    async def test_sampler_refines_a_stale_boot_bias_at_rest(self):
        """The running sampler low-passes a stale boot bias to the raw rate."""
        session = self._session()
        session.transport = "i2c"
        session.BIAS_REST_SETTLE_S = 0.05
        session.BIAS_REFINE_TAU_S = 0.02
        session._make_imu_driver = lambda: FakeDriver(rate=0.0, bias=-1.5)
        await session.start_imu()
        try:
            self.assertAlmostEqual(session.imu_status()["bias_deg_s"], -1.5, places=3)
            self.assertIsNone(session.imu_status()["rest_s"])
            session.connected = True
            session.odometry = self._rest_odometry()
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and abs(session.imu_bias_deg_s) >= 0.1:
                await asyncio.sleep(0.05)
            status = session.imu_status()
            self.assertEqual(status["state"], "ready")  # the sampler survived
            self.assertLess(abs(status["bias_deg_s"]), 0.1)
            self.assertIsNotNone(status["rest_s"])
            self.assertLess(abs(status["rate_deg_s"]), 0.2)
        finally:
            await session.stop_imu()

    async def test_manual_stop_and_start_cycle(self):
        """Stop disables the sampler (state stopped); start probes it again."""
        session = self._session()
        session.transport = "i2c"
        driver = FakeDriver(rate=0.0, bias=0.25)
        session._make_imu_driver = lambda: driver
        await session.start_imu()
        try:
            self.assertEqual(session.imu_status()["state"], "ready")
            for _ in range(50):
                if session.imu_status()["samples"] >= 1:
                    break
                await asyncio.sleep(0.01)
            await session.stop_imu_sampling()
            self.assertEqual(session.imu_status()["state"], "stopped")
            self.assertFalse(session.imu_status()["present"])
            self.assertIsNone(session._imu_task)
            samples = session.imu_status()["samples"]
            await asyncio.sleep(0.05)
            self.assertEqual(session.imu_status()["samples"], samples)  # no more reads
            # a second stop is idempotent; a second start while ready is a no-op
            await session.stop_imu_sampling()
            self.assertEqual(session.imu_status()["state"], "stopped")
            await session.start_imu_sampling()
            self.assertEqual(session.imu_status()["state"], "ready")
            task = session._imu_task
            await session.start_imu_sampling()
            self.assertIs(session._imu_task, task)
        finally:
            await session.stop_imu()

    async def test_manual_stop_keeps_absent_state(self):
        """Stopping an absent IMU does not pretend it was present."""
        session = self._session()
        session.transport = "i2c"
        session._make_imu_driver = lambda: FakeDriver(fail_open=True)
        await session.start_imu()
        self.assertEqual(session.imu_status()["state"], "absent")
        await session.stop_imu_sampling()
        self.assertEqual(session.imu_status()["state"], "absent")

    async def test_calibration_refuses_without_imu(self):
        """Bias/noise calibration fails cleanly when no IMU is present."""
        session = self._session()
        session.transport = "i2c"
        session._make_imu_driver = lambda: FakeDriver(fail_open=True)
        await session.start_imu()
        self.assertEqual(session.imu_status()["state"], "absent")
        with self.assertRaises(ValueError):
            await session.imu_recalibrate_bias()
        with self.assertRaises(ValueError):
            await session.imu_measure_noise(seconds=0.5)

    async def test_lost_imu_exits_the_sampler(self):
        """Repeated read failures end the task and report the IMU as lost."""
        session = self._session(imu_rate_hz=1000.0)
        session.transport = "i2c"
        session._make_imu_driver = lambda: FakeDriver(fail_read=True)
        await session.start_imu()
        for _ in range(200):
            if session.imu_status()["state"] == "lost":
                break
            await asyncio.sleep(0.01)
        self.assertEqual(session.imu_status()["state"], "lost")
        self.assertGreaterEqual(session.imu_status()["errors"], session.IMU_MAX_ERRORS)
        await session.stop_imu()

    async def test_auto_orientation_applies_the_detected_axis(self):
        """IMU_YAW_AXIS=auto: gravity detection decides the yaw axis and sign."""
        session = self._session(imu_yaw_axis="auto")
        session.transport = "i2c"
        orientation = MountOrientation(axis="z", sign=-1, tilt_deg=1.2, gravity_g=0.999)
        driver = FakeDriver(orientation=orientation)
        session._make_imu_driver = lambda: driver
        await session.start_imu()
        try:
            status = session.imu_status()
            self.assertEqual(status["state"], "ready")
            self.assertEqual(status["yaw_axis"], "-z")
            self.assertEqual(status["mount"]["axis"], "z")
            self.assertEqual(status["mount"]["sign"], -1)
            self.assertAlmostEqual(status["mount"]["tilt_deg"], 1.2, places=2)
            self.assertEqual(driver.detected, 1)
        finally:
            await session.stop_imu()

    async def test_fixed_yaw_axis_skips_the_detection(self):
        """An explicit IMU_YAW_AXIS is used as is - no gravity read at all."""
        session = self._session(imu_yaw_axis="-x")
        session.transport = "i2c"
        driver = FakeDriver(yaw_axis="-x")
        session._make_imu_driver = lambda: driver
        await session.start_imu()
        try:
            status = session.imu_status()
            self.assertEqual(status["yaw_axis"], "-x")
            self.assertIsNone(status["mount"])
            self.assertEqual(driver.detected, 0)
        finally:
            await session.stop_imu()

    def test_factory_passes_the_configured_axis(self):
        """_make_imu_driver maps IMU_YAW_AXIS (auto -> provisional z)."""
        self.assertEqual(self._session(imu_yaw_axis="-x")._make_imu_driver().yaw_axis, "-x")
        self.assertEqual(self._session(imu_yaw_axis="auto")._make_imu_driver().yaw_axis, "z")

    async def test_failed_detection_keeps_the_imu_usable(self):
        """A failed mount detection warns once, never loses the IMU."""
        session = self._session(imu_yaw_axis="auto")
        session.transport = "i2c"
        logs = []

        async def logger(level, message):
            """Capture one log line."""
            logs.append((level, message))

        session.logger = logger
        session._make_imu_driver = lambda: FakeDriver(fail_detect=True)
        await session.start_imu()
        try:
            self.assertEqual(session.imu_status()["state"], "ready")
            self.assertEqual(session.imu_status()["yaw_axis"], "z")
            self.assertTrue(
                any(level == "warn" and "mount detection failed" in message for level, message in logs)
            )
        finally:
            await session.stop_imu()
