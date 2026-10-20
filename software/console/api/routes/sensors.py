"""Sensor routes: IMU / gyro calibration and health (Docs/09 section 9.3).

The MPU6050 lives on the robot's own RPi (I2C6), not on the board: these
endpoints tune the RPi-side sampler the same way the settings registers tune
the board - from the console's Calibration tab.

* the hand-spin workflow derives ``GYRO_SCALE`` from the live yaw integral
  (``imu.integrated_deg``) and applies it at once;
* ``POST /bias`` re-averages the gyro bias while the robot stands still;
* ``POST /noise`` measures the rest noise and suggests ``GYRO_SIGMA_DEG_S``.

Every value applies LIVE (no restart) but is not persisted: the response
carries the exact ``docker/.env`` line to keep it across restarts.

File:    software/console/api/routes/sensors.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-09
"""

from __future__ import annotations

from typing import Dict

from fastapi import APIRouter, HTTPException

from ...runtime.state import state
from .deps import require_no_execution
from .models import ImuNoiseIn, ImuScaleIn

router = APIRouter()


def _imu_session():
    """The session when a healthy IMU sampler is running, else HTTP 409."""
    session = state.session
    status = session.imu_status()
    if not status.get("present") or status.get("state") != "ready":
        raise HTTPException(
            status_code=409,
            detail=f"IMU not available (state: {status.get('state')})",
        )
    return session


@router.get("/sensors/imu")
async def get_imu_status() -> Dict:
    """IMU health, gyro stream state and re-anchor state (calibration tab)."""
    session = state.session
    status = session.imu_status()
    return {
        "status": status,
        "stream": session.gyro_stream_status(),
        "reanchor": session.reanchor_status(),
        "available": bool(status.get("present")) and status.get("state") == "ready",
    }


@router.post("/sensors/imu/reset")
async def reset_imu_integral() -> Dict:
    """Zero the spin-test integral (start of the hand-rotation check)."""
    session = _imu_session()
    session.imu_reset_integral()
    await state.log("info", "IMU calibration: yaw integral reset - rotate the robot now")
    return {"ok": True, "integrated_deg": 0.0}


@router.post("/sensors/imu/stop")
async def stop_imu() -> Dict:
    """Stop the IMU sampler on purpose (bench fallback test, STM_docs/06 8-4).

    Allowed DURING motion - that is the point: the sampler and the gyro
    stream stop, the board returns to wheel heading within ~200 ms and the
    fusion degrades to wheels, never a fault. No wire work required.
    """
    session = state.session
    await session.stop_imu_sampling()
    await state.log(
        "warn", "IMU stopped on purpose (bench fallback test) - the board is on wheel heading"
    )
    return {"ok": True, "state": session.imu_status()["state"]}


@router.post("/sensors/imu/start")
async def start_imu() -> Dict:
    """Re-probe the IMU after a manual stop (idle only: bias re-average)."""
    require_no_execution()
    session = state.session
    await session.start_imu_sampling()
    status = session.imu_status()
    if status["state"] != "ready":
        raise HTTPException(status_code=409, detail=f"IMU not available (state: {status['state']})")
    await state.log("info", f"IMU restarted: bias re-averaged ({status['bias_deg_s']:.3f} deg/s)")
    return {"ok": True, "state": "ready", "bias_deg_s": status["bias_deg_s"]}


@router.post("/sensors/imu/scale")
async def apply_imu_scale(body: ImuScaleIn) -> Dict:
    """Apply a gyro scale directly, or compute it from a finished spin.

    Two callers: the spin workflow sends ``turns_done`` (the correction is
    derived from the live integral), manual tuning sends ``scale``. The
    value applies immediately and is echoed as the docker/.env line to keep.
    """
    require_no_execution()
    session = _imu_session()
    try:
        if body.scale is not None:
            previous, applied = session.imu_apply_scale(body.scale)
            await state.log(
                "info", f"IMU calibration: gyro scale {previous:.4g} -> {applied:.4g} (manual)"
            )
            return {
                "ok": True,
                "previous": previous,
                "applied": applied,
                "env_line": f"GYRO_SCALE={applied:.6g}",
            }
        turns = 1.0 if body.turns_done is None else float(body.turns_done)
        result = session.imu_spin_scale(turns)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    applied = result["applied"]
    await state.log(
        "info",
        f"IMU calibration: gyro scale {result['previous']:.4g} -> {applied:.4g} "
        f"({result['turns']:.2f} turn(s), integrated {result['integrated_deg']:.1f} deg)",
    )
    return {"ok": True, **result, "env_line": f"GYRO_SCALE={applied:.6g}"}


@router.post("/sensors/imu/bias")
async def recalibrate_imu_bias() -> Dict:
    """Re-average the gyro bias while the robot stands still (~0.2 s)."""
    require_no_execution()
    session = _imu_session()
    try:
        bias = await session.imu_recalibrate_bias()
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    await state.log("info", f"IMU calibration: bias re-averaged ({bias:.3f} deg/s)")
    return {"ok": True, "bias_deg_s": bias}


@router.post("/sensors/imu/noise")
async def measure_imu_noise(body: ImuNoiseIn) -> Dict:
    """Measure the yaw-rate noise at rest and suggest GYRO_SIGMA_DEG_S."""
    require_no_execution()
    session = _imu_session()
    try:
        result = await session.imu_measure_noise(body.seconds)
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    sigma = max(0.1, round(result["sigma_deg_s"], 2))
    await state.log(
        "info",
        "IMU calibration: noise over %.1f s - mean %.3f deg/s, sigma %.3f deg/s"
        % (result["seconds"], result["mean_deg_s"], result["sigma_deg_s"]),
    )
    return {
        "ok": True,
        **result,
        "sigma_suggested": sigma,
        "env_line": f"GYRO_SIGMA_DEG_S={sigma:g}",
    }
