"""REST API package: one router per domain, aggregated into ``router``.

URL paths are unchanged from the single-module version; all mutating robot
calls are serialized by the client lock and manual commands are refused while
a plan is executing (``deps.require_no_execution``).

* ``system``     - health, link mode, state, field, pose, board log, display, I2C trace
* ``motion``     - manual moves, execution control, E-STOP
* ``planning``   - plan / re-plan, last plan
* ``settings``   - limits, watchdog, board settings registers
* ``sensors``    - IMU / gyro calibration (spin scale, bias, noise) + health
* ``selftest``   - acceptance tests, readiness checklist, drift audits
* ``strategies`` - save / load / list / delete the saved strategies

File:    software/console/api/routes/__init__.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

from fastapi import APIRouter

from .motion import router as motion_router
from .planning import router as planning_router
from .selftest import router as selftest_router
from .sensors import router as sensors_router
from .settings import router as settings_router
from .strategies import router as strategies_router
from .system import router as system_router

router = APIRouter()
router.include_router(system_router)
router.include_router(motion_router)
router.include_router(planning_router)
router.include_router(settings_router)
router.include_router(sensors_router)
router.include_router(selftest_router)
router.include_router(strategies_router)
