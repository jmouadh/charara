"""The RPi software package - plan, play and command from the robot side.

Folder roles in this repository::

    charara/            the RPi repository (deployment: docker/, docs, ui)
    `-- charara/        the working checkout root (tests, strategies)
        `-- software/   THIS package - the Python software

Package layout (dependencies point downwards, ``software.core`` is pure
Python with no dependencies):

* ``software.core``    - domain: navigation, localization, world model,
                         mission engine (executor, orchestration, strategy)
* ``software.robot``   - board access: protocol codec, I2C/mock links, client
* ``software.sensors`` - RPi-local sensors: MPU6050 driver (optional - the
                         system degrades to wheel-only when absent)
* ``software.session`` - RobotSession: the shared composition root (link,
                         client, executor, localization, sensors, telemetry)
                         wrapped by the console and the brain alike
* ``software.console`` - the FastAPI console service (REST/WS + UI hosting)
                         used to configure, calibrate, test and plan
* ``software.brain``   - the RPi's main job: run a saved strategy headless
                         (plan -> execute -> avoid obstacles -> repeat)

File:    software/__init__.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.2.0
Date:    2026-10-19
"""

from __future__ import annotations

import os
from pathlib import Path

__version__ = "3.2.0"


def _build_id() -> str:
    """Build stamp written by ``docker/console.Dockerfile``.

    Empty in a plain checkout: the console then shows only the semver.
    """
    explicit = os.environ.get("CHARARA_BUILD", "").strip()
    if explicit:
        return explicit
    path = os.environ.get("CHARARA_BUILD_FILE", "/srv/BUILD_ID")
    try:
        return Path(path).read_text(encoding="utf-8").strip()
    except OSError:
        return ""


__build__ = _build_id()
