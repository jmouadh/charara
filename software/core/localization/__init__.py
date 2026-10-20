"""Localization: the RPi's own pose estimate (wheel odometry + future IMU).

* ``kinematics`` - wheel twist and dead reckoning (pure math)
* ``kalman``     - 4-state pose filter ``[x, y, theta, omega]`` (pure Python)
* ``service``    - LocalizationService: feed it board samples, read the pose

The rest of the service consumes :meth:`LocalizationService.pose` instead of
the raw board odometry, so upgrading the estimator (IMU fusion, map
constraints, ...) never reaches into planning or execution code.

File:    software/core/localization/__init__.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from .service import LocalizationService, WheelSample

__all__ = ["LocalizationService", "WheelSample"]
