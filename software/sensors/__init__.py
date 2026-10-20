"""RPi-local sensors (IMU). All optional; failures never block the robot.

File:    software/sensors/__init__.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-05
"""

from .mpu6050 import ImuError, MPU6050

__all__ = ["ImuError", "MPU6050"]
