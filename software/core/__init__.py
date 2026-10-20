"""Pure domain core: everything the robot knows, with no I/O and no deps.

* ``navigation``   - geometry, occupancy grid, routing, trajectory planning,
                     re-plan slicing, velocity profiles (pure math).
* ``localization`` - wheel kinematics + Kalman filter + fusion service
                     (gyro-ready), producing the pose everyone consumes.
* ``world``        - the obstacle map model (camera reports + merge rules).
* ``mission``      - executor (runs a motion program on a robot port),
                     orchestration (plan / mid-run re-plan), strategy model.

File:    software/core/__init__.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""
