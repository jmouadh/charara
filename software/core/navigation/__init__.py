"""Path planning domain (pure, portable - no I/O, no service code).

* ``geometry``   - curve fitting, segmentation, fillets, resampling
* ``grid``       - occupancy grid + Theta* routing
* ``velocity``   - trapezoidal speed profile
* ``options``    - PlanOptions (per-request planning knobs)
* ``clearance``  - robot footprint clearances + grid construction
* ``trajectory`` - the planning pipeline: elements -> motion program
* ``replan``     - slicing / remaining-route helpers for mid-run re-plans

File:    software/core/navigation/__init__.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""
