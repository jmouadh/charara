"""World model: everything the service knows about its surroundings.

* ``vision`` - the camera obstacle store (snapshot + TTL + source tracking)
* ``poses``  - the camera robot-pose store (us + opponent, raw precision)

The world layer is pure data: the transport (``api``) writes into it, the
mission layer (re-plan decisions) and the fusion read from it.

File:    software/core/world/__init__.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-05
"""

from .poses import PoseStore
from .vision import VisionStore, merged_obstacles

__all__ = ["PoseStore", "VisionStore", "merged_obstacles"]
