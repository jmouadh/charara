"""Runtime layer: the process-wide state and its background services.

* ``state``  - :class:`AppState`, the object the API and mission layers share
* ``trace``  - I2C trace ring buffer

Importing :mod:`app.runtime.state` builds the single ``state`` instance the
whole service runs on (one board, one I2C bus, one field).

File:    software/console/runtime/__init__.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from .state import AppState, state

__all__ = ["AppState", "state"]
