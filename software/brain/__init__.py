"""The RPi's main job, headless: play a saved strategy.

The console is the workshop (configure, calibrate, test, plan); the brain is
the player. It owns the robot link, the localization service, the executor
and a mission context, so the SAME planning / re-plan / avoidance engine the
console uses runs here without any web layer::

    python -m software.brain --strategy eurobot_main --transport i2c

Obstacle reports arrive through :meth:`Brain.set_obstacles` (the future
camera / companion-card integration wraps this or adds a small socket); the
watcher reacts exactly like the console's ``/ws/vision`` does.

File:    software/brain/__init__.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from .runner import Brain

__all__ = ["Brain"]
