"""Game clock and timed-action tests (pure runtime + document normalisation).

The match clock is server-authoritative: these tests drive it with a fake
monotonic clock so no test sleeps. They cover the limit latch, the red-warning
window, once-per-match action dispatch, the documented REST placeholder and
the forgiving game-document normalisation.

File:    tests/test_game_time.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 1.0.0
Date:    2026-11-03
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
# Importing the console state builds the process-wide Settings: force the
# mock transport BEFORE it, exactly like the other API suites. Without this
# a subset run (this file first) would boot the real i2c transport for the
# TestClient in test_map_api and its telemetry never answers the mock tests.
os.environ.setdefault("ROBOT_TRANSPORT", "mock")

from software.console.runtime.state import AppState  # noqa: E402
from software.core.mission.game_time import (  # noqa: E402
    GameClock,
    TimeActionContext,
    dispatch_time_action,
    register_time_action_handler,
)
from software.core.world.map_config import (  # noqa: E402
    DEFAULT_GAME_DURATION_S,
    DEFAULT_GAME_WARNING_S,
    MapConfig,
    normalize_game,
)

FIELD_DIR = Path(__file__).resolve().parents[1] / "ui" / "static" / "field"


class FakeTime:
    """A monotonic clock the test advances by hand."""

    def __init__(self):
        """Start at zero."""
        self.now = 0.0

    def __call__(self):
        """Current fake time."""
        return self.now

    def advance(self, seconds):
        """Move the clock forward."""
        self.now += float(seconds)


class GameClockTests(unittest.TestCase):
    """Start / warn / limit / once-per-match behaviour."""

    def setUp(self):
        """Fresh clock with the Eurobot-like 100 s / last 10 s config."""
        self.time = FakeTime()
        self.clock = GameClock(self.time)
        self.clock.configure({
            "duration_s": 100.0,
            "warning_s": 10.0,
            "actions": [
                {"id": "late", "at_s": 95.0, "kind": "log", "enabled": True,
                 "params": {"message": "last 5 s"}},
                {"id": "off", "at_s": 50.0, "kind": "log", "enabled": False,
                 "params": {"message": "disabled"}},
            ],
        })

    def test_clock_counts_down_and_warns_in_the_window(self):
        """The clock starts at the duration, warns in the last N seconds."""
        self.assertFalse(self.clock.status()["running"])
        self.assertTrue(self.clock.start())
        self.assertEqual(self.clock.status()["remaining_s"], 100.0)
        self.time.advance(89.0)
        status = self.clock.status()
        self.assertFalse(status["warning"], status)
        self.time.advance(1.0)
        self.assertTrue(self.clock.status()["warning"], "last 10 s must be red")

    def test_limit_latches_once(self):
        """expire_if_due returns True exactly once at the match limit."""
        self.clock.start()
        self.time.advance(99.9)
        self.assertFalse(self.clock.expire_if_due())
        self.time.advance(0.2)
        self.assertTrue(self.clock.expire_if_due())
        self.assertFalse(self.clock.expire_if_due(), "the limit is latched")
        self.assertTrue(self.clock.status()["expired"])

    def test_due_actions_fire_once_and_disabled_ones_never(self):
        """Only enabled actions at/before elapsed time are due, once each."""
        self.clock.start()
        self.time.advance(50.0)
        self.assertEqual(self.clock.due_actions(), [], "disabled action at 50 s")
        self.time.advance(45.0)
        due = self.clock.due_actions()
        self.assertEqual([action["id"] for action in due], ["late"])
        self.clock.mark_fired("late")
        self.assertEqual(self.clock.due_actions(), [])
        self.assertTrue(self.clock.status()["actions"][0]["fired"])

    def test_reset_rewinds_and_rearms_actions(self):
        """Reset stops the clock and clears the fired history."""
        self.clock.start()
        self.time.advance(95.0)
        self.clock.mark_fired("late")
        self.clock.reset()
        self.assertFalse(self.clock.status()["running"])
        self.assertFalse(self.clock.status()["actions"][0]["fired"])
        self.assertTrue(self.clock.start())
        self.assertEqual(self.clock.status()["elapsed_s"], 0.0)

    def test_pause_freezes_elapsed_and_resume_continues(self):
        """Pause / Resume freeze and continue the match time exactly."""
        self.clock.start()
        self.time.advance(30.0)
        self.assertTrue(self.clock.pause())
        status = self.clock.status()
        self.assertTrue(status["paused"])
        self.assertTrue(status["running"], "a paused clock is still a running match")
        self.time.advance(20.0)
        self.assertEqual(self.clock.status()["elapsed_s"], 30.0)
        self.assertEqual(self.clock.status()["remaining_s"], 70.0)
        self.assertFalse(self.clock.pause(), "already paused")
        self.assertTrue(self.clock.resume())
        self.assertFalse(self.clock.status()["paused"])
        self.time.advance(5.0)
        self.assertEqual(self.clock.status()["elapsed_s"], 35.0)
        self.assertEqual(self.clock.status()["remaining_s"], 65.0)
        self.assertFalse(self.clock.resume(), "not paused any more")

    def test_a_paused_clock_fires_nothing_and_cannot_expire(self):
        """No timed action and no limit latch while the clock is frozen."""
        self.clock.start()
        self.time.advance(96.0)
        self.clock.pause()
        self.time.advance(60.0)
        self.assertEqual(self.clock.due_actions(), [])
        self.assertFalse(self.clock.expire_if_due())
        self.assertFalse(self.clock.status()["expired"])
        self.clock.resume()
        self.assertEqual(
            [action["id"] for action in self.clock.due_actions()], ["late"]
        )
        self.time.advance(5.0)  # 101 s elapsed: the limit is due
        self.assertTrue(self.clock.expire_if_due())

    def test_reset_clears_a_paused_clock(self):
        """Redo (reset) restarts the counters from zero, even when paused."""
        self.clock.start()
        self.time.advance(40.0)
        self.clock.pause()
        self.clock.reset()
        status = self.clock.status()
        self.assertFalse(status["running"])
        self.assertFalse(status["paused"])
        self.assertEqual(status["remaining_s"], 100.0)
        self.assertTrue(self.clock.start())
        self.assertEqual(self.clock.status()["elapsed_s"], 0.0)

    def test_zero_duration_disables_the_clock(self):
        """A duration of 0 never starts and never expires."""
        clock = GameClock(self.time)
        clock.configure({"duration_s": 0, "warning_s": 10, "actions": []})
        self.assertFalse(clock.configured)
        self.assertFalse(clock.start())
        self.assertFalse(clock.expire_if_due())


class GameClockPauseStateTests(unittest.IsolatedAsyncioTestCase):
    """The state helpers the Pause / Resume / Execute routes call."""

    def _state(self, clock, transport="mock"):
        """Minimal AppState surface for the game-clock helpers."""
        logs = []
        events = []

        async def log(level, message):
            """Capture one log line."""
            logs.append((level, message))

        async def emit(event):
            """Capture one console event."""
            events.append(event)

        state = SimpleNamespace(
            game_clock=clock, transport=transport, log=log, emit=emit,
        )
        state.game_status = lambda: AppState.game_status(state)
        state.game_resume = lambda: AppState.game_resume(state)
        state.game_start = lambda: AppState.game_start(state)
        return state, logs, events

    async def test_pause_and_resume_publish_the_status(self):
        """Pause freezes the clock, Resume continues it, both go to the UI."""
        time = FakeTime()
        clock = GameClock(time)
        clock.configure({"duration_s": 100, "warning_s": 10, "actions": []})
        clock.start()
        time.advance(12.0)
        state, logs, events = self._state(clock)

        status = await AppState.game_pause(state)
        self.assertTrue(status["paused"])
        self.assertEqual(status["elapsed_s"], 12.0)
        self.assertEqual(events[-1]["type"], "game")
        self.assertTrue(any("paused" in message for _lvl, message in logs))

        time.advance(8.0)
        status = await AppState.game_resume(state)
        self.assertFalse(status["paused"])
        self.assertEqual(status["elapsed_s"], 12.0)
        time.advance(3.0)
        self.assertEqual(clock.elapsed_s(), 15.0)

    async def test_execute_continues_a_paused_clock(self):
        """Execute after a Pause resumes the match time instead of ignoring it."""
        time = FakeTime()
        clock = GameClock(time)
        clock.configure({"duration_s": 100, "warning_s": 10, "actions": []})
        clock.start()
        time.advance(10.0)
        clock.pause()
        state, _logs, _events = self._state(clock)
        await AppState.game_start_if_configured(state)
        self.assertFalse(clock.paused)
        time.advance(5.0)
        self.assertEqual(clock.elapsed_s(), 15.0)


class GameActionTests(unittest.IsolatedAsyncioTestCase):
    """The pluggable handler architecture and its two shipped handlers."""

    async def test_log_action_writes_a_line_and_emits_an_event(self):
        """The reference action logs and reports status 'done'."""
        logs = []
        events = []

        async def log(level, message):
            """Capture one log line."""
            logs.append((level, message))

        async def emit(event):
            """Capture one console event."""
            events.append(event)

        context = TimeActionContext(log=log, emit=emit)
        result = await dispatch_time_action({
            "id": "a1", "at_s": 95.0, "kind": "log",
            "params": {"message": "release the harvester"},
        }, context)
        self.assertEqual(result["status"], "done")
        self.assertTrue(any("95.0 s" in message and "release" in message for _lvl, message in logs))
        self.assertEqual(events[-1]["type"], "time_action")
        self.assertEqual(events[-1]["data"]["id"], "a1")

    async def test_rest_action_is_a_documented_placeholder(self):
        """The REST kind is accepted but performs no network request yet."""
        logs = []
        events = []

        async def log(level, message):
            """Capture one log line."""
            logs.append((level, message))

        async def emit(event):
            """Capture one console event."""
            events.append(event)

        context = TimeActionContext(log=log, emit=emit)
        result = await dispatch_time_action({
            "id": "a2", "at_s": 95.0, "kind": "rest",
            "params": {"url": "http://actuator/trigger", "method": "POST"},
        }, context)
        self.assertEqual(result["status"], "pending")
        self.assertTrue(any("no REST handler" in message for _lvl, message in logs))
        self.assertEqual(events[-1]["data"]["status"], "pending")

    async def test_a_future_handler_can_replace_a_kind(self):
        """Registering a handler is the only integration hook a REST bridge needs."""
        calls = []

        class FakeRest:
            """Stand-in for the future actuator bridge."""

            kind = "rest"

            async def run(self, action, context):
                """Record the call and succeed."""
                calls.append(action["params"]["url"])
                return {"status": "done", "message": "called"}

        async def noop(*args, **kwargs):
            """Discard logs/events."""
            return None

        original = None
        from software.core.mission import game_time as module
        original = module._HANDLERS["rest"]
        register_time_action_handler(FakeRest())
        try:
            context = TimeActionContext(log=noop, emit=noop)
            result = await dispatch_time_action({
                "id": "a3", "at_s": 95.0, "kind": "rest",
                "params": {"url": "http://actuator/trigger"},
            }, context)
        finally:
            module._HANDLERS["rest"] = original
        self.assertEqual(result["status"], "done")
        self.assertEqual(calls, ["http://actuator/trigger"])


class GameTickTests(unittest.IsolatedAsyncioTestCase):
    """The console tick: REAL stops at the limit, MOCK continues."""

    class FakeExecutor:
        """Executor double recording stop reasons."""

        def __init__(self):
            """Start idle-looking but 'running'."""
            self.stops = []

        def is_running(self):
            """The tick only stops a running program."""
            return True

        async def stop(self, reason=""):
            """Record the stop."""
            self.stops.append(reason)

    def _context(self, clock, transport):
        """Minimal AppState surface used by game_tick."""
        logs = []
        events = []

        async def log(level, message):
            """Capture one log line."""
            logs.append((level, message))

        async def emit(event):
            """Capture one console event."""
            events.append(event)

        def game_status():
            """The AppState status view (clock + transport override)."""
            status = clock.status()
            status["mock_override"] = bool(status["expired"] and transport == "mock")
            status["transport"] = transport
            return status

        return SimpleNamespace(
            game_clock=clock,
            transport=transport,
            log=log,
            emit=emit,
            game_status=game_status,
            vision_waiting={"reason": "obstacle"},
            executor=self.FakeExecutor(),
            client=SimpleNamespace(),
        ), logs, events

    async def test_real_mode_stops_immediately_at_the_limit(self):
        """A REAL timeout stops the run and clears the obstacle wait."""
        time = FakeTime()
        clock = GameClock(time)
        clock.configure({"duration_s": 100, "warning_s": 10, "actions": []})
        clock.start()
        time.advance(101.0)
        context, logs, events = self._context(clock, "i2c")
        await AppState.game_tick(context)
        self.assertEqual(context.executor.stops, ["game time limit reached"])
        self.assertIsNone(context.vision_waiting)
        self.assertTrue(clock.status()["expired"])
        self.assertTrue(any("GAME TIME LIMIT" in message for _lvl, message in logs))
        self.assertEqual(events[-1]["type"], "game")
        self.assertFalse(events[-1]["data"]["mock_override"])

    async def test_mock_mode_keeps_running_with_the_override_flag(self):
        """A MOCK timeout never stops the run; the UI gets the warning flag."""
        time = FakeTime()
        clock = GameClock(time)
        clock.configure({"duration_s": 100, "warning_s": 10, "actions": []})
        clock.start()
        time.advance(101.0)
        context, logs, events = self._context(clock, "mock")
        await AppState.game_tick(context)
        self.assertEqual(context.executor.stops, [])
        self.assertTrue(clock.status()["expired"])
        self.assertTrue(events[-1]["data"]["mock_override"])
        self.assertTrue(any("MOCK" in message for _lvl, message in logs))

    async def test_due_action_is_dispatched_from_the_tick(self):
        """A 95 s action fires from the live tick exactly once."""
        time = FakeTime()
        clock = GameClock(time)
        clock.configure({
            "duration_s": 100, "warning_s": 10,
            "actions": [{"id": "a", "at_s": 95, "kind": "log", "enabled": True,
                         "params": {"message": "actuator"}}],
        })
        clock.start()
        time.advance(95.0)
        context, logs, events = self._context(clock, "mock")
        await AppState.game_tick(context)
        self.assertTrue(any("actuator" in message for _lvl, message in logs))
        self.assertTrue(any(event["type"] == "time_action" for event in events))
        await AppState.game_tick(context)  # no second dispatch
        self.assertEqual(
            sum(1 for event in events if event["type"] == "time_action"), 1
        )


class RealStartPoseTests(unittest.IsolatedAsyncioTestCase):
    """A REAL link adopts the dedicated team start with SET_POSE."""

    class FakeClient:
        """Board double: idle and recording SET_POSE."""

        def __init__(self, idle=True):
            """Store the idle flag."""
            self.idle = idle
            self.poses = []

        async def odometry(self):
            """Board odometry (only ``is_idle`` is used)."""
            return SimpleNamespace(is_idle=self.idle)

        async def set_pose(self, x_mm, y_mm, theta_deg):
            """Record the adopted pose."""
            self.poses.append((x_mm, y_mm, theta_deg))

    def _state(self, client):
        """The AppState surface used by adopt_team_start."""
        logs = []

        async def log(level, message):
            """Capture one log line."""
            logs.append((level, message))

        return SimpleNamespace(
            team="yellow",
            team_starts={"yellow": (315.0, 1800.0, 90.0)},
            team_labels={"yellow": "Yellow"},
            executor=None,
            client=client,
            transport="i2c",
            redo_restore_pose=None,
            log=log,
        ), logs

    async def test_real_mode_sets_the_pose_at_the_team_start(self):
        """REAL issues SET_POSE (not motion) at the dedicated start."""
        client = self.FakeClient()
        context, logs = self._state(client)
        self.assertTrue(await AppState.adopt_team_start(context, force=True))
        self.assertEqual(client.poses, [(315.0, 1800.0, 90.0)])
        self.assertTrue(any("REAL robot pose" in message for _lvl, message in logs))

    async def test_a_moving_robot_is_not_repositioned(self):
        """SET_POSE waits until the board is idle."""
        client = self.FakeClient(idle=False)
        context, _logs = self._state(client)
        self.assertFalse(await AppState.adopt_team_start(context, force=True))
        self.assertEqual(client.poses, [])


class GameDocumentTests(unittest.TestCase):
    """The game block is normalised and persisted with the map document."""

    def test_defaults_are_the_eurobot_match(self):
        """An old document without ``game`` gets the 100 s default."""
        game = normalize_game({})
        self.assertEqual(game["duration_s"], DEFAULT_GAME_DURATION_S)
        self.assertEqual(game["warning_s"], DEFAULT_GAME_WARNING_S)
        self.assertEqual(game["actions"], [])

    def test_values_are_clamped_and_unknown_kinds_fall_back(self):
        """Broken/hand-edited values cannot produce an unsafe clock."""
        game = normalize_game({
            "duration_s": -5,
            "warning_s": 999,
            "actions": [
                {"id": "", "at_s": 999, "kind": "nope", "params": "bad"},
                {"id": "dup", "at_s": 10, "kind": "log", "params": {"message": "x"}},
                {"id": "dup", "at_s": 20, "kind": "rest", "params": {"url": "u"}},
            ],
        })
        self.assertEqual(game["duration_s"], 0.0)
        self.assertEqual(game["warning_s"], 0.0)
        self.assertEqual([action["id"] for action in game["actions"]],
                         ["action-1", "dup", "dup-2"])
        self.assertEqual(game["actions"][0]["kind"], "log")
        self.assertEqual(game["actions"][0]["params"], {})

    def test_the_game_block_round_trips_through_the_document(self):
        """Saving the clock/actions survives a MapConfig reload."""
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "map.json"
            config = MapConfig(path, FIELD_DIR, {"width_mm": 3000, "height_mm": 2000})
            config.update({"game": {
                "duration_s": 120,
                "warning_s": 5,
                "actions": [{"id": "a", "at_s": 95, "kind": "log",
                             "params": {"message": "go"}}],
            }})
            reloaded = MapConfig(path, FIELD_DIR, {"width_mm": 3000, "height_mm": 2000})
            game = reloaded.data["game"]
            self.assertEqual(game["duration_s"], 120.0)
            self.assertEqual(game["warning_s"], 5.0)
            self.assertEqual(game["actions"][0]["at_s"], 95.0)
            self.assertEqual(game["actions"][0]["params"]["message"], "go")
