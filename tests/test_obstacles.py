"""Obstacle model tests: permanent fixtures vs named game elements.

File:    tests/test_obstacles.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.3.0
Date:    2026-10-28
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from software.core.mission.strategy import Strategy  # noqa: E402
from software.core.mission.teams import mirror_elements  # noqa: E402
from software.core.world.obstacles import (  # noqa: E402
    TYPE_GAME_ELEMENT,
    TYPE_PERMANENT,
    apply_action,
    check_element_name,
    erase_game_element,
    normalize_obstacles,
    permanent_signature,
    permanents_kept,
    spawn_game_element,
    validate_actions,
)


class ObstacleModelTests(unittest.TestCase):
    """Type/name validation and the pure action helpers."""

    def test_legacy_obstacles_load_as_anonymous_permanents(self):
        """Documents from before the model split keep working unchanged."""
        obstacles = normalize_obstacles(
            [{"kind": "rect", "x": 10.0, "y": 20.0, "w": 30.0, "h": 40.0}]
        )
        self.assertEqual(obstacles[0]["type"], TYPE_PERMANENT)
        self.assertNotIn("name", obstacles[0])

    def test_a_game_element_needs_a_unique_name(self):
        """Game elements are referenced by name, so the name is mandatory."""
        with self.assertRaises(ValueError):
            normalize_obstacles([{"kind": "rect", "type": TYPE_GAME_ELEMENT}])
        with self.assertRaises(ValueError):
            normalize_obstacles([
                {"kind": "rect", "type": TYPE_GAME_ELEMENT, "name": "A"},
                {"kind": "circle", "type": TYPE_GAME_ELEMENT, "name": "a"},
            ])
        valid = normalize_obstacles([
            {"kind": "rect", "type": TYPE_GAME_ELEMENT, "name": "A"},
            {"kind": "rect", "type": TYPE_GAME_ELEMENT, "name": "B"},
        ])
        self.assertEqual([item["name"] for item in valid], ["A", "B"])

    def test_a_name_is_not_kept_on_a_permanent(self):
        """A stray name on a fixture must never become an action target."""
        item = normalize_obstacles(
            [{"kind": "rect", "type": TYPE_PERMANENT, "name": "WALL"}]
        )[0]
        self.assertNotIn("name", item)

    def test_names_are_short_and_start_with_a_letter_or_digit(self):
        """Names are visible chips on the map and in the sequence."""
        self.assertEqual(check_element_name("  cube_A "), "cube_A")
        for bad in ("", "/A", "-A", "A#B", "A" * 40):
            with self.assertRaises(ValueError):
                check_element_name(bad)

    def test_erase_spawn_are_idempotent(self):
        """Replaying a strategy must not fail on the current map state."""
        a = {"kind": "rect", "type": TYPE_GAME_ELEMENT, "name": "A", "x": 1, "y": 2, "w": 3, "h": 4}
        erased = erase_game_element([a], "A")
        self.assertEqual(erased, [])
        self.assertEqual(erase_game_element([], "A"), [])
        placed = spawn_game_element([], "A", {"kind": "circle", "x": 10, "y": 20, "r": 15})
        self.assertEqual(placed[0]["name"], "A")
        moved = spawn_game_element(placed, "A", {"kind": "circle", "x": 50, "y": 60, "r": 15})
        self.assertEqual(len(moved), 1)
        self.assertEqual((moved[0]["x"], moved[0]["y"]), (50, 60))
        self.assertEqual(apply_action(moved, "erase", "a"), [])

    def test_actions_are_validated(self):
        """A spawn carries usable geometry; a permanent is never targeted."""
        elements = [
            {"kind": "erase", "name": "A"},
            {"kind": "spawn", "name": "A", "obstacle": {"kind": "rect", "w": 10, "h": 10}},
        ]
        validate_actions(elements, [])  # both are legal, even from an empty map
        with self.assertRaises(ValueError):
            validate_actions([{"kind": "erase", "name": "BAD/NAME"}], [])
        with self.assertRaises(ValueError):
            validate_actions([{"kind": "spawn", "name": "A"}], [])
        with self.assertRaises(ValueError):
            validate_actions(
                [{"kind": "spawn", "name": "A", "obstacle": {"kind": "rect"}}], []
            )

    def test_permanent_signature_ignores_game_elements(self):
        """Moving a game piece never unlocks/breaks the fixture lock."""
        obstacles = [
            {"kind": "rect", "type": TYPE_PERMANENT, "x": 100, "y": 100, "w": 50, "h": 50},
            {"kind": "rect", "type": TYPE_GAME_ELEMENT, "name": "A", "x": 10, "y": 10, "w": 5, "h": 5},
        ]
        first = permanent_signature(obstacles)
        moved = [dict(obstacles[0]), {**obstacles[1], "x": 900}]
        self.assertEqual(permanent_signature(moved), first)
        changed = [dict(obstacles[0], x=200), obstacles[1]]
        self.assertNotEqual(permanent_signature(changed), first)

    def test_the_running_lock_allows_new_transients_but_not_moved_fixtures(self):
        """A camera-like addition is fine; moving the setup fixture is not."""
        wall = {"kind": "rect", "x": 100, "y": 100, "w": 50, "h": 50}
        transient = {"kind": "rect", "x": 900, "y": 900, "w": 50, "h": 50}
        self.assertTrue(permanents_kept([wall], [wall, transient]))
        self.assertFalse(permanents_kept([wall], [dict(wall, x=200)]))
        self.assertFalse(permanents_kept([wall], []))


class StrategyGameElementTests(unittest.TestCase):
    """Strategy documents validate the new model and mirror spawn geometry."""

    def test_invalid_action_rejects_the_document(self):
        """An unpalayable document must never reach the brain."""
        doc = {
            "name": "bad",
            "elements": [{"kind": "spawn", "name": "A"}],
            "obstacles": [{"kind": "rect", "type": TYPE_GAME_ELEMENT, "name": "A",
                           "x": 0, "y": 0, "w": 10, "h": 10}],
        }
        with self.assertRaises(ValueError):
            Strategy.from_dict(doc)

    def test_spawn_geometry_is_mirrored_for_the_other_side(self):
        """A spawn action is table data: the mirror reflects it too."""
        elements = [{
            "kind": "spawn",
            "name": "A",
            "obstacle": {"kind": "rect", "x": 100.0, "y": 200.0, "w": 40.0, "h": 50.0},
        }]
        mirrored = mirror_elements(elements, 3000.0)
        geometry = mirrored[0]["obstacle"]
        self.assertEqual(mirrored[0]["name"], "A")
        self.assertEqual(geometry["x"], 2860.0)
        self.assertEqual((geometry["y"], geometry["w"], geometry["h"]), (200.0, 40.0, 50.0))


if __name__ == "__main__":
    unittest.main()
