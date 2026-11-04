"""Map configuration document: zones, spawn poses, colours, persistence.

The map document is what makes the console fit a different game field, so
the tests pin its normalisation rules: clamping to the play area, a spawn
inside its own zone, one colour per team and the background artwork check.

File:    tests/test_map_config.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-10-23
"""

import json
import shutil
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from software.core.world.map_config import (  # noqa: E402
    DEFAULT_COLORS,
    MapConfig,
    normalize_background,
)
from software.core.world.pdf_background import (  # noqa: E402
    cache_path,
    ensure_background_png,
)


class MapConfigTests(unittest.TestCase):
    """The persisted map document: defaults, normalisation and reload."""

    def setUp(self):
        """A temp field folder with one artwork and an empty document."""
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        self.field_dir = self.root / "field"
        self.field_dir.mkdir()
        (self.field_dir / "table.png").write_bytes(b"png")
        (self.field_dir / "second.svg").write_text("<svg/>", encoding="utf-8")
        (self.field_dir / "notes.txt").write_text("not an image", encoding="utf-8")
        self.path = self.root / "map" / "map.json"
        self.config = MapConfig(
            self.path, self.field_dir, {"width_mm": 3000.0, "height_mm": 2000.0}
        )

    def tearDown(self):
        """Drop the temp folder."""
        self._tmp.cleanup()

    def test_defaults_use_the_configured_table_and_colours(self):
        """A missing document starts from the environment defaults."""
        data = self.config.snapshot()
        self.assertEqual(data["field"], {"width_mm": 3000.0, "height_mm": 2000.0})
        self.assertEqual(data["colors"], DEFAULT_COLORS)
        self.assertEqual(data["zones"], [])
        self.assertEqual(data["background"], "")
        self.assertFalse(self.path.exists(), "defaults must not write a file")

    def test_update_is_persisted_and_survives_a_reload(self):
        """Zones, colours and the table size round-trip through the file."""
        self.config.update(
            {
                "field": {"width_mm": 2500.0, "height_mm": 1500.0},
                "colors": {"yellow": "#112233"},
                "background": "field/table.png",
                "zones": [
                    {
                        "team": "yellow",
                        "type": "primary",
                        "x": 100.0,
                        "y": 800.0,
                        "w": 600.0,
                        "h": 500.0,
                        "spawn": {"x": 400.0, "y": 1000.0, "theta_deg": 270.0},
                    }
                ],
            }
        )
        self.assertTrue(self.path.is_file())
        reloaded = MapConfig(
            self.path, self.field_dir, {"width_mm": 3000.0, "height_mm": 2000.0}
        )
        data = reloaded.snapshot()
        self.assertEqual(data["field"], {"width_mm": 2500.0, "height_mm": 1500.0})
        self.assertEqual(data["background"], "field/table.png")
        self.assertEqual(data["colors"]["yellow"], "#112233")
        self.assertEqual(data["colors"]["blue"], DEFAULT_COLORS["blue"])
        zone = data["zones"][0]
        self.assertEqual((zone["team"], zone["type"]), ("yellow", "primary"))
        self.assertEqual(zone["spawn"], {"x": 400.0, "y": 1000.0, "theta_deg": -90.0})

    def test_zones_are_clamped_to_the_table_and_unknown_teams_dropped(self):
        """A hand-edited file can never put a zone off the field."""
        data = self.config.update(
            {
                "zones": [
                    {"team": "yellow", "type": "primary", "x": -200.0, "y": 1900.0,
                     "w": 5000.0, "h": 900.0},
                    {"team": "green", "x": 0.0, "y": 0.0, "w": 100.0, "h": 100.0},
                    {"team": "blue", "w": 10.0, "h": 10.0},
                ]
            }
        )
        self.assertEqual(len(data["zones"]), 2, "the unknown team is dropped")
        first, second = data["zones"]
        self.assertEqual((first["x"], first["y"]), (0.0, 1100.0))
        self.assertEqual((first["w"], first["h"]), (3000.0, 900.0))
        # a tiny rectangle is grown to the usable minimum
        self.assertEqual((second["w"], second["h"]), (50.0, 50.0))

    def test_primary_spawn_is_clamped_inside_its_zone(self):
        """A spawn cannot stand outside the rectangle that owns it."""
        data = self.config.update(
            {
                "zones": [
                    {"team": "yellow", "type": "primary", "x": 200.0, "y": 300.0,
                     "w": 400.0, "h": 300.0,
                     "spawn": {"x": 2000.0, "y": -500.0, "theta_deg": 540.0}}
                ]
            }
        )
        spawn = data["zones"][0]["spawn"]
        self.assertEqual((spawn["x"], spawn["y"]), (600.0, 300.0))
        self.assertEqual(spawn["theta_deg"], 180.0)

    def test_secondary_zone_never_keeps_a_spawn(self):
        """Only a primary zone carries the robot spawn pose."""
        data = self.config.update(
            {
                "zones": [
                    {"team": "blue", "type": "secondary", "x": 0.0, "y": 0.0,
                     "w": 200.0, "h": 200.0,
                     "spawn": {"x": 100.0, "y": 100.0, "theta_deg": 0.0}}
                ]
            }
        )
        self.assertIsNone(data["zones"][0]["spawn"])

    def test_zone_ids_are_unique(self):
        """Duplicate ids would make the list/popup edit the wrong zone."""
        data = self.config.update(
            {
                "zones": [
                    {"id": "same", "team": "yellow", "w": 100.0, "h": 100.0},
                    {"id": "same", "team": "blue", "w": 100.0, "h": 100.0},
                ]
            }
        )
        ids = [zone["id"] for zone in data["zones"]]
        self.assertEqual(len(ids), len(set(ids)))

    def test_background_must_be_an_existing_image_in_the_field_folder(self):
        """Only a real image file name is accepted (no path traversal)."""
        self.assertEqual(normalize_background("table.png", self.field_dir), "field/table.png")
        self.assertEqual(
            normalize_background("field/second.svg", self.field_dir), "field/second.svg"
        )
        self.assertEqual(normalize_background("../secret.png", self.field_dir), "")
        self.assertEqual(normalize_background("notes.txt", self.field_dir), "")
        self.assertEqual(normalize_background("missing.png", self.field_dir), "")

    def test_invalid_colours_fall_back_to_the_defaults(self):
        """A bad colour must never break the canvas painting."""
        data = self.config.update({"colors": {"yellow": "red", "blue": "#00ff00"}})
        self.assertEqual(data["colors"]["yellow"], DEFAULT_COLORS["yellow"])
        self.assertEqual(data["colors"]["blue"], "#00ff00")

    def test_backgrounds_lists_images_only(self):
        """The picker lists every image in the folder, nothing else."""
        names = [row["name"] for row in self.config.backgrounds()]
        self.assertEqual(names, ["second.svg", "table.png"])

    def test_pdf_background_is_selectable_and_rendered_by_the_service(self):
        """A published PDF table is a valid background (rendered to a PNG)."""
        (self.field_dir / "table.pdf").write_bytes(b"%PDF-1.4\n")
        self.assertEqual(
            normalize_background("table.pdf", self.field_dir), "field/table.pdf"
        )
        entries = {row["name"]: row for row in self.config.backgrounds()}
        self.assertEqual(entries["table.pdf"]["kind"], "pdf")
        self.assertEqual(entries["table.pdf"]["value"], "field/table.pdf")
        self.assertTrue(
            entries["table.pdf"]["url"].startswith("/api/map/background/table.pdf"),
            entries["table.pdf"]["url"],
        )
        self.assertEqual(entries["table.png"]["kind"], "image")
        self.assertTrue(entries["table.png"]["url"].startswith("field/table.png"))

    def test_background_path_rejects_traversal_and_other_files(self):
        """Only a plain file name of the field folder can be rendered."""
        (self.field_dir / "table.pdf").write_bytes(b"%PDF-1.4\n")
        self.assertIsNotNone(self.config.background_path("table.pdf"))
        for bad in ("../table.pdf", "sub/table.pdf", "..", "", "notes.txt"):
            self.assertIsNone(self.config.background_path(bad), bad)

    def test_spawn_for_returns_the_first_primary_pose(self):
        """The team match start is the first primary zone's spawn."""
        self.config.update(
            {
                "zones": [
                    {"team": "yellow", "type": "secondary", "x": 0.0, "y": 0.0,
                     "w": 100.0, "h": 100.0},
                    {"team": "yellow", "type": "primary", "x": 300.0, "y": 400.0,
                     "w": 200.0, "h": 200.0,
                     "spawn": {"x": 350.0, "y": 450.0, "theta_deg": 45.0}},
                ]
            }
        )
        self.assertEqual(self.config.spawn_for("yellow"), (350.0, 450.0, 45.0))
        self.assertIsNone(self.config.spawn_for("blue"))

    def test_a_broken_file_falls_back_to_the_defaults(self):
        """A half-written file must not take the console down."""
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text("{not json", encoding="utf-8")
        config = MapConfig(
            self.path, self.field_dir, {"width_mm": 3000.0, "height_mm": 2000.0}
        )
        self.assertEqual(config.snapshot()["field"]["width_mm"], 3000.0)
        self.assertEqual(config.snapshot()["zones"], [])

    def test_saved_document_is_plain_readable_json(self):
        """The file stays diffable/hand-editable (no compact blob)."""
        self.config.update({"zones": [{"team": "blue", "w": 100.0, "h": 100.0}]})
        text = self.path.read_text(encoding="utf-8")
        self.assertIn("\n  ", text)
        self.assertEqual(json.loads(text)["version"], 1)


class PdfBackgroundRenderTests(unittest.TestCase):
    """Rasterising the published PDF table into the map background."""

    #: a small tracked PDF: rendering it must not depend on the field folder
    SAMPLE_PDF = (
        Path(__file__).resolve().parents[1]
        / "Docs"
        / "STM_docs"
        / "Schematics"
        / "carte_deplacement.pdf"
    )

    def test_cache_key_follows_the_pdf_and_the_width(self):
        """Same PDF+width reuses the raster; a re-export invalidates it."""
        with tempfile.TemporaryDirectory() as folder:
            pdf = Path(folder) / "table.pdf"
            pdf.write_bytes(b"%PDF-1.4\n")
            cache = Path(folder) / "cache"
            first = cache_path(cache, pdf, 1200)
            self.assertEqual(cache_path(cache, pdf, 1200), first)
            self.assertEqual(first.parent, cache)
            self.assertEqual(first.suffix, ".png")
            self.assertTrue(first.name.startswith("table-"), first.name)
            self.assertNotEqual(cache_path(cache, pdf, 3000), first)
            pdf.write_bytes(b"%PDF-1.4\n% re-exported\n")
            self.assertNotEqual(cache_path(cache, pdf, 1200), first)

    def test_missing_poppler_is_reported_with_a_fix(self):
        """No poppler must not be a mystery: say what to rebuild."""
        with tempfile.TemporaryDirectory() as folder:
            pdf = Path(folder) / "table.pdf"
            pdf.write_bytes(b"%PDF-1.4\n")
            with patch(
                "software.core.world.pdf_background.tool_path", return_value=None
            ):
                with self.assertRaises(RuntimeError) as caught:
                    ensure_background_png(pdf, Path(folder) / "cache")
            self.assertIn("poppler", str(caught.exception))

    @unittest.skipIf(shutil.which("pdftoppm") is None, "poppler-utils not installed")
    def test_render_writes_a_cached_png_of_the_requested_width(self):
        """The raster lands in the cache, at the asked width, and is reused."""
        self.assertTrue(self.SAMPLE_PDF.is_file(), self.SAMPLE_PDF)
        with tempfile.TemporaryDirectory() as folder:
            cache = Path(folder) / "cache"
            png = ensure_background_png(self.SAMPLE_PDF, cache, width_px=1200)
            self.assertEqual(png.parent, cache)
            data = png.read_bytes()
            self.assertTrue(data.startswith(b"\x89PNG\r\n\x1a\n"))
            self.assertEqual(struct.unpack(">I", data[16:20])[0], 1200)
            self.assertEqual(
                ensure_background_png(self.SAMPLE_PDF, cache, width_px=1200), png
            )
            self.assertEqual(list(cache.glob("*.png")), [png])


if __name__ == "__main__":
    unittest.main()
