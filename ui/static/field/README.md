# Field background

The real Eurobot 2026 table is **bundled** as `table.png`
(3000 x 2001 px, 1 px = 1 mm) - it was rendered from
`Docs/Eurobot_2026/table_FINALE_1.0.pdf` in the repository (page =
3000 x 2000 mm), so the
UI shows the actual playing field behind your drawn paths.

To replace it: delete the old `table.png`, drop a new `table_FINALE_1.0.pdf`
here and rebuild the `ui` image (`docker compose up -d --build ui` from
`docker/`) - at container start `entrypoint.sh` converts the PDF to
`table.png` with `pdftoppm` (150 dpi). The web
console tries, in order:

1. `field/table.png`
2. `field/table.jpg`
3. `field/field-default.svg` (built-in schematic, always present)

So the UI works with or without the artwork.

**Several backgrounds.** Every image in this folder (`*.png`, `*.jpg`,
`*.jpeg`, `*.svg`, `*.webp`, `*.gif`) and every **PDF** is listed in the
console's settings rail -> **Game** (Game Configuration) -> *Background
artwork*. Pick one and the map redraws at once; the choice is saved in the
map document. Copy a new file here, press **Rescan** in the tab and it
becomes selectable - no rename and no restart needed. **Automatic** keeps the
historical search order above.

**PDFs are first-class.** Eurobot publishes the table as a PDF
(`table_FINALE_1.0_2027.pdf` is here for the 2027 field), so a PDF can be
selected directly: the service renders **page 1** to a ~3000 px wide PNG the
first time it is used (`pdftoppm` from poppler-utils, installed in the `api`
image) and caches it under `MAP_CACHE_DIR` (`/data/map/cache` in docker). No
manual conversion and no `ui` rebuild is needed; a re-exported PDF is
re-rendered automatically (the cache key follows the file's mtime/size).

The image is stretched to the configured play area (any resolution works);
set the real play area with **Field -> Play area** in the same tab.

Calibration tip: measure one known feature on the real table (a border, a
start line) and check that the drawn robot position matches when you place
the robot at that feature. If the artwork aspect does not match the play
area, re-export the PDF cropped to the play area (3000 x 2000 mm).
