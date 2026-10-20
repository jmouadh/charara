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

So the UI works with or without the artwork. To use a different image, save
it as `table.png` (any resolution; it is stretched to the configured field
size - set the real play area with the **Field** panel in the UI).

Calibration tip: measure one known feature on the real table (a border, a
start line) and check that the drawn robot position matches when you place
the robot at that feature. If the artwork aspect does not match the play
area, re-export the PDF cropped to the play area (3000 x 2000 mm).
