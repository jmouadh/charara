# Changelog

All notable changes to Charara are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the
project uses [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Security

- Credentials are no longer stored in tracked files.
  `deploy/host/configure_wifi.sh` now reads `SSID` / `PASSWORD` from
  `${CHARARA_NET_ENV:-/etc/charara/network.env}` (or the environment) and falls
  back to the hotspot when they are absent; `deploy/host/configure_vpn.sh`
  requires `TS_AUTHKEY` in the environment. Templates live in
  `deploy/host/network.env.example` and `docker/.env.example`; the real files are
  git-ignored.

### Added

- Console **dark mode** (Light / Dark / System, remembered per browser) with a
  header toggle; the table artwork is veiled so every overlay keeps contrast.
- Console **complete map palette**: drawn/planned/reference paths, turns &
  pauses, waypoints, our robot, other robots (the red camera squares),
  obstacles, goal, ruler and pose marker are all customizable, with a hex
  readout per swatch.
- **Resizable mission panel**: drag the border (col-resize grip), double-click
  to reset, arrow keys for fine steps; the width persists in the browser.
- Repository scaffolding: issue forms (bug / feature / task), pull request
  template, `CODEOWNERS`, CI (lint + full test suite + firmware tree check),
  `CONTRIBUTING.md` and this changelog.
- Documentation site: `mkdocs.yml` (Material theme, `Docs/` as source) and the
  `pages` workflow, published at <https://jmouadh.github.io/charara/>.
- `deploy.ps1` - pack, upload over SSH, provision the I2C buses and rebuild the
  docker stack on the robot, with `-NoRebuild`, `-NoClean`, `-NoProvision`,
  `-UseKey` and `-SetupKey` (the entry point `deploy.cmd` already referenced).
- `SECURITY.md` - private vulnerability reporting and the credential policy.
- Console screenshot set under `Docs/assets/screenshots/`, embedded in the
  `README.md` gallery and the console guide. The set now shows **both themes**
  (a dark hero, a light/dark pair of the same mission) plus a dedicated
  **theme &amp; map-palette** shot.
- `LICENSE` (BSD-3-Clause), with the vendor licenses of the ST HAL and CMSIS
  called out.

### Changed

- Hardware galleries (`README.md`, `Mecanical/README.md`) use **equal-height
  photos**, and `board.jpg` / `drivetrain.jpg` are rotated upright, so the rows
  line up instead of mixing portrait and landscape thumbnails. The spring
  coupling is documented for what it is: the two tension springs hold the
  **odometry (encoder) wheel** on the floor through shocks, so it keeps
  measuring instead of skipping.
- The goal is edited on the map: placing a goal opens a **goal popup** next to
  the marker (X / Y / arrival heading, Apply / delete / close, Enter / Escape),
  and clicking an already placed goal reopens it. The numeric **Goal theta**
  box was removed from the Mission card.
- The Release controls use a small **inline UI confirmation card** under the
  button (Cancel/Escape closes it) instead of the native `window.confirm`
  dialog; nothing is sent until Release is confirmed.
- The Tools grid no longer overflows a narrow mission panel: it keeps four
  columns at normal widths and switches to three below ~310 px.
- Console header redesigned: one recessed status cluster (camera, IMU, link,
  robot, state, execution) with subtle state tints and a compact two-row
  layout below 960 px (three rows under 380 px); no more oversized/circular
  pills. The map's "START - ..." text labels were removed - the solid/dashed
  start ghosts are enough.
- Console dark mode is now a neutral carbon-black theme (black/graphite
  surfaces, no navy tint), including the map backing, glows and primary
  actions.
- Console settings rail: the collapse icon and every outline glyph follow the
  current text colour (they were black-on-black in dark mode), and the
  unselected tabs are legible outlined chips with a clear hover in both
  themes.
- Console sequence rows use a fixed index / description / actions grid, so the
  per-step speed and waypoint fields wrap on narrow panels instead of sliding
  under the move/delete buttons.
- `deploy/host/configure_wifi.sh`: `HAVE_WIFI_CREDS` is derived from the loaded
  environment; without credentials the script goes straight to the hotspot so
  the robot is always reachable.
- `pyproject.toml`: `dev` extra now includes `pytest-rerunfailures` (the
  websocket re-plan test is timing sensitive).

## [3.2.0] - 2026-10-19

Baseline import of the working robot stack.

### Added

- **Console** (`ui/`, `software/console`): field canvas with the Eurobot 2026
  table, freehand drawing fitted to lines and arcs, waypoints, goal, obstacles,
  ghost preview, strategies, calibration and bench tests, live I2C monitor,
  REST + WebSocket API.
- **Planner** (`software/core/navigation`): grid A* / Theta* with smoothing,
  native arcs, obstacle inflation from the robot geometry, dynamic re-planning
  around camera-reported obstacles, fused execution with watchdog.
- **Brain** (`software/brain`): headless strategy player for the Raspberry Pi.
- **Firmware** (`STM32/Firmware`): STM32F722 control and odometry firmware,
  I2C protocol v3, settings registers persisted to flash, fault latch, deadman
  watchdog, MPU6050 fusion.
- **Documentation** (`Docs/`): system overview, backend architecture, planner
  algorithms, console guide, API reference, vision integration, development
  guide, brain and strategies, sensor fusion, roadmap, plus the STM32 hand-off
  documents and the Eurobot 2026 rules.
