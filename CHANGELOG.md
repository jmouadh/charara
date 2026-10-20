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
  `README.md` gallery and the console guide.
- `LICENSE` (BSD-3-Clause), with the vendor licenses of the ST HAL and CMSIS
  called out.

### Changed

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
