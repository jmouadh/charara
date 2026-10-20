# 07 - Development guide

Everything a developer (human or AI) needs to work on this repository.

## 1. Repository layout

```
charara/                       the RPi repository
|-- README.md                  quick start, feature list
|-- pyproject.toml             the 'charara' package (extras: console, hardware, dev)
|-- docker/                    compose files, Dockerfiles, .env.example, README
|-- strategies/                saved strategy documents (shared with the brain)
|-- tests/                     unittest suite (147 tests, no hardware needed)
|-- ui/static/                 the console (index.html, css/, js/app.js, field/, logo/)
`-- software/                  the Python package (see 02 - Software architecture)
    |-- core/                  pure domain: navigation, localization, world, mission
    |-- robot/                 board protocol, links, client, models
    |-- console/               FastAPI service (REST/WS + runtime state)
    `-- brain/                 headless strategy player (CLI)
```

The STM32 firmware lives in its own repository (`Core/`, `App/`, `Drivers/` -
not part of this checkout); the Python codec mirrors its wire protocol in
`software/robot/protocol.py` (CRC-8 test vector included).

## 2. Development setup (no hardware)

```powershell
# from the repository root - the console with the simulated board
$env:ROBOT_TRANSPORT="mock"
python -m uvicorn software.console.main:app --host 127.0.0.1 --port 8090
# open http://127.0.0.1:8090  (the service serves the console itself)

# or play a saved strategy headless (the brain)
python -m software.brain --strategy demo --transport mock
```

* **No `--reload`**: restart the process after code edits.
* Frontend edits: just reload the browser page. Assets are versioned -
  **bump `?v=N` in `ui/static/index.html`** when JS/CSS change, otherwise
  browsers may serve stale files (they apply heuristic caching).
* Look and feel: the console runs the **"cobalt blueprint"** theme, taken from
  the team logo (navy ink, cobalt/azure instruments, an ice-cyan glow on a cool
  white field). Every colour is a `:root` custom property at the top of
  `ui/static/css/style.css` - retune the tokens there rather than the
  individual rules. The source artwork is `ui/static/logo/logo.png`;
  `charara-emblem.png` (header mark) and `charara-mark.png` (favicon and the
  faded stamp on the field) are crops derived from it.
* The console keeps warnings/logs in the page; UI state (paths, obstacles)
  lives in the page memory only - **a reload clears the drawing**.

Docker (real robot): `cd docker && docker compose up -d --build` on the Pi;
the stack includes the `vpn` Tailscale sidecar, so with `TS_AUTHKEY` in
`docker/.env` the console is remote too: `http://<tailnet-name>:8080` from
any of your devices. Dev variant: `docker compose -f compose.dev.yml up
--build` (mock, no `/dev/i2c`). See `docker/README.md`.

### Deploy to the robot from Windows (`deploy.ps1`)

`deploy.ps1` (repository root; `deploy.cmd` for a double click) packs the
checkout, uploads it over SSH to `charara@192.168.0.192:/home/charara`
(replacing the managed files, keeping `docker/.env`) and rebuilds the docker
stack. Before that it provisions the I2C buses the stack needs (idempotent):
`dtparam=i2c_arm=on` for the board on `/dev/i2c-1` and
`dtoverlay=i2c6,pins_22_23` for the MPU6050 on `/dev/i2c-6` (GPIO22/23),
rebooting the robot once when the running kernel does not expose them yet.
Options: `-NoRebuild` (files only), `-NoClean`, `-NoProvision`,
`-UseKey` / `-SetupKey` (passwordless deploys). The robot may be on the home
Wi-Fi (`192.168.0.192`) or on its own hotspot (`192.168.0.30`): `-Target`
takes several `user@host` candidates (that is the default) and the first one
that answers SSH is used, so a deploy works on either network.

The same step installs the robot's OS-level network bootstrap from
`deploy/host/` (`configure_wifi.sh` + the `configure_hotspot.service` and
`configure_wifi-watch.timer` units): connect to the home Wi-Fi when it is in
range, otherwise raise the CHARARA hotspot, re-checking every 2 minutes. See
`deploy/host/README.md`; the script can also be run by hand
(`sudo /usr/local/bin/configure_wifi.sh [--dry-run|--reeval]`).

## 3. Tests

```bash
# from the repository root (so `software/` is importable)
python -m unittest discover -s tests          # everything (430 tests)
python -m unittest discover -s tests -p "test_vision.py" -v
```

| File | Covers |
|---|---|
| `test_protocol.py` | frame codec, CRC-8 (firmware vector), helpers |
| `test_geometry.py` | RDP, circle fitting, line/arc segmentation, resampling, primitives |
| `test_planner.py` | grid inflation, Theta\*, routing rules, fillet behaviour, waypoints, slicing, **shortest detour** |
| `test_localization.py` | kinematics, Kalman filter, fusion service (adoption, jumps, at-rest tightening, gyro hook) |
| `test_executor.py` | the segment runner (pause/stop/timeout) and tail rebuild (`remaining_elements`) |
| `test_mock_link.py` | the simulated board (short-way turns, faults, deadman, arc geometry, **settings registers + flash simulation, event log, SET_VELOCITY stream**) |
| `test_api.py` | end-to-end REST: plan/execute/pause, dynamic re-plans, **no-op re-check**, mode switch, trace, WS telemetry, **robot config endpoints, board errors, acceptance card audits + fault drills** |
| `test_vision.py` | `/ws/vision`: merge into plans, TTL expiry, mid-run camera re-plan |
| `test_strategy.py` | the strategy document model (v1 elements / v2 kind+tree) |
| `test_strategy_api.py` | `/api/strategies` round trip incl. **v2 tree validation at save time** |
| `test_brain.py` | the headless runner on the mock board + CLI behaviour |
| `test_brain_match.py` | strategy v2 playback: leaf catalogue, full tree runs, KPI log passthrough |
| `test_bt.py` | behavior-tree scheduler semantics (composites, decorators, spec loader) |
| `test_pursuit.py` | pure-pursuit geometry, curvature feed-forward, RAMSETE law, kinematic simulator (Track B) |
| `test_follower.py` | SET_VELOCITY mock parity, stale-stop, deadman, PathFollower runs + capability gate |
| `test_stream.py` | silent SET_GYRO stream (session task, firmware gate, bench stop/start) |
| `test_imu.py` | MPU6050 sampler, bias refinement, spin-scale workflow |
| `test_diagnostics.py` | pure helpers of the acceptance card (square/arc geometry, drift stats) |
| `test_corrections.py` | idle re-anchor policy loop |
| `test_session.py` | the shared session lifecycle |

The API tests use FastAPI's `TestClient` with the mock link - no hardware,
no network. Keep them deterministic: set the pose first, `wait_for` states,
avoid wall-clock assertions.

## 4. Conventions

* Python 3.11, `from __future__ import annotations`, dataclasses, type hints.
* **pydantic v1 and v2 compatibility** - the dev PC has pydantic 1.x while
  docker gets v2. Never call `model_dump()` directly; use the `to_dict()`
  helper in `software/console/api/routes/deps.py`.
* The planner works in **millimetres, degrees, +y up, theta CCW** - screen
  conversions exist only in the UI (`worldToScreen` / `screenToWorld`).
* Warnings are user-facing text: state clearly **what** was deviated from and
  **why** ("re-routed to stay clear...", "squeezed...", "moved to the closest
  allowed point..."). Never hide a deviation.
* Any change to planning behaviour needs a regression test with a realistic
  geometry (see the existing "mess fix" tests).
* Async: never block the loop; I2C goes through `RobotClient` (thread
  off-loading + lock). Planning is CPU-bound and briefly blocks - accepted,
  serialised by the mission context's `plan_lock`.

## 5. How-tos

### Add a REST endpoint
1. Add the pydantic model to `software/console/api/routes/models.py` and the
   handler to the matching domain module under `software/console/api/routes/`
   (use `call()` from `deps.py` for robot actions so `RobotError`/`LinkError`
   map to status codes).
2. Document it in `Docs/05-api-reference.md`.
3. Add a test in `tests/test_api.py`.

### Add a plan option
1. Add the field to `PlanOptions` (software/core/navigation/options.py) with a sane default.
2. Use it where needed; expose it in `ui/static/js/app.js` `planOptions()`
   and the Config tab if the operator should control it.
3. Update `Docs/03-planner-algorithms.md` section 1 and add tests.

### Change routing behaviour
Routing lives in `software/core/navigation/trajectory._route` (runs, tiers, warning texts) and
`grid.theta_star`. The rules to preserve:
* drawn geometry wins; only body-blocked runs are detoured;
* one detour per run; detour = shortest feasible (full/squeezed/body
  candidates, 20 mm clearance tie-break);
* detours are RDP-simplified at `res*0.8`, drawn geometry untouched.
Add/adjust tests before and after (`test_planner.py`).

### Work on the vision channel
Protocol handling: `software/console/api/ws.py`; store: `VisionStore` in
`software/core/world/vision.py`; reactions (broadcast, re-plan):
`software/core/mission/watcher.py`. Keep the protocol documented in
`Docs/06-vision-camera-integration.md` in sync - timestamp fields (`t`,
`t_server`) are part of the contract.

### Work on the brain / strategies
The document model is `software/core/mission/strategy.py`; the headless
runner is `software/brain/runner.py` (its own mission context + the shared
engine); the CLI is `software/brain/__main__.py`. Format, storage and usage:
[08 - Brain & strategies](08-brain-and-strategies.md). Keep the console REST
(`/api/strategies`), the brain and the format doc in sync.

### Work on localization
`software/core/localization/`: `kinematics.py` (wheel twist, pose
integration), `kalman.py` (`PoseKalmanFilter`, the IMU hook
`update_angular_rate`), `service.py` (`LocalizationService` - adoption, jump
detection, at-rest tightening). Everything the rest of the software calls
`pose()` goes through here; keep `test_localization.py` green.

### Debug a board issue
Open the console's **I2C monitor** or `GET /api/trace`. Errors, sequence
numbers and payloads are all there. In mock mode the same frames show the
simulated traffic.

## 6. Gotchas (learned the hard way)

| Gotcha | Rule |
|---|---|
| Browser serving stale JS/CSS | bump `?v=N` in `index.html` when assets change |
| Reloading the console clears the drawing | drawings live in the page; reload only when the canvas may be lost (or re-draw) |
| pydantic v1 vs v2 | use `to_dict()` (`software/console/api/routes/deps.py`) |
| `uvicorn` without `--reload` | restart the server after code edits |
| console AND brain on one board | separate processes, both own `/dev/i2c-1` - run one at a time |
| Two `TestClient`s in one process | `switch_transport` rebuilds the link - never keep asyncio primitives across lifespans (a stale lock is bound to a dead loop) |
| Over-inflated obstacles | grid inflation is Euclidean (Minkowski); axis-aligned inflation over-blocks corners by up to sqrt2 |
| Fillet starving | never RDP-dedupe below ~2 mm; fillet samples are dense by design (`DEDUPE_NODE_MM = 2`) |
| Re-plan drift | always slice the **mission elements**; never rebuild from a previous plan's nodes |
| Re-plan churn | camera reports are coalesced (0.25 s) and quantised (10 mm); the affected-check tolerates <2 % length drift |

## 7. Extending to new hardware

* **New sensors/cameras** - connect them to `/ws/vision` (obstacles) or add a
  new input channel next to it; feed data into `AppState` and reuse the
  mission re-check (`mission.execute_plan`) so behaviour stays uniform.
* **Autonomy** - the brain already runs a saved strategy headless
  (`python -m software.brain`); everything the console does is also available
  over REST (`POST /api/plan`, `POST /api/execute`, `GET /api/state`, `/ws`
  events), so external tools or companion-card logic can drive the same
  software without a browser.

## 8. Glossary

See [01 - System overview section 7](01-system-overview.md#7-glossary) for the domain
vocabulary (element, run, node, body clearance, margin, slice, TTL, ...).
