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

- **Game tab, match clock and timed actions**: the former **Map** tab is now
  **Game** and owns the match clock next to the table configuration. Configure
  a game duration (default 100 s; 0 disables it) and the red-warning window
  (default the last 10 s). Execute starts the clock; the header shows a
  countdown game pill (teal running / amber warning / red limit). At the limit
  a **REAL** run stops immediately and refuses to restart until Reset, while
  **MOCK** keeps running with a red overtime `!` mark. Timed actions fire once
  per match at an elapsed second (`log` implemented; a pluggable `rest`
  handler is reserved for a future actuator API - see Docs/11). The clock and
  actions persist in the game document (`/api/game`, `PUT /api/map`).
- **REAL match-start pose adoption**: when the link becomes ready, the team
  changes or a start zone is saved, a REAL robot now receives `SET_POSE` at
  its primary start-zone spawn (never a motion command), so the console shows
  it in its dedicated starting position instead of a stale corner pose.
- **Permanent vs game-element obstacles**: the Box / Zone / Polygon tools now
  set a `type` (default `permanent` for legacy maps) and a unique `name` for
  `game_element` pieces (painted as a chip on the map, dashed outline). The
  old **Custom** tool is renamed **Polygon** in the UI and docs (the wire
  `kind` was already `polygon`). Names are validated and unique; permanents
  stay anonymous.
- **Erase / Spawn strategy steps**: the Sequence card picks a named game
  element and inserts `{"kind":"erase","name":...}` or
  `{"kind":"spawn","name":...,"obstacle":{...}}`. The planner applies the
  actions in mission order and rebuilds the routing grids at each one, so
  `drive to A -> erase A -> drive through -> spawn A elsewhere` is planned
  correctly (the spawn captures the piece's current map geometry). The
  executor runs each action between two motions (segment backend only),
  updates the live obstacle map used by later camera re-plans, and emits a
  `game_element` event so the console marker follows. Actions are idempotent
  (a strategy replays from any current map state).
- **Permanent-obstacle lock**: once the sequence has steps, the console
  blocks creating/editing/removing table fixtures (game elements stay free),
  and the service rejects a mid-run re-plan that moves or deletes a fixture
  of the executing plan. New anonymous obstacles arriving in a re-plan are
  treated like camera reports.
- **Redo on a paused run**: while the mission is paused, the **Redo** button
  lights up (indigo), **ends the paused run** and puts the robot **back at the
  start of the sequence** (`POST /api/mission/redo` accepts a paused run), so
  **Execute** replays the whole path from its beginning - the same thing Redo
  does after a finished run. (The per-node "rewind" of the previous build is
  gone: it moved the robot back to the previous junction, not to the start.)
- **Timer icon**: the task clock in the status row is a stopwatch pill with a
  turning dial while it runs, tinted indigo (never the status green) with a
  slate idle/finished state.
- **Task timer** in the header: the duration the plan is expected to take
  (`≈ 7.40 s`, refreshed by every plan - drawing, waypoints, a loaded
  strategy), then a clock that starts at **0.00 s** when **Execute** is
  pressed and runs until the run really ends (a mid-run re-plan or a wait
  keeps counting); the final time stays on screen.
- **Background opacity** slider in the MAP Configuration tab (Field card):
  fades the field artwork 0-100 % so the printed table stops fighting with the
  drawn mission - remembered in the browser.
- **Pose clearance** (Robot tab, extra mm, default 0): the Pose tool follows
  the waypoint rule, so the robot can be placed with its **edge on the field
  border** and is pushed out of the drawn obstacles by that margin.
- **Reset robot to start** (MAP Configuration, under the start zones) and
  `POST /api/pose/start`: put the robot back on our configured start (the
  primary zone spawn) - MOCK teleports, REAL logs the pose to adopt.
- **Opponent heading**: every reported opponent robot carries a heading with
  its own θ field, is drawn rotated, and its id (`R1`) is painted **on the
  front face, parallel to it**, so it turns with the robot; the heading is
  reported to the service as the bounding box of the rotated body.
- **Custom obstacle tool**: click the points of any shape on the map - they
  are joined in order and the last one closes the outline back to the first
  (Enter, a double-click or a click on the first point finishes, Esc cancels;
  duplicate points from the double-click are dropped). Triangles, hexagons,
  angled bumpers: the service handles the `polygon` kind everywhere a box or a
  zone already worked (routing grid, stop clearances, rotated-body check,
  mid-run obstacle reactions, mirroring for the other side, table clamping).
- **Obstacle editor**: clicking a Box, Zone or Custom obstacle - with its own
  tool or with **Pan** - opens a popup with its coordinates (X / Y / W / H,
  centre X / Y + radius, or one X / Y row per vertex with a per-vertex delete);
  Apply moves it and re-plans, the bin removes it, Esc closes.
- **MAP Configuration tab** in the right settings rail (between **UI** and
  **Calibrate**): the console is now generic for any game field. It edits a
  persisted map document
  (`MAP_CONFIG_FILE`; `/data/map/map.json` on its own docker volume) and
  manages:
  * the **play-area size** (saved, so it survives restarts) and the
    **background artwork** - every image **and PDF** in `ui/static/field` is
    listed (a PDF, e.g. the table Eurobot publishes, is rasterised to a cached
    PNG by the service), the chosen one is drawn at once, **Rescan** picks up
    new files;
  * **starting zones** drawn with a new drag-a-rectangle tool: team
    (Yellow/Blue), type (**Primary** with its robot spawn X / Y / theta, or
    **Secondary**), editable afterwards on the map, from the list, with the
    erase tool or the row's bin;
  * one **colour per team**, stored with the map, used to paint the zones and
    the primary spawn marker.
  A primary zone's spawn **owns its team's match start** (header, MOCK
  placement and rehearsal scenario), `TEAM_START_*` becoming the fallback.
  Saving zones **places the robots in them**: the MOCK robot is teleported to
  our primary spawn (the REAL robot only gets a "place it and press Set Pose"
  log line, as with a team switch) and the console's reported opponent R1 is
  moved into the other primary zone. The pre-match **readiness** check now
  reports the `start zones` and measures `start pose` against the zone spawn
  (never disturbing a run in progress).
  The service normalises the document (zones clamped to the table, a spawn
  clamped inside its zone, unknown teams dropped) and broadcasts every change
  on `/ws` (`map` event).
- REST: `GET /api/map`, `PUT /api/map`, `GET /api/map/backgrounds`,
  `GET /api/map/background/{name}` and `DELETE /api/map/zones/{id}`
  (Docs/05 section 1).
- Console **dark mode** (Light / Dark / System, remembered per browser) with a
  header toggle; the table artwork is veiled so every overlay keeps contrast.
- Console **complete map palette**: drawn/planned/reference paths, turns &
  pauses, waypoints, our robot, other robots (the red camera squares),
  obstacles, goal, ruler and pose marker are all customizable, with a hex
  readout per swatch, plus a **layer opacity** slider for our robot, the other
  robots and the obstacles - **100 % draws the shape solid in its colour**,
  lower values let the artwork through - remembered in the browser.
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
- **Load-time geometry validation** (issue #10): `GET /api/strategies/{name}`
  and `POST /api/strategies/{name}/load` now return a `warnings` list, one
  line per motion **leg** that leaves the playfield, whose rotated body
  crosses the field border, or whose recorded start pose does not fit the
  table the robot is on now (`software/core/navigation/validation.py`). The
  console logs every line as a `warn` event and the brain prints them at
  startup. Advisory by design - an old strategy stays loadable - with a 5 mm
  border tolerance so the normal start-zone pose (a body flush against the
  wall) does not cry wolf, and the start-to-first-target connector is left to
  the planner, which already projects it.
- **Plan outcome badge** (issue #2): an amber `⚠ N` pill next to the plan
  stats counts the warnings that change what the robot actually drives
  (re-route, geometry, waits, arrival-angle fallbacks, reverse arrivals) and
  lists them, grouped, in its tooltip; the full list stays in the stats block
  and disappears with the plan when an edit invalidates it.
- **Golden protocol frames** (issue #7): `tests/test_protocol.py` freezes the
  command and status tables plus 28 request frames byte for byte, so a wire
  format change cannot slip in unnoticed.
- **Styled photo pickers** (issue #16): the Robot/opponents photo rows are a
  proper **Choose file** button with the selected filename beside it
  (`No file chosen` when empty, `saved photo` when restored), instead of a
  clipped native file input.
- **One troubleshooting table** (issue #3): Docs/04 section 14 is now
  *Symptom | Probable cause | Check | Fix*, split into console/UI and
  board/firmware faults (no telemetry, `NODATA`, `BUSY`, `RANGE`, `BAD_LEN`,
  `STORAGE`, latched `FAULT`, `DEADMAN`, `CONFIG_FALLBACK`, lost settings,
  odometry drift, plan warnings, `422` rejections, stale UI cache).

### Changed

- **A run that ends with a relative move no longer drives back**: the closing
  pose audit used the last ABSOLUTE node as its target, so a plan ending with
  a drive/strafe (close-waypoint departure, release step) dragged the robot
  back to that node after the last move. A relative move has no absolute exit,
  so there is nothing to correct towards.
- **The chain pump no longer floods the log**: a BUSY answer for the queued
  ARC was retried on every poll (~15 `BUSY_REJECT` lines per second in the
  log). The board's single pending slot only frees when its segment counter
  moves, so the answer is remembered per counter value.
- **Pause → Resume finishes the interrupted node** instead of skipping it: the
  remaining arc sweep / turn / drive distance is re-sent from the live pose, so
  the mission continues exactly where it stopped (the old code dropped the rest
  of the node and started the next one from the pose the robot happened to
  hold - an ARC is RELATIVE, so the whole path after the pause bent away: "it
  changes the path and heads to the next and it was a mess").
- **The slice that joins a drawing drawn FROM the robot never lands behind
  the start escape**: the escape end is projected onto the stroke's first
  leg, and when that projection lies beside/behind the escaped pose the plan
  drove back and hooked onto the drawing it had just left (the reported
  "non-sense first movements": forward 105 mm, then a 33 mm back-step, then a
  117 deg loop onto the drawing). The join now slides along the stroke to the
  first point that lies within `STROKE_JOIN_MAX_TURN_DEG` (60 deg) of the
  escape direction **and** is met within `STROKE_JOIN_MAX_KINK_DEG` (20 deg)
  of the drawing's own heading, so the launch is a forward step and a curve -
  the same mission, 85 mm shorter, no loop and no pivot onto the drawing.
- **The mission launch is a CURVE, never an in-place pivot**: when the first
  node of the route lies beside or behind the robot (the drawing start is
  elsewhere, the corridor to it runs along a border), the whole opening -
  leading in-place turns plus the straight legs they fed - is replaced by a
  short straight along the CURRENT heading and ONE arc that joins the line
  into that node tangentially (`_launch_fillet`, option `smooth_launch`,
  default on). The (lead, radius) pair is scanned: the arc must exit on the
  heading the route leaves the node with, clear the table with the rotated
  body, and cost at most 600 mm more than the straight it replaces - a
  robot facing away next to an obstacle keeps the pivot instead of driving
  a 3 m loop. Warning: *"the mission launches with a 60 mm straight then one
  217 deg arc (R 60 mm) instead of a 160 deg in-place turn (no stop at the
  start)"*. `smooth_launch=false` restores the pivot.
- **A planned turn now needs the full turn clearance**: `half_diagonal +
  6 mm + turn_margin_mm` (new option, **Turn clearance (extra)** in the Robot
  tab, default 30 mm). Two consequences: the straight **departure** that
  leaves a close waypoint (or any stop against a border) ends where the body
  can really rotate with that margin, so it is longer (reported case: 75 mm
  -> 105 mm), and a mission that starts in its start box no longer turns in
  place at the pose - when the first drawn geometry is a curve the opening is
  woven into ONE tangent connector arc (unchanged), otherwise the plan starts
  with a short **straight escape move** along the current heading to the
  first turn-safe point and turns there (warning included). A drawing that
  starts AT the robot is joined from the end of that escape, so the plan
  never drives forward and back to the old pose. The start point is no longer
  silently projected to a "closest allowed point".
- **Switching side mirrors the mission**: besides moving both robots, a team
  change reflects paths, waypoints (position + arrival heading), rotations,
  the goal and the drawn obstacles about the table centre line - the same
  transformation a strategy authored for the other side gets when it is
  loaded - and places R1 in the opponent start **facing our robot**.
- The console's **Field** card (table size + fit view) moved from the UI
  settings tab to the new **MAP Configuration** tab of the same settings rail,
  next to the background artwork and the starting zones; the UI tab keeps a
  shortcut that opens it.
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

### Fixed

- **Mid-run re-plans froze the console**: the reaction ladder ran the
  candidate planner whenever an obstacle was merely *near* the robot (it
  fell through on `clip_gap = inf`), i.e. on every 75 mm of progress, and
  every re-plan planned the same remainder twice (assessment + final plan).
  The ladder now only plans when the remaining route is physically clipped
  inside the reaction bands, and the assessment's candidate is reused as the
  plan when the robot did not coast away and the obstacle map is unchanged:
  one planner run per re-plan. The log line now reports the re-plan time.
- A moving obstacle whose **safety bubble already overlaps the robot** (an
  opponent parked beside the route) no longer stops it: the stop -> assess ->
  re-plan -> restart -> stop loop pinned the robot at its start and ran the
  planner back to back. The robot drives clear at caution speed while the
  body route is clear; the contact bubble still guards a real hit.
- **Re-plans corrupted the drawn mission**: a remainder plan's
  `resolved_waypoints` were stamped onto the FIRST waypoints of the full
  drawing (server `_apply_resolved_waypoints` and the UI's map broadcast),
  turning a 4-waypoint mission into duplicated checkpoints and a route that
  looped back over its own tail. Both now remap only when the plan covers
  the whole drawing; mid-run broadcasts are display-only.
- A loaded strategy restores its **Robot-tab geometry/clearance options**
  (width, height, obstacle margin, waypoint margin, turn margin). Replaying
  Game1 (saved with a 50 mm margin) used the tab's 150 mm, planning a much
  more conservative route and shrinking the dynamic reaction bands so the
  robot stopped far from a parked opponent.
- Planner line-of-sight checks inline the grid bounds/cell lookup - they
  were the single largest cost of a Pi plan (Theta* relaxation).
- Planner no longer stops and pivots between two tangent-capable ARCs. Junction
  connectors are also accepted when they stay inside the configured obstacle
  margin but the exact rotated body is clear (the narrow corridor next to an
  obstacle pivoted for nothing), and junction rounding now runs before the
  straight merge so an ARC -> straight exit is rounded too. The reported plan
  danced between R = 23871 mm and R = 413 mm; it now drives as one tangent
  ARC chain.
- Close waypoints prefer a comfortably clear pre-rotation stand-off instead of
  the first minimally turn-safe one. The last waypoint `(1390, 1850)` mm used
  to turn only 225 mm from the border (7 mm under the body corner); it now
  turns 325 mm from the border and drives the final 175 mm straight.
- A camera re-plan landing during the DELAY on a close waypoint kept the
  pause but dropped the reverse departure behind it: the re-planned route
  rotated in place on the waypoint, the body clipped the wall and the run
  ended with "replanned path still intersects ... the robot remains stopped".
  The hold now owns its departure (the delay is skipped when the reverse is
  looked up), both in the mid-run assessment and in the re-planned program,
  so the robot backs up and continues instead of stopping.
- **The match clock obeys Pause / Resume / Redo**: Pause froze the robot but
  the clock kept running, Resume left the lost time behind and Redo kept the
  old elapsed value. `GameClock` now records the paused span (timed actions
  neither fire nor expire while paused, and a running clock is resumed by
  Execute), `/api/pause` and `/api/resume` drive it, and Redo rewinds the
  clock to *not started* together with the sequence.
- **A returning route no longer completes its last waypoint at the start**:
  the re-plan cursor matched samples by distance, so a closed-loop mission
  whose final waypoint sits a couple of millimetres from the start counted it
  as visited on the first sample - the run then skipped the real final stop,
  the arrival angle and the closing reverse. The cursor now advances
  monotonically along the driven route, so a waypoint is visited only after
  the previous element completed.
- **A game element spawned earlier in the mission is not a blockage**: the
  mid-run obstacle check compared the *remainder* plan against the *whole*
  mission's actions and treated a piece that the sequence had just removed
  (or not yet spawned) as an obstacle, re-planning an unneeded arc. The check
  now receives the full mission ladder and the request's action names, so the
  plan is clipped against what is on the table at that moment.
- **A waypoint that must be reached backwards is approached backwards**: a
  second waypoint whose arrival angle points back along the approach was
  reached with a 180 deg turn on the spot or a wide unneeded arc. The planner
  now looks for a stand-off on the side the robot already faces (90 deg
  tolerance, 300 mm minimum run), turns to the arrival angle there and drives
  the last stretch in reverse, so the final pose matches the requested angle
  and the drawn arc disappears.

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
