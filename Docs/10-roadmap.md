# 10 - Implementation roadmap

*Author: Mouadh Jeridi <jeridimouadh@gmail.com>*

*Status: planning document (2026-10-09). Freezes the next implementation
tracks agreed after the 2026-10-08/09 field sessions - A: motion
continuity and tuning, B: streaming pursuit (Pure Pursuit / RAMSETE),
C: behavior-tree match layer - with enough design detail to implement
each one without re-deriving it. Update the progress log (section 10) as
items land.*

*Date: 2026-10-09*

## 1. Purpose and how to use this document

The robot today drives segment by segment: the board stops at every GOTO
node, and only chained ARCs (`QUEUE_ARC`) run without stopping in between.
That model is safe and accurate, but it pays a time cost at every junction,
and the brain replays one fixed strategy - it cannot DECIDE (time, score,
camera, faults). The three tracks below address exactly those two limits.

Rules for everything in this document:

* the robot hardware is frozen; STM32 changes are **additive and optional
  only** (the `SET_GYRO` precedent: implement, build, bench-validate, then
  gate it),
* **software first**: when a track needs new firmware, the Python side
  lands first behind a capability probe and changes nothing on the robot
  until the command exists there,
* one track at a time; every change keeps the whole test suite green,
  ships with new tests and updates the documentation,
* ASCII only, file headers with the author line, docstring on every
  function (repo conventions, see `07-development-guide.md`).

## 2. Decisions frozen here

| Decision | Choice | Rationale |
|---|---|---|
| **ROS2** | **No** | Two processes on one Pi (console, brain) already share one async core (`RobotSession` + `core.mission`). ROS2 would add a middleware dependency, a second build system and a rewrite of a working engine, without a concrete requirement. Revisit only if multi-robot coordination or hardware abstraction becomes real. The current stack does everything the match needs. |
| **Motion smoothing** | Pure Pursuit first, RAMSETE optional | Pure Pursuit is a geometric law on top of the path the planner already produces; RAMSETE (a full tracking controller) is the refinement to consider only if the simple law is not enough. Both are software-only until firmware `SET_VELOCITY` exists (Track B). |
| **Match layer** | Behavior trees in the brain | The console stays the workshop (draw, plan, test, calibrate); the brain gains a small, pure BT runtime so match logic can branch (time, score, camera, faults) instead of replaying one drawing (Track C). |
| **Firmware policy** | additive-only, gated, bench-validated | The safety invariants of `09-sensor-fusion-and-control-architecture.md` section 13 stay untouched: single writer, STOP/E-STOP highest priority, deadman unchanged. |

## 3. Baseline (2026-10-09) - what exists, what stays

| Area | Already there | This roadmap changes |
|---|---|---|
| Board firmware | protocol v3, GOTO/ARC/QUEUE_ARC, ramps, supervision (v3.3, bench-validated) | additive commands only (Track B, proposal) |
| Planner | drawing-wins routing, native arcs, near-straight ARCs, cusp collapse, re-anchors (`03-planner-algorithms.md` 9.2-9.4) | tuning values only (Track A) |
| Executor | segment model, `QUEUE_ARC` chaining, deadman/timeout supervision, KPI log lines | optional gated FOLLOW mode (Track B) |
| Brain | strategy v1 playback (`play(strategy)`), obstacle re-plan | BT runtime + strategy v2 documents (Track C) |
| Sensing | MPU6050 + silent gyro stream, at-rest bias refinement, bench stop/start, re-anchor (phase 4) | nothing (done) |
| Tools | Calibration tab with acceptance card: square / arc / chain / rest-drift / readiness (`/api/selftest/*`) | Tier-2 audits (section 8) |
| Camera | interface contract frozen (Docs/09 section 8) | Track C conditions consume it; calibration card when hardware arrives |

Open field items carried over: the KPI baseline on real missions, the
firmware bench checklist (`STM_docs/06` section 8), optionally setting
`GYRO_SIGMA_DEG_S` from the measured gyro noise, and the two known flaky
e2e tests (`test_api.py`, unrelated to this work).

## 4. KPIs we track (and where they come from)

| Metric | Source | Target |
|---|---|---|
| Segments / blended | `run finished: N segment(s), M blended without stopping` after every run | as many blended as the geometry allows; every stop explainable |
| Closing error | `run ended X mm / Y deg from the plan's last node` (warn at >= 20 mm / 3 deg) | below the warn threshold on a clean mission |
| Square accuracy | acceptance card `square` (fused + board odometry) | <= 20 mm / 2 deg |
| Arc accuracy | acceptance card `arc` (both directions) | <= 20 mm / 2 deg |
| Chain audit | acceptance card `chain` | 0 junction stops, closing error within tolerance |
| Rest drift | acceptance card `rest-drift` | <= 2 mm / 0.5 deg, no motion, no faults |
| Gyro noise | IMU card "Measure noise" | sigma recorded; feeds `GYRO_SIGMA_DEG_S` |
| Mission time | planner `est_s` vs real run duration (log timestamps) | less time, same accuracy (tracks A and B) |

## 5. Track A - continuity and tuning (no new code paths)

**Goal**: fewer stops and less time on real missions, at the same accuracy,
by changing PLANNER KNOBS only - every measurement comes from the existing
KPI lines and the acceptance card.

Why it may be enough on its own: the 2026-10-07 mission went from
17 segments / 16 board stops to 6 / 4 after the cusp-collapse and
native-arc work. The stops that remain are junction breaks the chaining
cannot remove and the periodic `CHAIN_REANCHOR_MM` anchors (one stop every
~900 mm of relative chaining).

Levers (one at a time!):

| Knob | Default | Effect when increased |
|---|---|---|
| `arc_fit_tol_mm` | 8 | more spans become native ARCs instead of GOTO chains |
| `line_tol_mm` | 3 | more straights become flat ARCs (fewer GOTO stops), at a small geometric bow |
| `corner_radius_mm` | 200 | smoother fillets, larger waypoint cut |
| `line_pair_mm` | 700 | longer flat S pairs (fewer segment boundaries) |
| `CHAIN_REANCHOR_MM` | 900 (constant in `trajectory.py`) | fewer absolute re-anchors (fewer stops) but more accumulated slip before correction |
| `speed_mm_s` / `lat_accel_mm_s2` | 500 / 300 | overall pace on curves |

Procedure (field session):

1. Run the acceptance card as the baseline (square, arc, chain, rest-drift).
2. Run one full real mission; copy the `run finished` / `run ended` lines
   and the wall clock out of the log.
3. Change ONE knob, re-plan, re-run the affected test and the mission;
   record before/after in the progress log of this document.
4. Repeat until either the KPI targets hold with few stops left, or the
   remaining stops are junction chains that no knob can remove - that is
   the Track B decision point.

**Acceptance**: chain audit 0 junction stops; no `run ended` warnings on a
normal mission; square/arc still within tolerance. Remote part: knob
arithmetic, doc updates, log analysis. On-robot part: everything measured.

## 6. Track B - streaming pursuit (software first, gated)

**Goal**: continuous path following - the board receives velocity targets
(v, omega) at 20-50 Hz and never stops at junctions; the path and its speed
law still come from the planner.

**Conditional track**: only worth it if Track A leaves junction stops that
hurt, or when the camera arrives and vision-servo headroom is wanted. The
groundwork below is cheap and off-robot, so it can start immediately.

**Status (2026-10-09): B1-B3 landed** - `pursuit.py` / `tracking.py` /
`sim.py` with the acceptance tests below (tracking within 15 mm in
simulation), the protocol constant + silent client + MockLink parity + the
deadman/stale-stop semantics with tests, and the gated `PathFollower`
(refuses to send a single frame unless the session's capability check
passes) with its own suite (`tests/test_follower.py`). The firmware command
is MERGED in 3.4 source (bench checklist: `STM_docs/06` section 9). B4
(go/no-go) still needs Track A field data; B6 remains.

**Status (2026-10-12): the executor FOLLOW mode + host wiring landed** -
`Executor.start(..., follow_path=...)` drives the plan polyline through
`PathFollower` inside the executor's own supervision shell (deadman,
stale-fault abort, caution `set_speed_scale`, PAUSE = STOP + hold, STOP
keeping the operator's reason, closing-error audit); the session injects
`supports_velocity_stream` as the capability and a requested stream is
REFUSED on an unsupported board - never a silent fallback. The console
`/execute` accepts `follow_path` (the UI posts the plan polyline; the
`STREAMING_EXEC` gate decides) and shows "streaming mode active" with a
live mm-to-go progress line; the brain runner streams strategy runs the
same way; mid-run re-plans and wait-resumes restart in the SAME backend
(`executor.last_stream`). New tests: `FollowModeTests` / `FollowCruiseTests`
(`tests/test_executor.py`), `FollowerHookTests` (`tests/test_follower.py`)
and a streaming end-to-end on the mock (`tests/test_api.py`). B4 (go/no-go)
still needs Track A field data; B6 (field validation, acceptance card in
streaming mode) remains.

**What it enables**: no stop-go at all on smooth paths, a place to put the
opponent-avoidance steering later, and a second execution backend to keep
the stop-turn program mode as the safe fallback.

### 6.1 Design (RPi side - implementable and testable off-robot)

| Piece | New/change | Content |
|---|---|---|
| `software/core/navigation/pursuit.py` | new, pure | lookahead point, curvature command, speed law (formulas below) |
| `software/core/navigation/tracking.py` | new, optional | RAMSETE variant (formulas below) |
| `software/core/navigation/sim.py` | new, pure | kinematic simulator (v, omega integrated) so tests close the loop without a robot |
| `software/robot/protocol.py` | change | reserve `CMD_SET_VELOCITY = 0x19` (proposal below) + capability probe |
| `software/robot/link.py` / `client.py` | change | write-only `set_velocity(v, omega)`, silent latest-wins |
| `software/core/mission/executor.py` | change | FOLLOW mode behind the `STREAMING_EXEC` gate; refuses to start when the board lacks the command |
| `software/robot/link.py` (`MockLink`) | change | simulate `SET_VELOCITY` so the whole chain is testable on the mock |
| console | later | banner "streaming mode active"; no workflow change |

Pure Pursuit (per tick):

```
L      = clamp(k_l * v, L_min, L_max)      # lookahead distance
target = first path point at distance >= L # walked forward from the robot
alpha  = atan2(...) - theta                # angle to the lookahead point
if |alpha| > align_deg:                    # target behind the robot
    v = 0; omega = clamp(align_gain * alpha, +-align_max)   # turn in place
else:
    kappa = 2 * sin(alpha) / L             # commanded curvature
    omega = v * kappa                      # yaw-rate command
    v     = min(v_cruise, sqrt(a_lat / |kappa|))   # keep lateral accel in budget
```

The alignment branch is mandatory: with a target almost behind the robot the
curvature is tiny while the speed stays full, so the robot drives straight
ahead instead of turning (2026-10-18 field incident: it drove out of the
field).

`v` and `L` depend on each other: iterate once per tick. `v_cruise` per
node comes from the existing planner speed profile.

RAMSETE (optional refinement, designed for a reference trajectory):

```
e   = R(theta)^T * (p_ref - p)             # robot-frame error
eth = wrap(theta_ref - theta)
v     = v_ref * cos(eth) + k_x * e_x
omega = omega_ref + v_ref * (k_y * e_y + k_theta * sin(eth))
```

### 6.2 Protocol proposal (freeze with the firmware author)

`SET_VELOCITY` (0x19, proposal): payload `i16 v [mm/s]` (signed),
`i16 omega [0.1 deg/s]` (signed), silent latest-wins like `SET_GYRO`.
Board ramps internally to the targets, keeps stall/limit supervision;
when no valid frame arrives for ~200 ms the board ramps to a controlled
stop (NOT the deadman - the deadman stays a link watchdog). The command is
accepted only when the board is idle or explicitly in streaming mode;
mixing it with program commands is rejected (`BUSY`/`RANGE`). The probe:
send it once, `UNKNOWN_CMD` means "not supported" and the software gate
stays off.

### 6.3 Steps

| Step | Where | Deliverable |
|---|---|---|
| B1 | off-robot | `pursuit.py` + `sim.py` + tests (tracking error on circle / fillet square / S-curve) |
| B2 | off-robot | protocol constant + link/client methods + mock support + tests |
| B3 | off-robot | executor FOLLOW mode behind the gate + tests (mock closes the loop) |
| B4 | off-robot | Track A data read; go/no-go decision recorded here |
| B5 | on-robot | firmware `SET_VELOCITY` + bench checklist (future `STM_docs` note) |
| B6 | on-robot | field validation: acceptance card in streaming mode, KPI comparison vs Track A |

**Acceptance (simulation)**: tracking error <= 15 mm at 500 mm/s on a
300 mm radius circle, a filleted square and an S-curve; no speed
oscillation. **Acceptance (robot)**: square and arc pass in streaming
mode; stop count per mission effectively 0.

## 7. Track C - behavior-tree match layer

**Goal**: the brain can DECIDE during a match: run the main mission, react
to failure (fall back to a backup path), stop when time is up, use the
camera or not. The drawing stays the mission description; trees describe
the DECISIONS around it.

### 7.1 Architecture

```
strategy v2 (kind "bt") --> brain match runner --> bt.tick loop
                                                    |-- actions    --> mission engine
                                                    |                 (execute_plan, watcher, executor)
                                                    |-- conditions --> session
                                                                      (pose, IMU, vision, team, clock)
```

| Piece | New/change | Content |
|---|---|---|
| `software/core/mission/bt.py` | new, pure | Status (RUNNING/SUCCESS/FAILURE), composites (Sequence, Selector, Parallel), decorators (Inverter, Retry, Timeout, ForceSuccess), blackboard, async tick |
| `software/brain/match/nodes.py` | new | the leaf catalogue (tables below) wrapping existing services only |
| `software/core/mission/strategy.py` | change | version 2: `"kind": "bt"` + `tree` field; v1 documents keep working (the runner wraps them as a one-leaf tree) |
| `software/brain/runner.py` | change | `play()` accepts both kinds; the leaf implementation calls the SAME `execute_plan` / watcher functions as today |
| console | later | step 1 read-only tree view, step 2 visual editor (the drawing workflow is untouched) |

Node catalogue v1 (implemented in `software/brain/match/nodes.py`; unknown
names fail at LOAD time, never mid-match):

| Conditions | Meaning |
|---|---|
| `at_pose(x, y, theta?, tol_mm?, tol_deg?)` | fused pose within tolerance of a pose |
| `executor_idle()` | no mission segment running |
| `camera_fresh()` | the vision snapshot is inside its TTL |
| `time_left(seconds)` | `match_s` deadline has time left (no deadline = True) |
| `score_at_least(points)` | blackboard score reached |
| `team_is(side)` | configured `TEAM` matches `yellow` / `blue` |

| Actions | Meaning |
|---|---|
| `run_document(doc)` | plan + execute a v1 document (the existing `play` path) |
| `wait(seconds)` | pause the tree (async, cancellable by `timeout`) |
| `log(message)` | one line into the brain log |
| `add_score(points)` | raise the blackboard score (manual/referee today) |
| `abort_run(reason)` | stop a running mission right now |

Document sketch (v2):

```json
{
  "name": "match-2026",
  "version": 2,
  "kind": "bt",
  "tree": {
    "type": "sequence",
    "children": [
      {"type": "action", "name": "run_document", "doc": "collect-main"},
      {"type": "selector", "children": [
        {"type": "action", "name": "run_document", "doc": "collect-backup"},
        {"type": "action", "name": "log", "message": "backup path failed"}
      ]},
      {"type": "action", "name": "wait", "seconds": 1.0}
    ]
  }
}
```

### 7.2 Rules

* Leaves call the EXISTING engine (`execute_plan`, watcher, executor) -
  no new motion path, no new board commands; parity with the console is
  by construction (both hosts share `RobotSession`).
* One plan at a time (`plan_lock`), one segment run at a time (executor);
  STOP/E-STOP remain independent of the tree.
* The tick loop is deterministic in tests: clock, score and vision are
  injected; no wall-clock sleeps inside nodes.
* v1 strategies keep playing unchanged; the console keeps saving v1 while
  the v2 editor does not exist.

### 7.3 Steps

| Step | Where | Deliverable |
|---|---|---|
| C1 | off-robot | `bt.py` + scheduler unit tests (all composites/decorators, running-child resumption) |
| C2 | off-robot | `nodes.py` + strategy v2 schema + loader + tests (mock, fake clock) |
| C3 | off-robot | brain plays trees (`--strategy` auto-detects kind); parity tests vs v1 behaviour |
| C4 | on-robot (later) | field rehearsal: a real v2 strategy plays a full mission; console tree view |

**Acceptance**: a v2 tree expresses "main mission, fallback document,
time cutoff" and runs to completion on the mock; every v1 test unaffected.

**Status (2026-10-09): C1-C3 landed** - `core/mission/bt.py` with the
scheduler tests, `brain/match/` (registry + `play_tree`), strategy v2
(`kind` + `tree`, v1 documents load unchanged) and full tree playback on
the mock board (`tests/test_brain_match.py`); the CLI plays both kinds
(`--repeat` is v1-only - a tree decides its own repetition). C4 (console
tree view + field rehearsal) pending.

## 8. Backlog (smaller items)

| Tier | Item | Remote? | Status |
|---|---|---|---|
| T1 | Brain KPI log passthrough: `Brain.on_event` prints `{"type": "log"}` events (the `run finished` / `run ended` lines are dropped headless today) | yes (code + tests) | done 2026-10-09 |
| T1 | Bus health check: readiness probe extension measuring ping round-trip jitter and error counts over N seconds | yes (code), robot validates | proposed |
| T1 | OLED test buttons: console card sending `DISPLAY_TEXT` (firmware already supports it) | yes (code), robot validates | proposed |
| T2 | Turn repeatability + scrub audit: N x 90 deg both directions, error distribution per direction | code yes, run on robot | done 2026-10-09 (mock-tested) |
| T2 | Loop-drift audit: repeated closed loops, separating control error from estimation error (board vs fused vs ticks) | code yes, run on robot | done 2026-10-09 (mock-tested) |
| T2 | Speed/ramp audit: step response of commanded speeds, overshoot and settle time | code yes, run on robot | done 2026-10-09 (mock-tested) |
| T2 | Guided fault drills: deadman expiry, e-stop rehearsal (telemetry-pause helper in the session) | code yes, run on robot | done 2026-10-09 (mock-tested) |
| T2 | Encoder check: forward/backward/turn tick scale, direction and polarity checks | code yes, run on robot | done 2026-10-09 (mock-tested) |
| T3 | Camera calibration card | blocked (camera hardware) | later |

## 9. Recommended working order (remote first)

Off-robot, in this order:

1. **T1 brain KPI log passthrough** - tiny, unblocks reading mission KPIs
   from `docker logs` during the next field session.
2. **Track C, C1 + C2** - the BT core and node catalogue are pure Python,
   fully unit-testable on the mock; highest strategic value.
3. **Track B, B1-B3** - pursuit + simulator + protocol reserve + gated
   follow mode; nothing changes on the robot (gate off, probe absent).
4. **T2 audit endpoints** - write the code now, they run at the robot.

At the robot, next session:

5. Track A baseline + tuning, acceptance card, T2 audits live,
   Track B go/no-go (B4).
6. If go: B5 firmware + bench; then C3/C4 field rehearsal.

## 10. Progress log

| Date | Item | Result |
|---|---|---|
| 2026-10-09 | Roadmap document created | tracks A/B/C frozen as above |
| 2026-10-09 | T1 brain KPI passthrough | done - log events print headless; test added |
| 2026-10-09 | Track B B1 | done - pursuit/tracking/sim + acceptance tests |
| 2026-10-09 | Track B B2 | done - protocol 0x19, silent client, mock parity + tests |
| 2026-10-09 | Track B B3 | done - gated PathFollower + tests over the mock |
| 2026-10-09 | Track B B5 (source only) | done - firmware SET_VELOCITY merged (fw 3.4); IAR build + bench pending |
| 2026-10-09 | Track C C1-C3 | done - bt.py, match nodes, strategy v2, brain playback + tests |
| 2026-10-09 | Tier-2 audits (T2) | done in code - turn-repeat, loop-drift, speed-ramp, encoders + UI + tests |
| 2026-10-09 | Fault drills (T2) | done in code - deadman + e-stop drills, telemetry pause + tests |
| 2026-10-12 | Track B: executor FOLLOW mode | done - `follow_path` runs the follower behind the capability gate; refusal on unsupported boards; caution scale, PAUSE/RESUME and STOP hooks reused; tests over the mock |
| 2026-10-12 | Track B: host wiring + console | done - `/execute` accepts `follow_path` (UI posts the plan polyline, streaming banner), brain runner streams strategy runs, mid-run re-plans keep the backend; e2e test on the mock |
| 2026-10-18 | Field safety: full obstacle margin + combined arrival | done - routing prefers the full margin (tighter tiers only when there is no room, always warned), waypoints/goals are projected out of every inflated zone, `navigation/arrival.py` plans the shortest legal combined arrival (intermediate point + exact ARC exit, field/obstacle checked), console clamps waypoints/goals, follower turns in place instead of driving at a target behind it, field-edge geofence + final-heading alignment; tests added |
| 2026-10-19 | Configurable close-waypoint clearance | done - Robot > Waypoint clearance (extra mm over half the body side, 0 = 150 mm for a 300 mm body); waypoints/borders use the stop-clearance rule, a close point is reached exactly by a straight final leg, the arrival angle is performed at a turn-safe stand-off first, and a continuing mission departs with a reverse/forward move to a turn-safe point before rotating; rotated-body checks, mid-run reaction ladder made body-aware, segment backend forced over streaming; planner + follower tests |
| TBD | Track A baseline + B4 decision | needs a field session |
| TBD | Track C C4 (console tree view, field rehearsal) | pending |

## 11. References

* [02 - Software architecture](02-backend-architecture.md),
  [03 - Planner & algorithms](03-planner-algorithms.md),
  [04 - Console guide](04-console-guide.md),
  [05 - API reference](05-api-reference.md),
  [07 - Development guide](07-development-guide.md),
  [08 - Brain & strategies](08-brain-and-strategies.md),
  [09 - Sensor fusion & control architecture](09-sensor-fusion-and-control-architecture.md)
* Firmware set: `STM_docs/03-i2c-protocol-spec.md` (command table),
  `STM_docs/06-bringup-testing-tuning.md` (bench checklist)
* Code: `software/core/navigation/` (planner, velocity, pursuit,
  tracking, sim), `software/core/mission/` (executor, follower, bt,
  orchestrator, watcher, strategy), `software/brain/runner.py`,
  `software/brain/match/` (nodes + play_tree), `software/robot/protocol.py`,
  `software/session/session.py`
