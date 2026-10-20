# 03 - Planner & algorithms

Everything the planner does happens in `software/core/navigation/` plus the
orchestration in `software/core/mission/orchestrator.py`. This document explains each stage,
the design rules behind it, and the exact formulas.

```
elements --> line/arc fit --> routing (Theta*) --> corner fillets -->
curvature-aware sampling --> velocity profile --> GOTO/ARC/TURN/PAUSE segments
```

The **one rule above all others: the robot never enters a clearance zone.**
The planner re-routes every leg that would enter the configured obstacle
margin (`half_diagonal + obstacle_margin_mm`) or the legal field border, and
detours are the *shortest* route on that safe grid. A tighter tier is used
only when the full margin leaves no room at all, and it is always named in a
warning. **Goals** are held to the full margin (the robot stands there and
may turn in place). **Waypoints** have their own configurable stop clearance
(`waypoint_margin_mm`): 0 mm lets the centre stand half the robot's larger
body side from an obstacle or the field border, and a larger value adds to
that floor. A waypoint inside the routing margin is reached by a **straight
final leg**; when it is closer than the half-diagonal (turning in place would
sweep a body corner into the obstacle), the arrival angle is performed at a
turn-safe stand-off BEFORE that leg. For a running mission, a new obstacle
only causes a stop + re-route when the **remaining** route is really affected.

## 1. Input model

```json
elements: [
  {"kind": "stroke",   "points": [[x, y], ...]},   // freehand / polyline
  {"kind": "waypoint", "x": ..., "y": ...},          // must-pass point
  {"kind": "rotate",   "angle_deg": 90},         // + = CCW
  {"kind": "pause",    "seconds": 2},
  {"kind": "release"}                           // release stepper drivers
]
goal:      {"x": ..., "y": ..., "theta_deg": 90|null}
obstacles: [{"kind":"rect","x","y","w","h"} | {"kind":"circle","x","y","r"}]
options:   PlanOptions (below, all optional)
```

`PlanOptions` (defaults):

| Option | Default | Meaning |
|---|---|---|
| `speed_mm_s` | 500 | cruise speed used for the whole plan |
| `turn_speed_mm_s` | 150 | speed of in-place turns (rotations) |
| `min_speed_mm_s` | 100 | floor sent to the board (its creep speed) |
| `accel_mm_s2` / `decel_mm_s2` | 700 / 700 | trapezoidal profile |
| `lat_accel_mm_s2` | 300 | lateral acceleration limit on curves |
| `goto_step_mm` / `step_mm` | 250 / 50 | node spacing on **straights** / fine steps |
| `node_chord_tol_mm` | 6 | max chord error of curved sections |
| `max_node_turn_deg` | 15 | max heading change per node |
| `min_node_step_mm` | 40 | never place nodes closer than this |
| `corner_radius_mm` | 200 | fillet radius (0 = no rounding) |
| `waypoint_cut_mm` | 50 | the corner fillet stays within this distance of the waypoint |
| `fit_tol_mm` | 8 | line/arc fit tolerance |
| `native_arcs` | `true` | fitted arcs become one firmware ARC segment |
| `robot_width_mm` / `robot_height_mm` | 300 / 300 | body size (position = centre) |
| `obstacle_margin_mm` | 150 | full safety clearance around obstacles used by the ROUTER |
| `waypoint_margin_mm` | 0 (console) / unset (API) | extra stop clearance around waypoints; 0 = half the larger body side, unset = legacy full routing clearance |
| `waypoint_cut_mm` | 50 | how "deep" a fillet may sweep past a waypoint (0 = exact) |
| `inflate_mm` | 0 | legacy override of the obstacle inflation (0 = derive) |
| `grid_mm` | 50 | occupancy grid resolution |
| `min_speed_mm_s` | 40 | floor for node speeds |
| `native_arcs` | true | emit one ARC segment for constant-curvature chains |
| `arc_fit_tol_mm` | 8 | max circle-fit residual on an ARC span |
| `arc_min_radius_mm` | 60 | smallest radius an ARC segment may have (the tightest the firmware drives well) |
| `arc_min_sweep_deg` / `arc_max_sweep_deg` | 8 / 340 | sweep range of an ARC segment |
| `native_lines` | true | near-straight gaps are driven as flat ARCs, then merged into ONE GOTO per straight run (no GOTO stop-go, no alternating-S wiggle) |
| `line_pair_mm` | 700 | max length of one flat S-pair on a straight run |
| `line_tol_mm` | 3 | how far samples may stray from the span chord and still count as straight |
| `smooth` / `fit_enabled` / `avoid_obstacles` | true | stage switches |
| `max_segments` | 600 | warning threshold |
| `resume_mid_mission` | false | internal: suppress the first-connector warning on re-plans |

## 2. Stroke fitting (`geometry.py`)

A drawn stroke is a noisy point list. It is converted to **exact primitives**
(lines and arcs) so that a drawn arc is driven as an arc:

1. **RDP pre-pass** - micro-jitter below `fit_tol_mm` is removed.
2. **Robust circle fit (Kasa)** - algebraic fit; up to 15 % of outliers are
   trimmed and the fit is kept only if every remaining point is within
   `fit_tol`.
3. **Recursive line/arc segmentation** - the span is split at the maximum
   chord-deviation index; a span is accepted as a line if all points lie
   within `fit_tol` of the chord, otherwise an arc is attempted. An arc is
   accepted only if it **follows the drawn polyline** (probe samples must hug
   it) and **lands on the end point** - a sparse 3-point corner (which fits a
   perfect circle) stays a corner, not a semicircle.
4. Accepted primitives are regenerated from their parameters
   (`primitive_to_points`) - arcs come back as true arcs, not as the noisy
   original samples.

Deduplication and the anchor logic happen in the pipeline (below).

## 3. Occupancy grid (`grid.py`)

`OccupancyGrid(width, height, res, inflation_mm, border_mm)`:

* A cell centre is **blocked** when its **Euclidean (Minkowski) distance** to
  any obstacle shape is below `inflation_mm`. This matters: the earlier
  axis-aligned expansion over-blocked rectangle corners by up to sqrt2.
* Cells whose centre is within `border_mm` of the field edge are blocked -
  the whole body stays on the table.
* `theta_star(start, goal)` - 8-connected Theta\* with any-angle LOS
  shortcutting, octile heuristic, no corner cutting. Blocked endpoints are
  snapped to the nearest free cell within 500 mm and re-attached exactly if
  they were free. `los(a, b)` samples the segment every `res/3`.

Three grids are built per plan from the robot geometry:

| Tier | Inflation | Used for |
|---|---|---|
| **full** | `half_diagonal + obstacle_margin_mm` (300x300 + 150 -> 362.1) | the routing grid: any leg entering it is re-routed |
| **relaxed** ("squeezed") | `half_diagonal + 0.5 margin` (-> 287.1) | fallback only when the full margin has no corridor |
| **body** | `half_diagonal + max(30, 0.6-grid)` (-> 242.1) | last resort (a stop point standing inside a fresh zone); always warned |

`half_diagonal = hypot(width, height)/2` (-> 212.1 mm), `border = half_diagonal`.

Waypoints and goals are **stop points**: their position is first projected out
of every inflated zone to the closest fully-clear point
(`clearance.closest_clear_point`, Euclidean projection onto the rounded
rectangle / circle), then the route must reach that exact point on the full
grid. The console applies the same projection when a waypoint or goal is
placed, so the point the operator sees is the point the planner uses.

## 4. Routing - safe and short (`trajectory._route`)

The fitted geometry is walked leg by leg on the **full** grid:

1. A free leg is kept as drawn. A leg that enters the margin starts a
   **run**: the chain of consecutive blocked legs. The whole run gets **one**
   Theta\* detour from the last reached point `a` to the chain's free end
   `b`. (Older versions re-routed leg-by-leg; that produced the "mess" of
   zig-zags.)
2. **Detour tiers** are tried in order of safety, not length:
   * full-margin theta\* -> warning *"leg i re-routed to stay clear (obstacle
     or field border)"*,
   * relaxed theta\* -> *"leg i squeezed through the obstacle margin - no room
     for the full margin there"*,
   * body theta\* -> *"leg i re-routed at body clearance (the drawn line is
     blocked)"* (both run ends may be snapped: a stop point inside a freshly
     inflated zone).
   Theta\* already returns the **shortest** route on its grid, so the first
   feasible tier is both the safest and the shortest legal answer. The tier
   that ran is always named; the console never silently drives closer than
   the configured margin.
3. Detour polylines are simplified with RDP at `res-0.8` (grid jitter), while
   the drawn geometry keeps its original shape. There is **no blanket
   simplification of the whole route afterwards** - a coarse RDP at `res-0.5`
   used to flatten exactly the 5 deg-sampled fitted arcs back into a handful of
   long chords (and with them the node placement the sampler derives).
4. If no route exists, the drawn run is kept with a warning
   (*"blocked but no route found - kept direct"* / *"...lies inside the robot
   clearance zone...- kept as drawn"*).

Connectors: when a stroke starts further than `ANCHOR_SNAP_MM = 15 mm` from
the previous anchor (previous stroke end / waypoint / the robot pose), a
connector leg `[anchor, start]` is routed first and merged into the run, so
joints get filleted too. If the very first stroke starts away from the robot,
a warning says so.

## 5. Corner rounding - tangent fillets (`geometry.round_corners`)

Every sharp vertex (turn between 10 deg and 170 deg) of a merged run is replaced by
a **tangent fillet**:

* tangent length `= min(r-tan(theta/2), 0.45-leg1, 0.45-leg2)` - the fillet never
  eats more than 45 % of either leg;
* sharper corners get a wider blend: radius x 1.0 ... 1.8;
* **per-corner veto** - a fillet is rejected when any of its points would
  enter the FULL clearance grid (the same obstacle margin as routing); the
  corner then falls back to 0.6x and 0.3x radius, and if all fail it **keeps
  its vertex** (counted in one warning: *"N corner(s) kept sharp where
  rounding would leave the safety zone"*). Other corners are unaffected.
* `radius_limit_for(vertex)` caps the fillet *tolerance* at waypoints (next
  section) and at forced waypoints (inside a clearance zone) the veto is
  lifted so the path still sweeps.

## 6. Waypoints

A waypoint is a point that must be crossed. Two behaviours:

* **swept (default, `waypoint_cut_mm = 50`)** - the run continues through the
  point; the corner fillet is capped so that the path stays within
  `cut` mm of the exact waypoint:
  `radius = min(cut / (sec(half) - 1), 800)` - for a 90 deg corner a 50 mm cut
  means a ~20.7 mm fillet apex distance. The robot glides past the point
  (like a rounded drawn corner), it does not stop on it.
* **exact (`waypoint_cut_mm = 0`)** - the program is *cut* around the
  waypoint: the run ends ON the point (a GOTO node lands exactly there), the
  next run starts there. Knife-point mode for precision actions.

A waypoint element may carry an optional `speed_mm_s`. It caps the
connector and straight final leg that reach the point (and, for a close
point, its departure move); leave it out for the system speed. The same
field is accepted by strokes, rotations and moves. For a negative `drive`
the speed is carried into the reverse `DRIVE` segment instead of falling
back to the turn speed.

A waypoint has its own stop clearance, `waypoint_margin_mm` (Robot tab):

* **0 mm** - the centre may stand half the robot's larger body side from an
  obstacle or the field border (150 mm for a 300 mm body).
* **X > 0 mm** - that distance grows by X (300 mm for the default 150 mm
  margin + 50 mm).
* The rotated body is checked at the final approach heading too: a waypoint
  hugging a wall diagonally is pushed out until the body CORNER clears, not
  just the half-side circle (`clearance.closest_body_clear_point`).

A waypoint may therefore intentionally stand inside the routing margin. It
is not moved out to the full margin any more: the planner routes to a
turn-safe **stand-off** on the full grid, then drives a **straight final
leg** into the point. The leg is sampled and checked against the real
rotated body, so the approach itself cannot touch the obstacle/border. When
the point is closer than the half-diagonal + 6 mm, rotating on it would sweep
a body corner into the obstacle: the arrival angle is performed at the
stand-off (the GOTO's final heading) and the robot then drives the last
stretch already facing the commanded angle.

A mission that CONTINUES after such a close waypoint still reaches the
point **exactly**. The departure is planned as a straight longitudinal move
(`trajectory` program step `depart`, emitted as a `DRIVE` segment): the
robot reverses out of the tight spot (reverse tried first, forward when the
obstacle is behind it) to a turn-safe, route-free point, rotates there and
continues. The move is inserted before the next motion, so a `pause` still
happens AT the waypoint. When the next element is a stroke that starts on
the waypoint, the leading piece is replaced by a short join from the
departure point; the join point itself is required to be turn-safe too, so
the merge cannot swing the body into the border. If no longitudinal escape
exists at all (a pocket), only then is the stop moved to the closest
turn-safe point.

If the commanded final heading cannot be reached by a straight leg (e.g. it
runs parallel to the wall: the straight line would keep the same wall
distance), safety wins: the stop is moved to the closest turning-safe point
with the warning *"waypoint ... too close to rotate in place and no safe
straight final leg exists - moved to the closest turning-safe point ..."*,
so the angle and the departure are performed legally.

Omitting `waypoint_margin_mm` keeps the historical behaviour (projection to
the full routing clearance, warning *"waypoint (x, y) mm sits inside the
robot clearance zone - moved to the closest allowed point (x, y) mm"*); the
console always sends its configured value (default 0). The console clamps
new/moved waypoints with the same stop clearance; goals keep the full
clearance.

A waypoint that coincides with the robot's current start (the usual first
element: "start here, facing this way") is **not** a stop: it adds no cut,
no zero-length GOTO and no arrival manoeuvre - the robot is already there,
and the first real movement handles the heading. Before this guard, the
zero-length GOTO's arrival heading disagreed with the outgoing leg and
forced the whole plan onto the stop-and-turn segment backend ("the first
steps did a lot of unnecessary turns", 2026-10-18).

Obstacle geometry is clipped to the playfield at the top of `plan` (and by
the console when drawing/loading): a Box or Zone can no longer extend
outside the game area, so the grid, the drawing and the saved strategy all
agree.

### Arrival angle (`theta_deg`, UI *arrival* + *combine with the approach*)

A waypoint may carry an arrival angle: the robot has to HOLD that heading ON
the point. It is performed for every approach shape and BEFORE any `pause`
that follows the waypoint, so the mission order is *reach the point -> face
the commanded angle -> wait -> turn away toward the next step*. That last
alignment turn really comes after the pause
(`trajectory._align_arc_after_arrival`); it is needed because the board builds
every ARC's circle from the CURRENT heading, so an arc entered right after the
waypoint would start on a different circle ("the robot went completely to the
right", 2026-10-01).

* **not combined** - the node landing on the waypoint carries the angle. A
  GOTO node turns to it as its final phase; when the point is the EXIT of a
  near-straight ARC (straights are emitted as flat ARCs, so this is the common
  shape and used to drop the command silently, 2026-10-09) an explicit
  in-place pivot from the arc's exit heading is inserted right there. A
  zero-length GOTO would first turn to its degenerate travel heading - a
  double turn the operator sees as a dance.
* **combined** (`"combine": true`) - the arrival is planned BEFORE the run is
  emitted (`navigation/arrival.py`):
  1. Candidate manoeuvres are circular arcs that END on the waypoint with the
     exact exit tangent `theta`, for both turn directions and a grid of
     radii (60..2000 mm) and sweeps (20..170 deg). Every candidate is
     sampled every 5 deg and must stay inside the legal border AND outside
     every inflated obstacle (full margin, line-of-sight between samples).
  2. When the waypoint closes the drawn run, the search walks the last
     700 mm of that run: any point on it may become the arc start `Q`, the
     run's tail is replaced by the arc. When the waypoint is free-standing,
     the planner routes (Theta\*, full grid) to the most promising arc starts
     - the "intermediate point" of the manoeuvre.
  3. The cheapest **total** wins: route/prefix length + arc length + the
     equivalent cost of any in-place turn at `Q` (2.2 mm per degree) + a
     penalty for cutting drawn geometry. An arc is preferred over a stopping
     turn while it costs at most 25 % + 30 mm more (that is what "combine"
     asks for); a wide loop near a wall is never chosen.
  4. The arc is emitted as an explicit ARC segment (the sampler cannot
     flatten it), the node at `Q` is tagged so the wire sanitiser keeps it
     facing the arc entry, and the exit is exactly `theta`. No legal arc ->
     the robot drives to the (already safe) waypoint and turns in place,
     with the warning *"the arrival angle at waypoint (x, y) mm could not be
     woven into the approach - the robot turns in place there"*.

  **Close waypoints bypass this weave.** When `waypoint_margin_mm` is
  configured and the point stands inside the routing margin, the planner
  does not try to swing an arc into it: it searches the reverse final
  heading for a stand-off that is turn-safe and fully route-reachable, marks
  that leg GOTO-only (`close_waypoint`) and drives a straight final leg.
  This also forces the segment backend in the executor - the streaming
  follower would cut the stand-off corner towards the obstacle.

  The old implementation only rewrote the last ~100 mm of the emitted GOTO
  tail, could not route to an intermediate point and validated arcs against
  body clearance only - near a wall it either refused or swung the sweep
  outside the game area (2026-10-18 field session).
* When even the fallback cannot land on the waypoint the warning is
  *"waypoint (x, y) mm is not a stop node on the route - its arrival angle
  cannot be performed"*. Safe waypoints make this unreachable in practice:
  the point is projected to full clearance before any of this runs.

## 7. Sampling - curvature-aware node spacing

`path_resample(points, goto_step, chord_tol, max_turn, min_step)` walks the
smoothed path chord by chord; every limit is measured **on the polyline
itself** and applies to the chord being placed:

* **deviation** - the skipped geometry stays within `chord_tol` of the chord
  (checked point by point, not from an estimated radius),
* **span** - the tangent change inside one chord is <= `max_turn`,
* **alignment** - the chord direction stays within `max_turn / 2` of the
  tangent it ends on (so the next chord can always satisfy the turn limit),
* **turn** - each chord turns <= `max_turn` from the previous one,
* **length** - no chord exceeds `goto_step` (`min_step` floors the base).

This is what makes the firmware follow an arc as an arc: a 90 deg fillet becomes
a chain of <= 15 deg node turns instead of one 90 deg pivot; when such a same-sign
chain is clean enough, segment emission replaces it with a **single native
ARC segment** (section 9.1) and the node chain only feeds the preview. Because the
limits are
local, a tight fillet densifies only its own nodes - the long straights around
it keep the coarse `goto_step` spacing (an index-windowed radius estimate used
to bleed a fillet's small radius backwards and filled a 1.3 m straight with
3-80 mm nodes).

## 8. Velocity profile (`velocity.py`)

`speed_profile` computes a trapezoidal profile over the sample distances with
`accel`, `decel` and a **lateral limit** on curves
(`v = sqrt(lat_accel / kappa)`, `kappa` = the local curvature); each node gets
`v = clamp(speed_i, min_speed, 900)` (900 = absolute protocol ceiling; the board's stored speed limit -
default 800 mm/s - clamps further at execution time). Estimated time `est_s`
is the sum of segment times plus pause seconds.

## 9. Segment emission

* Nodes -> `{"kind": "goto", "x", "y", "th", "v"}`; **theta faces the next node**,
  recomputed across joints; nodes closer than `DEDUPE_NODE_MM = 2 mm` are
  dropped (a larger threshold once swallowed small fillets - the robot made
  one pivot where the path was smooth).
* A **same-sign turning chain** of nodes whose circle fit holds within
  `arc_fit_tol_mm` -> one `{"kind": "arc", ...}` segment (native firmware ARC,
  section 9.1); a failed conversion is harmless - those nodes stay GOTOs.
* `rotate` -> `turn` chunks <= 170 deg (firmware TURN accepts <= 180 deg).
* `pause` -> `pause` segments (0.1-600 s).
* `release` -> one `{"kind": "release"}` segment with no geometry; the executor sends the
  idle-only `RELEASE_MOTORS` command after the previous movement is fully stopped. The next
  motion segment re-energizes the drivers automatically.
* The **goal** is a normal element at the end of the mission: its leg is
  routed like any other; if the goal itself is blocked it is force-appended
  (*"goal lies inside the robot clearance zone ...- reached directly"*), and
  `theta_deg` (if given) becomes the heading of the **last** node.
* Node headings look *through* pause segments to the continuation, so the
  robot is aligned when it resumes instead of turning twice.

### 9.1 Native arc emission (`pipeline._native_arcs`)

Walking the samples, the longest chain that still hugs a single circle is
grown into a *turn span*. A span becomes **one firmware ARC command** only
when all of this holds:

* the circle fitted to the span hugs every kept sample within
  `arc_fit_tol_mm` - wobbly ends are trimmed off (the trimmed ends fall back
  to GOTO nodes), and so are **straight stubs**: an end whose travel
  direction disagrees with the circle tangent by more than 10 deg is dropped
  and the circle refitted, so the arc starts and ends on the bend itself;
* radius within `arc_min_radius_mm` .. 20000 mm, sweep within
  `arc_min_sweep_deg` .. `arc_max_sweep_deg`, arc length <= 2900 mm (firmware
  limits);
* a span starting at the very beginning of a run has no entry node to
  pre-align the heading: it needs the previous GOTO to already face the arc
  tangent, or a known robot heading within 5 deg (first motion of the plan).

The emitted segment carries `x0, y0` (entry), `x1, y1` (exit), signed
`radius_mm` (+ = centre on the left of travel), signed `sweep_deg` (+ = CCW,
same sign as the radius), `th0_deg` (entry tangent), `v` (the slowest node
speed of the span) and `points` (the samples, for the console preview). The
node before the arc is re-headed to `th0_deg`; firmware drives the whole
sweep as one continuous move - no per-node turn/stop, no micro-jitter of
short chords, and a program many segments shorter. `native_arcs: false`
disables the conversion (everything stays GOTO nodes).

### 9.2 Near-straight ARCs (`pipeline._near_straight_arcs`)

The board brakes to a **full stop at every GOTO node** (TURN -> DRIVE ->
TURN with a ramp reset per segment); only consecutive ARCs queued through
`QUEUE_ARC` are blended into one continuous move. A straight run emitted as
the resampler's 250 mm node chain was therefore stop-and-go, and every
arc -> straight junction cost a stop plus an in-place turn. The straight
gaps between the fitted arcs are now emitted as **very flat ARCs**:

* the sweep is an **exact multiple of the 0.1 deg wire quantum** and the
  radius follows from it (`R = half / (units * 0.1 deg)`). The firmware
  re-derives its arc length as `sweep * R`, so the span is driven at
  exactly its measured length - at 32767 mm one 0.1 deg quantum is already
  57 mm of arc length, a rounded sweep would move the robot tens of mm;
* a span up to `line_pair_mm` drives as ONE arc; longer spans are cut into
  half-arcs that **alternate side** (an extremely flat S), so the span
  leaves and re-joins the run direction and the junctions stay tangent. The
  bulge (`half^2 / 2R`) stays below ~2 mm, well inside `fit_tol_mm`;
* samples straying more than `line_tol_mm` from their chord, or bowed enough
  to tilt the ends by more than ~0.5 deg, are **not** straight (the circular
  fit refused them for a reason): they keep their GOTO nodes and the
  stop/turn fallback;
* very short spans (< 30 mm) and spans whose samples are too sparse to split
  keep their GOTO nodes.

A chain of circular arcs can never be a *perfect* straight line: the flat S
keeps the drawn geometry within ~2 mm over a normal run (the sagitta of a
250 mm piece at 32767 mm radius), which is well inside `fit_tol_mm`.

**One straight move, not an S.** After the arrival/tail passes, a run of
flat ARCs and GOTO nodes that stays within `STRAIGHT_MERGE_MAX_DEV_MM`
(6 mm) of its chord and turns less than 4 deg is collapsed into **one
GOTO** (`pipeline._merge_straight_runs`). The real robot read the tiny
curvature sign alternation of the flat S as *"a lot of acceleration and
deceleration and unnecessary turns"* (2026-10-18); one GOTO gives exactly
one acceleration, a straight drive and one deceleration. The merged
segment keeps its sampled polyline in `points`, so the map preview still
draws the full run, and the arrival tangent is kept as the GOTO's final
heading. Runs with different per-stroke speeds are never merged.

### 9.3 Cusp collapse (`pipeline._collapse_cusps`)

A hand drawn reversal is often tighter than the firmware's 50 mm ARC floor
(real mission 2026-10-07: the sampler followed a 10 mm switchback as **14
GOTO nodes 2-4 mm apart, turning 10-13 deg each** - the robot stopped at
every one: *"a lot of turning unnecessary and not smooth"*). The whole
excursion fits inside the robot's own footprint (300 mm), so no wheel
geometry can make it smooth.

A run of GOTO nodes whose **equivalent radius** (travelled length over
turned angle) is below the drivable ARC minimum is replaced by its **last
node** - the exit corner, which is also the entry node of the following ARC
when there is one. The board drives straight there and turns in place
**once** (the GOTO final turn points the robot at the next step). Guards:

the run must turn at least 70 deg end to end, must stay under 150 mm of
travel, must make progress (an out-and-back **jab** returns to its start and
is kept - it may be a target the operator drew), and the straight cut must
stay in the legal area. The console preview is rebuilt from the final
segments so the map shows what is driven.

The plan for that mission went from **17 segments / 16 board stops to 6
segments / 4 stops** (one pivot through the switchback, the rest chained
ARCs).

### 9.4 Absolute re-anchors and the ARC wire window

Every ARC is driven **relative** (radius + sweep from the board's *current*
pose), so the per-arc completion tolerance and the wheel slip (never fed
back) accumulate along a chain. The real mission of 2026-10-07 finished
**~70 mm short of the plan's own end node** with the heading correct - and
the same run was rejected mid-flight: the board answered RANGE (BAD_PARAM
`aux 22 = ARC`, `aux 23 = QUEUE_ARC`) for arcs outside its window, the
console then lost its chain, and a 7 s plan crawled for 29 s (stop at every
junction). Three passes close that gap (`pipeline._reanchor_turns`,
`_cap_chain_length`, `_sanitize_arcs`):

* **`_reanchor_turns`** - a mid-path in-place TURN (a junction break the
  angles left over) becomes an **absolute GOTO at the junction point** with
  `th` = the next arc's entry tangent. One stop, exactly like the TURN it
  replaces, but the position is taken from the plan again. The opening
  alignment and explicit rotations stay TURNs.
* **`_cap_chain_length`** - past `CHAIN_REANCHOR_MM` (900 mm) of continuous
  relative chaining the plan inserts one absolute GOTO at the junction it is
  about to pass. Longer runs are re-anchored in ~900 mm slices.
* **`_sanitize_arcs`** - the board drives radius + sweep from its *own*
  pose, so an ARC's declared exit is a promise the wire has to keep. Every
  ARC is re-derived so the plan IS the geometry the board drives:

  * it is **aimed at its declared exit**: a circle through two points that is
    tangent to the entry heading is unique and turns by twice the bearing
    error, so the sweep becomes that turn (quantised to the `0.1 deg` wire)
    and the radius follows from the chord, whole mm
    (`R = chord / (2 * sin(sweep/2))`). The board then ends exactly where the
    plan says - which is what removes the residual it used to leave behind
    (**20.4 mm over 26 arcs, worst 7.5 mm**, on the loaded waypoint drawing
    2026-10-09) that the next absolute GOTO had to creep away: a tiny
    movement plus a touch-up turn at the junction;
  * when the wire cannot hold the aimed curvature - a tight junction
    connector, a turn under the 50 mm radius / 2 mm length floors, or a
    fitted radius below the floor (a hand drawn corner, which stays a GOTO)
    - the **fitted curvature is kept**, the sweep rides the arc-length floor,
    and the exit is **re-declared at the point the board really reaches**;
  * the exit, the **next segment's start** and the **heading labels** follow
    from that arithmetic, and each segment's own samples are rebuilt as the
    arc the board drives (start and end exact). The heading is propagated
    only where the fit already wanted a tangent junction
    (`ARC_CHAIN_TANGENT_DEG`): a drawing corner keeps its own entry heading,
    or the aim would turn the next span into a hairpin. A **GOTO node within
    5 mm of the arc before it** (`ARC_NODE_SNAP_MM`) is that same junction -
    the node comes from the reference path, the arc from the fit - so it is
    moved onto the arc's true exit: one stop with nothing left to drive,
    never a stop, a sub-centimetre creep and a turn;
  * a **near-straight** ARC (|R| > 20000 mm) keeps its radius on its
    **chord** - never on the sampled polyline, which overshot by millimetres
    whenever a span ended on a curve - because the sweep travels in 0.1 deg
    units (54 mm at R = 31011 mm) and the radius has to absorb that rounding;
    without it a trimmed or re-aimed straight ended up to ~27 mm from the
    exit it declares, which is also what used to make a junction connector
    unable to trim such an ARC at all;
  * the window holds: radius 50..32767 mm, sweep < 360 deg, arc length
    2..2900 mm (`motion.c` rejects under 1 mm), sweep sign = radius sign. An
    ARC that cannot be represented becomes a GOTO to its own exit (one stop,
    but drivable and in the plan's frame); two coincident re-anchor GOTOs
    merge into one.

The plan printed in the console is therefore the geometry the board drives:
same radii, same sweeps, same junctions. The executor adds the matching
robustness on its side (a transient queue/telemetry failure no longer kills
chaining or the run - see the executor notes) and logs, after every run,
*"run finished: N segment(s), M blended without stopping"* plus the closing
error against the plan's last node, so accuracy is measurable.

### 9.5 Junction rounding - corner connectors (`pipeline._smooth_arc_junctions`)

The fitter cannot make every junction tangent-continuous: two arcs fitted to
one drawn curve meet with a small **tangent break**, and the board drives an
ARC from the CURRENT heading, so the break has to be absorbed. Emitting the
in-place TURN that the emission inserts here costs a **full stop plus a
pivot** at the junction (`_reanchor_turns` turns it into an absolute GOTO,
which is the same stop), which is exactly the "dance" between arcs. The
rounding pass replaces that turn with a **connector ARC** tangent to both
neighbours; the chain then drives through the junction under `QUEUE_ARC`
without stopping. Three constructions are tried, most faithful first:

* **relocated junction** - a tight-to-wide transition of a drawn curve comes
  out of the fit as two arcs whose *circles* are (nearly) tangent or slightly
  crossing, with the curvature step sitting exactly on the junction; there no
  connector tangent to both can exist (it would have to touch the second
  circle BEHIND the junction). Where the two circles actually meet they are
  within millimetres of each other, so the junction **slides along both
  circles onto that meeting point** (up to `JUNCTION_RELOCATE_MAX_MM`): the
  last millimetres of the incoming arc are driven as part of the outgoing one,
  the downstream geometry does not move at all, and the joint becomes tangent
  (touching pair) or small enough for a connector (crossing pair);
* **anchored construction** - for a **near-straight** neighbour (|R| >
  20000 mm) the connector touches it at its own end point or a whole
  `0.1 deg` wire quantum away, and the OTHER neighbour carries the trim. The
  radius comes from a closed form (`|centre - o| = r +- rho` with the centre
  on the touch point's normal - the same equation family the launch weave
  solves);
* **tangent search** - a circle tangent to BOTH neighbour circles (both may
  be trimmed a little), scanned over the radius window
  `ARC_CONNECT_MIN_RADIUS_MM` .. `ARC_CONNECT_MAX_RADIUS_MM`. It is also the
  fallback when the anchored construction has no solution (e.g. a straight
  meeting a *tight* curve): it may then trim the near-straight ARC by up to
  `ARC_CONNECT_FAR_TRIM_MM`, which is representable because `_sanitize_arcs`
  re-derives a near-straight ARC's radius from its chord - however the
  straight is shortened it still lands exactly on the exit it declares.

All three are held to the same limits: the trims leave the neighbours
drivable (`ARC_CONNECT_MIN_SWEEP_DEG` kept, `ARC_CONNECT_TRIM_MAX_DEG` and
`ARC_CONNECT_FAR_TRIM_MM` caps), the connector fills the break within the
firmware window, it stays inside the legal area, and it may not cut the drawn
corner by more than `ARC_CONNECT_MAX_DEV_MM` (24 mm). **The cut is measured
against the neighbour POLYLINES, never against their sample vertices**: a
fitted ARC keeps the sampler's node spacing (up to ~250 mm on a large radius),
so distance-to-vertex over-stated a local cut by up to half a chord and
rejected good connectors (measured 36.8 mm where the true cut was 1.6 mm -
loaded strategy, 2026-10-09).

A junction keeps its stop-and-pivot when none of the three constructions
applies, or the break is too big (`> ARC_CONNECT_MAX_DEG = 55 deg`). Those are
tight manoeuvres anyway (the wheels must scrub).

## 10. Dynamic re-planning (mid-run)

Triggered by a console obstacle edit (`POST /api/plan` while executing) or a
camera report (`/ws/vision` -> coalesced watcher, one re-plan / 0.25 s).

### 10.1 Is the remaining route affected? And how urgently?

```
remaining = state.last_plan.path cut at the robot position
candidate = plan(remaining route (or the sliced drawing if the mission was
                 edited), new obstacle map, from the robot pose)
affected  = |len(candidate) - len(remaining)| > max(100 mm, 2 % of len(remaining))
```

Because the candidate is built from the route the robot is *actually* on (not
by reattaching to the drawing), an obstacle behind the robot, far from what it
still has to drive, or one that only brushes the soft margin => **no stop, no
trajectory change** (log: *"obstacle change does not affect the remaining
route - trajectory unchanged"*) - also while the robot is mid-detour. If the
candidate cannot be planned at all, the route is checked for a physical
(body-clearance) blockage - conservative, so a truly blocked route still
stops. A longer candidate = a detour is needed; a shorter one = the route can
be optimised again.

A candidate of the **same length** can still be affected: when no detour
exists and the planner keeps the drawn line, the route would drive *through*
the obstacle. The candidate path is therefore also tested for clipping the
new obstacles at the **physical** clearance - sampled geometrically along the
route (every 25 mm) against `half-diagonal + 2 mm`, deliberately **not** a
grid line-of-sight test: 50 mm cells re-introduce ~25 mm of quantisation and
made the check hair-triggered (a legal pass at ~215 mm from a robot was
flagged because the neighbouring cell centre sat inside the test inflation).
A detour that hugs an obstacle is not a clip; driving through it is. A
clipped candidate is *affected* and goes to section 10.3.

**Reaction ladder (distance, not just "blocked").** Mid-run decisions only
consider obstacles **near the robot** (within `half-diagonal + REACT_GAP_MM`)
and then react by **how far along the route the first physical clip is**
(`clip_gap_mm`, pure geometry on the already-planned path - no re-plan for the
cheap half of the decision):

| clip distance | reaction |
|---|---|
| beyond `CAUTION_GAP_MM` (700 mm) | **watch only** - another robot usually moves on; the watchdog re-checks every 0.5 s as we advance |
| inside `CAUTION_GAP_MM` | **creep**: every motion command is scaled to `CAUTION_SCALE` (45 %) - the plan and its speeds are untouched, `state.exec.speed_scale` shows it |
| inside `STOP_GAP_MM` (250 mm) | **stop and resolve**: wait <= `WAIT_MAX_S` for a moving obstacle, detour when there is nowhere to wait |
| any distance, but the blockage point is **tight** (a border or a static obstacle closer than `half-diagonal + TIGHT_ROOM_MM`) | act at once: re-planning from there would have no room left, so the extra room is bought while it still exists |

A robot parked on the route 1 m ahead therefore changes nothing at all; the
`proximity_loop` (0.5 s, quiet) escalates by itself as the distance closes,
so no new camera message is needed. (Planning itself - drawing a mission,
pressing Plan - still uses the **full** obstacle map: the preview always
shows the route around everything known.)

### 10.2 The swap

1. Stop the robot (`STOP`, reason *"re-planning the remaining route (dynamic
   obstacle)"*), wait for the board to report IDLE, read a fresh pose.
2. Build the remainder to plan:
   * **same mission** (the normal obstacle-change case): the not-yet-executed
     program (`executor.remaining_elements()`) - the robot continues on the
     **route it is actually driving** (arcs and previous detours included),
     re-routed only where the new map blocks it. It is never sent back to the
     drawing while mid-detour, and a half-driven arc contributes only its
     exit point (its start lies behind the robot), so repeated obstacle
     changes cannot ping-pong the robot backwards. The re-fit of the smooth
     route is stable (measured drift <= ~20 mm per generation).
   * **edited mission**: `slice_remaining_elements` trims the new drawing at
     the point nearest to the robot (progress prior from the running plan's
     path for self-crossings), keeps everything after it verbatim and plans
     that - a mid-run edit is honoured from the robot's position.
3. Plan the remainder on the merged obstacle map (console + camera).
4. Restart the executor on the new segments - instantly, or **paused** when
   the run was paused.

Planning is serialised by `state.plan_lock`, so concurrent requests (console
+ camera burst) can never interleave a stop/plan/start cycle.

### 10.3 Waiting instead of detouring (moving obstacles)

A camera obstacle (another robot) is not a box: it moves, so it costs nothing
to let it pass. When the affected route would need a detour, the service
chooses between **driving around** and **waiting** - the robot only re-routes
when the candidate actually avoids the obstacle AND the detour is reasonable:

```
wait  <=>  candidate clips the obstacle at the physical clearance   (no way around)
     or  len(candidate) > max(1.6 x len(remaining), len(remaining) + 500 mm)
```

When it waits: the run stops (`STOP`, reason *"waiting for the moving obstacle
to clear"*), the log says so, `GET /api/state` exposes `waiting`, and the
console pill shows **EXEC waiting**. The route captured at the stop point is
kept in `state.vision_waiting`; every following camera change (and TTL expiry)
triggers a **resume attempt**: re-plan the captured route on the current map -
when it is free again (or the detour finally reasonable) the run restarts from
the stop position on its own, still **paused** when the operator had paused
it. A manual `Plan`, `Execute >`, `Stop`, `E-STOP`, or a manual pose
(`/api/pose`, `/api/odometry/reset`) clears the waiting state (the operator
took over). Both the wait decision and the resume re-plan also
respect the reaction ladder: only obstacles near the robot count, so a far
robot on the captured route neither blocks the wait nor the resume.

**A stop is never forever - and never a lie (2026-10-09).** Three guards:

* the planner runs **off the event loop** (`asyncio.to_thread`): a 1-2 s
  re-plan on the Pi no longer starves telemetry polling, which is what let the
  board's deadman lapse while the wheels were still winding down - the latch
  that made every later restart answer `robot status FAULT`;
* every (re)start is **verified**: `executor.start` returns as soon as the
  runner task exists, so the caller now waits `REPLAN_VERIFY_S` and checks the
  state - an immediate abort is reported (and the wait state kept) instead of
  logging "resuming the mission" while nothing drives the robot;
* a **DEADMAN latch left by the previous stop is cleared** (and logged) when a
  run starts; a latched deadman is the watchdog's "the link went quiet while
  moving" bit and the operator pressing Execute/Resume is the human in the
  loop. Every other fault still aborts with its own message, and a deadman
  that lapses DURING a run still stops it.

`WAIT_RESUME_MAX_FAILS` resume attempts after a failed restart end the wait
with a clear error (*"the mission stays stopped: the resumed run keeps
aborting (...) - fix the board, then press Execute"*) instead of sitting on
"waiting" with nothing driving the robot.

## 11. Warnings catalogue (honest reporting)

| Warning | Meaning |
|---|---|
| `leg N re-routed to stay clear (obstacle or field border)` | a blocked run was detoured at the full margin (the normal case) |
| `leg N squeezed through the obstacle margin - no room for the full margin there` | no corridor fits the full margin; half the margin was used |
| `leg N re-routed at body clearance (the drawn line is blocked)` | last resort: only the physical clearance fits (warned) |
| `leg N lies inside the robot clearance zone (field border or obstacle) - kept as drawn` | even the body would clip there - kept, warned |
| `leg N blocked but no route found - kept direct` | no Theta\* route exists |
| `N corner(s) kept sharp where rounding would leave the safety zone` | fillet veto everywhere |
| `waypoint (x, y) mm sits inside the robot clearance zone - moved to the closest allowed point (x, y) mm` | legacy (`waypoint_margin_mm` unset): the stop point was projected out of the full routing margin |
| `waypoint (x, y) mm sits inside the configured waypoint clearance (N mm) - moved to the closest allowed point (x, y) mm` | the configured stop clearance (half body side + margin) or the rotated body required a move |
| `waypoint (x, y) mm stands inside the routing margin - approached straight from a safe stand-off at (x, y) mm` | the close waypoint is reached by a straight final leg; with an arrival angle the rotation is finished at that stand-off |
| `waypoint (x, y) mm is too close to rotate in place and no safe straight final leg exists - moved to the closest turning-safe point (x, y) mm` | the requested final heading cannot be approached straight (e.g. parallel to a wall); safety wins and the angle is performed at a turn-safe point |
| `waypoint (x, y) mm is too close to rotate in place - reached exactly, then the robot backs up N mm to (x, y) mm before turning to continue` | normal close-waypoint departure: exact stop, reverse to a turn-safe point, rotate, continue ("moves forward" when the obstacle is behind) |
| `waypoint (x, y) mm sits inside the robot clearance zone - closest allowed approach (x, y) mm, N mm short` | fallback when the projection still is not reachable (a pocket): the run stops at the closest reachable cell |
| `the arrival angle at waypoint (x, y) mm could not be woven into the approach - the robot turns in place there` | `combine` was ticked but no legal arrival arc exists (corner/wall); the angle is still performed as an in-place turn |
| `waypoint (x, y) mm is not a stop node on the route - its arrival angle cannot be performed` | nothing lands on the waypoint (should not happen after the projection); reported instead of pretending |
| `goal (x, y) mm sits inside the robot clearance zone - moved to the closest allowed point (x, y) mm` | the goal is a stop point: projected out of the margin before routing |
| `the first path starts away from the robot - connector leg added from the robot position` | first stroke starts elsewhere |
| `obstacle change does not affect the remaining route - trajectory unchanged` | the no-op re-check (section 10.1) |
| `path start inside the robot clearance zone (field border or obstacle) - moved to the closest allowed point (x, y) mm` | the run's first point was blocked (fresh obstacle / start hugging the wall) |
| *log* `moving obstacle blocks the remaining route - waiting for it to clear (...)` | wait instead of a huge/blocked detour (section 10.3) |
| *log* `the route is clear again - resuming the mission (N segments)` | a waiting run restarted itself |

## 12. Performance notes

* Planning a typical mission (3 m, a handful of primitives) takes 50-300 ms,
  dominated by Theta\* on the 60x40 grid. Worst case per blocked run: three
  Theta\* calls (full / squeezed / body). A `combine` waypoint adds up to 14
  routed arrival candidates plus the cheap arc sampling - still a few
  hundred milliseconds, and the planner runs in a worker thread.
* The affected re-check adds one candidate plan (~same cost) per obstacle
  change; camera bursts are coalesced to 1 / 0.25 s.
* The plan (`segments` list) is exactly what the executor sends, one frame at
  a time - there is no on-board trajectory interpolation.
