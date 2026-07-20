# Codebase Audit — current state at `d691a3a` (v2, 2026-07-20)

Supersedes the 2026-07-20 v1 audit (written against `67b520f`). Basis: file-by-file source audit of v1, diff-verified fix commits `e7da777` (21 audited low-hanging bugs) and `d691a3a` (path densification + segmentation-collapse guard), and two observed sim runs on the fixed code. Every claim cites its file; runtime claims cite the sim run that produced them.

## Commit history this audit covers
- `67b520f` — baseline audited in v1 (verdicts: 8×🟡, 2×✅, decision **GO**).
- `e7da777` — all 21 v1 low-hanging fixes landed; each diff verified against spec, greps clean.
- `14c8449` — audit + test plan docs moved into `docs/`.
- `d691a3a` — coverage path densified to ≤0.5 m pose spacing (`coverage_server/utils.h`); mission aborts to ERROR if segmentation retains <50% of path poses (`mission_manager_node.cpp`). Both verified in source; both confirmed working in sim run 2.

## Sim evidence (2 runs, Webots, stock map)
**Run 1 (`e7da777`):** exposed that F2C swaths are 2-point lines (~9–13 m legs), so `splitPathIntoSegments`' 1.0 m jump threshold split *inside every swath* → 51 degenerate fragments dropped → 1 segment survived → MOWING lasted 2.6 ms and "completed" without mowing. Root cause: path sampling density, not the drop logic — the old merge behaviour had been silently reassembling the fragments (and re-introducing 10 m intra-segment jumps) all along.
**Run 2 (`d691a3a`):** 2 segments, 0 drop warnings, collapse guard armed but not needed. Robot genuinely swath-followed for 6 m 05 s at ~0.26 m/s (matching `desired_linear_vel`). **The real stall is now isolated:** 5× `controller_server: Failed to make progress` during MOWING (t+44/184/197/257/305 s), robot moving between retries, all failures in the same x≈−4 band, retries exhausted → clean ERROR ("follow_path repeatedly failed"). Caveat: two EKF "Failed to meet update rate" errors ≈t+164 s suggest sandbox CPU contention may contribute to timing-based failures.

## Verdict table (current)

| Module | v1 | Now | Owner focus |
|---|---|---|---|
| map_server | 🟡 | ✅ | one open design decision (exclusion buffering) |
| map_recorder | 🟡 | 🟡 | covariance stub, thread-safety |
| coverage_server | 🟡 | ✅ | turn-generation decision deferred until stall closed |
| docking_helper | 🟡 | ✅ | cosmetic race; E2E blocked on dock reconciliation |
| mission_manager | ✅ | ✅ | battery-scale decision; ERROR diagnostics |
| sim | 🟡 | 🟡 | dock triple-position mismatch (uncommitted local fix) |
| lib / msg / srv / action | 🟡 | ✅ | — |
| Localization | ✅ | ✅ | update-rate margin (seen failing in run 2) |
| Nav2 integration | 🟡 | 🟡 | **active workstream: mid-swath progress failures** |
| Launch wiring | 🟡 | 🟡 | timer sequencing; hardware launch deps; naming |

## Per-module: what was fixed, what remains
(“Fixed” items are diff-verified in `e7da777`/`d691a3a`; “Remaining” is the team worklist.)

### map_server — ✅
Fixed: `declare_parameter` re-declare crash on republish (params now declared once in ctor); grid-truncation warning added; transient-local datum publisher persists; buggy unused `polygonsIntersect`/`splitPolygonIntoParts`/`findLongestEdge` deleted (`polygonArea` kept).
Remaining:
- **Exclusion buffering (design decision, tied to stall hypothesis 2):** exclusions are stamped at true polygon size (`map_server_node.cpp` pass 3) and no costmap layer inflates them, so a path within `robot_radius` 0.48 m of an exclusion puts the footprint on lethal cells. Not exercised by the committed map (no exclusion features) but will bite on first recorded exclusion. Options: dilate lethal outward around exclusions in map_server (mirror of the existing free-region dilation), or buffer exclusions in coverage_server, or add a Nav2 inflation layer. Pick one, record it.
- Minor: `PolygonGridIterator::next()` pre-increments, so column `min_x_` is never sampled (masked by the edge-tolerance check at 0.1 m resolution); raw `new` for `map_io_`/`gaussian_filter_`; Gaussian-blur params use inconsistent prefixes (`grid.use_gaussian_blur` vs `map.gaussian_blur.kernel`).

### map_recorder — 🟡
Fixed: area recording no longer permanently blocked after a dock recording (`is_active()` check); `isValidPolygon` made inline.
Remaining:
- `checkPositionCovariance()` returns `true` with a TODO while the abort message still promises a 2 cm covariance check (`map_recorder_node.cpp:635` area) — implement or delete the claim.
- Detached recording thread reads `current_boundary_points_` while `handleAddBoundaryPoint` writes it — unsynchronised; add a mutex or route through the executor.
- `isValidPolygon` still validates nothing beyond point count despite its comment.

### coverage_server — ✅
Fixed: pose yaw = direction of travel (was bearing-from-map-origin via `getAngleFromPoint()`); path densified to ≤0.5 m spacing (run 2: 2 segments, 0 drops); `coverage/path` topic now actually published; dead `PolygonCoverage.srv` include removed.
Remaining:
- **No turn/connection generation** — the path is still densified straight swaths with no planned turnarounds; RPP improvises every 180° reversal. Deliberately deferred: revisit only if the stall battery (S3–S5) convicts the turnaround geometry; the fix would be F2C's route/path planners or per-swath goals.
- Headland rings are appended after mainland (perimeter mowed last, one big inter-block jump) — works, undocumented; document or reorder.
- `findAreasInPolygon` still returns all operation areas with a TODO intersection test — unused trap; delete or implement.

### docking_helper — ✅
Fixed: hang-on-rejection (shared atomics, post-loop abort); cancel honoured end-to-end (Nav2 goal cancelled, `finished` guard against late results); TF exception in `dockPose` no longer terminates the process (nullptr + graceful abort).
Remaining:
- `nav2_goal_handle_holder` shared_ptr written from executor thread, read from wait loop, non-atomic — same benign-on-x86 class as the `current_status`/`current_retries` counters; fold into next touch.
- End-to-end docking is untestable until the dock-position reconciliation (see sim) — run 2 confirmed the goal targets the GeoJSON dock (staging ≈(5.37, 2.21)) while the physical dock is at (8, 6).

### mission_manager — ✅
Fixed: battery-low now also handled during NAVIGATING_TO_START (cancels navigate, docks); segment-merge can no longer re-introduce >1 m intra-segment jumps (drops fragment, warns); **new collapse guard: ERROR if segmentation retains <50% of received poses** — run 1's "completed without mowing" failure mode is now structurally impossible; battery-scale WARN_ONCE added.
Remaining:
- Battery percentage contract (0–100 here vs 0–1 `sensor_msgs` convention) — decision pending; sim publishes 0–100 so consistent today, hardware drivers may not be.
- ERROR state carries only a log string — no diagnostics topic; consider publishing last-failure context for the dashboard/tests.
- `follow_path` retry resends the whole segment (recovery via RPP closest-pose pruning) — worked as designed in run 2; acceptable, revisit only if stall fix changes segment structure.

### sim — 🟡
Unchanged since v1, still the blocker for docking E2E:
- **Dock triple-position mismatch:** Webots physical dock (8, 6) (`openmower.wbt`); GeoJSON dock ≈(4.92, 2.23); sim charger-contact params (1.82, 1.5) (`sim.launch.py`). The 2026-07-07 Notion entry says dock+map+exclusion were reconciled locally — that fix was never committed. Recover it from the machine it was made on, or redo: move GeoJSON dock + contact params to match (8, 6) and add the dock-footprint exclusion. Until then RETURNING_TO_DOCK can navigate but CHARGING is unreachable.
- Physical dock sits inside the operation polygon with a collision body while costmaps are static-layer-only — headland swaths pass through x 7.6–8.4, y 5.8–6.2.

### lib / msg / srv / action — ✅
Fixed: dead `PolygonCoverage.srv` removed from rosidl + disk; `Area.msg` comment corrected; empty `src/lib/*` gitlinks removed.
Remaining: none. Note for hardware phase: `gps.launch.py`/`micro_ros_agent.launch.py` now depend on externally installed `ublox`/`ntrip`/`micro_ros_agent`/`vesc` packages (they were never buildable in-repo).

### Localization — ✅
No code changes (none needed). Remaining verification items:
- **Run 2 observed `ekf_se_map`/`ekf_se_odom` "Failed to meet update rate" errors** (~t+164 s) — likely CPU contention (Webots+Nav2+dual EKF), but it directly perturbs the pose stream the progress checker consumes. Measure real-time factor + EKF/controller loop rates before attributing timing-based failures to planning (test S0).
- `ekf_se_odom` `sensor_timeout: 0.025` demands ≥40 Hz wheel odom — measure margin.
- Confirm Webots IMU publishes orientation on `imu/data_raw` (EKFs fuse roll/pitch from it).

### Nav2 integration — 🟡 (the active workstream)
Fixed: `desired_linear_vel` 0.26 aligned with velocity_smoother (confirmed at speed in run 2).
Remaining — this is where the stall lives:
- **Mid-swath progress failures (run 2):** 5× `SimpleProgressChecker` aborts spread over ~5 min, same x≈−4 band, robot moving between retries. Prime suspects, in order: (1) RPP turnaround behaviour at swath ends — carrot aliasing across 0.3 m-spaced parallel swaths (`min_lookahead_dist` 0.3, `max_robot_pose_search_dist` 10.0 spans ~30 swaths); (2) progress-checker envelope (0.5 m / 10 s) vs. time actually spent in rotate-to-heading + chatter at hairpins; (3) sim CPU contention starving control loops (EKF rate errors are the tell). Run the stall battery S0→S8 in the test plan; do not tune blind.
- **No obstacle source anywhere:** both costmaps are static-layer-only — nothing dynamic (dock, chair, person) can ever appear; physical collisions manifest as progress failures. Needs a decision: sensor-based obstacle layer, keepout filters, or accept-and-document for sim.
- No inflation layer + unbuffered exclusions (see map_server decision).
- Minor: NavFn `tolerance: 0.5` silently accepts goals 0.5 m off; `docking_server.controller.use_collision_detection: false` with an in-code "doesn't work well" comment.

### Launch wiring — 🟡
Fixed: env-var guard with actionable message (`localization.launch.py`); dead files removed (`maps/world.yaml`, placeholder `scripts/drive_coverage_path.py`, unused `mow_bt.xml` + behaviortree dep); orphaned map_server pytest registered.
Remaining:
- **Registered `load_geojson_map.test.py` will fail as-is:** it launches `map_server_node` without the required `datum` param (ctor throws) and is a `launch_testing` test registered via `ament_add_pytest_test`. Fix: add `'datum': [-22.9, -43.2]` and register via `add_launch_test`.
- Timer-based startup sequencing (3/5/10 s) instead of readiness events — tolerated by retry loops; races on slow machines.
- Duplicate `find_package(ament_cmake_pytest)`; legacy `open_mower_next` naming throughout (rename is a coordinated, separate change).

## Standing open decisions (need a human owner each)
1. Dock reconciliation (sim module) — blocks docking E2E and SPINE-5/6.
2. Exclusion buffering approach (map_server vs coverage vs inflation layer).
3. Battery percentage contract (0–100 vs 0–1) before any hardware driver lands.
4. Obstacle sensing strategy (none exists; decide sim-acceptable vs required).
5. Turn generation in coverage_server — decide after stall battery verdict.
