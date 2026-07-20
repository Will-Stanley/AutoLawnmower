# Phase 1 — Correctness & Structural Audit (AutoLawnmower)

Audited from source at `main` (cloned 2026-07-20). Every finding cites the file it came from; items I could not verify statically are marked as such.

## Verdict summary

| Module | Verdict |
|---|---|
| map_server | 🟡 MINOR ERRORS |
| map_recorder | 🟡 MINOR ERRORS |
| coverage_server | 🟡 MINOR ERRORS |
| docking_helper | 🟡 MINOR ERRORS |
| mission_manager | ✅ SOUND |
| sim | 🟡 MINOR ERRORS |
| lib / msg / srv / action | 🟡 MINOR ERRORS (interfaces sound; dead items) |
| Localization (dual EKF + navsat) | ✅ SOUND |
| Nav2 integration (config) | 🟡 MINOR ERRORS |
| Launch / integration wiring | 🟡 MINOR ERRORS |

**Overall: GO** — no module needs a rewrite. Details below.

---

## map_server — 🟡 MINOR ERRORS
Based on: `src/map_server/map_server_node.cpp`, `geo_json_map.cpp`, `polygon_iterator.hpp`, `polygon_utils.hpp`, `some_gaussian_filter.hpp`.

Architecture is right: load GeoJSON → lat/lon→map via GeographicLib LocalCartesian (same datum params as navsat, consistent) → occupancy grid (default-lethal, paint operation/navigation free, dilate free region by `robot_radius` 0.48, stamp exclusions last). The paint-order + dilation design (`mapToOccupancyGrid`, lines 279–392) is coherent and deliberately coordinated with Nav2's `robot_radius` (comment at line 35, matches `nav2_params.yaml`).

Bugs:
1. **Re-publish after save throws.** `mapToOccupancyGrid()` calls `declare_parameter("grid.resolution")` and `declare_parameter("grid.max_size")` on every invocation (lines 322, 325). Second call (any `saveArea`/`saveDockingStation` → `saveAndPublishMap` → `publishMap`) throws `ParameterAlreadyDeclaredException`. It's swallowed by the service handler's try/catch (`saveAreaHandler`, line 154), which then reports failure even though the file *was* saved, and the updated map/grid is never republished. Record-then-mow without a node restart is broken. One-line fix (declare in constructor).
2. **Exclusions not buffered.** Pass 3 stamps exclusion polygons at true size (line 375) after dilation, but nothing dilates *lethal* back around exclusions, and Nav2 runs no inflation layer (see Nav2 section). A path passing < 0.48 m from an exclusion puts the robot's footprint on lethal cells. Not exercised by the current sim map (no exclusions) but will bite the moment one is recorded.
3. **Datum publisher is dead on arrival.** `publishDatum()` creates a transient-local publisher, publishes once, then `reset()`s it (`geo_json_map.cpp` lines 340–343) — destroying the durability cache, so no late subscriber can ever receive `map/datum`.
4. **Dead + buggy utility code.** `polygonsIntersect` has a typo — `u` numerator uses `(p1.y - p1.y)` (= 0) instead of `(p1.y - q1.y)` (`polygon_utils.hpp` line 47). It, `findLongestEdge`, and `splitPolygonIntoParts` are referenced nowhere outside the header (grep-verified). Delete or fix before anyone trusts them.
5. Minor: `grid.max_size` clamps width/height silently — an area larger than 200 m in either axis gets truncated with no warning; raw `new` for `map_io_`/`gaussian_filter_` (leak, harmless); `PolygonGridIterator::next()` pre-increments so column `min_x_` is never sampled (edge-tolerance check masks it at 0.1 m resolution).
6. `src/map_server/test/load_geojson_map.test.py` exists but is **not registered in CMakeLists.txt** — a dead test.

## map_recorder — 🟡 MINOR ERRORS
Based on: `src/map_recorder/map_recorder_node.cpp/.hpp`, `utils.hpp`.

Action-server flows (drive-until-charging for docks; auto/manual boundary point capture) are logically sound and hand off correctly to map_server's `save_*` services. Frames used (`mower`, `charging_port`) exist in `description/robot_core.xacro` (lines 106, 186).

Bugs:
1. **Area recording permanently blocked after any dock recording.** `docking_goal_handle_` is set in `handleDockingAccepted` (line 90) and never cleared. `handleAreaBoundaryGoal` rejects when `docking_goal_handle_` is truthy (line 288) — it checks the pointer, not `is_active()`. After one dock recording (success or failure), all boundary recordings are rejected until restart.
2. **Covariance check is a stub.** `checkPositionCovariance()` returns `true` with a TODO (line 635), yet the action can abort with "Position covariance exceeds threshold (2cm)" — the message promises a check the code doesn't do. Contradicts its own docs.
3. Shared state (`is_recording_area_`, `auto_recording_mode_`, `current_boundary_points_`) is mutated from detached `std::thread`s and service callbacks with no mutex (e.g. lines 306–330 vs 544–601). Benign under the default single-threaded executor for callbacks, but the detached recording thread reads `current_boundary_points_` while `handleAddBoundaryPoint` writes it — a real, if low-probability, race.
4. `utils.hpp` `isValidPolygon` is a non-inline function defined in a header (ODR hazard if ever included twice) and validates nothing beyond point count despite its comment.

## coverage_server — 🟡 MINOR ERRORS
Based on: `src/coverage_server/coverage_server_node.cpp`, `utils.h`.

F2C usage (Cells from polygon + holes, `ConstHL` headlands, `BruteForce` swaths + `BoustrophedonOrder`, headland ring swaths appended) is correct as geometry generation. Service contract (`AreaCoverage.srv`) is clean. Subscription to `/mowing_map` matches the launch remap (`localization.launch.py` line 66).

Bugs / contract gaps — these matter for the stall (see Stall section):
1. **Wrong pose orientations in the output path.** `utils::toMsg(swaths, …)` (utils.h line 184) sets each pose's yaw from `point.getAngleFromPoint()`, which in Fields2Cover is the angle of the vector *from the origin to the point* — not the direction of travel. Every intermediate pose orientation in `/area_coverage`'s path is garbage. mission_manager patches only segment endpoints (`fixSegmentEndpointOrientations`). RPP mostly ignores path orientations, so this is latent — but it poisons any future controller/goal-checker that trusts them.
2. **No connection/turn generation.** The path is a raw concatenation of swath endpoints — no headland-following transitions between swaths and no turns; `min_turning_radius` is stored in the F2C `Robot` (line 109) but never used (no `f2c::pp` path planner is invoked). The downstream contract is implicitly "Nav2 RPP will improvise every 180° turnaround." That works only inside a narrow envelope of RPP tuning (see Nav2 section). Fixable in place (use F2C's route/path planning, or emit per-swath goals), not a rewrite.
3. **Mainland/headland ordering:** headland ring swaths are appended *after* mainland (lines 250–267), so the perimeter is mowed last and the mainland→headland transition is a large jump that mission_manager must split. Intentional-looking but undocumented.
4. `findAreasInPolygon` returns all operation areas with a TODO for the actual intersection test (line 157) — currently unused by anything that matters, but it's a trap.
5. Note: `handleAreaCoverageRequest` publishes visualization markers but the returned `path` is the only navigation product; `coverage/path` publisher (line 34) is created and **never published to** — dead topic.

## docking_helper — 🟡 MINOR ERRORS
Based on: `src/docking_helper/docking_helper_node.cpp/.hpp`, `charger_presence_charging_dock.cpp`, `plugins.xml`.

Structure is right: thin orchestration over Nav2's `/dock_robot`, dock DB from `/mowing_map`, staging pose math (`dockPose`, lines 135–175: 180° flip + charging-port offset) is geometrically coherent with the `staging_x_offset: -1.0` in `nav2_params.yaml`. The `ChargerPresenceChargingDock` plugin is a correct minimal `opennav_docking_core::ChargingDock` (detection = `/power/charger_present`).

Bugs:
1. **Infinite hang if Nav2 rejects the goal.** `executeDockingAction` spins `while (docking_active …)` (line 314); `docking_active` is only cleared in the result callback (line 284). A rejected goal never produces a result → the detached thread loops forever and the mission_manager's dock action never completes (its own retry logic can't fire because the action neither succeeds nor aborts).
2. **Cancel is accepted but ignored.** Both cancel handlers return `ACCEPT` yet `executeDockingAction` never checks `goal_handle->is_canceling()` and never cancels the underlying Nav2 goal.
3. **Uncaught TF throw in a detached thread → process death.** `dockPose()` calls `lookupTransform` outside any try/catch (line 149) and is invoked at line 242 inside the detached thread; a TF exception there calls `std::terminate`.
4. `docking_active` is a stack `std::atomic` captured by reference in the result callback (line 278) — safe only because the loop outlives the single callback; fragile pattern worth fixing while in there.

## mission_manager — ✅ SOUND
Based on: `src/mission_manager/mission_manager_node.cpp/.hpp`.

The FSM matches the spec (`IDLE → REQUESTING_COVERAGE → NAVIGATING_TO_START → MOWING → RETURNING_TO_DOCK → CHARGING`, plus `ERROR`), transitions are mutex-guarded, every async result checks it's still in the expected state before acting (stale-result guards at lines 381, 449), retries are bounded (`kMaxActionRetries = 5`), and battery-low preemption cancels follow_path before docking (lines 111–119). Segment handling (`splitPathIntoSegments` at >1 m jumps, degenerate-segment merging, reversal-spike removal, endpoint orientation fix) shows the path contract was thought about. This is safe to build tests on.

Gaps (small, listed for completeness — none warrant 🟡 on their own):
- Battery-low is only checked in `MOWING` (line 111); a low battery during `NAVIGATING_TO_START` or a long `REQUESTING_COVERAGE` retry loop is ignored.
- `follow_path` retry resends the **whole segment** from its start (`sendFollowPathGoal(pending_path_)`, line 484); recovery relies on RPP's closest-pose pruning. Works, but means a geometric failure repeats identically 5× then hard-ERRORs — exactly the observed stall signature.
- Degenerate-segment merge can re-introduce a >1 m jump *inside* a merged segment (`splitPathIntoSegments` lines 236–247 prepends a far fragment onto the next segment) — contradicting the function's own `kMaxSegmentJump` invariant.
- Battery contract: compares `msg->percentage` against `30.0`/`95.0` — assumes 0–100. The sim publishes 0–100 so it's internally consistent, but `sensor_msgs/BatteryState` convention is 0–1; a standards-compliant hardware driver would trigger instant "battery low". Contract to pin down in Phase 2.
- `ERROR` is terminal except via the `mission/start` service; nothing publishes diagnostics on why.

## sim — 🟡 MINOR ERRORS
Based on: `src/sim/sim_node.cpp/.hpp`, `worlds/openmower.wbt`, `protos/DockingStation.proto`.

Battery/charger simulation logic is fine (linear V/s model, percentage from voltage window; charge detection = charging_port TF within a tolerance box of a configured contact pose).

Bugs / config contradictions:
1. **Three different dock locations in the sim setup.** Physical dock in Webots: (8, 6) (`openmower.wbt` line 77; world frame ≡ map frame since `gpsReference -22.9 -43.2` matches `OM_DATUM_*`). Dock recorded in the map GeoJSON: (4.92, 2.23). sim_node's charger contact pose: (1.82, 1.5) (`sim.launch.py` lines 139–141). As wired, Nav2 will dock at the GeoJSON pose, the charger will "detect" at a third location, and the physical dock is somewhere else entirely — end-to-end docking in sim **cannot currently succeed**. (The Notion "3.Navigation Software" page already flags dock/costmap handling as an open decision — this is the concrete state of it.)
2. **The physical dock sits inside the operation area** (bbox x[−5.1, 8.2], y[−3.3, 7.8]) with a bounding object (`DockingStation.proto` line 85), and the costmaps have no layer that can see it (static-layer-only, no sensor input, no exclusion in the map). Any path through x≈7.6–8.4, y≈5.8–6.2 — the 3-loop headland rings pass exactly there — ends in an invisible physical collision.
3. Percentage published 0–100 (line 180) vs BatteryState's 0–1 convention — see mission_manager note.

## lib / msg / srv / action — 🟡 (interfaces sound; dead items)
`msg`/`srv`/`action` definitions are clean, self-documenting, and match their users (verified against all call sites above). Issues:
- `PolygonCoverage.srv` is compiled (CMakeLists rosidl list) but **no node implements or calls it** — dead interface.
- `src/lib/*` (fields2cover, micro_ros_agent, ntrip_client, ublox_f9p, vesc) are **empty directories** — no submodules (`.gitmodules` absent), no content. Either restore them as submodules or delete; right now they're misleading.
- `behaviortree_cpp` is a `find_package`/`package.xml` dependency but no source uses it; `config/mow_bt.xml` is loaded by nothing (bt_navigator uses its default BT). Both dead.
- `Area.msg` comment says "0 = obstacle" while the constant is `TYPE_EXCLUSION` — trivial doc drift.

## Localization — ✅ SOUND
Based on: `config/robot_localization.yaml`, `launch/localization.launch.py`, `resource/openmower_webots.urdf`.

Textbook dual-EKF: `ekf_se_odom` (wheel vx/vy/vyaw + IMU attitude/rates/accel, world=odom) → odom→base_link; `ekf_se_map` (same + GPS x/y from `odometry/gps`, world=map) → map→odom; navsat with `use_local_cartesian: true` and the **same datum** as map_server's GeoJSON projection (both fed from `OM_DATUM_*`) — so the grid and GPS live in one consistent map frame. Topic wiring verified end-to-end: Webots publishes `/gps/fix` and `/imu/data_raw` (`openmower_webots.urdf` lines 9, 18); both EKFs remap `imu/data` → `imu/data_raw`; navsat gets `odometry/filtered` → `odometry/filtered/map`.

Notes (not defects, just unverified-statically):
- `ekf_se_odom` `sensor_timeout: 0.025` is aggressive; if wheel odom drops below 40 Hz the filter free-runs. Worth a measurement, not a rewrite.
- EKFs consume `imu/data_raw` with roll/pitch fusion enabled — this assumes the Webots IMU publishes orientation on the *raw* topic (webots_ros2 does when an InertialUnit is bundled). Confirm once in sim; on hardware a raw-only IMU would need a filter (madgwick) in front.
- `ekf_se_map` process noise x/y = 1.0 makes map pose GPS-dominated; fine with clean sim GPS, revisit with RTK dropout on hardware.

## Nav2 integration — 🟡 MINOR ERRORS
Based on: `config/nav2_params.yaml`, `launch/nav2.launch.py`.

The composed bringup (container + lifecycle manager, cmd_vel chain `controller/behaviors/docking → cmd_vel_raw → velocity_smoother → cmd_vel_nav → twist_mux → diff_drive`) is coherent and verified against `twist_mux.yaml`. Costmap design (static-layer-only on `/map_grid`, with map_server pre-dilating free space instead of running an inflation layer) is unconventional but internally consistent *for the empty-lawn case*.

Issues:
1. **No obstacle source anywhere.** Both costmaps have only `static_layer`. Nothing — not the dock, not a garden chair — can ever appear as an obstacle. Combined with sim finding #2 this guarantees physical collisions are invisible to planning and manifest exactly as "Failed to make progress".
2. **No inflation layer + unbuffered exclusions** (map_server bug #2): RPP's `use_collision_detection: true` checks footprint cost against the costmap; near exclusion polygons the first lethal cell it can see is the exclusion itself, at which point the footprint is already overlapping.
3. **Speed contract mismatch:** RPP `desired_linear_vel: 0.5` vs velocity_smoother `max_velocity: [0.26, …]` — every RPP command is silently halved downstream (open-loop feedback). Regulated velocity scaling and time-to-collision estimates inside RPP are computed at speeds the robot never does.
4. **RPP vs. raw-swath path envelope** (the stall driver): swath spacing in sim is 0.3 m (`operation_width: 0.3`, `sim.launch.py` line 158) while RPP `min_lookahead_dist: 0.3` and `max_robot_pose_search_dist: 10.0`. At a 180° swath turnaround the closest-pose search and the lookahead carrot can land on the *adjacent, opposite-direction* swath (0.3 m away — within localization noise), producing rotate-in-place chatter and no net translation → `SimpleProgressChecker` (0.5 m / 10 s) aborts. This is a tuning/contract problem, not broken logic.
5. Minor: `local_costmap` 3×3 m is small relative to `max_robot_pose_search_dist` 10 m; NavFn `tolerance: 0.5` can silently accept a goal half a meter from a segment start; `docking_server.controller.use_collision_detection: false` with an in-code comment admitting "it doesn't work well now".

## Launch / integration wiring — 🟡 MINOR ERRORS
Based on: `launch/sim.launch.py`, `openmower.launch.py`, `localization.launch.py`, `nav2.launch.py`, CMakeLists, `.devcontainer/*`.

What actually starts in sim: Webots + driver + ros2_control spawners, robot_state_publisher, twist_mux, sim_node, map_server (delayed 3 s) + map_recorder + docking_helper + dual EKF + navsat, full Nav2 stack, coverage_server (delayed 5 s), mission_manager (delayed 10 s, autostart, hardcoded `target_area_id` matching the devcontainer map). The topology is coherent; remaps all check out (verified `/mowing_map`, `/map_grid`, cmd_vel chain, gps/imu above).

Issues:
1. `localization.launch.py` calls `float(os.getenv("OM_DATUM_LAT"))` at parse time (lines 55–57) — if the env vars aren't set, the whole launch dies with a bare `TypeError`. Guard with a readable error.
2. Timer-based sequencing (3/5/10 s) instead of lifecycle/readiness events — works on a fast machine, races on a slow one (mission_manager tolerates it via its retry loop, so this is cosmetic for now).
3. `openmower.launch.py` (hardware) references `launch/gps.launch.py` and micro-ROS; those depend on the **empty** `src/lib` vendor dirs — hardware bringup as committed cannot work. Also still named/branded openmower (`package open_mower_next`, namespace `open_mower_next::`) — cleanup item, as you noted.
4. Dead/broken files: `maps/world.yaml` references `world.pgm` which doesn't exist; `scripts/drive_coverage_path.py` literally contains the text "[paste the full script content from above]" — a placeholder that was never filled in; `config/mow_bt.xml` unused (above); map_server's pytest unregistered (above).
5. CI (`.github/workflows/ci.yml`) builds and can run the two Webots integration tests behind `OPEN_MOWER_NEXT_ENABLE_INTEGRATION_TESTS` — good bones for Phase 2.

---

## The ~10% stall — judgment

**Not a structural flaw in mission_manager.** Its FSM, segmentation, and retry logic behave exactly as designed; the 5×-retry-then-ERROR signature is it faithfully reporting a downstream failure. The evidence points at the **coverage_server → Nav2 contract plus config**, with two concrete, testable candidate mechanisms, both 🟡-class fixes:

1. **RPP turnaround failure / path aliasing** (Nav2 #4): 0.3 m swath spacing ≈ RPP min lookahead, 10 m closest-pose search across ~30 parallel swaths, garbage intermediate orientations (coverage #1), no generated turns (coverage #2). Predicts a stall at a *specific* hairpin where geometry/localization bias is worst — consistent with "reliably the same segment, ~10% in". Discriminating test: log RPP's pruned-path start index and carrot pose at the stall; if the carrot sits on an adjacent swath, this is it. Independently: shrink `max_robot_pose_search_dist` to < 0.15 m-equivalent window / raise swath spacing, and see if the stall moves or vanishes.
2. **Invisible physical dock** (sim #2): guaranteed collision when the path (headland rings, or transit between segments) crosses (7.6–8.4, 5.8–6.2). Predicts stalls late in the path or during NavigateToPose transits — if your "~10%" is measured on a path that starts near the NE corner, this could be the one. Discriminating test: hide/move the dock out of the operation polygon in the world file and rerun; or overlay the stall pose on the dock bbox.

Either way the fix is contract/config (generate real turns or per-swath goals; add the dock as exclusion/keepout + reconcile the three dock poses), not a redesign of any module.

---

## GO / NO-GO: **GO**

All ten audited areas are ✅ or 🟡; nothing needs architectural rework. The code is worth writing modular test plans against — with the caveat that Phase 2 should put its first tests exactly where the 🟡s cluster: the coverage-path contract (shape, spacing, orientations), the RPP turnaround envelope, and the dock-pose consistency (map ↔ world ↔ sim params ↔ docking_helper).

Fix-first shortlist (all small, all in-place):
1. map_server `declare_parameter` re-declare crash (blocks record→mow loop).
2. docking_helper hang-on-reject + ignored cancel + uncaught TF throw (blocks RETURNING_TO_DOCK reliability).
3. map_recorder `docking_goal_handle_` never cleared (blocks area recording).
4. Reconcile the three dock positions; move the dock out of (or exclude it from) the operation area.
5. Align RPP `desired_linear_vel` with velocity_smoother limits.
