# Phase 2 — Module Map & Test Plans (AutoLawnmower)

Derived from source audited in Phase 1. Test types: **U** = unit, **I** = integration (2+ modules' contract, mocks allowed), **S** = sim/behavioural (Webots, measurable metric). Every test lists explicit pass criteria.

## Module hierarchy

```
1 Mission Orchestration
  1.1 mission_manager FSM            (mission_manager_node.cpp)
  1.2 Path post-processing            (splitPathIntoSegments / removeReversalSpikes /
                                       fixSegmentEndpointOrientations — same file)
2 Coverage Planning
  2.1 coverage_server service         (coverage_server_node.cpp)
  2.2 F2C↔ROS conversion              (coverage_server/utils.h)
3 Nav2 Integration
  3.1 Controller path-following (RPP) (nav2_params.yaml controller_server)
  3.2 Costmaps                        (nav2_params.yaml local/global_costmap)
  3.3 Planner + BT navigation         (planner_server, bt_navigator)
  3.4 cmd_vel chain                   (velocity_smoother, twist_mux)
4 Localization
  4.1 ekf_se_odom                     (robot_localization.yaml)
  4.2 ekf_se_map + navsat             (robot_localization.yaml, localization.launch.py)
5 Mapping
  5.1 GeoJSON IO                      (geo_json_map.cpp)
  5.2 Occupancy-grid generation       (map_server_node.cpp, polygon_iterator.hpp)
  5.3 map_server services             (map_server_node.cpp)
  5.4 map_recorder                    (map_recorder_node.cpp)
6 Docking
  6.1 docking_helper orchestration    (docking_helper_node.cpp)
  6.2 ChargerPresenceChargingDock     (charger_presence_charging_dock.cpp)
  6.3 docking_server config           (nav2_params.yaml)
7 Sim Support                         (sim_node.cpp, openmower.wbt, DockingStation.proto)
8 Interfaces                          (src/msg, src/srv, src/action)
9 Launch Wiring                       (launch/*.py)
```

### Test infrastructure prerequisites (do first)
- **P1** Extract 1.2 (path post-processing) and 2.2 (utils.h) into a header-only lib target so they're unit-testable without ROS executors. Both are pure functions already; this is a CMake change.
- **P2** Register `src/map_server/test/load_geojson_map.test.py` in CMakeLists (currently dead).
- **P3** Add a `gtest`/`pytest` harness with mock action servers (follow_path, navigate_to_pose, dock_robot_nearest) and a transient-local `/mowing_map` publisher fixture. One reusable fixture file serves modules 1, 2, 6.
- **P4** Fixture data: `.devcontainer/home/map.geojson` (real), plus a synthetic map with one exclusion (none exists today — exclusion paths are untested everywhere).
- **P5** Webots headless (`mode=fast`, `gui=false`) + supervisor ground-truth pose reader — CI already gates integration tests behind `OPEN_MOWER_NEXT_ENABLE_INTEGRATION_TESTS`.

---

# PRIORITY 1 — mission_manager + its contracts (stall isolation)

## 1.1 mission_manager FSM
**Purpose:** Orchestrate IDLE→…→CHARGING via coverage service, NavigateToPose, FollowPath, DockRobotNearest.
**Source:** `src/mission_manager/mission_manager_node.cpp/.hpp`.
**Interface contract:**
- In: `/area_coverage` (srv AreaCoverage), `/power` (BatteryState, percentage assumed 0–100), `mission/start` (Trigger).
- Out: actions `/navigate_to_pose`, `/follow_path` (controller_id="FollowPath"), `/dock_robot_nearest`; topic `mission/state` (String, transient-local).
- Params: `target_area_id`, `battery_low_percent` 30, `battery_charged_percent` 95, `autostart`.
**Dependencies:** coverage_server, Nav2 controller/bt_navigator, docking_helper.

| test | type | setup & inputs | action | pass criterion | notes |
|---|---|---|---|---|---|
| fsm_happy_path | I | mock coverage srv (returns 2-segment path), mock nav/follow/dock servers (succeed) | autostart | `mission/state` sequence exactly IDLE→REQUESTING_COVERAGE→NAVIGATING_TO_START→MOWING→(NAVIGATING_TO_START→MOWING)→RETURNING_TO_DOCK→CHARGING | fixture P3 |
| fsm_coverage_retry | I | mock coverage srv fails 3× then succeeds | autostart | stays REQUESTING_COVERAGE during failures; proceeds after success; ≥3 requests observed | tick() retry loop |
| fsm_follow_retry_cap | I | mock follow_path aborts every goal | reach MOWING | exactly 6 follow_path goals (1+5 retries) then state==ERROR, reason logged | pins kMaxActionRetries semantics |
| fsm_stale_result_guard | I | mock follow_path; force battery-low mid-goal, then deliver stale SUCCEEDED result | battery msg 25.0 | state==RETURNING_TO_DOCK; stale result does NOT trigger advanceToNextSegment | guards at .cpp:381/449 |
| fsm_battery_low_preempt | I | mock servers; battery 25.0 during MOWING | publish /power | follow_path receives cancel; dock goal sent within 2 s; state==RETURNING_TO_DOCK | |
| fsm_battery_low_navigating | I | battery 25.0 during NAVIGATING_TO_START | publish /power | **expected-fail today**: nothing happens (audit gap). Test documents desired: dock goal sent | decide desired behaviour first |
| fsm_battery_scale_contract | U/I | publish BatteryState percentage=0.9 (0–1 convention) | observe | **expected-fail today**: instantly "battery low". Pin contract: define 0–100 in an interface doc + reject/scale <1.0 inputs | cross-module contract w/ sim + hardware |
| fsm_charge_complete | I | state CHARGING, battery 96.0 | publish /power | state→REQUESTING_COVERAGE within 2 ticks | |

## 1.2 Path post-processing (stall-critical)
**Purpose:** Split raw coverage path into follow_path-safe segments; scrub spikes; fix endpoint yaw.
**Source:** `mission_manager_node.cpp` lines 181–329.
**Interface contract:** pure: `Path → vector<Path>`; invariant: no intra-segment consecutive-pose gap > 1.0 m; all segments ≥3 poses & ≥0.3 m; first/last pose yaw = travel direction.
**Dependencies:** none (after P1 extraction).

| test | type | setup & inputs | action | pass criterion | notes |
|---|---|---|---|---|---|
| split_boustrophedon | U | synthetic snake path, 0.3 m swath gaps, one 5 m mainland→headland jump | splitPathIntoSegments | 2 segments; every intra-segment gap ≤1.0 m | |
| split_merge_bug | U | degenerate 2-pose fragment 3 m before next segment | split | **expected-fail today**: merged segment contains a 3 m internal jump (audit: lines 236–247 violate own invariant) | fix, then flip to pass |
| split_all_degenerate | U | path of 2 poses total | split | returns 1 segment, no throw, poses preserved | edge case |
| spike_removal | U | swath path + one 0.1 m/170° spur vertex | removeReversalSpikes | spur removed; legitimate 90° swath-transition corners retained (pose count check) | guards over-pruning |
| spike_no_false_positive | U | clean boustrophedon, 0.3 m transitions (2×90° corners) | removeReversalSpikes | zero vertices removed | 90°<150° threshold |
| endpoint_yaw | U | segment with garbage orientations | fixSegmentEndpointOrientations | first pose yaw == atan2 to pose[1] ±1e-6; last likewise; **middle poses still garbage — assert & document** | ties to test 2.2 orient_fix |

## Stall isolation battery (cross-module, run in this order)
Fixture: standard sim bringup, record rosbag of `/odometry/filtered/map`, `/plan`, `/cmd_vel_nav`, RPP debug topics, `mission/state`, TF; supervisor ground-truth pose.

| test | type | setup & inputs | action | pass criterion / discriminates |
|---|---|---|---|---|
| stall_S1_reproduce | S | stock sim, autostart mission | run to stall | Reproduce + record: stall pose ±0.2 m stable across 3 runs; segment index & % logged. Baseline for all below |
| stall_S2_dock_hypothesis | S | move DockingStation to (12,6) (outside operation area) in `openmower.wbt` | rerun | stall persists ⇒ dock not the cause; stall gone ⇒ dock collision confirmed (fix: exclusion/keepout) |
| stall_S3_carrot_alias | S | stock run, log RPP lookahead ("carrot") pose + pruned-path closest index at 20 Hz | inspect at stall | If carrot's nearest path pose lies on a *different* swath than robot's current swath (lateral offset ≈0.3 m, heading ≥90° off) for >1 s ⇒ aliasing confirmed |
| stall_S4_spacing_sweep | S | operation_width ∈ {0.3, 0.6, 1.0} (sim.launch.py) | rerun each | stall vanishes for spacing ≥2×min_lookahead ⇒ aliasing; persists ⇒ look elsewhere |
| stall_S5_search_dist | S | `max_robot_pose_search_dist`: 10.0→0.6 | rerun | stall gone ⇒ closest-pose search latching wrong swath confirmed |
| stall_S6_loc_error | S | stock run | compare EKF map pose vs supervisor truth at stall ±5 s | ‖error‖>0.15 m (½ swath spacing) at stall ⇒ localization contributes; <0.05 m ⇒ pure controller geometry |
| stall_S7_follow_direct | I/S | bypass mission_manager: send recorded stall segment straight to `/follow_path` from robot parked at segment start | observe | stalls without mission_manager in loop ⇒ definitively exonerates FSM; completes ⇒ suspect state interaction (unlikely per audit) |
| stall_S8_progress_margin | S | instrument SimpleProgressChecker: log distance-moved in each 10 s window near turnarounds | clean run at 0.6 m spacing | min window movement ≥2× required_movement_radius (0.5 m) ⇒ healthy margin; <1.2× ⇒ checker tuned too tight for turnarounds |

---

# PRIORITY 2 — modules feeding the stall

## 2.1 coverage_server
**Purpose:** AreaCoverage srv → F2C headlands+swaths → nav_msgs/Path.
**Source:** `src/coverage_server/coverage_server_node.cpp`.
**Interface contract:**
- In: `/mowing_map` (Map, transient-local), `/area_coverage` (AreaCoverage: area_id, with_exclusions, headland_loops, swath_angle).
- Out: response path+coverage_geometry; `coverage/visualization` markers. (`coverage/path` publisher is dead — audit.)
- Params: `robot_width`, `operation_width`, `min_turning_radius` (unused today).
**Dependencies:** map_server (map topic), Fields2Cover.

| test | type | setup & inputs | action | pass criterion | notes |
|---|---|---|---|---|---|
| cov_rect_geometry | I | publish fixture map (13×11 m rect), width=0.3, 3 loops | call srv | CODE_SUCCESS; path ≥2 poses/swath; adjacent parallel-swath lateral spacing 0.3±0.03 m; all poses inside polygon | mock-free: needs only a map publisher |
| cov_coverage_ratio | U/I | same; rasterize path with 0.3 m disc at 0.05 m grid | compute | ≥95% of (polygon ⊖ headland band) cells covered; 100% of cells within headland rings' band covered | the actual "does it mow the lawn" number |
| cov_orientation_contract | I | same | inspect poses | **expected-fail today**: pose yaw == direction to next pose ±5°. Documents getAngleFromPoint bug; flip to pass after fix | audit coverage #1 |
| cov_exclusion_hole | I | fixture map + 2×2 m exclusion inside | call srv with_exclusions=true | no path pose inside exclusion+0.48 m buffer; CODE_SUCCESS | buffer part fails today (unbuffered) — decide fix (buffer in coverage or inflation in Nav2) then pin |
| cov_exclusion_swallows | I | exclusion covering whole area | call srv | CODE_INVALID_EXCLUSION, message non-empty, no crash | lines 103–107 |
| cov_bad_area | I | unknown id; empty id; navigation-type id | 3 calls | CODE_INVALID_AREA each; node stays alive | |
| cov_no_map_yet | I | call srv before any map published | call | CODE_INVALID_AREA (not crash/hang) | current_map_ default-empty |
| cov_headland_order | I | 3 loops | inspect path | headland ring poses appear only after last mainland pose; document as intended or re-order | audit coverage #3 |

## 2.2 F2C↔ROS conversion (utils.h)
**Purpose:** Polygon↔LinearRing/Cell(s), Swaths→Path.
**Source:** `src/coverage_server/utils.h`. **Dependencies:** none (P1).

| test | type | setup & inputs | action | pass criterion |
|---|---|---|---|---|
| ring_closure | U | open 4-pt polygon | toLinearRing | ring closed (first==last), size 5 |
| ring_roundtrip | U | closed polygon → ring → toMsg | compare | point count preserved (closure dupe stripped), coords ±1e-6 |
| cells_difference | U | rect + overlapping exclusion | toCells | result area == rect − intersection ±1%; disjoint exclusion leaves area unchanged |
| swaths_to_path_orient | U | 2 hand-built swaths | toMsg(swaths) | **expected-fail today**: yaw == travel direction (see cov_orientation_contract) |

## 4 Localization
**Purpose:** odom→base_link (wheel+IMU), map→odom (+GPS).
**Source:** `config/robot_localization.yaml`, `launch/localization.launch.py`.
**Interface contract:** In: `diff_drive_base_controller/odom`, `imu/data_raw`, `gps/fix`→navsat→`odometry/gps`. Out: `/odometry/filtered` (odom), `/odometry/filtered/map`, TF odom→base_link, map→odom. Params: datum == map_server datum (both from OM_DATUM_*).
**Dependencies:** ros2_control odom, Webots sensors (sim) / drivers (hw).

| test | type | setup & inputs | action | pass criterion | notes |
|---|---|---|---|---|---|
| loc_static_consistency | S | robot parked 30 s | record | map-pose drift <0.05 m; odom-pose drift <0.01 m; TF tree exactly map→odom→base_link (no dual publishers) | |
| loc_square_drive | S | teleop 10×10 m square via cmd_vel script, return to start | compare EKF vs supervisor truth | map-frame RMSE <0.15 m over run; final-return error <0.2 m | 0.15 m = ½ swath spacing → feeds stall_S6 |
| loc_datum_agreement | I | launch map_server + navsat with fixture env | project geojson dock via map_server; project same lat/lon via navsat filtered-GPS | same map xy ±0.05 m | catches datum/projection divergence (LocalCartesian both sides — should pass) |
| loc_odom_rate_margin | S | record `diff_drive_base_controller/odom` rate | measure | rate ≥1/sensor_timeout (40 Hz) with margin ≥1.5×; else retune timeout | audit note |
| loc_imu_orientation_present | S | echo `imu/data_raw` | inspect | orientation quaternion non-identity & covariance ≠ −1 (EKF fuses roll/pitch — verify Webots actually provides it) | unverified in audit |
| loc_gps_dropout | S | pause GPS device 10 s mid-drive | observe | map pose degrades gracefully (<0.5 m jump on reacquire); no TF flicker | hardware-relevant robustness |

## 3 Nav2 Integration
**Purpose:** Path following, planning, costmaps, velocity chain per `nav2_params.yaml`.
**Dependencies:** map_server grid, localization TF.

| test | type | setup & inputs | action | pass criterion | notes |
|---|---|---|---|---|---|
| nav_costmap_matches_map | I | bringup map_server+Nav2 only | compare `/map_grid` vs global costmap | every lethal grid cell lethal in costmap; free interior free; costmap extent ⊇ polygon+0.48 m | verifies static-layer + dilation contract |
| nav_hairpin_unit | S | park robot at hand-built 2-swath hairpin path (0.3 m gap), send to /follow_path | observe | completes <60 s; no progress-checker abort; cross-track error <0.15 m on straights | minimal repro of the turnaround envelope; sweep gap 0.3/0.6/1.0 |
| nav_speed_contract | S | straight 8 m follow_path | measure cmd_vel_nav & actual speed | steady-state |v| == min(RPP desired, smoother max) and params agree after fix (align 0.5 vs 0.26 — audit Nav2 #3) | |
| nav_progress_checker_bounds | I | mock: publish zero cmd_vel odometry while follow_path active | wait | abort at 10±1 s with progress-checker error code surfaced to client | pins the failure mode the stall shows |
| nav_planner_tolerance | I | request path to pose 0.4 m inside lethal region | planner | NavFn returns goal within `tolerance` 0.5 m of request but **in free space**; mission_manager start-pose still reachable | silent-tolerance trap |
| nav_no_obstacle_source | S | place 0.5 m box on lawn mid-swath | run follow_path through it | **documents today's behaviour**: costmap never shows it; robot collides; progress abort. Pass = test detects collision via supervisor contact/velocity mismatch. Drives the "add obstacle layer or keepouts" decision | audit Nav2 #1 |
| nav_cmdvel_chain | I | publish on controller output side | trace | msg arrives at `/diff_drive_base_controller/cmd_vel` only via smoother→twist_mux; joystick topic preempts nav (priority 100>10) within 0.5 s | |

---

# PRIORITY 3 — remaining modules

## 5.1 GeoJSON IO
**Source:** `geo_json_map.cpp`. **Contract:** file↔`msg::Map`; LL↔map via LocalCartesian(datum).

| test | type | setup & inputs | action | pass criterion |
|---|---|---|---|---|
| geo_roundtrip | U/I | load fixture → save → load | compare | areas/docks count equal; vertices ±0.01 m; dock yaw ±0.5° |
| geo_dock_pose | U | dock LineString fixture | load | pose position == first coord; yaw == atan2(second−first) ±1e-3 |
| geo_malformed | U | empty file; non-FeatureCollection; Point geometry; 1-coord LineString | load each | empty/partial map returned, warning logged, **no throw** |
| geo_datum_missing | I | node without datum param | construct | clean invalid_argument with readable message |

## 5.2 Occupancy-grid generation
**Source:** `map_server_node.cpp` 279–602, `polygon_iterator.hpp`.

| test | type | setup & inputs | action | pass criterion |
|---|---|---|---|---|
| grid_interior_free | U | 10×10 rect, res 0.1 | mapToOccupancyGrid | all cells ≥0.05 m inside polygon ==0; all cells ≥0.48+0.05 m outside ==100 |
| grid_dilation_width | U | same | measure free band beyond polygon edge | 0.48±0.1 m on all 4 sides (circular kernel) |
| grid_exclusion_wins | U | rect + exclusion overlapping dilated band | generate | exclusion cells ==100 even where dilation freed them (paint-order contract, .cpp:369–378) |
| grid_republish_after_save | I | running node; call save_area twice | observe | **expected-fail today**: 2nd publish OK (declare_parameter crash — fix-first item #1). Flip to pass after fix |
| grid_oversize_clamp | U | 300 m polygon | generate | warning logged; grid ≤2000² **and** truncation surfaced in response/log (today: silent) |
| grid_thin_polygon | U | 0.05 m-wide sliver (below res) | generate | no crash; sliver cells free via edge-tolerance path |

## 5.3 map_server services
| test | type | setup & inputs | action | pass criterion |
|---|---|---|---|---|
| srv_area_crud | I | running node, tmp geojson | save→update→remove area | codes SUCCESS/SUCCESS/SUCCESS; file on disk reflects each step; `/mowing_map` republished each time (after fix #1) |
| srv_remove_missing | I | remove unknown id | call | CODE_NOT_FOUND; map unchanged |
| srv_dock_crud | I | same for docking station | as above | as above |

## 5.4 map_recorder
**Contract:** actions `record_area_boundary`, `record_docking_station`; consumes TF map→mower/charging_port, `/power/charger_present`, DriveOnHeading; calls save services.

| test | type | setup & inputs | action | pass criterion | notes |
|---|---|---|---|---|---|
| rec_area_auto | I | fake TF publisher moving 0.06 m/step; mock save_area | start action auto mode, finish after square | points ≈ path length/0.05; saved polygon matches TF trace ±0.02 m; services removed after finish | |
| rec_area_manual | I | manual mode | add 4 points via trigger, finish | 4 points exact; auto-add rejected in manual mode | |
| rec_after_dock_recording | I | run a dock recording to completion first | then request area recording | **expected-fail today**: rejected (goal handle never cleared — fix-first #3). Flip after fix | |
| rec_dock_happy | I | mock drive_on_heading; publish charger_present=true after 2 s; static TF | record dock | saved pose == charging_port pose rotated 180° ±1e-3; drive goal cancelled | |
| rec_dock_no_charge | I | never publish charging | record dock | abort CODE_NO_CHARGING_DETECTED at 60±2 s | |
| rec_covariance_stub | — | — | — | Track: implement checkPositionCovariance or delete the abort message promising it | audit map_recorder #2 |

## 6 Docking
**Contract:** `dock_robot_nearest`/`dock_robot_to` → nav2 `/dock_robot` with computed dock_pose; plugin reports docked via `/power/charger_present`.

| test | type | setup & inputs | action | pass criterion | notes |
|---|---|---|---|---|---|
| dock_nearest_selection | U/I | publish map with 3 docks; static TF at known pose | call find_nearest srv | returns geometrically nearest id | |
| dock_pose_math | U | dock at (2,0,yaw 0); base_link→charging_port = (0.3,0) | dockPose | result: yaw π, position (2.3, 0) ±1e-3 (offset *behind* flipped pose) | pins the 180°+offset math |
| dock_reject_hang | I | mock /dock_robot that **rejects** goal | send dock_robot_nearest | **expected-fail today**: action aborts ≤10 s (currently hangs forever — fix-first #2). Flip after fix | |
| dock_cancel | I | mock /dock_robot slow; cancel client goal | observe | nav2 goal cancelled; result CANCELED ≤2 s (today: ignored) | |
| dock_no_tf_crash | I | no charging_port TF | send goal | action aborts with message; **process survives** (today: std::terminate risk) | |
| dock_plugin_states | U | drive `/power/charger_present` true/false | poll plugin | isDocked/isCharging track topic ≤100 ms; hasStoppedCharging inverse | |
| dock_e2e_sim | S | after dock-pose reconciliation (fix-first #4) | full RETURNING_TO_DOCK→CHARGING in Webots | charger_present true ≤90 s from dock goal; mission reaches CHARGING; 3/3 runs | blocked on fix #4 today |

## 7 Sim support
| test | type | setup & inputs | action | pass criterion |
|---|---|---|---|---|
| sim_battery_discharge | I | run sim_node alone, charger absent | 60 s | voltage drops 0.3±0.03 V; percentage math matches (v−21.7)/7·100 ±0.5 |
| sim_charge_detect_box | I | static TF placing charging_port at contact pose ± offsets | sweep | charger_present true iff |dx|<0.20 & |dy|<0.12 (frame-correct: offsets in dock frame) |
| sim_dock_pose_consistency | I | parse world file, geojson, sim params in one script | compare | webots dock, geojson dock, contact pose mutually ≤0.1 m/5° — **fails today by design; becomes the regression guard after fix #4** |

## 8 Interfaces
| test | type | setup & inputs | action | pass criterion |
|---|---|---|---|---|
| iface_codes_unique | U | parse .srv/.action files | script | result codes unique per file; every code producible by grep in exactly ≥1 source site; flags dead codes |
| iface_dead_srv | — | — | — | Track: delete or implement `PolygonCoverage.srv` (compiled, zero references) |

## 9 Launch wiring
| test | type | setup & inputs | action | pass criterion |
|---|---|---|---|---|
| launch_smoke_sim | S | `sim.launch.py` headless, env set | 120 s | all lifecycle nodes ACTIVE; `/mowing_map`, `/map_grid`, `/odometry/filtered/map`, `mission/state` all publishing; zero node deaths (extends existing test_webots_smoke.py) |
| launch_env_guard | U | unset OM_DATUM_LAT; parse localization.launch.py | expect | readable error naming the variable (today: bare TypeError) — add guard, then pin |
| launch_param_rewrite | U | render nav2 RewrittenYaml with use_sim_time=false | inspect | no `use_sim_time: True` survives (hardware-launch correctness) |

---

# Integration spine (minimum cross-module proof)

Run order = dependency order; each assumes the previous passed. All S-type, Webots headless, supervisor ground truth, 3-run repeatability unless noted.

| test | modules proven | action | pass criterion |
|---|---|---|---|
| SPINE-1 map→costmap | 5.1→5.2→3.2 | bringup mapping+Nav2 | nav_costmap_matches_map criteria on the real fixture map |
| SPINE-2 loc→nav | 4→3 | NavigateToPose to 4 corners of lawn | each goal reached within xy_goal_tolerance 0.25 m (supervisor-verified, not self-reported); no recovery behaviours triggered |
| SPINE-3 coverage→follow | 2→1.2→3.1 | mission_manager disabled; script: call /area_coverage, post-process, send segment 1 to /follow_path | segment completes; supervisor-measured coverage of segment corridor ≥90%; no progress aborts (this is the stall gate — expected to fail until stall fix lands) |
| SPINE-4 full mission | 1+2+3+4+5 | autostart mission on stock map (dock relocated per fix #4) | mission reaches RETURNING_TO_DOCK with ≥95% lawn coverage (rasterized supervisor track, mow width 0.3 m); zero ERROR states |
| SPINE-5 dock+charge | 6+7+1 | continue SPINE-4 | CHARGING reached; charger_present true; battery percentage rising |
| SPINE-6 battery loop | 1+6+7 | start mission with initial_battery_voltage low (≈25 V) | mid-mow preempt → dock → CHARGING → (battery≥95%) → REQUESTING_COVERAGE again; full loop ≤ sim 30 min |
| SPINE-7 soak | all | 3 consecutive full missions, SPINE-4 map | zero stalls, zero node restarts, map-frame localization RMSE <0.15 m throughout |

**Sequencing note:** SPINE-3 is the stall gate — run the Stall isolation battery (S1–S8) first, land the indicated fix (turn generation / RPP retune / dock keepout), then SPINE-3 flips green and unblocks 4–7.

## Known gaps the plan deliberately exposes
Tests marked **expected-fail today** double as regression guards for every Phase 1 🟡: map_server redeclare crash (grid_republish_after_save), docking hang/cancel/TF-crash (dock_reject_hang, dock_cancel, dock_no_tf_crash), recorder goal-handle leak (rec_after_dock_recording), segment-merge invariant break (split_merge_bug), orientation contract (cov_orientation_contract), battery scale (fsm_battery_scale_contract), dock-pose triple mismatch (sim_dock_pose_consistency), env-var crash (launch_env_guard). Fix each, flip the test, and the bug can never return silently.
