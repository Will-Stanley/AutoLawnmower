# Module Test Plans — v2 (current at `d691a3a`, 2026-07-20)

Supersedes v1. Changes from v1: 11 tests flipped from *expected-fail* to **regression guards** (their bugs are fixed — the test now pins the fix); 5 new tests added (marked **NEW**); stall battery updated with sim run-2 evidence and a new S0 timing-sanity gate; prerequisite P2 replaced by a repair item. Test types: **U** unit, **I** integration (mocks allowed), **S** sim/behavioural (Webots, measurable metric). Every test has an explicit pass criterion.

## Module hierarchy

```
1 Mission Orchestration
  1.1 mission_manager FSM            (mission_manager_node.cpp)
  1.2 Path post-processing            (splitPathIntoSegments / removeReversalSpikes /
                                       fixSegmentEndpointOrientations / collapse guard)
2 Coverage Planning
  2.1 coverage_server service         (coverage_server_node.cpp)
  2.2 F2C↔ROS conversion + densify    (coverage_server/utils.h)
3 Nav2 Integration
  3.1 Controller path-following (RPP) (nav2_params.yaml controller_server)
  3.2 Costmaps                        (local/global_costmap)
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

### Infrastructure prerequisites
- **P1** Extract 1.2 and 2.2 into header-only lib targets for ROS-free unit tests (pure functions; CMake change).
- **P2 (revised)** Repair the now-registered `load_geojson_map.test.py`: add `'datum': [-22.9, -43.2]` to the map_server node params (ctor throws without it) and register via `add_launch_test` (`launch_testing_ament_cmake`) instead of `ament_add_pytest_test`. It WILL fail until repaired.
- **P3** Mock action-server fixtures (follow_path, navigate_to_pose, dock_robot_nearest) + transient-local `/mowing_map` publisher fixture; shared by modules 1, 2, 6.
- **P4** Fixture maps: the committed `.devcontainer/home/map.geojson`, plus a synthetic map **with an exclusion** (exclusion handling is untested everywhere; the committed map has none).
- **P5** Webots headless (`mode=fast`, `gui=false`) + supervisor ground-truth pose reader; CI gate `OPEN_MOWER_NEXT_ENABLE_INTEGRATION_TESTS` exists.

---

# PRIORITY 1 — the stall (now isolated to MOWING) + mission_manager contracts

## Stall battery — updated for run-2 evidence
Run 2 (`d691a3a`) established: densified path → 2 segments, real swath-following at 0.26 m/s, then **5× `SimpleProgressChecker` "Failed to make progress" during MOWING** (t+44/184/197/257/305 s), all in the x≈−4 band, robot moving between retries, 5th failure → ERROR. Also observed: EKF "Failed to meet update rate" errors (~t+164 s) — timing contamination is now a live hypothesis. Segmentation and orientation causes are ruled out (fixed and verified in this run).

Fixture: standard sim bringup; rosbag `/odometry/filtered/map`, `/cmd_vel_nav`, `/plan`, controller_server logs, `mission/state`, TF; supervisor ground truth.

| test | type | setup & inputs | action | pass criterion / discriminates |
|---|---|---|---|---|
| **stall_S0_timing_sanity (NEW, run first)** | S | stock run, no changes | measure Webots real-time factor, EKF actual rates vs 50 Hz target, controller_server loop rate vs 20 Hz | all ≥90% of nominal with zero "Failed to meet update rate" errors ⇒ timing is clean, proceed; else fix compute budget (headless mode, lower EKF frequency, `mode=fast`) and re-baseline — timing contamination invalidates every timeout-based conclusion below |
| stall_S1_reproduce | S | stock run | run to ERROR | baseline: failure times/poses stable across 3 runs (±0.5 m); log which swath + whether mid-swath or at a turnaround — run-2 data suggests turnarounds in the x≈−4 column; confirm |
| stall_S3_carrot_alias | S | log RPP lookahead pose + pruned-path closest index at 20 Hz | inspect at each failure | carrot's nearest path pose on a *different* swath than the robot's (lateral ≈0.3 m, heading ≥90° off) for >1 s ⇒ aliasing confirmed |
| stall_S4_spacing_sweep | S | `operation_width` ∈ {0.3, 0.6, 1.0} | rerun each | failures vanish for spacing ≥2×`min_lookahead_dist` ⇒ aliasing; persist ⇒ look at S8 |
| stall_S5_search_dist | S | `max_robot_pose_search_dist` 10.0→0.6 | rerun | failures gone ⇒ closest-pose search latching wrong swath confirmed |
| stall_S6_loc_error | S | stock run | EKF map pose vs supervisor truth at each failure ±5 s | ‖error‖>0.15 m (½ swath spacing) ⇒ localization contributes; <0.05 m ⇒ pure controller geometry |
| stall_S7_follow_direct | I/S | send the failing segment straight to `/follow_path`, robot parked at its start, mission_manager not running | observe | fails without mission_manager ⇒ FSM definitively exonerated (expected, per run 2 behaviour) |
| stall_S8_progress_margin | S | instrument SimpleProgressChecker: distance moved per 10 s window around each turnaround | clean run (0.6 m spacing if needed) | min window movement ≥2× `required_movement_radius` 0.5 m ⇒ healthy; <1.2× ⇒ checker envelope too tight for turnaround duration — retune (e.g. radius 0.3 / allowance 15 s) *with* S3 evidence, not instead of it |
| stall_S2_dock_hypothesis | S | (demoted — run-2 failures at x≈−4 are far from the dock at (8,6)) move dock out of polygon | rerun | only if S3–S8 all come back clean |

## 1.1 mission_manager FSM
**Purpose:** orchestrate IDLE→…→CHARGING via coverage service, NavigateToPose, FollowPath, DockRobotNearest.
**Source:** `src/mission_manager/mission_manager_node.cpp/.hpp`.
**Contract:** in — `/area_coverage`, `/power` (BatteryState, 0–100 assumed), `mission/start`; out — actions `/navigate_to_pose`, `/follow_path` (controller_id "FollowPath"), `/dock_robot_nearest`; topic `mission/state` (transient-local). Params: `target_area_id`, `battery_low_percent` 30, `battery_charged_percent` 95, `autostart`.
**Deps:** coverage_server, Nav2, docking_helper.

| test | type | setup & inputs | action | pass criterion | notes |
|---|---|---|---|---|---|
| fsm_happy_path | I | mock coverage srv (2-segment path), mock nav/follow/dock succeed | autostart | state sequence exactly IDLE→REQUESTING_COVERAGE→NAVIGATING_TO_START→MOWING→(NAVIGATING_TO_START→MOWING)→RETURNING_TO_DOCK→CHARGING | fixture P3 |
| fsm_coverage_retry | I | coverage srv fails 3× then succeeds | autostart | stays REQUESTING_COVERAGE; ≥3 requests; proceeds after success | |
| fsm_follow_retry_cap | I | follow_path aborts every goal | reach MOWING | exactly 6 goals (1+5 retries) → ERROR with reason | **regression guard — verified live in run 2** |
| fsm_stale_result_guard | I | battery-low mid-goal, then stale SUCCEEDED result | publish 25.0 | RETURNING_TO_DOCK; stale result ignored | |
| fsm_battery_low_preempt | I | battery 25.0 during MOWING | publish /power | follow_path cancelled; dock goal ≤2 s | |
| fsm_battery_low_navigating | I | battery 25.0 during NAVIGATING_TO_START | publish /power | navigate cancelled; dock goal sent; RETURNING_TO_DOCK | **regression guard (fixed e7da777)** |
| fsm_battery_scale_contract | U/I | publish percentage=0.9 | observe | *still expected-fail*: WARN fires (guard exists) but semantics undecided — pin 0–100 contract when decision lands | open decision #3 |
| fsm_charge_complete | I | CHARGING, battery 96.0 | publish | REQUESTING_COVERAGE ≤2 ticks | |
| **fsm_coverage_collapse_guard (NEW)** | I | mock coverage srv returns a path engineered so segmentation retains <50% of poses | autostart | ERROR "coverage path collapsed during segmentation"; NAVIGATING_TO_START never entered | regression guard for run-1 failure mode (d691a3a) |

## 1.2 Path post-processing
**Contract:** pure `Path → vector<Path>`; invariants: no intra-segment gap >1.0 m; segments ≥3 poses & ≥0.3 m; endpoint yaw = travel direction; fragments violating merge-gap are dropped with warning; caller guards ≥50% retention.

| test | type | setup & inputs | action | pass criterion | notes |
|---|---|---|---|---|---|
| split_boustrophedon | U | synthetic snake, 0.3 m transitions, one 5 m block jump | split | 2 segments; all intra-segment gaps ≤1.0 m | |
| split_merge_bug | U | degenerate 2-pose fragment 3 m before next segment | split | fragment dropped + warning; no segment contains the 3 m jump | **regression guard (fixed e7da777)** |
| **split_sparse_swath (NEW)** | U | path with consecutive poses 10 m apart (raw 2-point swaths, pre-densify format) | split | documents behaviour: splits into fragments; paired with cov_path_density this pins why densification is load-bearing | regression guard for run-1 collapse |
| split_all_degenerate | U | 2-pose path | split | 1 segment, no throw | |
| spike_removal | U | swath path + 0.1 m/170° spur | removeReversalSpikes | spur removed; 90° transition corners retained | |
| spike_no_false_positive | U | clean boustrophedon | removeReversalSpikes | zero removals | |
| endpoint_yaw | U | garbage orientations | fixSegmentEndpointOrientations | first/last yaw = travel direction ±1e-6 | middle poses now correct at source (utils.h fix) — assert that too |

---

# PRIORITY 2 — modules feeding the stall

## 2.1 coverage_server
**Contract:** in — `/mowing_map` (transient-local), `/area_coverage`; out — response path (+`coverage_geometry`), `coverage/path` (transient-local), `coverage/visualization`. Params: `robot_width`, `operation_width`, `min_turning_radius` (unused).

| test | type | setup & inputs | action | pass criterion | notes |
|---|---|---|---|---|---|
| cov_rect_geometry | I | fixture map 13×11 m, width 0.3, 3 loops | call srv | SUCCESS; parallel-swath spacing 0.3±0.03 m; all poses inside polygon | |
| **cov_path_density (NEW)** | I | same | inspect path | max consecutive-pose gap ≤0.5 m within swaths; gaps >1.0 m only at genuine block transitions (count them, expect ≤ headland_loops+1) | regression guard (d691a3a); this property is load-bearing for 1.2 |
| cov_orientation | I | same | inspect poses | every pose yaw = direction to next pose ±5° (last pose: from previous) | **regression guard (fixed e7da777)** |
| cov_coverage_ratio | U/I | rasterize path with 0.3 m disc, 0.05 m grid | compute | ≥95% of (polygon ⊖ headland band) covered; headland band covered by ring swaths | the "does it mow the lawn" number |
| cov_exclusion_hole | I | fixture map + 2×2 m exclusion (P4) | with_exclusions=true | no pose inside exclusion; SUCCESS. Buffered clause (no pose within 0.48 m) *still expected-fail* — pending open decision #2 | |
| cov_exclusion_swallows | I | exclusion covers area | call | CODE_INVALID_EXCLUSION, no crash | |
| cov_bad_area / cov_no_map_yet | I | unknown/empty/navigation id; call before map | calls | CODE_INVALID_AREA each; node alive | |
| **cov_path_topic (NEW)** | I | after a successful call, late-join subscriber on `coverage/path` | subscribe | receives the same path (transient-local delivery) | regression guard (fixed e7da777) |
| cov_headland_order | I | 3 loops | inspect | headland poses strictly after mainland; document as intended | |

## 2.2 F2C↔ROS conversion (utils.h)

| test | type | setup & inputs | action | pass criterion |
|---|---|---|---|---|
| ring_closure / ring_roundtrip | U | open + closed polygons | toLinearRing/toMsg | closure enforced; roundtrip preserves points ±1e-6 |
| cells_difference | U | rect + overlapping / disjoint exclusion | toCells | area == rect − intersection ±1%; disjoint unchanged |
| swaths_to_path | U | hand-built 2-point 10 m swath | toMsg(swaths) | 21 poses (≤0.5 m spacing), all yaw = leg direction; single-point swath → 1 pose, no throw | **regression guard (fixed e7da777 + d691a3a)** |

## 4 Localization
**Contract:** in — `diff_drive_base_controller/odom`, `imu/data_raw`, `gps/fix`; out — `/odometry/filtered`, `/odometry/filtered/map`, TF map→odom→base_link. Datum shared with map_server via `OM_DATUM_*`.

| test | type | setup & inputs | action | pass criterion | notes |
|---|---|---|---|---|---|
| **loc_update_rate (NEW)** | S | stock run 5 min | count EKF "Failed to meet update rate" + measure achieved Hz | zero errors, ≥45 Hz achieved (50 target) | run 2 saw failures — same evidence gate as stall_S0 |
| loc_static_consistency | S | parked 30 s | record | map drift <0.05 m; odom drift <0.01 m; single TF publisher per edge | |
| loc_square_drive | S | scripted 10×10 m square | EKF vs supervisor truth | map RMSE <0.15 m; return error <0.2 m | 0.15 m = ½ swath spacing, feeds stall_S6 |
| loc_datum_agreement | I | map_server + navsat, fixture env | project same lat/lon both ways | same map xy ±0.05 m | |
| loc_odom_rate_margin | S | measure wheel-odom rate | compare | ≥1.5× the 40 Hz implied by `sensor_timeout: 0.025`, else retune | |
| loc_imu_orientation_present | S | echo `imu/data_raw` | inspect | orientation non-identity, covariance ≠ −1 | |
| loc_gps_dropout | S | pause GPS 10 s mid-drive | observe | graceful degradation; <0.5 m jump on reacquire | hardware-relevant |

## 3 Nav2 Integration

| test | type | setup & inputs | action | pass criterion | notes |
|---|---|---|---|---|---|
| nav_costmap_matches_map | I | map_server + Nav2 only | compare `/map_grid` vs global costmap | lethal↔lethal, free↔free, extent ⊇ polygon+0.48 m | |
| nav_hairpin_unit | S | robot parked at hand-built 2-swath hairpin (gap 0.3/0.6/1.0 m), send to /follow_path | observe | completes <60 s, no progress abort, cross-track <0.15 m on straights | minimal turnaround-envelope repro; the stall's unit test |
| nav_speed_contract | S | straight 8 m follow_path | measure | steady-state ≈0.26 m/s; RPP desired ≤ smoother max asserted by config check | **regression guard — verified live in run 2** |
| nav_progress_checker_bounds | I | zero cmd_vel while follow_path active | wait | abort at 10±1 s, progress-checker error surfaced | pins the observed failure mode |
| nav_planner_tolerance | I | goal 0.4 m inside lethal | plan | returned goal within 0.5 m AND in free space | |
| nav_no_obstacle_source | S | 0.5 m box on lawn mid-swath | follow_path through it | documents: costmap blind, collision detected only via supervisor | drives open decision #4 |
| nav_cmdvel_chain | I | trace controller output | inspect | reaches diff drive only via smoother→twist_mux; joystick preempts ≤0.5 s | |

---

# PRIORITY 3 — remaining modules

## 5.1 GeoJSON IO
| test | type | inputs | pass criterion |
|---|---|---|---|
| geo_roundtrip | U/I | load→save→load fixture | counts equal; vertices ±0.01 m; dock yaw ±0.5° |
| geo_dock_pose | U | dock LineString | position = first coord; yaw = atan2(second−first) ±1e-3 |
| geo_malformed | U | empty / non-FC / Point / 1-coord LineString | partial map + warning, no throw |
| geo_datum_missing | I | node without datum | clean invalid_argument |
| **geo_datum_latch (NEW)** | I | start map_server, then late-join subscriber on `map/datum` | late subscriber receives datum (transient-local cache) | regression guard (fixed e7da777) |

## 5.2 Occupancy-grid generation
| test | type | inputs | pass criterion |
|---|---|---|---|
| grid_interior_free | U | 10×10 rect, res 0.1 | inside ==0; ≥0.48+0.05 m outside ==100 |
| grid_dilation_width | U | same | free band 0.48±0.1 m, circular |
| grid_exclusion_wins | U | rect + exclusion in dilated band | exclusion cells ==100 (paint-order contract) |
| grid_republish_after_save | I | call save_area twice | both publishes succeed, grid updates | **regression guard (fixed e7da777)** |
| grid_oversize_clamp | U | 300 m polygon | truncation warning fires; grid ≤2000² | **regression guard (warning added e7da777)** |
| grid_thin_polygon | U | 0.05 m sliver | no crash; cells free via edge tolerance |

## 5.3 map_server services
| test | type | action | pass criterion |
|---|---|---|---|
| srv_area_crud / srv_dock_crud | I | save→update→remove | SUCCESS codes; file reflects each step; `/mowing_map` republished each time |
| srv_remove_missing | I | remove unknown id | CODE_NOT_FOUND; map unchanged |

## 5.4 map_recorder
| test | type | setup | pass criterion | notes |
|---|---|---|---|---|
| rec_area_auto / rec_area_manual | I | fake TF walker; mock save_area | points match trace ±0.02 m; services removed after finish; manual add rejected in auto mode & vice versa | |
| rec_after_dock_recording | I | complete a dock recording first | area recording accepted | **regression guard (fixed e7da777)** |
| rec_dock_happy / rec_dock_no_charge | I | mock drive_on_heading ± charger topic | saved pose = charging_port flipped 180° ±1e-3; timeout abort at 60±2 s | |
| rec_covariance_stub | — | — | open: implement `checkPositionCovariance` or delete the promising abort message | worklist item |

## 6 Docking
| test | type | setup | pass criterion | notes |
|---|---|---|---|---|
| dock_nearest_selection | U/I | 3 docks + static TF | nearest id returned | |
| dock_pose_math | U | dock (2,0,yaw 0), port offset (0.3,0) | yaw π, position (2.3,0) ±1e-3 | |
| dock_reject_hang | I | mock /dock_robot rejects | action aborts ≤10 s | **regression guard (fixed e7da777)** |
| dock_cancel | I | cancel mid-dock | Nav2 goal cancelled; CANCELED ≤2 s | **regression guard (fixed e7da777)** |
| dock_no_tf_crash | I | no charging_port TF | aborts with message; process survives | **regression guard (fixed e7da777)** |
| dock_plugin_states | U | drive `/power/charger_present` | isDocked/isCharging track ≤100 ms | |
| dock_e2e_sim | S | **blocked on open decision #1** (dock reconciliation) | RETURNING_TO_DOCK→CHARGING ≤90 s, 3/3 runs | run 2 confirmed goal targets the GeoJSON dock, physical dock elsewhere |

## 7 Sim support
| test | type | pass criterion |
|---|---|---|
| sim_battery_discharge | I | 60 s → voltage −0.3±0.03 V; percentage formula correct |
| sim_charge_detect_box | I | charger_present iff \|dx\|<0.20 & \|dy\|<0.12 in dock frame |
| sim_dock_pose_consistency | I | webots dock, geojson dock, contact pose mutually ≤0.1 m/5° — *still expected-fail*; becomes the regression guard after open decision #1 |

## 8 Interfaces
| test | type | pass criterion |
|---|---|---|
| iface_codes_unique | U | result codes unique; every code produced by ≥1 source site |

## 9 Launch wiring
| test | type | pass criterion |
|---|---|---|
| launch_smoke_sim | S | all lifecycle nodes ACTIVE; key topics publishing; zero node deaths in 120 s |
| launch_env_guard | U | unset OM_DATUM_LAT → readable RuntimeError naming the variable | **regression guard (fixed e7da777)** |
| launch_param_rewrite | U | use_sim_time=false render leaves no `True` |
| **launch_pytest_repair (NEW)** | I | after P2 repair, `colcon test` runs load_geojson_map green | pins the repair |

---

# Integration spine

| test | modules | pass criterion |
|---|---|---|
| SPINE-1 map→costmap | 5→3.2 | costmap matches grid on fixture map |
| SPINE-2 loc→nav | 4→3 | 4 corner goals reached within 0.25 m (supervisor-verified) |
| SPINE-3 coverage→follow | 2→1.2→3.1 | **the stall gate.** Run-2 status: swath-following works at 0.26 m/s for minutes; fails on repeated progress-checker aborts. Blocked on stall battery verdict (S0→S8) + resulting fix. Pass: one full segment, ≥90% corridor coverage, zero progress aborts |
| SPINE-4 full mission | 1–5 | ≥95% lawn coverage (supervisor track, 0.3 m width), zero ERROR |
| SPINE-5 dock+charge | 6,7,1 | CHARGING reached — **blocked on open decision #1** |
| SPINE-6 battery loop | 1,6,7 | low-start → preempt → dock → charge → resume, ≤30 sim-min |
| SPINE-7 soak | all | 3 consecutive missions, zero stalls/restarts, loc RMSE <0.15 m |

**Sequencing:** P1/P2 infra → stall battery S0 (timing gate) → S1,S3–S8 → land the indicated fix → SPINE-3 → SPINE-4 → (after dock reconciliation) SPINE-5–7. Unit/integration tables can proceed in parallel per module owner.
