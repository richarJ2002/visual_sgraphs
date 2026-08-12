# WP4: Simulation Verification Harness

This document describes how every vs_graphs change for **WP1** (room context
carryover), **WP2** (wall loop completion), and **WP3** (deterministic map
merge) is verified. Implementation changes must be validated by running the
full SITL simulation — a successful compile is **not** sufficient evidence.

## 1. Overview

The harness runs the Alpha waypoint mission (240+ waypoints covering all 12
office rooms in a 2×6 grid; each room 3.4 m × 5.0 m, corridor 1.8 m wide) and
collects the vs_graphs runtime logs. It then reports PASS/FAIL against the
acceptance criteria of each work package.

| WP | Feature | Runtime signal | RViz marker | Acceptance criterion |
|----|---------|----------------|-------------|----------------------|
| WP1 | Room context carryover | `[System] Map restart detected` / `[Atlas] Matched room ... tagged with identity "room_N"` | room labels keep consistent tags across restarts | ≥1 room tagged after a map restart |
| WP2 | Wall loop completion | `[SemMgr] Room#N boundary=COMPLETE (K walls)` | green `room_complete` wireframe | ≥3 distinct rooms COMPLETE |
| WP3 | Map merge | `[Atlas::MergeMapPair] Merged map X into map Y fused N rooms` | rooms/maps fused, tags persist | ≥1 successful merge |

The Alpha MAVLink flight is **not modified** during verification — only
vs_graphs behaviour is observed.

## 2. Manual reference flow

Run the integrated launcher directly to observe the flight interactively:

```bash
./scripts/launch_test.sh earth_afternoon office_clean/office_clean alpha
```

This opens a 4-pane tmux session (Gazebo, PX4 SITL, scene spawn, ROS2 offboard
which runs `vs_graphs`). Watch for the log lines listed above, and use RViz to
inspect the green `room_complete` markers and room tags.

## 3. Automated harness

The harness automates building, running, log capture, and the PASS/FAIL report:

```bash
# Default run: build vs_graphs, launch the office mission, capture for 25 min
src/visual_sgraphs/scripts/run_sim_verification.sh

# Skip the build and allow a longer flight
src/visual_sgraphs/scripts/run_sim_verification.sh --no-build -n 30

# Keep the tmux session alive after the report (for manual RViz inspection)
src/visual_sgraphs/scripts/run_sim_verification.sh --keep-session
```

Arguments: `-w|--world`, `-s|--scene`, `-m|--model`, `-t|--session`,
`-n|--minutes`, `--no-build`, `--keep-session`.

The harness exits `0` only when all acceptance criteria are met and `1`
otherwise. Internally it builds its own 4-pane tmux session that mirrors the
commands of `scripts/launch_test.sh` (Gazebo, PX4 SITL, scene spawn, offboard
`ros2 launch alpha`), so it can run unattended/headless without relying on the
interactive attach-based reference launcher.

## 4. Log collection convention

Every run writes to a timestamped directory:

```text
/tmp/vsgraphs_test_run_<YYYYmmdd_HHMMSS>/
├── pane0.log ... pane3.log   # per-pane output (vs_graphs is on the offboard pane)
└── VERIFICATION_REPORT.txt   # WP1/WP2/WP3 PASS/FAIL summary
```

Manual grep equivalent:

```bash
grep -E "(Map restart detected|Matched room|Merged map|boundary=COMPLETE|boundary=INCOMPLETE|Fused duplicate)" \
    /tmp/vsgraphs_test_run_*/pane*.log
```

Monitor a live run manually with:

```bash
tmux capture-pane -t sgcose:0.3 -p | tail -50   # ROS2 / vs_graphs pane
```

## 5. Expected log messages

| Message | Meaning |
|---------|---------|
| `[System] Map restart detected (mapId: 1 -> 2)` | Tracking was lost and a new map was created |
| `[Atlas] Exported room context: 4 rooms (mapId: 1)` | Prior-map room geometry snapshotted before the map is stranded |
| `[Atlas] Matched room 5 in new map, tagged with identity "room_3" (prior room 3, dist=0.42 m)` | New-map room assigned a prior-map identity (WP1) |
| `[SemMgr] Room#5 boundary=COMPLETE (4 walls)` | Room boundary closed per `boundary_topology` (WP2) |
| `[SemMgr] Room#5 boundary=INCOMPLETE (2 walls)` | Too few / non-closing walls for a complete boundary |
| `[Atlas::MergeMapPair] Merged map 2 into map 1 fused 4 rooms into current map.` | Two tagged maps merged into one (WP3) |

## 6. Iteration guidance

If the acceptance criteria are not met:

| WP | Likely issue | Adjustment |
|----|--------------|------------|
| WP1 | Room matching threshold too tight | Relax `kRoomContextMatchThreshold_m` in `core/include/Atlas.h` |
| WP2 | Too few rooms COMPLETE | Adjust `boundary_topology.maximum_corner_gap` / `minimum_enclosed_area` in `config/system_params.yaml` |
| WP3 | No map restart occurred | Force a tracking loss: fast camera pan, or temporarily occlude the RGB-D sensor in Gazebo |

Record each iteration (log directory timestamp + report) so the flight history
is traceable.

## 7. Notes

- The simulation does **not** pause on errors — always capture the full log.
- `colcon test --packages-select vs_graphs` has no effect until unit tests are
  added; the SITL simulation is the authoritative verification.
- Build the package first if `--no-build` is not used:
  `colcon build --packages-select vs_graphs --cmake-args -DCMAKE_BUILD_TYPE=Release`
