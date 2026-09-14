# visual_sgraphs Test Suite

## 1. How to run the suites

From the workspace root `/opt/rnd/SgcOse`:

- **Run all tests** (BUILD_TESTING=On):
  ```bash
  . /opt/ros/jazzy/setup.bash
  colcon test --packages-select vs_graphs --event-handlers console_cohesion+ 2>&1 | tail -30
  ```

- **Run a single test binary** via ctest (after `colcon test`):
  ```bash
  cd /opt/rnd/SgcOse/build/vs_graphs
  ctest --output-on-failure -R <test_name>
  ```
  or run the binary directly:
  ```bash
  ./lib/vs_graphs/test_<test_name>
  ```

- **With overlay build** (Release mode):
  ```bash
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  . install/setup.bash
  colcon test --packages-select vs_graphs --event-handlers console_cohesion+
  ```

## 2. Test binaries (registered in `src/visual_sgraphs/CMakeLists.txt`)

Total: 31 gtest targets** + 1 non-test executable. Each entry verified to map to an existing source file on disk. No stale entries discovered.

| Test Binary | Source File(s) | Registration Line |
|---|---|---|
| `test_unit_skeleton` | `test/test_unit_skeleton.cpp`, `core/src/Semantic/RoomTracker.cc` | line 533 |
| `test_RoomTracker` | `test/test_RoomTracker.cpp`, `core/src/Semantic/RoomTracker.cc` | line 538 |
| `test_CandidateGen` | `test/test_CandidateGen.cpp`, `core/src/Semantic/SemanticCandidates.cc` | line 543 |
| `test_atlas_lock_order` | `test/test_atlas_lock_order.cpp` | line 548 |
| `test_RoomContextPersist` | `test/test_RoomContextPersist.cpp` | line 555 |
| `test_GeometricVerify` | `test/test_GeometricVerify.cpp` | line 562 |
| `test_room_tracker_integration` | `test/test_room_tracker_integration.cpp` | line 569 |
| `test_verification_wiring_integration` | `test/test_verification_wiring_integration.cpp` | line 631 |
| `test_WallAdmission` | `test/test_WallAdmission.cpp` | line 644 |
| `test_SemanticBootstrapPhase1` | `test/test_SemanticBootstrapPhase1.cpp` | line 654 |
| `test_WallPairing` | `test/test_WallPairing.cpp` | line 667 |
| `test_FloorFlatness` | `test/test_FloorFlatness.cpp` | line 680 |
| `test_RoomCreationBudget` | `test/test_RoomCreationBudget.cpp` | line 694 |
| `test_BoundaryLoopOutlierPruning` | `test/test_BoundaryLoopOutlierPruning.cpp` | line 705 |
| `test_RoomObservationGaps` | `test/test_RoomObservationGaps.cpp` | line 718 |
| `test_rgbd_observability` | `test/test_rgbd_observability.cpp`, `src/RgbdObservability.cc`, `src/RgbdAllPointsCadence.cc`, `core/src/ResetCause.cc` | line 728 |
| `test_SparseClusterVerdict` | `test/test_SparseClusterVerdict.cpp`, `src/classifySparseMarker.cc` | line 739 |
| `test_GroundPlaneFilter` | `test/test_GroundPlaneFilter.cpp` | line 747 |
| `test_StereoMatchOutlierRejection` | `test/test_StereoMatchOutlierRejection.cpp`, `core/src/StereoMatchOutlierRejection.cc` | line 760 |
| `test_OptimizerEdgeLookup` | `test/test_OptimizerEdgeLookup.cpp` | line 770 |
| `test_SemanticFixtures` | `test/test_SemanticFixtures.cpp`, `test/SemanticFixtures.cc` | line 782 |
| `test_GlobalWallMetrics` | `test/test_GlobalWallMetrics.cpp`, `test/GlobalWallMetrics.cc` | line 797 |
| `test_SemanticGraphSnapshot` | `test/test_SemanticGraphSnapshot.cpp`, `test/SemanticFixtures.cc`, `test/SemanticGraphSnapshotTestHelpers/findRoomRecord.cc`, `test/SemanticGraphSnapshotTestHelpers/findWallRecord.cc`, `test/SemanticGraphSnapshotTestHelpers/findPassageRecord.cc`, `test/SemanticGraphSnapshotTestHelpers/findMapSnapshot.cc` | line 809 |
| `test_SemanticAxiomEvaluator` | `test/test_SemanticAxiomEvaluator.cpp`, `test/SemanticFixtures.cc` | line 832 |
| `test_SemanticCanonicalSerialization` | `test/test_SemanticCanonicalSerialization.cpp`, `test/SemanticFixtures.cc` | line 844 |
| `test_SemanticReportCache` | `test/test_SemanticReportCache.cpp`, `test/SemanticFixtures.cc` | line 854 |
| `test_MissionHealthTopologyJson` | `test/test_MissionHealthTopologyJson.cpp` | line 864 |
| `test_SemanticDiagnostics` | `test/test_SemanticDiagnostics.cpp` | line 873 |
| `test_LegacyCaptureReplay` | `test/test_LegacyCaptureReplay.cpp`, `test/LegacyCaptureReplay.cc` | line 883 |
| `test_PassageTraversalRepro` | `test/test_PassageTraversalRepro.cpp` | line 891 |
| `test_ConsecutiveMapMatcher` | `test/test_ConsecutiveMapMatcher.cpp` | line 902 |

**Non-test executable:** `legacy_capture_replay` — `test/legacy_capture_replay_main.cc`, `test/LegacyCaptureReplay.cc` (line 898, not a gtest).

**Verification:** All 31 test binaries' source files were confirmed present on disk via `find`/`ls`. No registered test binary was missing its source file. The full list of 32 `test_*.cpp`/`.cc` files in `src/visual_sgraphs/test/` all have corresponding registrations or are helper sources included by registered tests.

## 3. How to add a new test

1. **Create the source file** at `src/visual_sgraphs/test/test_<name>.cpp` (or `.cc` if it needs legacy compatibility). Follow the existing `TEST()` pattern from gtest, e.g.:
   ```cpp
   TEST(TestSuite, CaseName) {
      // exercise the behavior under test
   }
   ```
2. **Register the test** in `src/visual_sgraphs/CMakeLists.txt` inside the `if(BUILD_TESTING)` block, after the existing entries, using one of these patterns:
   - `ament_add_gtest(test_<name> test/test_<name>.cpp)` — links the production library.
   - `ament_add_gtest(test_<name> test/test_<name>.cpp core/src/Semantic/RoomTracker.cc)` — adds a test-hook dependency.
   - `ament_add_gtest(test_<name> test/test_<name>.cpp src/RgbdObservability.cc ...)` — add extra source files as needed.
   - For library-linked tests, add a following `target_link_libraries(test_<name> ${PROJECT_NAME}_lib)` or `${PROJECT_NAME}_room_tracker_test_lib` as appropriate.
3. **Rebuild**: `colcon build --symlink-install` then `colcon test --packages-select vs_graphs`.

## 4. Sim-led workflow note

- **Lockstep launch**: `./scripts/launch_test.sh --lockstep --lockstep-slice 3.0 earth_afternoon office_clean/office_clean alpha` — starts a tmux session `sgcose` with four panes (Gazebo world, scene/model spawn, PX4 SITL + QGC, ROS overlay + alpha.launch.py). Lockstep keeps sim time and the controller synchronized.
- **Clean teardown**: detach with `Ctrl+B D`, then `tmux kill-session -t sgcose`. Verify all processes are gone (PID-checked):
  ```bash
  pgrep -af "gz sim|sim_lockstep_controller|ros_rgbd|px4"  # should return nothing
  ```
- **sgraph evidence**: after a run, `test_runs/<run_id>/` holds the sgraph time series under `output/sgraph/sgraph_*.json` and RViz screenshots (`rviz_*.png`) at the run root alongside the run log.
