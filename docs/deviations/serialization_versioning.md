# Serialization Versioning — WP-00.8.1 Decision

- **Rule:** Boost `serialize(Archive&, unsigned)` signatures are ABI-frozen
  (see plan §1 exemptions; `Map.h:61`, `Atlas.h:102`). No signature change
  in WP-00.8.1.
- **Version bump:** None. Existing `version` parameters are unused and left
  as-is. A future rename (WP-01A namespace `ORB_SLAM3` → `vs_graphs::core`)
  will require a version bump + migration note; that bump is explicitly
  out of scope here.
- **Cross-version support:** None. Archives are self-consistent only:
  HEAD writes, HEAD reads. No guarantee across branches, no forward/backward
  compat, no long-term `.osa` stability.
- **Self-consistency contract:** `test_SerializationRoundTrip` (gtest,
  ROS/Gazebo-free) saves each Boost site to a Boost archive (binary
  in-memory; text file in `/tmp` for Bias/Map/Atlas) and loads back,
  comparing deep value:
  - POD/containers by value; `set`/`map` via sorted comparison helpers.
  - Pointer identity via Boost tracking IDs (polymorphic
    `vector<GeometricCamera*>` duplicate-pointer test).
  - `mutex`/`atomic`/thread handles and unsaved raw pointers (`tvr`,
    `mpCamera`, `mpORBvocabulary`, viewer/DB pointers) are SKIPPED by
    design; post-load objects are asserted default-constructed by
    exercising them (getters, `SetPose`, `IntegrateNewMeasurement`,
    `clearMap`, `GetAllMaps` — no deadlock, no crash).
  - Deterministic ordering enforced via sorted helpers.
- **PreSave/PostLoad note:** `Map`/`Atlas` serialize backup vectors only
  (`mvpBackup*`), not live sets. Tests call `PreSave` where needed to
  populate backups deterministically (`MapPoint`, `KeyFrame`,
  `KeyFrameDatabase`); `Atlas` empty-map `PreSave` retires the map by
  design, so the file test asserts the loaded camera/init-ID state rather
  than the transient active set. `Map::mbBad`/`Atlas` atomics are not
  serialized and intentionally not compared.
- **Verification:** `test_SerializationRoundTrip` GREEN on HEAD required
  before WP-01A.
- **Approver:** Pending maintainer sign-off.
- **Status:** Referenceable; gates WP-01A.
