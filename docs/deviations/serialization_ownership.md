# Serialization Ownership — WP-00.8.2 Decision

- **Scope:** `Atlas::mspMaps` (`std::set<Map*>`) + `Map::mspKeyFrames`
  (`std::set<KeyFrame*>`) and sibling raw-pointer sets
  (`mspMapPoints`, `mvpBackup*` vectors). Prototypes compared:
  - (a) Raw borrowed + Boost tracking + docs (status quo).
  - (b) `shared_ptr`/`weak_ptr`.
  - (c) Stable-ID custom (e.g. `unordered_map<Id, unique_ptr<T>>` + ID
    fix-ups on load).
- **Method:** Reasoned micro-analysis using existing fixtures
  (`Atlas(0)` single-map, `Map` empty/small, `test_SerializationRoundTrip`
  light fixtures). No committed benchmark binary per task allowance
  (`/tmp` prototype not retained); numbers below are sizeof/complexity
  estimates, not wall-clock runs. Sign-off pending.
- **Size (64-bit):**
  - Raw `T*`: 8 B per pointer + `std::set` RB-tree node (~32 B) per element.
  - `shared_ptr<T>`: 16 B (ptr + control ptr) + control block (~32 B:
    refcounts + deleter + allocator) + same set node. ~2–3× per-element
    overhead vs raw for N = 1–3 maps, N = 10–100s keyframes.
  - `weak_ptr<T>`: 16 B + same control block (kept alive by weak refs).
  - Stable-ID: ID (4–8 B) + `unordered_map` node (~32 B + bucket overhead)
    + owned `unique_ptr` (8 B + heap object). Similar to (b) plus map
    overhead; extra ID→pointer table during load.
- **Time:**
  - (a) Deref O(1), no atomic. Boost tracking restores identity via
    pointer-ID table O(N) on load; `PreSave`/`PostLoad` ID fix-ups already
    O(N log N) sorted (`Atlas::PreSave` sorts `mvpBackupMaps`).
  - (b) Copy/inc/dec refcount = atomic RMW (~20–30 ns each); traversal
    pays double indirection. No benefit in single-threaded
    `PreSave`/`PostLoad` paths; adds contention on shared readers
    (`GetAllMaps`/`GetAllKeyFrames`).
  - (c) Lookup = hash + equality (~50–100 ns); load = two passes
    (create, then fix-up). Deterministic but rewrites every accessor.
- **Memory/lifetime:**
  - (a) Atlas owns `Map*` (deleted once at shutdown from union of
    active/bad/retired sets); `Map` does not own `KeyFrame*`/`MapPoint*`
    beyond its lifetime contract documented on `setStartingRoom`/
    `setFollowingMap`/`p_followingMap`. No cycles introduced.
  - (b) `shared_ptr` cycles (`KeyFrame`↔`MapPoint` observations,
    `Atlas`↔`Map` current-map link) require disciplined `weak_ptr`
    breakage; otherwise leaks. Migration touches every accessor.
  - (c) Clear ownership (`unique_ptr`) but requires ID stability across
    merge/fusion (`observeRoomIdentity` pattern) and rewrites
    `Get*ById` indexes.
- **Decision:** (a) first — least invasive, preserves ABI (frozen
  `serialize` signatures, no layout change to `mspMaps`/`mspKeyFrames`),
  zero overhead, matches existing `PreSave`/`PostLoad` + Boost tracking
  design. (b)/(c) only if (a) fails post-rename (e.g. tracking-ID
  breakage from `vs_graphs::core` rename, use-after-free in
  retired-map window, or ASan ownership error). Re-confirm at WP-04.5
  before any pointer-member conversion (plan §WP-04.5).
- **Verification:** `test_SerializationRoundTrip` polymorphic tracking test
  (`PolymorphicTrackingPreservesIdentity`) proves (a) preserves identity
  on HEAD. No new failure introduced.
- **Approver:** Pending maintainer sign-off (sign-off pending per task).
- **Status:** Referenceable; gates WP-01A.
