# Fault-Injection API — WP-00.8.3 Design (Approval Pending)

- **Goal:** Deterministic failure tests for WP-04 strict-status conversion
  (every `_SUCCESS`/`_FAILURE` path exercised without hardware/sim).
- **Location (proposed):** `core/include/Utils.h` (or new
  `core/include/FaultInjection.h` included by `Utils.h`). No production
  behaviour change; header-only, test-only hooks.
- **Toggle:** `VS_GRAPHS_ENABLE_FAULT_INJECTION` (CMake option, default
  `OFF`). Production builds compile hooks out entirely. Test builds pass
  `-DVS_GRAPHS_ENABLE_FAULT_INJECTION=1` for the fault-injection gtests
  only (same pattern as `VS_GRAPHS_ENABLE_ATLAS_LOCK_ORDER_TEST_HOOK` in
  `CMakeLists.txt`).
- **Macro:** `VS_GRAPHS_FAULT_INJECT(name_in)` — single probe point.
  Disabled build expands to `((void)0)` (zero cost: no branch, no TLS
  access, no function call; compiler eliminates it). Enabled build expands
  to a thread-local registry lookup + optional early-return/throw per the
  registered action.
- **Thread-local registry (enabled only):**
  ```cpp
  #ifdef VS_GRAPHS_ENABLE_FAULT_INJECTION
  #include <functional>
  #include <string>
  #include <unordered_map>
  namespace vs_graphs::testing
  {
  using FaultAction = std::function<bool()>;
  // Returns false = no fault; true = fault consumed (caller returns error).
  inline std::unordered_map<std::string, FaultAction> &
  FaultRegistry()
  {
      thread_local std::unordered_map<std::string, FaultAction> registry;
      return registry;
  }
  inline void RegisterFault(const std::string &name_in, FaultAction action_in);
  inline void ClearFaults();
  inline bool CheckFault(const std::string &name_in);
  } // namespace vs_graphs::testing
  #define VS_GRAPHS_FAULT_INJECT(name_in)                                      \
      do                                                                       \
      {                                                                        \
          if (::vs_graphs::testing::CheckFault(name_in))                        \
          {                                                                    \
              return false; /* or status enum per WP-04 signature */            \
          }                                                                    \
      } while (0)
  #else
  #define VS_GRAPHS_FAULT_INJECT(name_in) ((void)0)
  #endif
  ```
  Registry is `thread_local` so parallel gtests and worker threads do not
  cross-trigger. `RegisterFault`/`ClearFaults` are test-only helpers;
  production code never calls them. Scoped guard (`ScopedFault`) registers
  on construction and clears on destruction for exception safety.
- **Probe placement (WP-03.8, not here):** Fallible allocation/IO/BAsolver
  entry points only (e.g. `Map::AddKeyFrame`, `Atlas::CreateNewMap`,
  optimizer entry). No probes in hot loops, no probes in `serialize()`
  (ABI-frozen), no probes in ROS callbacks/timers (exempt list).
- **gtest example:**
  ```cpp
  #include <gtest/gtest.h>
  #include "Utils.h"
  TEST(FaultInjection, MapAddKeyFrameFailsCleanly)
  {
  #ifdef VS_GRAPHS_ENABLE_FAULT_INJECTION
      ::vs_graphs::testing::RegisterFault(
          "Map::AddKeyFrame", []() { return true; });
      Map map;
      EXPECT_FALSE(map.AddKeyFrameChecked(nullptr)); // strict-status wrapper
      ::vs_graphs::testing::ClearFaults();
      EXPECT_NO_THROW(map.GetAllKeyFrames());
  #else
      GTEST_SKIP() << "fault injection disabled";
  #endif
  }
  ```
- **Zero-cost-when-disabled analysis:** With the toggle `OFF`, each probe
  is `((void)0)`: no code generated (verified via `-Wunused-macros` clean
  + disassembly spot-check in WP-03.8), no TLS, no branch, no size change
  to hot functions, no ABI change (no new members, no virtuals, no layout
  change). Enabled builds pay one `thread_local` map lookup per probe hit
  only — acceptable for failure-path gtests, never shipped.
- **Approval:** Pending maintainer approval before WP-03/WP-04. No
  implementation in WP-00.8.3 (design only).
- **Status:** Draft; gates WP-03.8 implementation.
