/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 *
 * Test-only failure-hook skeleton (design:
 * `src/visual_sgraphs/docs/design/fault_injection_api.md`). Test-only
 * failure hooks for strict-status conversion. This TU ships only in
 * the `test_FaultInjection` target; it is never linked into the production
 * library, and no production signature is changed.
 *
 * Zero-cost-when-disabled: with `VS_GRAPHS_ENABLE_FAULT_INJECTION` unset,
 * `VS_GRAPHS_FAULT_INJECT` expands to `((void)0)` (no branch, no TLS
 * access, no call) and `VS_GRAPHS_FAULT_CHECK` folds to `false`.
 */

#ifndef VS_GRAPHS_CORE_SEMANTIC_FAULT_INJECTION_H
#define VS_GRAPHS_CORE_SEMANTIC_FAULT_INJECTION_H

#include <functional>
#include <string>
#include <unordered_map>

namespace vs_graphs
{
namespace testing
{

/*!
 * @brief           Fault action for one named probe.
 *
 *                  Returns true when the fault fires (caller takes its
 *                  error path); returns false to pass through.
 */
using FaultAction = std::function<bool()>;

/*!
 * @brief           Returns the calling thread's fault registry.
 *
 *                  Thread-local so parallel gtests and worker threads do
 *                  not cross-trigger. Test-only; production code never
 *                  calls this.
 */
std::unordered_map<std::string, FaultAction> &FaultRegistry();

/*!
 * @brief           Arms one named probe for the calling thread.
 *
 * @param[in]       name_in
 *                  Probe name (e.g. "Map::AddKeyFrame").
 * @param[in]       action_in
 *                  Action evaluated when the probe fires.
 */
void RegisterFault(const std::string &name_in, FaultAction action_in);

/*!
 * @brief           Disarms all probes for the calling thread.
 */
void ClearFaults();

/*!
 * @brief           Fires the named probe when armed.
 *
 * @param[in]       name_in
 *                  Probe name to look up.
 *
 * @return          True when a registered action fired; false otherwise.
 */
bool CheckFault(const std::string &name_in);

/*!
 * @brief           RAII guard arming one probe for a scope.
 *
 *                  Registers on construction, clears the calling thread's
 *                  registry on destruction (exception-safe).
 */
class ScopedFault
{
  public:
    /*!
     * @brief           Arms the named probe.
     *
     * @param[in]       name_in
     *                  Probe name to arm.
     * @param[in]       action_in
     *                  Action evaluated when the probe fires.
     */
    ScopedFault(const std::string &name_in, FaultAction action_in) :
        name(name_in)
    {
        RegisterFault(name_in, action_in);
    }

    /*!
     * @brief           Disarms all probes for the calling thread.
     */
    ~ScopedFault()
    {
        ClearFaults();
    }

    ScopedFault(const ScopedFault &other_in)            = delete;
    ScopedFault &operator=(const ScopedFault &other_in) = delete;
    ScopedFault(ScopedFault &&other_in)                 = default;
    ScopedFault &operator=(ScopedFault &&other_in)      = default;

  private:
    /*!
     * @brief           Armed probe name (documentation only; clearing is
     *                  registry-wide).
     */
    std::string name;
};

} /* namespace testing */
} /* namespace vs_graphs */

#ifdef VS_GRAPHS_ENABLE_FAULT_INJECTION
#define VS_GRAPHS_FAULT_INJECT(name_in)                                        \
    do                                                                         \
    {                                                                          \
        if (::vs_graphs::testing::CheckFault(name_in))                         \
        {                                                                      \
            return false;                                                      \
        }                                                                      \
    }                                                                          \
    while (0)
#define VS_GRAPHS_FAULT_CHECK(name_in)                                         \
    (::vs_graphs::testing::CheckFault(name_in))
#else
#define VS_GRAPHS_FAULT_INJECT(name_in) ((void)0)
#define VS_GRAPHS_FAULT_CHECK(name_in)  (false)
#endif

#endif /* VS_GRAPHS_CORE_SEMANTIC_FAULT_INJECTION_H */
