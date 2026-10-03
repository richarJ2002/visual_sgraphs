/*!
 * @file            test_FaultInjection.cpp
 *
 * @brief           Unit tests for the fault-injection registry that failure
 *                  tests use (FaultInjection).
 */

/*
 * Fault-injection example: inject a failure, verify the status path, verify
 * the disabled path. Uses a local strict-status-shaped probe function so no
 * production signature is touched. Test-only; ROS/Gazebo-free.
 */

#include "Semantic/FaultInjection.h"

#include <gtest/gtest.h>

#include <cstdint>

namespace
{

enum class ProbeStatus : std::uint8_t
{
    PROBE_STATUS_SUCCESS          = 0U,
    PROBE_STATUS_INJECTED_FAILURE = 1U
};

ProbeStatus DoFallibleWork()
{
    if (VS_GRAPHS_FAULT_CHECK("DoFallibleWork"))
    {
        return ProbeStatus::PROBE_STATUS_INJECTED_FAILURE;
    }
    return ProbeStatus::PROBE_STATUS_SUCCESS;
}

bool DoBoolWork()
{
    VS_GRAPHS_FAULT_INJECT("DoBoolWork");
    return true;
}

} // namespace

/*!
 * @brief        Checks that an injected fault makes the probe function return
 *               its injected-failure status; skipped when fault injection is
 *               compiled out.
 */
TEST(FaultInjection, InjectedFailureTakesStatusPath)
{
#ifdef VS_GRAPHS_ENABLE_FAULT_INJECTION
    const ::vs_graphs::testing::ScopedFault scopedFault("DoFallibleWork",
                                                        []() { return true; });
    EXPECT_EQ(DoFallibleWork(), ProbeStatus::PROBE_STATUS_INJECTED_FAILURE);
#else
    GTEST_SKIP() << "fault injection disabled";
#endif
}

/*!
 * @brief        Checks that, with no fault armed, the probe functions take
 *               their normal success path.
 */
TEST(FaultInjection, DisabledPathSucceeds)
{
    ::vs_graphs::testing::clearFaults();
    EXPECT_EQ(DoFallibleWork(), ProbeStatus::PROBE_STATUS_SUCCESS);
    EXPECT_TRUE(DoBoolWork());
}
