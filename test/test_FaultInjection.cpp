/**
 * WP-03.8 skeleton example: inject a failure, verify the status path, verify
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

TEST(FaultInjection, DisabledPathSucceeds)
{
    ::vs_graphs::testing::ClearFaults();
    EXPECT_EQ(DoFallibleWork(), ProbeStatus::PROBE_STATUS_SUCCESS);
    EXPECT_TRUE(DoBoolWork());
}
