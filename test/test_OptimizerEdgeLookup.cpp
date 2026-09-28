/*!
 * @file test_OptimizerEdgeLookup.cpp
 * @brief B4 regression coverage: edgeSourceKeyFrame() must not read past the
 *        end of a shorter parallel edge-keyframe vector.
 */

#include "OptimizerEdgeLookup.h"

#include <gtest/gtest.h>

#include <vector>

namespace vs_graphs
{
namespace core
{

TEST(OptimizerEdgeLookup, ReturnsThePointerAtAnInBoundsIndex)
{
    int a = 1, b = 2, c = 3;

    std::vector<int *> edgeKeyFrames = {&a, &b, &c};

    int *p_keyFrame = nullptr;
    ASSERT_EQ(edgeSourceKeyFrame(
                  edgeKeyFrames, static_cast<std::size_t>(0), p_keyFrame),
              OptimizerEdgeLookupStatus::OPTIMIZER_EDGE_LOOKUP_STATUS_SUCCESS);
    EXPECT_EQ(p_keyFrame, &a);
    ASSERT_EQ(edgeSourceKeyFrame(
                  edgeKeyFrames, static_cast<std::size_t>(2), p_keyFrame),
              OptimizerEdgeLookupStatus::OPTIMIZER_EDGE_LOOKUP_STATUS_SUCCESS);
    EXPECT_EQ(p_keyFrame, &c);
}

TEST(OptimizerEdgeLookup, ReturnsNullptrForAnOutOfBoundsIndex)
{
    /* Reproduces B4: a loop bounded by one edge vector's length (e.g.
     * vpEdgesStereo) indexing into a shorter parallel vector (e.g.
     * vpEdgeKFMono) used to read out of bounds. */
    int a = 1;

    std::vector<int *> shortEdgeKeyFrames = {&a};

    int *p_keyFrame = &a;
    ASSERT_EQ(edgeSourceKeyFrame(
                  shortEdgeKeyFrames, static_cast<std::size_t>(1), p_keyFrame),
              OptimizerEdgeLookupStatus::OPTIMIZER_EDGE_LOOKUP_STATUS_SUCCESS);
    EXPECT_EQ(p_keyFrame, nullptr);
    p_keyFrame = &a;
    ASSERT_EQ(
        edgeSourceKeyFrame(
            shortEdgeKeyFrames, static_cast<std::size_t>(100), p_keyFrame),
        OptimizerEdgeLookupStatus::OPTIMIZER_EDGE_LOOKUP_STATUS_SUCCESS);
    EXPECT_EQ(p_keyFrame, nullptr);
}

TEST(OptimizerEdgeLookup, ReturnsNullptrForAnEmptyVector)
{
    std::vector<int *> emptyEdgeKeyFrames;

    int  a          = 1;
    int *p_keyFrame = &a;
    ASSERT_EQ(edgeSourceKeyFrame(
                  emptyEdgeKeyFrames, static_cast<std::size_t>(0), p_keyFrame),
              OptimizerEdgeLookupStatus::OPTIMIZER_EDGE_LOOKUP_STATUS_SUCCESS);
    EXPECT_EQ(p_keyFrame, nullptr);
}

} // namespace core
} // namespace vs_graphs
