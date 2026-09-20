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

    EXPECT_EQ(edgeSourceKeyFrame(edgeKeyFrames, static_cast<std::size_t>(0)),
              &a);
    EXPECT_EQ(edgeSourceKeyFrame(edgeKeyFrames, static_cast<std::size_t>(2)),
              &c);
}

TEST(OptimizerEdgeLookup, ReturnsNullptrForAnOutOfBoundsIndex)
{
    /* Reproduces B4: a loop bounded by one edge vector's length (e.g.
     * vpEdgesStereo) indexing into a shorter parallel vector (e.g.
     * vpEdgeKFMono) used to read out of bounds. */
    int a = 1;

    std::vector<int *> shortEdgeKeyFrames = {&a};

    EXPECT_EQ(
        edgeSourceKeyFrame(shortEdgeKeyFrames, static_cast<std::size_t>(1)),
        nullptr);
    EXPECT_EQ(
        edgeSourceKeyFrame(shortEdgeKeyFrames, static_cast<std::size_t>(100)),
        nullptr);
}

TEST(OptimizerEdgeLookup, ReturnsNullptrForAnEmptyVector)
{
    std::vector<int *> emptyEdgeKeyFrames;

    EXPECT_EQ(
        edgeSourceKeyFrame(emptyEdgeKeyFrames, static_cast<std::size_t>(0)),
        nullptr);
}

} // namespace core
} // namespace vs_graphs
