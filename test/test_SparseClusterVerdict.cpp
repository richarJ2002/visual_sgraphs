/*!
 * @file            test_SparseClusterVerdict.cpp
 *
 * @brief           Tests deterministic sparse-graph marker classification.
 */

#include "../src/SparseClusterVerdict.h"

#include <cstddef>
#include <string>

#include <gtest/gtest.h>

namespace
{

using vs_graphs::sparse::classifySparseMarker;
using vs_graphs::sparse::SparseMarkerVerdict;

/* Type codes mirror visualization_msgs/msg/semantic::Marker.msg (LINE_LIST=5,
 * CUBE_LIST=6); the header under test deliberately avoids the ROS include. */
constexpr int TEST_CUBE_LIST_TYPE  = 6;
constexpr int TEST_LINE_LIST_TYPE  = 5;
constexpr int TEST_LINE_STRIP_TYPE = 4;

TEST(SparseClusterVerdictTest, IgnoresMarkersOutsideFreeSpaceContract)
{
    EXPECT_EQ(classifySparseMarker(TEST_LINE_STRIP_TYPE,
                                   "connected_vertices_0",
                                   40U,
                                   10U),
              SparseMarkerVerdict::SPARSE_MARKER_IGNORED);
    EXPECT_EQ(classifySparseMarker(TEST_CUBE_LIST_TYPE, "vertices", 40U, 10U),
              SparseMarkerVerdict::SPARSE_MARKER_IGNORED);
    EXPECT_EQ(classifySparseMarker(TEST_LINE_LIST_TYPE,
                                   "connected_edges_0",
                                   40U,
                                   10U),
              SparseMarkerVerdict::SPARSE_MARKER_IGNORED);
    EXPECT_EQ(classifySparseMarker(TEST_CUBE_LIST_TYPE, "", 40U, 10U),
              SparseMarkerVerdict::SPARSE_MARKER_IGNORED);
}

TEST(SparseClusterVerdictTest, RejectsEmptyMarkersByKind)
{
    EXPECT_EQ(classifySparseMarker(TEST_CUBE_LIST_TYPE,
                                   "connected_vertices_0",
                                   0U,
                                   10U),
              SparseMarkerVerdict::SPARSE_CLUSTER_EMPTY);
    EXPECT_EQ(classifySparseMarker(TEST_LINE_LIST_TYPE, "edges", 0U, 10U),
              SparseMarkerVerdict::SPARSE_EDGE_EMPTY);
}

TEST(SparseClusterVerdictTest, RejectsUndersizedClustersAtBoundary)
{
    EXPECT_EQ(classifySparseMarker(TEST_CUBE_LIST_TYPE,
                                   "connected_vertices_2",
                                   9U,
                                   10U),
              SparseMarkerVerdict::SPARSE_CLUSTER_UNDERSIZED);
    EXPECT_EQ(classifySparseMarker(TEST_CUBE_LIST_TYPE,
                                   "connected_vertices_2",
                                   10U,
                                   10U),
              SparseMarkerVerdict::SPARSE_CLUSTER_CANDIDATE);
    EXPECT_EQ(classifySparseMarker(TEST_CUBE_LIST_TYPE,
                                   "connected_vertices_2",
                                   11U,
                                   10U),
              SparseMarkerVerdict::SPARSE_CLUSTER_CANDIDATE);
}

TEST(SparseClusterVerdictTest, AdmitsEligibleMarkersAsCandidates)
{
    EXPECT_EQ(classifySparseMarker(TEST_CUBE_LIST_TYPE,
                                   "connected_vertices_0",
                                   46U,
                                   10U),
              SparseMarkerVerdict::SPARSE_CLUSTER_CANDIDATE);
    EXPECT_EQ(classifySparseMarker(TEST_LINE_LIST_TYPE, "edges", 248U, 10U),
              SparseMarkerVerdict::SPARSE_EDGE_CANDIDATE);
}

} /* namespace */
