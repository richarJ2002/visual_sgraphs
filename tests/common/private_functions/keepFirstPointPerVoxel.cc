/*!
 * @file            keepFirstPointPerVoxel.cc
 *
 * @brief           Tests keepFirstPointPerVoxel: one point per voxel, the
 *                  first one kept, cells split at floor() across zero, and
 *                  non-finite or off-grid points dropped.
 */

/* Matching Declaration Include */
#include "common/private_functions/private_functions.h"

/* C++ Standard Library Includes */
#include <limits>
#include <vector>

/* External Library Includes */
#include <Eigen/Core>
#include <gtest/gtest.h>

namespace
{

/* Voxel edge as float, so the test follows the production constant. */
const float VOXEL_SIZE_M =
    static_cast<float>(vs_graphs::common::MAP_POINT_ARCHIVE_VOXEL_SIZE_M);

} /* namespace */

TEST(KeepFirstPointPerVoxel, KeepsTheFirstPointOfEachVoxel)
{
    const std::vector<Eigen::Vector3f> pointPositions_m = {
        /* Voxel (0, 0, 0): first point, kept. */
        {0.1f * VOXEL_SIZE_M, 0.1f * VOXEL_SIZE_M, 0.1f * VOXEL_SIZE_M},
        /* Same voxel: dropped. */
        {0.9f * VOXEL_SIZE_M, 0.5f * VOXEL_SIZE_M, 0.2f * VOXEL_SIZE_M},
        /* Voxel (1, 0, 0): kept. */
        {1.1f * VOXEL_SIZE_M, 0.1f * VOXEL_SIZE_M, 0.1f * VOXEL_SIZE_M},
        /* Voxel (0, 1, 0): kept; the axes do not alias in the key. */
        {0.1f * VOXEL_SIZE_M, 1.1f * VOXEL_SIZE_M, 0.1f * VOXEL_SIZE_M},
        /* Voxel (-1, 0, 0): floor(), not truncation toward zero; kept. */
        {-0.1f * VOXEL_SIZE_M, 0.1f * VOXEL_SIZE_M, 0.1f * VOXEL_SIZE_M},
        /* Same voxel (-1, 0, 0): dropped. */
        {-0.9f * VOXEL_SIZE_M, 0.2f * VOXEL_SIZE_M, 0.3f * VOXEL_SIZE_M},
        /* Exact repeat of the first point: dropped. */
        {0.1f * VOXEL_SIZE_M, 0.1f * VOXEL_SIZE_M, 0.1f * VOXEL_SIZE_M},
    };
    const std::vector<Eigen::Vector3f> expectedPositions_m = {
        pointPositions_m[0],
        pointPositions_m[2],
        pointPositions_m[3],
        pointPositions_m[4],
    };

    EXPECT_EQ(vs_graphs::common::keepFirstPointPerVoxel(pointPositions_m),
              expectedPositions_m);
}

TEST(KeepFirstPointPerVoxel, DropsNonFiniteAndOffGridPoints)
{
    const float notANumber = std::numeric_limits<float>::quiet_NaN();
    const float infinity_m = std::numeric_limits<float>::infinity();

    /* Voxel index 1e6 lies inside the (-2^20, 2^20) grid, 2e6 outside. */
    const float insideGrid_m  = 1.0e6f * VOXEL_SIZE_M;
    const float outsideGrid_m = 2.0e6f * VOXEL_SIZE_M;

    const std::vector<Eigen::Vector3f> pointPositions_m = {
        {notANumber, 0.0f, 0.0f},
        {0.0f, infinity_m, 0.0f},
        {0.0f, 0.0f, -infinity_m},
        {outsideGrid_m, 0.0f, 0.0f},
        {insideGrid_m, 0.0f, 0.0f},
        {-insideGrid_m, 0.0f, 0.0f},
    };
    const std::vector<Eigen::Vector3f> expectedPositions_m = {
        pointPositions_m[4],
        pointPositions_m[5],
    };

    EXPECT_EQ(vs_graphs::common::keepFirstPointPerVoxel(pointPositions_m),
              expectedPositions_m);
}
