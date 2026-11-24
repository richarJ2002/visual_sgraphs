/*!
 * @file            private_functions.h
 *
 * @brief           Declares the helpers used only inside the common package.
 */

#ifndef VS_GRAPHS_COMMON_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_COMMON_PRIVATE_FUNCTIONS_H

/* C++ Standard Library Includes */
#include <vector>

/* External Library Includes */
#include <Eigen/Core>

namespace vs_graphs::common
{

/*!
 * @brief           Edge length of the cubic voxels that thin the map points
 *                  archived with each S-Graph snapshot, in metres.
 */
inline constexpr double MAP_POINT_ARCHIVE_VOXEL_SIZE_M = 0.1;

/*!
 * @brief           Keeps the first point of each occupied voxel of an
 *                  axis-aligned grid with MAP_POINT_ARCHIVE_VOXEL_SIZE_M
 *                  cells, one corner at the origin of the points' frame.
 *
 *                  A point lies in voxel floor(coordinate / size) on each
 *                  axis. Non-finite points, and points whose voxel index on
 *                  some axis is not within (-2^20, 2^20) (about 104 km from
 *                  the origin at 0.1 m), are dropped.
 *
 * @param[in]       pointPositions_m_in
 *                  Point positions in any one frame, metres.
 *
 * @return          The kept points in input order, in the same frame and
 *                  units.
 */
[[nodiscard]] std::vector<Eigen::Vector3f> keepFirstPointPerVoxel(
    const std::vector<Eigen::Vector3f> &pointPositions_m_in);

} /* namespace vs_graphs::common */

#endif /* VS_GRAPHS_COMMON_PRIVATE_FUNCTIONS_H */
