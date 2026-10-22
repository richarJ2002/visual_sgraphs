/*!
 * @file            keepFirstPointPerVoxel.cc
 *
 * @brief           Implements keepFirstPointPerVoxel declared in
 *                  private_functions.h.
 */

/* Matching Declaration Include */
#include "common/private_functions/private_functions.h"

/* C++ Standard Library Includes */
#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <vector>

/* External Library Includes */
#include <Eigen/Core>

namespace vs_graphs::common
{

std::vector<Eigen::Vector3f> keepFirstPointPerVoxel(
    const std::vector<Eigen::Vector3f> &pointPositions_m_in)
{
    /*!
     * Three 21-bit voxel indices pack into one 64-bit key. Each index is
     * offset by 2^20, so every accepted index is a positive 21-bit value.
     * ponytail: this caps the grid at about 104 km from the origin; use a
     * key of three full integers if a map ever spans more.
     */
    constexpr unsigned int VOXEL_INDEX_BITS   = 21U;
    constexpr std::int64_t VOXEL_INDEX_OFFSET = std::int64_t{1} << 20U;

    std::vector<Eigen::Vector3f>      keptPositions_m;
    std::unordered_set<std::uint64_t> occupiedVoxelKeys;
    for (const Eigen::Vector3f &pointPosition_m : pointPositions_m_in)
    {
        std::uint64_t voxelKey     = 0U;
        bool          isInsideGrid = true;
        for (Eigen::Index axisIndex = 0; axisIndex < 3; axisIndex++)
        {
            const double voxelIndex =
                std::floor(static_cast<double>(pointPosition_m[axisIndex]) /
                           MAP_POINT_ARCHIVE_VOXEL_SIZE_M);

            /* The comparison is also false for NaN and infinity. */
            if (!(std::abs(voxelIndex) <
                  static_cast<double>(VOXEL_INDEX_OFFSET)))
            {
                isInsideGrid = false;
                break;
            }
            voxelKey =
                (voxelKey << VOXEL_INDEX_BITS) |
                static_cast<std::uint64_t>(
                    static_cast<std::int64_t>(voxelIndex) + VOXEL_INDEX_OFFSET);
        }

        /* insert() reports false when an earlier point owns the voxel. */
        if (isInsideGrid && occupiedVoxelKeys.insert(voxelKey).second)
        {
            keptPositions_m.push_back(pointPosition_m);
        }
    }

    return keptPositions_m;
}

} /* namespace vs_graphs::common */
