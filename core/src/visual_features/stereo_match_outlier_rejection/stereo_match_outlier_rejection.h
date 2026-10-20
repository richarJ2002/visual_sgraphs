/*!
 * @file            stereo_match_outlier_rejection.h
 *
 * @brief           Public interface of the stereo_match_outlier_rejection
 *                  module.
 */

#ifndef VS_GRAPHS_CORE_SRC_STEREO_MATCH_OUTLIER_REJECTION_STEREO_MATCH_OUTLIER_REJECTION_H
#define VS_GRAPHS_CORE_SRC_STEREO_MATCH_OUTLIER_REJECTION_STEREO_MATCH_OUTLIER_REJECTION_H

#include "visual_features/stereo_match_outlier_rejection/objects/StereoMatchOutlierRejectionStatusEnum.h"

#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace stereo_match_outlier_rejection
{

/*!
 * @brief           Rejects stereo matches whose ORB descriptor distance is far
 *                  from the median of all accepted matches (Frame's original
 *                  1.5 * 1.4 * median threshold).
 *
 * @param[in,out]   distanceIndices_inout
 *                  (distance, left-keypoint-index) pairs for every keypoint
 *                  that received a stereo match. Sorted in place.
 *
 * @param[in,out]   mvuRight_inout
 *                  Per-left-keypoint matched right-image u coordinate; rejected
 *                  entries are set to -1.
 *
 * @param[in,out]   depths_inout
 *                  Per-left-keypoint depth; rejected entries are set to -1.
 *
 * @return          STEREO_MATCH_OUTLIER_REJECTION_STATUS_SUCCESS.
 *
 * @note            No-ops when distanceIndices_inout is empty -- a frame with
 *                  zero qualifying stereo matches has nothing to compute a
 *                  median from, and must not read an empty vector.
 */
[[nodiscard]] StereoMatchOutlierRejectionStatus rejectOutlierStereoMatches(
    std::vector<std::pair<int, int>> &distanceIndices_inout,
    std::vector<float>               &mvuRight_inout,
    std::vector<float>               &depths_inout);

} /* namespace stereo_match_outlier_rejection */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_STEREO_MATCH_OUTLIER_REJECTION_STEREO_MATCH_OUTLIER_REJECTION_H \
        */
