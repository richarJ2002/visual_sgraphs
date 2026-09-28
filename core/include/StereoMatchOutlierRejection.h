/*!
 * @file StereoMatchOutlierRejection.h
 * @brief Declares the median-distance outlier rejection step used by
 *        Frame::computeStereoMatches(), isolated for direct unit testing.
 */

#ifndef VS_GRAPHS_CORE_STEREO_MATCH_OUTLIER_REJECTION_H
#define VS_GRAPHS_CORE_STEREO_MATCH_OUTLIER_REJECTION_H

#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{

/*!
 * Rejects stereo matches whose ORB descriptor distance is far from the
 * median of all accepted matches (Frame::computeStereoMatches()'s original
 * 1.5 * 1.4 * median threshold).
 *
 * @param[in,out] distanceIndices_inout  (distance, left-keypoint-index) pairs
 * for every keypoint that received a stereo match. Sorted in place.
 * @param[in,out] mvuRight_inout  Per-left-keypoint matched right-image u
 *                          coordinate; rejected entries are set to -1.
 * @param[in,out] depths_inout   Per-left-keypoint depth; rejected entries are
 * set to -1.
 *
 * @note No-ops when vDistIdx is empty -- a frame with zero qualifying stereo
 *       matches has nothing to compute a median from, and must not read
 *       vDistIdx[vDistIdx.size() / 2] (out of bounds on an empty vector).
 */
void rejectOutlierStereoMatches(
    std::vector<std::pair<int, int>> &distanceIndices_inout,
    std::vector<float>               &mvuRight_inout,
    std::vector<float>               &depths_inout);

} // namespace core
} // namespace vs_graphs

#endif
