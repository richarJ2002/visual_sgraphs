/**
 * @file StereoMatchOutlierRejection.h
 * @brief Declares the median-distance outlier rejection step used by
 *        Frame::ComputeStereoMatches(), isolated for direct unit testing.
 */

#ifndef ORB_SLAM3_STEREO_MATCH_OUTLIER_REJECTION_H
#define ORB_SLAM3_STEREO_MATCH_OUTLIER_REJECTION_H

#include <utility>
#include <vector>

namespace ORB_SLAM3
{

/**
 * Rejects stereo matches whose ORB descriptor distance is far from the
 * median of all accepted matches (Frame::ComputeStereoMatches()'s original
 * 1.5 * 1.4 * median threshold).
 *
 * @param[in]     vDistIdx  (distance, left-keypoint-index) pairs for every
 *                          keypoint that received a stereo match. Sorted in
 *                          place.
 * @param[in,out] mvuRight  Per-left-keypoint matched right-image u
 *                          coordinate; rejected entries are set to -1.
 * @param[in,out] mvDepth   Per-left-keypoint depth; rejected entries are set
 *                          to -1.
 *
 * @note No-ops when vDistIdx is empty -- a frame with zero qualifying stereo
 *       matches has nothing to compute a median from, and must not read
 *       vDistIdx[vDistIdx.size() / 2] (out of bounds on an empty vector).
 */
void rejectOutlierStereoMatches(std::vector<std::pair<int, int>> &vDistIdx,
                                std::vector<float>                &mvuRight,
                                std::vector<float>                &mvDepth);

} // namespace ORB_SLAM3

#endif
