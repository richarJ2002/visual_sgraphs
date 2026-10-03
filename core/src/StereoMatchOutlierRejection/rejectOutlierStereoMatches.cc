/*!
 * @file            rejectOutlierStereoMatches.cc
 *
 * @brief           Defines the median-distance outlier rejection step used by
 *                  Frame::computeStereoMatches().
 */

#include "StereoMatchOutlierRejection.h"

#include <algorithm>

namespace vs_graphs
{
namespace core
{

StereoMatchOutlierRejectionStatus rejectOutlierStereoMatches(
    std::vector<std::pair<int, int>> &distanceIndices_inout,
    std::vector<float>               &mvuRight_inout,
    std::vector<float>               &depths_inout)
{
    if (distanceIndices_inout.empty())
    {
        return StereoMatchOutlierRejectionStatus::
            STEREO_MATCH_OUTLIER_REJECTION_STATUS_SUCCESS;
    }

    std::sort(distanceIndices_inout.begin(), distanceIndices_inout.end());
    const float median =
        distanceIndices_inout[distanceIndices_inout.size() / 2].first;
    const float thresholdDistance = 1.5f * 1.4f * median;

    for (int distanceIndex = static_cast<int>(distanceIndices_inout.size()) - 1;
         distanceIndex >= 0;
         distanceIndex--)
    {
        if (distanceIndices_inout[distanceIndex].first < thresholdDistance)
        {
            break;
        }

        mvuRight_inout[distanceIndices_inout[distanceIndex].second] = -1;
        depths_inout[distanceIndices_inout[distanceIndex].second]   = -1;
    }

    return StereoMatchOutlierRejectionStatus::
        STEREO_MATCH_OUTLIER_REJECTION_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
