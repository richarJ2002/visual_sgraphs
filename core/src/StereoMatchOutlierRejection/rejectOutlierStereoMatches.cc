/*!
 * @file rejectOutlierStereoMatches.cc
 * @brief Defines the median-distance outlier rejection step used by
 *        Frame::computeStereoMatches().
 */

#include "StereoMatchOutlierRejection.h"

#include <algorithm>

namespace vs_graphs
{
namespace core
{

void rejectOutlierStereoMatches(std::vector<std::pair<int, int>> &vDistIdx,
                                std::vector<float>               &mvuRight,
                                std::vector<float>               &mvDepth)
{
    if (vDistIdx.empty())
    {
        return;
    }

    std::sort(vDistIdx.begin(), vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size() / 2].first;
    const float thDist = 1.5f * 1.4f * median;

    for (int i = static_cast<int>(vDistIdx.size()) - 1; i >= 0; i--)
    {
        if (vDistIdx[i].first < thDist)
        {
            break;
        }

        mvuRight[vDistIdx[i].second] = -1;
        mvDepth[vDistIdx[i].second]  = -1;
    }
}

} // namespace core
} // namespace vs_graphs
