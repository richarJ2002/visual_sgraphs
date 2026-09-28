/*!
 * @file test_StereoMatchOutlierRejection.cpp
 * @brief B3 regression coverage: Frame::computeStereoMatches()'s extracted
 *        outlier-rejection step must not read past an empty match list.
 */

#include "StereoMatchOutlierRejection.h"

#include <gtest/gtest.h>

#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{

TEST(StereoMatchOutlierRejection, NoOpsOnAnEmptyMatchList)
{
    /* Before the fix, vDistIdx[vDistIdx.size() / 2] on an empty vDistIdx
     * read vDistIdx[0] out of bounds -- reachable whenever a frame has zero
     * qualifying stereo matches. */
    std::vector<std::pair<int, int>> vDistIdx;
    std::vector<float>               mvuRight(3, -1.0f);
    std::vector<float>               mvDepth(3, -1.0f);

    ASSERT_EQ((rejectOutlierStereoMatches(vDistIdx, mvuRight, mvDepth)),
              StereoMatchOutlierRejectionStatus::
                  STEREO_MATCH_OUTLIER_REJECTION_STATUS_SUCCESS);

    EXPECT_EQ(mvuRight, std::vector<float>(3, -1.0f));
    EXPECT_EQ(mvDepth, std::vector<float>(3, -1.0f));
}

TEST(StereoMatchOutlierRejection, RejectsMatchesFarAboveTheMedianDistance)
{
    /* Three good matches clustered near distance 10, one clear outlier at
     * 1000 -- comfortably past 1.5 * 1.4 * median. */
    std::vector<std::pair<int, int>> vDistIdx = {{10, 0},
                                                 {11, 1},
                                                 {1000, 2},
                                                 {9, 3}};
    std::vector<float>               mvuRight(4, 5.0f);
    std::vector<float>               mvDepth(4, 2.0f);

    ASSERT_EQ((rejectOutlierStereoMatches(vDistIdx, mvuRight, mvDepth)),
              StereoMatchOutlierRejectionStatus::
                  STEREO_MATCH_OUTLIER_REJECTION_STATUS_SUCCESS);

    EXPECT_EQ(mvuRight[2], -1.0f);
    EXPECT_EQ(mvDepth[2], -1.0f);
    EXPECT_EQ(mvuRight[0], 5.0f);
    EXPECT_EQ(mvuRight[1], 5.0f);
    EXPECT_EQ(mvuRight[3], 5.0f);
}

} // namespace core
} // namespace vs_graphs
