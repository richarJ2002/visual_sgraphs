/*!
 * @file test_GroundPlaneFilter.cpp
 * @brief B1 regression coverage: computeGroundPlaneHeight() must not read
 *        past an empty (or single-point) support cloud.
 *
 * Before the fix, an empty support cloud made numPoint (= yVals.size() / 2)
 * equal to 0, and yVals[numPoint - 1] underflowed to yVals[SIZE_MAX] -- an
 * out-of-bounds read. A freshly constructed geometric::Plane (never given
 * points via setMapClouds/replaceMapClouds) is already in exactly this state,
 * since geometric::Plane's constructor allocates a valid but empty point cloud
 * rather than a null one.
 */

#include "Atlas.h"
#include "Geometric/Plane.h"
#include "Map.h"
#include "SemanticsManager.h"

#include <gtest/gtest.h>

#include <optional>

namespace vs_graphs
{
namespace core
{

TEST(GroundPlaneFilter, ReturnsNulloptForAnEmptySupportCloud)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticsManager manager(&atlas);

    /* A freshly constructed geometric::Plane has a valid but empty support
     * cloud -- exactly the state a plane can be in before its first successful
     * refit, or right after replaceMapClouds() clears it. */
    geometric::Plane groundPlane;
    ASSERT_EQ((groundPlane.setId(1)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((groundPlane.setMap(p_map)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);

    std::optional<float> height{};
    ASSERT_EQ((manager.computeGroundPlaneHeightForTest(&groundPlane, height)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_FALSE(height.has_value());
}

TEST(GroundPlaneFilter, ReturnsNulloptForASinglePointSupportCloud)
{
    /* numPoint = yVals.size() / 2 is also 0 for a single-point cloud, not
     * only for an empty one -- the same underflow is reachable here too. */
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticsManager manager(&atlas);

    geometric::Plane groundPlane;
    ASSERT_EQ((groundPlane.setId(1)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((groundPlane.setMap(p_map)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointXYZRGBA point;
    point.x = 0.0f;
    point.y = 0.5f;
    point.z = 0.0f;
    cloud->push_back(point);
    ASSERT_EQ((groundPlane.setMapClouds(cloud)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);

    std::optional<float> height{};
    ASSERT_EQ((manager.computeGroundPlaneHeightForTest(&groundPlane, height)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_FALSE(height.has_value());
}

TEST(GroundPlaneFilter, ReturnsAValueForAMultiPointSupportCloud)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticsManager manager(&atlas);

    geometric::Plane groundPlane;
    ASSERT_EQ((groundPlane.setId(1)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((groundPlane.setMap(p_map)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (int index = 0; index < 5; ++index)
    {
        pcl::PointXYZRGBA point;
        point.x = 0.0f;
        point.y = static_cast<float>(index) * 0.1f;
        point.z = 0.0f;
        cloud->push_back(point);
    }
    ASSERT_EQ((groundPlane.setMapClouds(cloud)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);

    std::optional<float> height{};
    ASSERT_EQ((manager.computeGroundPlaneHeightForTest(&groundPlane, height)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_TRUE(height.has_value());
}

} // namespace core
} // namespace vs_graphs
