/*!
 * @file test_WallPairing.cpp
 * @brief Wall-pairing coverage: SemanticsManager::reconcileWallFacePairs()
 *        links the two opposite-facing geometric::Plane hypotheses of one
 *        physical wall (axiom (e)), and unlinks a pair that stops being
 *        plausible.
 */

#include "Atlas.h"
#include "Geometric/Plane.h"
#include "Map.h"
#include "SemanticsManager.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <memory>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace
{

/*! Builds a WALL geometric::Plane whose surface sits at x = planeX_m_in, with
 * normal along +/-X (normalXSign_in), an observation origin stamped along that
 * same axis, and a genuine on-plane point cloud spanning [yMin,yMax] x
 * [zMin,zMax]
 * -- exactly on the plane, so footprint projection is faithful regardless of
 * which in-plane basis reconcileWallFacePairs() happens to pick. */
std::unique_ptr<geometric::Plane>
    makeWallFace(int                    id_in,
                 Map                   *p_map_in,
                 double                 planeX_m_in,
                 double                 normalXSign_in,
                 const Eigen::Vector3d &observationOrigin_world_in,
                 double                 yMin_m_in,
                 double                 yMax_m_in,
                 double                 zMin_m_in,
                 double                 zMax_m_in)
{
    std::unique_ptr<geometric::Plane> wall =
        std::make_unique<geometric::Plane>();
    if (wall->setId(id_in) != geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (wall->setMap(p_map_in) != geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (wall->setPlaneType(geometric::Plane::PlaneVariant::WALL) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPlaneType returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    const Eigen::Vector4d equation(normalXSign_in,
                                   0.0,
                                   0.0,
                                   -normalXSign_in * planeX_m_in);
    if (wall->setGlobalEquation(g2o::Plane3D(equation)) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setGlobalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (wall->setCentroid(Eigen::Vector3d(planeX_m_in,
                                          (yMin_m_in + yMax_m_in) / 2.0,
                                          (zMin_m_in + zMax_m_in) / 2.0)) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (wall->setObservationOrigin_world(observationOrigin_world_in) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setObservationOrigin_world returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    constexpr int steps = 10;
    for (int yIndex = 0; yIndex < steps; ++yIndex)
    {
        const double y = yMin_m_in + (yMax_m_in - yMin_m_in) *
                                         static_cast<double>(yIndex) /
                                         static_cast<double>(steps - 1);
        for (int zIndex = 0; zIndex < steps; ++zIndex)
        {
            const double z = zMin_m_in + (zMax_m_in - zMin_m_in) *
                                             static_cast<double>(zIndex) /
                                             static_cast<double>(steps - 1);
            pcl::PointXYZRGBA point;
            point.x = static_cast<float>(planeX_m_in);
            point.y = static_cast<float>(y);
            point.z = static_cast<float>(z);
            cloud->push_back(point);
        }
    }
    if (wall->setMapClouds(cloud) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMapClouds returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    return wall;
}

} // namespace

TEST(WallPairing, LinksAPlausibleTwinPairSymmetrically)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticsManager manager(&atlas);

    /* Face A at x=0, normal +X, camera at x=1 (its own +X exterior side). */
    std::unique_ptr<geometric::Plane> faceA =
        makeWallFace(1,
                     p_map,
                     0.0,
                     1.0,
                     Eigen::Vector3d(1.0, 0.0, 0.0),
                     -1.0,
                     1.0,
                     1.0,
                     2.0);
    /* Face B at x=-0.2 (0.2 m wall thickness), normal -X, camera at x=-1
     * (the opposite exterior side). Footprint overlaps A's. */
    std::unique_ptr<geometric::Plane> faceB =
        makeWallFace(2,
                     p_map,
                     -0.2,
                     -1.0,
                     Eigen::Vector3d(-1.0, 0.0, 0.0),
                     -0.9,
                     0.9,
                     1.1,
                     1.9);

    ASSERT_EQ((atlas.addMapPlane(faceA.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.addMapPlane(faceB.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    ASSERT_EQ((manager.reconcileWallFacePairsForTest()),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);

    geometric::Plane *p_getTwinFace = nullptr;
    ASSERT_EQ((faceA->getTwinFace(p_getTwinFace)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_EQ(p_getTwinFace, faceB.get());
    geometric::Plane *p_getTwinFace2 = nullptr;
    ASSERT_EQ((faceB->getTwinFace(p_getTwinFace2)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_EQ(p_getTwinFace2, faceA.get());
}

TEST(WallPairing, DoesNotLinkPlanesThinnerThanAnyPlausibleWall)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticsManager manager(&atlas);

    /* Only 0.01 m apart -- thinner than any plausible physical wall. */
    std::unique_ptr<geometric::Plane> faceA =
        makeWallFace(1,
                     p_map,
                     0.0,
                     1.0,
                     Eigen::Vector3d(1.0, 0.0, 0.0),
                     -1.0,
                     1.0,
                     1.0,
                     2.0);
    std::unique_ptr<geometric::Plane> faceB =
        makeWallFace(2,
                     p_map,
                     -0.01,
                     -1.0,
                     Eigen::Vector3d(-1.0, 0.0, 0.0),
                     -0.9,
                     0.9,
                     1.1,
                     1.9);

    ASSERT_EQ((atlas.addMapPlane(faceA.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.addMapPlane(faceB.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    ASSERT_EQ((manager.reconcileWallFacePairsForTest()),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);

    geometric::Plane *p_getTwinFace = nullptr;
    ASSERT_EQ((faceA->getTwinFace(p_getTwinFace)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_EQ(p_getTwinFace, nullptr);
    geometric::Plane *p_getTwinFace2 = nullptr;
    ASSERT_EQ((faceB->getTwinFace(p_getTwinFace2)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_EQ(p_getTwinFace2, nullptr);
}

TEST(WallPairing, DoesNotLinkPlanesFartherApartThanAnyPlausibleWall)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticsManager manager(&atlas);

    /* 3 m apart -- two different walls, not two faces of one wall. */
    std::unique_ptr<geometric::Plane> faceA =
        makeWallFace(1,
                     p_map,
                     0.0,
                     1.0,
                     Eigen::Vector3d(1.0, 0.0, 0.0),
                     -1.0,
                     1.0,
                     1.0,
                     2.0);
    std::unique_ptr<geometric::Plane> faceB =
        makeWallFace(2,
                     p_map,
                     -3.0,
                     -1.0,
                     Eigen::Vector3d(-4.0, 0.0, 0.0),
                     -0.9,
                     0.9,
                     1.1,
                     1.9);

    ASSERT_EQ((atlas.addMapPlane(faceA.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.addMapPlane(faceB.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    ASSERT_EQ((manager.reconcileWallFacePairsForTest()),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);

    geometric::Plane *p_getTwinFace = nullptr;
    ASSERT_EQ((faceA->getTwinFace(p_getTwinFace)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_EQ(p_getTwinFace, nullptr);
    geometric::Plane *p_getTwinFace2 = nullptr;
    ASSERT_EQ((faceB->getTwinFace(p_getTwinFace2)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_EQ(p_getTwinFace2, nullptr);
}

TEST(WallPairing, DoesNotLinkPlanesObservedFromTheSameExteriorSide)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticsManager manager(&atlas);

    /* Both cameras on the +X side -- inconsistent with being two opposite
     * faces of one wall (a genuine twin pair is observed from opposite
     * exterior sides). */
    std::unique_ptr<geometric::Plane> faceA =
        makeWallFace(1,
                     p_map,
                     0.0,
                     1.0,
                     Eigen::Vector3d(1.0, 0.0, 0.0),
                     -1.0,
                     1.0,
                     1.0,
                     2.0);
    std::unique_ptr<geometric::Plane> faceB =
        makeWallFace(2,
                     p_map,
                     -0.2,
                     -1.0,
                     Eigen::Vector3d(1.0, 0.0, 0.0),
                     -0.9,
                     0.9,
                     1.1,
                     1.9);

    ASSERT_EQ((atlas.addMapPlane(faceA.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.addMapPlane(faceB.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    ASSERT_EQ((manager.reconcileWallFacePairsForTest()),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);

    geometric::Plane *p_getTwinFace = nullptr;
    ASSERT_EQ((faceA->getTwinFace(p_getTwinFace)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_EQ(p_getTwinFace, nullptr);
    geometric::Plane *p_getTwinFace2 = nullptr;
    ASSERT_EQ((faceB->getTwinFace(p_getTwinFace2)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_EQ(p_getTwinFace2, nullptr);
}

TEST(WallPairing, DoesNotLinkPlanesWithNoFootprintOverlap)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticsManager manager(&atlas);

    /* Parallel, plausibly wall-thick apart, opposite sides -- but their
     * footprints sit at completely different Y ranges (two unrelated wall
     * segments on parallel planes, not one physical wall). */
    std::unique_ptr<geometric::Plane> faceA =
        makeWallFace(1,
                     p_map,
                     0.0,
                     1.0,
                     Eigen::Vector3d(1.0, 0.0, 0.0),
                     -1.0,
                     1.0,
                     1.0,
                     2.0);
    std::unique_ptr<geometric::Plane> faceB =
        makeWallFace(2,
                     p_map,
                     -0.2,
                     -1.0,
                     Eigen::Vector3d(-1.0, 0.0, 0.0),
                     10.0,
                     12.0,
                     1.1,
                     1.9);

    ASSERT_EQ((atlas.addMapPlane(faceA.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.addMapPlane(faceB.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    ASSERT_EQ((manager.reconcileWallFacePairsForTest()),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);

    geometric::Plane *p_getTwinFace = nullptr;
    ASSERT_EQ((faceA->getTwinFace(p_getTwinFace)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_EQ(p_getTwinFace, nullptr);
    geometric::Plane *p_getTwinFace2 = nullptr;
    ASSERT_EQ((faceB->getTwinFace(p_getTwinFace2)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_EQ(p_getTwinFace2, nullptr);
}

TEST(WallPairing, UnlinksAPreviouslyPairedPlaneThatDrifted)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticsManager manager(&atlas);

    std::unique_ptr<geometric::Plane> faceA =
        makeWallFace(1,
                     p_map,
                     0.0,
                     1.0,
                     Eigen::Vector3d(1.0, 0.0, 0.0),
                     -1.0,
                     1.0,
                     1.0,
                     2.0);
    std::unique_ptr<geometric::Plane> faceB =
        makeWallFace(2,
                     p_map,
                     -0.2,
                     -1.0,
                     Eigen::Vector3d(-1.0, 0.0, 0.0),
                     -0.9,
                     0.9,
                     1.1,
                     1.9);

    ASSERT_EQ((atlas.addMapPlane(faceA.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.addMapPlane(faceB.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    ASSERT_EQ((manager.reconcileWallFacePairsForTest()),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    geometric::Plane *p_getTwinFace = nullptr;
    ASSERT_EQ((faceA->getTwinFace(p_getTwinFace)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ(p_getTwinFace, faceB.get());
    geometric::Plane *p_getTwinFace2 = nullptr;
    ASSERT_EQ((faceB->getTwinFace(p_getTwinFace2)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ(p_getTwinFace2, faceA.get());

    /* Simulate a later refit drifting faceB far away -- no longer a
     * plausible twin (too far apart to be one physical wall). */
    ASSERT_EQ((faceB->setGlobalEquation(
                  g2o::Plane3D(Eigen::Vector4d(-1.0, 0.0, 0.0, 3.0)))),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ(
        (faceB->setObservationOrigin_world(Eigen::Vector3d(-4.0, 0.0, 0.0))),
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS);

    ASSERT_EQ((manager.reconcileWallFacePairsForTest()),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);

    geometric::Plane *p_getTwinFace3 = nullptr;
    ASSERT_EQ((faceA->getTwinFace(p_getTwinFace3)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_EQ(p_getTwinFace3, nullptr);
    geometric::Plane *p_getTwinFace4 = nullptr;
    ASSERT_EQ((faceB->getTwinFace(p_getTwinFace4)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_EQ(p_getTwinFace4, nullptr);
}

} // namespace core
} // namespace vs_graphs
