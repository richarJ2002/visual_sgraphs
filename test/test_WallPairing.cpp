/**
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

namespace vs_graphs
{
namespace core
{
namespace
{

/** Builds a WALL geometric::Plane whose surface sits at x = planeX_m_in, with normal
 * along +/-X (normalXSign_in), an observation origin stamped along that same
 * axis, and a genuine on-plane point cloud spanning [yMin,yMax] x [zMin,zMax]
 * -- exactly on the plane, so footprint projection is faithful regardless of
 * which in-plane basis reconcileWallFacePairs() happens to pick. */
std::unique_ptr<geometric::Plane> makeWallFace(int                     id_in,
                                    Map                    *p_map_in,
                                    double                  planeX_m_in,
                                    double                  normalXSign_in,
                                    const Eigen::Vector3d  &observationOrigin_World_in,
                                    double                  yMin_m_in,
                                    double                  yMax_m_in,
                                    double                  zMin_m_in,
                                    double                  zMax_m_in)
{
    auto wall = std::make_unique<geometric::Plane>();
    wall->setId(id_in);
    wall->SetMap(p_map_in);
    wall->setPlaneType(geometric::Plane::planeVariant::WALL);

    const Eigen::Vector4d equation(
        normalXSign_in, 0.0, 0.0, -normalXSign_in * planeX_m_in);
    wall->setGlobalEquation(g2o::Plane3D(equation));
    wall->setCentroid(Eigen::Vector3d(
        planeX_m_in, (yMin_m_in + yMax_m_in) / 2.0, (zMin_m_in + zMax_m_in) / 2.0));
    wall->setObservationOrigin_World(observationOrigin_World_in);

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
    wall->setMapClouds(cloud);
    return wall;
}

} // namespace

TEST(WallPairing, LinksAPlausibleTwinPairSymmetrically)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.GetCurrentMap();
    SemanticsManager manager(&atlas);

    /* Face A at x=0, normal +X, camera at x=1 (its own +X exterior side). */
    std::unique_ptr<geometric::Plane> faceA = makeWallFace(
        1, p_map, 0.0, 1.0, Eigen::Vector3d(1.0, 0.0, 0.0), -1.0, 1.0, 1.0, 2.0);
    /* Face B at x=-0.2 (0.2 m wall thickness), normal -X, camera at x=-1
     * (the opposite exterior side). Footprint overlaps A's. */
    std::unique_ptr<geometric::Plane> faceB = makeWallFace(2,
                                                p_map,
                                                -0.2,
                                                -1.0,
                                                Eigen::Vector3d(-1.0, 0.0, 0.0),
                                                -0.9,
                                                0.9,
                                                1.1,
                                                1.9);

    atlas.AddMapPlane(faceA.get());
    atlas.AddMapPlane(faceB.get());

    manager.reconcileWallFacePairsForTest();

    EXPECT_EQ(faceA->getTwinFace(), faceB.get());
    EXPECT_EQ(faceB->getTwinFace(), faceA.get());
}

TEST(WallPairing, DoesNotLinkPlanesThinnerThanAnyPlausibleWall)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.GetCurrentMap();
    SemanticsManager manager(&atlas);

    /* Only 0.01 m apart -- thinner than any plausible physical wall. */
    std::unique_ptr<geometric::Plane> faceA = makeWallFace(
        1, p_map, 0.0, 1.0, Eigen::Vector3d(1.0, 0.0, 0.0), -1.0, 1.0, 1.0, 2.0);
    std::unique_ptr<geometric::Plane> faceB = makeWallFace(2,
                                                p_map,
                                                -0.01,
                                                -1.0,
                                                Eigen::Vector3d(-1.0, 0.0, 0.0),
                                                -0.9,
                                                0.9,
                                                1.1,
                                                1.9);

    atlas.AddMapPlane(faceA.get());
    atlas.AddMapPlane(faceB.get());

    manager.reconcileWallFacePairsForTest();

    EXPECT_EQ(faceA->getTwinFace(), nullptr);
    EXPECT_EQ(faceB->getTwinFace(), nullptr);
}

TEST(WallPairing, DoesNotLinkPlanesFartherApartThanAnyPlausibleWall)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.GetCurrentMap();
    SemanticsManager manager(&atlas);

    /* 3 m apart -- two different walls, not two faces of one wall. */
    std::unique_ptr<geometric::Plane> faceA = makeWallFace(
        1, p_map, 0.0, 1.0, Eigen::Vector3d(1.0, 0.0, 0.0), -1.0, 1.0, 1.0, 2.0);
    std::unique_ptr<geometric::Plane> faceB = makeWallFace(2,
                                                p_map,
                                                -3.0,
                                                -1.0,
                                                Eigen::Vector3d(-4.0, 0.0, 0.0),
                                                -0.9,
                                                0.9,
                                                1.1,
                                                1.9);

    atlas.AddMapPlane(faceA.get());
    atlas.AddMapPlane(faceB.get());

    manager.reconcileWallFacePairsForTest();

    EXPECT_EQ(faceA->getTwinFace(), nullptr);
    EXPECT_EQ(faceB->getTwinFace(), nullptr);
}

TEST(WallPairing, DoesNotLinkPlanesObservedFromTheSameExteriorSide)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.GetCurrentMap();
    SemanticsManager manager(&atlas);

    /* Both cameras on the +X side -- inconsistent with being two opposite
     * faces of one wall (a genuine twin pair is observed from opposite
     * exterior sides). */
    std::unique_ptr<geometric::Plane> faceA = makeWallFace(
        1, p_map, 0.0, 1.0, Eigen::Vector3d(1.0, 0.0, 0.0), -1.0, 1.0, 1.0, 2.0);
    std::unique_ptr<geometric::Plane> faceB = makeWallFace(2,
                                                p_map,
                                                -0.2,
                                                -1.0,
                                                Eigen::Vector3d(1.0, 0.0, 0.0),
                                                -0.9,
                                                0.9,
                                                1.1,
                                                1.9);

    atlas.AddMapPlane(faceA.get());
    atlas.AddMapPlane(faceB.get());

    manager.reconcileWallFacePairsForTest();

    EXPECT_EQ(faceA->getTwinFace(), nullptr);
    EXPECT_EQ(faceB->getTwinFace(), nullptr);
}

TEST(WallPairing, DoesNotLinkPlanesWithNoFootprintOverlap)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.GetCurrentMap();
    SemanticsManager manager(&atlas);

    /* Parallel, plausibly wall-thick apart, opposite sides -- but their
     * footprints sit at completely different Y ranges (two unrelated wall
     * segments on parallel planes, not one physical wall). */
    std::unique_ptr<geometric::Plane> faceA = makeWallFace(
        1, p_map, 0.0, 1.0, Eigen::Vector3d(1.0, 0.0, 0.0), -1.0, 1.0, 1.0, 2.0);
    std::unique_ptr<geometric::Plane> faceB = makeWallFace(2,
                                                p_map,
                                                -0.2,
                                                -1.0,
                                                Eigen::Vector3d(-1.0, 0.0, 0.0),
                                                10.0,
                                                12.0,
                                                1.1,
                                                1.9);

    atlas.AddMapPlane(faceA.get());
    atlas.AddMapPlane(faceB.get());

    manager.reconcileWallFacePairsForTest();

    EXPECT_EQ(faceA->getTwinFace(), nullptr);
    EXPECT_EQ(faceB->getTwinFace(), nullptr);
}

TEST(WallPairing, UnlinksAPreviouslyPairedPlaneThatDrifted)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.GetCurrentMap();
    SemanticsManager manager(&atlas);

    std::unique_ptr<geometric::Plane> faceA = makeWallFace(
        1, p_map, 0.0, 1.0, Eigen::Vector3d(1.0, 0.0, 0.0), -1.0, 1.0, 1.0, 2.0);
    std::unique_ptr<geometric::Plane> faceB = makeWallFace(2,
                                                p_map,
                                                -0.2,
                                                -1.0,
                                                Eigen::Vector3d(-1.0, 0.0, 0.0),
                                                -0.9,
                                                0.9,
                                                1.1,
                                                1.9);

    atlas.AddMapPlane(faceA.get());
    atlas.AddMapPlane(faceB.get());

    manager.reconcileWallFacePairsForTest();
    ASSERT_EQ(faceA->getTwinFace(), faceB.get());
    ASSERT_EQ(faceB->getTwinFace(), faceA.get());

    /* Simulate a later refit drifting faceB far away -- no longer a
     * plausible twin (too far apart to be one physical wall). */
    faceB->setGlobalEquation(g2o::Plane3D(Eigen::Vector4d(-1.0, 0.0, 0.0, 3.0)));
    faceB->setObservationOrigin_World(Eigen::Vector3d(-4.0, 0.0, 0.0));

    manager.reconcileWallFacePairsForTest();

    EXPECT_EQ(faceA->getTwinFace(), nullptr);
    EXPECT_EQ(faceB->getTwinFace(), nullptr);
}

} // namespace core
} // namespace vs_graphs
