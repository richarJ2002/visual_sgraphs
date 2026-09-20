/*!
 * Focused tests for two wall-admission fixes reported directly against a
 * live sim run (office_clean):
 *
 *  1. evaluateWallAdmissionEvidence()'s height/width gate used to build its
 *     in-plane bounding-box axes from Eigen's Vector3d::unitOrthogonal(),
 *     which is arbitrary relative to the physical wall it is measuring. A
 *     narrow, always-vertical real-world object (a door-frame post) can
 *     rotate relative to that arbitrary axis and have BOTH measured
 *     extents inflate past the width/height thresholds even though its
 *     true width is a few centimetres -- exactly the "reducing the point
 *     cloud threshold and area threshold doesn't filter it out" symptom
 *     reported live. The fix anchors the two axes to the ground plane
 *     (horizontal tangent + vertical), matching the pattern
 *     buildFiniteWallSegment2d already uses elsewhere in this file.
 *
 *  2. admitWallToRoom() had no check for a wall observed from the opposite
 *     side of the room it was about to be bound to -- "connected to a wall
 *     on the wrong side" reported live. The fix orients the wall equation
 *     toward the room's own centroid, then rejects when the keyframes that
 *     actually observed the wall have a confident sign-consensus on the
 *     FAR side (geometric::Plane::getObservationSideSnapshot(), previously
 * read-only diagnostic logging, now also a gate).
 *
 * Both fixes are exercised through SemanticsManager's private production
 * methods via the existing VS_GRAPHS_ENABLE_ROOM_TRACKER_TEST_HOOK
 * test-only wrappers, not by reconstructing the logic by hand.
 */

#include "Atlas.h"
#include "Geometric/Plane.h"
#include "KeyFrame.h"
#include "Map.h"
#include "Semantic/Passage.h"
#include "Semantic/Room.h"
#include "SemanticsManager.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <memory>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace
{

/*! Builds a wall geometric::Plane with a genuine, production-computed geometry
 * snapshot: a flat rectangular grid of points, spanning [-halfU, halfU]
 * along axisU_World and [-halfV, halfV] along axisV_World, offset from the
 * plane's own centroid, all exactly on-plane. Mirrors
 * test_RoomContextPersist.cpp's makeRefitWallPlane pattern (feed a
 * synthetic cloud through the real geometric::Plane::updateSizeOfPlane() path,
 * the same function evaluateWallAdmissionEvidence's
 * geometric::Plane::getGeometrySnapshot() call reads from). */
void makeWallWithGridCloud(geometric::Plane      &wall_inout,
                           int                    id_in,
                           Map                   *p_map_in,
                           const Eigen::Vector4d &equation_World_in,
                           const Eigen::Vector3d &axisU_World_in,
                           const Eigen::Vector3d &axisV_World_in,
                           double                 halfU_m_in,
                           double                 halfV_m_in)
{
    wall_inout.setId(id_in);
    wall_inout.setMap(p_map_in);
    wall_inout.setPlaneType(geometric::Plane::PlaneVariant::WALL);
    /* evaluateWallAdmissionEvidence's wallDominatesSemantics gate compares
     * getPlaneType() against getExpectedPlaneType(), which is derived from
     * semanticVotes rather than settable directly -- cast a vote so the two
     * agree, matching what real wall classification does over time. */
    wall_inout.castWeightedVote(geometric::Plane::PlaneVariant::WALL, 1.0);
    wall_inout.setGlobalEquation(g2o::Plane3D(equation_World_in));
    wall_inout.setCentroid(Eigen::Vector3d::Zero());

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    constexpr int steps = 20;
    for (int uIndex = 0; uIndex < steps; ++uIndex)
    {
        const double u = -halfU_m_in + (2.0 * halfU_m_in) *
                                           static_cast<double>(uIndex) /
                                           static_cast<double>(steps - 1);
        for (int vIndex = 0; vIndex < steps; ++vIndex)
        {
            const double v = -halfV_m_in + (2.0 * halfV_m_in) *
                                               static_cast<double>(vIndex) /
                                               static_cast<double>(steps - 1);
            const Eigen::Vector3d point_World =
                u * axisU_World_in + v * axisV_World_in;

            pcl::PointXYZRGBA pclPoint;
            pclPoint.x = static_cast<float>(point_World.x());
            pclPoint.y = static_cast<float>(point_World.y());
            pclPoint.z = static_cast<float>(point_World.z());
            cloud->push_back(pclPoint);
        }
    }
    wall_inout.setMapClouds(cloud);
    wall_inout.updateSizeOfPlane();
}

} // namespace

/* ---------------------------------------------------------------------- *
 * Fix 1: ground-aligned height/width gate (door-frame-post filter)
 * ---------------------------------------------------------------------- */

TEST(WallAdmission, RejectsNarrowDoorFramePostWhenGroundAligned)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);

    /* Wall normal along world X; ground normal along world Z (Z-up). The
     * post's true width (0.08 m, well under the 0.30 m minimum) runs along
     * world Y, its true height (2.2 m) along world Z -- a physically
     * ordinary vertical door-frame post. */
    geometric::Plane wall;
    makeWallWithGridCloud(wall,
                          1,
                          p_map,
                          Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                          Eigen::Vector3d(0.0, 1.0, 0.0),
                          Eigen::Vector3d(0.0, 0.0, 1.0),
                          0.04,
                          1.1);

    const bool admissible =
        manager.evaluateWallAdmissionEvidenceAdmissibleForTest(
            &wall,
            Eigen::Vector3d(0.0, 0.0, 1.0));
    EXPECT_FALSE(admissible);
}

TEST(WallAdmission, AdmitsPhysicallyAdequateWallWhenGroundAligned)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);

    /* Same orientation as above, but an ordinary wall-sized panel: 2 m wide,
     * 2.2 m tall -- comfortably above minimumMajorExtent_m (0.80),
     * minimumMinorExtent_m (0.30) and minimumArea_m2 (0.40). */
    geometric::Plane wall;
    makeWallWithGridCloud(wall,
                          1,
                          p_map,
                          Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                          Eigen::Vector3d(0.0, 1.0, 0.0),
                          Eigen::Vector3d(0.0, 0.0, 1.0),
                          1.0,
                          1.1);

    const bool admissible =
        manager.evaluateWallAdmissionEvidenceAdmissibleForTest(
            &wall,
            Eigen::Vector3d(0.0, 0.0, 1.0));
    EXPECT_TRUE(admissible);
}

TEST(WallAdmission, RejectsNarrowDoorFramePostOnObliqueWall)
{
    /* The fix must generalise beyond a wall normal aligned to a world axis:
     * most real walls face an arbitrary horizontal direction. Ground
     * normal stays world Z; the wall normal here is a generic horizontal
     * direction (0.8, 0.6, 0). The post's true width/height are laid out
     * along this wall's own ground-derived horizontal tangent and world Z,
     * exactly as a real, always-vertical door-frame post would be,
     * regardless of which way the wall faces. */
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);

    const Eigen::Vector3d normal_World(0.8, 0.6, 0.0);
    const Eigen::Vector3d groundNormal_World(0.0, 0.0, 1.0);
    const Eigen::Vector3d horizontalTangent_World =
        groundNormal_World.cross(normal_World).normalized();
    const Eigen::Vector3d vertical_World(0.0, 0.0, 1.0);

    geometric::Plane wall;
    makeWallWithGridCloud(wall,
                          1,
                          p_map,
                          Eigen::Vector4d(normal_World.x(),
                                          normal_World.y(),
                                          normal_World.z(),
                                          0.0),
                          horizontalTangent_World,
                          vertical_World,
                          0.04,
                          1.1);

    const bool admissible =
        manager.evaluateWallAdmissionEvidenceAdmissibleForTest(
            &wall,
            groundNormal_World);
    EXPECT_FALSE(admissible);
}

TEST(WallAdmission,
     EvidenceGateStillAdmitsGenerouslySizedWallWithoutGroundPlane)
{
    /* Early in a mission, no ground plane may exist yet: groundNormal_World
     * is the zero vector, and the gate must fall back to its previous
     * behaviour rather than reject every wall outright. A generously-sized
     * (2 m x 2 m) cloud clears the width/height thresholds under any
     * in-plane axis choice, so this is a fallback-safety check, not a
     * reproduction of the axis-dependent bug the two tests above target. */
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);

    geometric::Plane wall;
    makeWallWithGridCloud(wall,
                          1,
                          p_map,
                          Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                          Eigen::Vector3d(0.0, 1.0, 0.0),
                          Eigen::Vector3d(0.0, 0.0, 1.0),
                          1.0,
                          1.0);

    const bool admissible =
        manager.evaluateWallAdmissionEvidenceAdmissibleForTest(
            &wall,
            Eigen::Vector3d::Zero());
    EXPECT_TRUE(admissible);
}

/* ---------------------------------------------------------------------- *
 * Fix 2: wrong-side backstop (observation-side gate)
 * ---------------------------------------------------------------------- */

namespace
{

/*! A default-constructed KeyFrame positioned at cameraCenter_World via
 * setPose(). Only isBad()/getCameraCenter() are exercised by the code under
 * test, both safe on a default-constructed KeyFrame (KeyFrame's default
 * constructor is a plain member-initialiser list; SetPose/GetCameraCenter
 * only touch mTcw/mTwc under mMutexPose). */
std::unique_ptr<KeyFrame>
    makeKeyFrameAt(const Eigen::Vector3d &cameraCenter_World_in)
{
    /* setPose() takes Tcw (world-to-camera): with identity rotation,
     * getCameraCenter() (== Twc.translation()) works out to -Tcw.translation.
     * Negate here so the resulting camera centre is the position callers
     * actually asked for. */
    auto keyFrame = std::make_unique<KeyFrame>();
    keyFrame->setPose(Sophus::SE3f(Eigen::Matrix3f::Identity(),
                                   -cameraCenter_World_in.cast<float>()));
    return keyFrame;
}

geometric::Plane::Observation makeMinimalObservation()
{
    geometric::Plane::Observation observation;
    observation.pointPlaneConstraintMatrix = Eigen::Matrix4d::Zero();
    observation.confidence                 = 1.0;
    return observation;
}

/*! A wide, admissible wall (passes evaluateWallAdmissionEvidence
 * unconditionally) at x=0, normal along +X, so the admission decision in
 * these tests turns entirely on the wrong-side gate. */
std::unique_ptr<geometric::Plane> makeAdmissibleWallAtOrigin(int  id_in,
                                                             Map *p_map_in)
{
    auto wall = std::make_unique<geometric::Plane>();
    makeWallWithGridCloud(*wall,
                          id_in,
                          p_map_in,
                          Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                          Eigen::Vector3d(0.0, 1.0, 0.0),
                          Eigen::Vector3d(0.0, 0.0, 1.0),
                          1.0,
                          1.1);
    return wall;
}

} // namespace

TEST(WallAdmission, RejectsWallConfidentlyObservedFromTheFarSide)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);

    std::unique_ptr<geometric::Plane> wall =
        makeAdmissibleWallAtOrigin(1, p_map);

    /* Four keyframes, all confidently on the +X side (x=1.0, well past the
     * 0.10 m reliable-side-distance floor). isWallFaceForeignToRoom() reads
     * face identity from the single stamped observationOrigin_World_m (the
     * 2026-09-04 rewrite), not from observation history -- stamp it here to
     * match what GeoSemHelpers::createMapPlane() does in production. */
    wall->setObservationOrigin_World(Eigen::Vector3d(1.0, 0.0, 0.0));
    std::vector<std::unique_ptr<KeyFrame>> keyFrames;
    for (int index = 0; index < 4; ++index)
    {
        keyFrames.push_back(
            makeKeyFrameAt(Eigen::Vector3d(1.0, 0.1 * index, 0.0)));
        wall->addObservation(keyFrames.back().get(), makeMinimalObservation());
    }

    semantic::Room room;
    room.setId(1);
    room.setMap(p_map);
    /* semantic::Room centroid on the -X side: opposite the keyframes that
     * actually observed this wall. */
    room.setCentroid(Eigen::Vector3d(-1.0, 0.0, 0.0));

    const bool admitted = manager.admitWallToRoomForTest(&room, wall.get());
    EXPECT_FALSE(admitted);
    EXPECT_TRUE(room.getWalls().empty());
}

TEST(WallAdmission, AdmitsWallObservedFromTheSameSideAsTheRoom)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);

    std::unique_ptr<geometric::Plane> wall =
        makeAdmissibleWallAtOrigin(1, p_map);

    /* Same four keyframes' side as the room this time. */
    wall->setObservationOrigin_World(Eigen::Vector3d(-1.0, 0.0, 0.0));
    std::vector<std::unique_ptr<KeyFrame>> keyFrames;
    for (int index = 0; index < 4; ++index)
    {
        keyFrames.push_back(
            makeKeyFrameAt(Eigen::Vector3d(-1.0, 0.1 * index, 0.0)));
        wall->addObservation(keyFrames.back().get(), makeMinimalObservation());
    }

    semantic::Room room;
    room.setId(1);
    room.setMap(p_map);
    room.setCentroid(Eigen::Vector3d(-1.0, 0.0, 0.0));

    const bool admitted = manager.admitWallToRoomForTest(&room, wall.get());
    EXPECT_TRUE(admitted);
    EXPECT_EQ(room.getWalls().size(), 1U);
}

TEST(WallAdmission, AdmitsWallWithNoObservationEvidenceRatherThanRejectingBlind)
{
    /* Fail-open: with no keyframe evidence at all, getObservationSideSnapshot
     * reports Face::UNKNOWN, not NEGATIVE -- the gate must not reject a wall
     * it simply lacks side evidence for. */
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);

    std::unique_ptr<geometric::Plane> wall =
        makeAdmissibleWallAtOrigin(1, p_map);

    semantic::Room room;
    room.setId(1);
    room.setMap(p_map);
    room.setCentroid(Eigen::Vector3d(-1.0, 0.0, 0.0));

    const bool admitted = manager.admitWallToRoomForTest(&room, wall.get());
    EXPECT_TRUE(admitted);
}

/* ---------------------------------------------------------------------- *
 * Fix 3 (B2): far-side aperture-crossing test must not go blind just
 * because the admitting room's own centroid sits on top of its passage
 * plane -- common for a sparsely-observed (e.g. single-wall) room.
 * ---------------------------------------------------------------------- */

TEST(WallAdmission, ReroutesFarSideWallWhenRoomCentroidIsOnThePassagePlane)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);

    /* semantic::Passage at x=0, generous aperture, with a stable prospective
     * room on the far (+x) side. */
    semantic::Passage passage;
    passage.setId(1);
    passage.setMap(p_map);
    passage.setGlobalEquation(
        g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)));
    passage.setCentroid(Eigen::Vector3d(0.0, 0.0, 0.0));
    passage.setWidth(2.0);
    passage.setHeight(2.0);
    passage.setPassable(true);
    /* Known near side is -X (established elsewhere, e.g. by an earlier
     * wall's admission on this passage) -- the B2 fix only substitutes a
     * synthesized near-side point when this is available; guessing from the
     * ambiguous room centroid's own residual sign would be as likely to
     * point the wrong way as the right one. */
    passage.setKnownSideDirection(Eigen::Vector3d(-1.0, 0.0, 0.0));

    semantic::Room prospective;
    prospective.setId(2);
    prospective.setMap(p_map);
    prospective.setCentroid(Eigen::Vector3d(1.0, 0.0, 0.0));
    passage.setProspectiveRoom(&prospective);

    atlas.addMapPassage(&passage);

    /* Candidate wall unambiguously on the far side of the passage. */
    geometric::Plane wall;
    wall.setId(3);
    wall.setMap(p_map);
    wall.setCentroid(Eigen::Vector3d(1.0, 0.0, 0.0));

    semantic::Room room;
    room.setId(4);
    room.setMap(p_map);
    /* Degenerate case: essentially sitting on the passage plane itself,
     * well inside any plausible minimumSideDistance_m. Before the B2 fix,
     * this made segmentCrossesPassageOpening silently report "no crossing",
     * letting the far-side wall fall through to ordinary admission on the
     * near room instead of being routed to the prospective far-side room. */
    room.setCentroid(Eigen::Vector3d(0.0005, 0.0, 0.0));

    const bool admitted = manager.admitWallToRoomForTest(&room, &wall);
    EXPECT_TRUE(admitted);
    EXPECT_TRUE(room.getWalls().empty());
    ASSERT_EQ(prospective.getWalls().size(), 1U);
    EXPECT_EQ(prospective.getWalls().front(), &wall);
}

TEST(WallAdmission, LeavesTheDegenerateCaseUnresolvedWithoutAKnownSideDirection)
{
    /* Same degenerate room-centroid geometry as above, but the passage has
     * no established KnownSideProvenance yet -- the B2 fix must not guess a
     * near-side direction from the ambiguous centroid's own residual sign
     * (that sign is noise and can point either way), so this stays exactly
     * as conservative as before the fix. */
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);

    semantic::Passage passage;
    passage.setId(1);
    passage.setMap(p_map);
    passage.setGlobalEquation(
        g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)));
    passage.setCentroid(Eigen::Vector3d(0.0, 0.0, 0.0));
    passage.setWidth(2.0);
    passage.setHeight(2.0);
    passage.setPassable(true);

    semantic::Room prospective;
    prospective.setId(2);
    prospective.setMap(p_map);
    prospective.setCentroid(Eigen::Vector3d(1.0, 0.0, 0.0));
    passage.setProspectiveRoom(&prospective);

    atlas.addMapPassage(&passage);

    geometric::Plane wall;
    wall.setId(3);
    wall.setMap(p_map);
    wall.setCentroid(Eigen::Vector3d(1.0, 0.0, 0.0));

    semantic::Room room;
    room.setId(4);
    room.setMap(p_map);
    room.setCentroid(Eigen::Vector3d(0.0005, 0.0, 0.0));

    const bool admitted = manager.admitWallToRoomForTest(&room, &wall);
    EXPECT_FALSE(admitted);
    EXPECT_TRUE(room.getWalls().empty());
    EXPECT_TRUE(prospective.getWalls().empty());
}

/* ---------------------------------------------------------------------- *
 * Cross-room wall-intersection rejection -- a candidate wall
 * whose finite segment decisively crosses another room's already-admitted
 * wall must be rejected, without perturbing the foreign room's wall.
 * ---------------------------------------------------------------------- */

namespace
{

/*! Builds a GROUND geometric::Plane at z=0 with a genuine, production-refit
 * geometry snapshot (Map::GetBiggestGroundPlane() requires cloudGeneration ==
 * successfulRefitGeneration and a finite support count). */
std::unique_ptr<geometric::Plane> makeRefitGroundPlaneAtOrigin(int  id_in,
                                                               Map *p_map_in)
{
    auto ground = std::make_unique<geometric::Plane>();
    ground->setId(id_in);
    ground->setMap(p_map_in);
    ground->setPlaneType(geometric::Plane::PlaneVariant::GROUND);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (int index = 0; index < 50; ++index)
    {
        pcl::PointXYZRGBA point;
        point.x = static_cast<float>(index) * 0.1f - 2.5f;
        point.y = static_cast<float>(index) * 0.05f - 1.25f;
        point.z = 0.0f;
        cloud->push_back(point);
    }
    ground->replaceMapClouds(cloud);

    const auto snapshot = ground->beginMapCloudRefit();
    ground->completeMapCloudRefit(
        snapshot->cloudGeneration,
        Eigen::Vector3d::Zero(),
        g2o::Plane3D(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0)),
        50U);
    return ground;
}

/*! Builds a long, admissible WALL geometric::Plane whose horizontal (in-plane)
 * span runs along axisAlong_World_in through the origin, at a given normal
 * direction, wide enough to produce a decisive interior crossing. */
std::unique_ptr<geometric::Plane>
    makeLongWallThroughOrigin(int                    id_in,
                              Map                   *p_map_in,
                              const Eigen::Vector3d &normal_World_in,
                              const Eigen::Vector3d &axisAlong_World_in)
{
    auto wall = std::make_unique<geometric::Plane>();
    wall->setId(id_in);
    wall->setMap(p_map_in);
    wall->setPlaneType(geometric::Plane::PlaneVariant::WALL);
    wall->castWeightedVote(geometric::Plane::PlaneVariant::WALL, 1.0);
    wall->setGlobalEquation(g2o::Plane3D(Eigen::Vector4d(normal_World_in.x(),
                                                         normal_World_in.y(),
                                                         normal_World_in.z(),
                                                         0.0)));
    wall->setCentroid(Eigen::Vector3d(0.0, 0.0, 1.5));

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    constexpr int steps = 20;
    for (int alongIndex = 0; alongIndex < steps; ++alongIndex)
    {
        const double along = -2.0 + 4.0 * static_cast<double>(alongIndex) /
                                        static_cast<double>(steps - 1);
        for (int heightIndex = 0; heightIndex < steps; ++heightIndex)
        {
            const double height = 1.0 + 1.0 * static_cast<double>(heightIndex) /
                                            static_cast<double>(steps - 1);
            const Eigen::Vector3d point_World =
                along * axisAlong_World_in + Eigen::Vector3d(0.0, 0.0, height);
            pcl::PointXYZRGBA pclPoint;
            pclPoint.x = static_cast<float>(point_World.x());
            pclPoint.y = static_cast<float>(point_World.y());
            pclPoint.z = static_cast<float>(point_World.z());
            cloud->push_back(pclPoint);
        }
    }
    wall->setMapClouds(cloud);
    return wall;
}

} // namespace

TEST(WallAdmission, RejectsACandidateWallThatCrossesAnotherRoomsWall)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);

    std::unique_ptr<geometric::Plane> ground =
        makeRefitGroundPlaneAtOrigin(1, p_map);
    atlas.addMapPlane(ground.get());

    /* Wall X: normal +Y, runs along world X through the origin -- owned by
     * semantic::Room A. */
    std::unique_ptr<geometric::Plane> wallX =
        makeLongWallThroughOrigin(2,
                                  p_map,
                                  Eigen::Vector3d(0.0, 1.0, 0.0),
                                  Eigen::Vector3d(1.0, 0.0, 0.0));
    semantic::Room roomA;
    roomA.setId(1);
    roomA.setMap(p_map);
    roomA.setWalls(wallX.get());
    atlas.addDetectedMapRoom(&roomA);

    /* Wall Y: normal +X, runs along world Y through the origin -- crosses
     * Wall X decisively at the origin, well inside both walls' interiors. */
    std::unique_ptr<geometric::Plane> wallY =
        makeLongWallThroughOrigin(3,
                                  p_map,
                                  Eigen::Vector3d(1.0, 0.0, 0.0),
                                  Eigen::Vector3d(0.0, 1.0, 0.0));

    semantic::Room roomB;
    roomB.setId(2);
    roomB.setMap(p_map);

    const bool admitted = manager.admitWallToRoomForTest(&roomB, wallY.get());

    EXPECT_FALSE(admitted);
    EXPECT_TRUE(roomB.getWalls().empty());
    ASSERT_EQ(roomA.getWalls().size(), 1U);
    EXPECT_EQ(roomA.getWalls().front(), wallX.get());
}

/* ---------------------------------------------------------------------- *
 * Far-side wall churn: the per-cycle passage-side sweep must not evict a
 * wall from the prospective room the aperture backstop routed it to.
 * Live-observed: Wall#1 removed from semantic::Room#2, redirected back and
 * re-admitted every cycle (42 admissions, stable owner). The wall's
 * observation origin sits on the near side while a deep far-side room's
 * centroid sits beyond the wall plane, so the bare face check always
 * fires; only the aperture exemption keeps the placement stable.
 * ---------------------------------------------------------------------- */

TEST(WallAdmission, SweepKeepsFarSideWallRoutedToItsProspectiveRoom)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);

    /* semantic::Passage at x=0, generous aperture, known near side -X. */
    semantic::Passage passage;
    passage.setId(1);
    passage.setMap(p_map);
    passage.setGlobalEquation(
        g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)));
    passage.setCentroid(Eigen::Vector3d(0.0, 0.0, 0.0));
    passage.setWidth(2.0);
    passage.setHeight(2.0);
    passage.setPassable(true);
    passage.setKnownSideDirection(Eigen::Vector3d(-1.0, 0.0, 0.0));

    /* Prospective far-side room whose centroid lies beyond the wall plane,
     * as a deep room's free-space centroid naturally does. */
    semantic::Room prospective;
    prospective.setId(2);
    prospective.setMap(p_map);
    prospective.setRoomVariant(semantic::Room::RoomVariant::UNDEFINED);
    prospective.setCentroid(Eigen::Vector3d(3.0, 0.0, 0.0));
    passage.setProspectiveRoom(&prospective);

    atlas.addMapPassage(&passage);
    atlas.addDetectedMapRoom(&prospective);

    /* Far-side wall at x=1.5, first observed from the near side. */
    geometric::Plane wall;
    wall.setId(3);
    wall.setMap(p_map);
    wall.setGlobalEquation(g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, -1.5)));
    wall.setCentroid(Eigen::Vector3d(1.5, 0.0, 0.0));
    wall.setObservationOrigin_World(Eigen::Vector3d(-1.0, 0.0, 0.0));

    /* Pre-place the wall as the aperture backstop would have routed it. */
    prospective.setWalls(&wall);

    manager.enforcePassageSideInvariantForTest();

    ASSERT_EQ(prospective.getWalls().size(), 1U);
    EXPECT_EQ(prospective.getWalls().front(), &wall);
}

TEST(WallAdmission, SweepStillEvictsRoutedWallWithoutAKnownSideDirection)
{
    /* Same geometry as above, but the passage has no established
     * near-side direction: the exemption must not guess, so the face
     * check evicts exactly as before the fix. */
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);

    semantic::Passage passage;
    passage.setId(1);
    passage.setMap(p_map);
    passage.setGlobalEquation(
        g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)));
    passage.setCentroid(Eigen::Vector3d(0.0, 0.0, 0.0));
    passage.setWidth(2.0);
    passage.setHeight(2.0);
    passage.setPassable(true);

    semantic::Room prospective;
    prospective.setId(2);
    prospective.setMap(p_map);
    prospective.setRoomVariant(semantic::Room::RoomVariant::UNDEFINED);
    prospective.setCentroid(Eigen::Vector3d(3.0, 0.0, 0.0));
    passage.setProspectiveRoom(&prospective);

    atlas.addMapPassage(&passage);
    atlas.addDetectedMapRoom(&prospective);

    geometric::Plane wall;
    wall.setId(3);
    wall.setMap(p_map);
    wall.setGlobalEquation(g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, -1.5)));
    wall.setCentroid(Eigen::Vector3d(1.5, 0.0, 0.0));
    wall.setObservationOrigin_World(Eigen::Vector3d(-1.0, 0.0, 0.0));

    prospective.setWalls(&wall);

    manager.enforcePassageSideInvariantForTest();

    EXPECT_TRUE(prospective.getWalls().empty());
}

TEST(WallAdmission, BoundsTrimStrayOutliersButKeepTheGridSurface)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();

    /* 3.0 m x 2.0 m grid wall on the x=0 plane: 400 support points. */
    geometric::Plane wall;
    makeWallWithGridCloud(wall,
                          1,
                          p_map,
                          Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                          Eigen::Vector3d(0.0, 1.0, 0.0),
                          Eigen::Vector3d(0.0, 0.0, 1.0),
                          1.5,
                          1.0);

    /* Six far-flung but on-plane strays: without trimming these alone
     * would stretch the quad to +-10 m. */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = wall.getMapClouds();
    for (const double stray : {-10.0, 10.0})
    {
        for (int axis = 0; axis < 3; ++axis)
        {
            pcl::PointXYZRGBA point;
            point.x = 0.0F;
            point.y = 0.0F;
            point.z = 0.0F;
            if (axis == 0)
            {
                point.y = static_cast<float>(stray);
            }
            else if (axis == 1)
            {
                point.z = static_cast<float>(stray);
            }
            else
            {
                point.y = static_cast<float>(stray);
                point.z = static_cast<float>(stray);
            }
            cloud->push_back(point);
        }
    }
    wall.setMapClouds(cloud);
    wall.updateSizeOfPlane();

    const geometric::Plane::GeometrySnapshot geometry =
        wall.getGeometrySnapshot();
    /* Production projects onto its own deterministic in-plane axes: for
     * this x=0 wall axisU is world +Z and axisV is world -Y, so the
     * 3.0 m generator-U span lands on V and the 2.0 m generator-V span
     * lands on U. Grid step is ~0.11 m in U and ~0.16 m in V: trimmed
     * bounds must sit within a couple of steps of the true surface. */
    EXPECT_GT(geometry.minPlaneU_m, -1.3);
    EXPECT_LT(geometry.minPlaneU_m, -0.5);
    EXPECT_GT(geometry.maxPlaneU_m, 0.5);
    EXPECT_LT(geometry.maxPlaneU_m, 1.3);
    EXPECT_GT(geometry.minPlaneV_m, -1.9);
    EXPECT_LT(geometry.minPlaneV_m, -1.0);
    EXPECT_GT(geometry.maxPlaneV_m, 1.0);
    EXPECT_LT(geometry.maxPlaneV_m, 1.9);
}

} // namespace core
} // namespace vs_graphs
