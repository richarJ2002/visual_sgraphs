/**
 * @file SemanticFixtures.h
 * @brief Deterministic, ROS/Gazebo-free builders for semantic-axiom test
 *        fixtures (semantic-axiom-reliability-plan.md, P0.4).
 *
 * Several existing test files (test_WallAdmission.cpp,
 * test_RoomContextPersist.cpp, test_room_tracker_integration.cpp) each
 * hand-rolled their own local copy of "build a wall Plane from a synthetic grid
 * cloud" / "build a KeyFrame at a world camera center" helpers. This header
 * canonicalizes those patterns so later semantic-axiom-plan phases (frame
 * equivariance, passage endpoints, wall ownership, non-convex boundaries) build
 * fixtures the same way instead of re-deriving construction order and required
 * setters from scratch.
 *
 * Every builder here constructs plain, in-memory model objects
 * (Atlas/Map/Plane/Room/Passage/Floor/KeyFrame) with no ROS node, message, or
 * Gazebo dependency, so these fixtures run in a bare GTest binary.
 */

#pragma once

#include "Atlas.h"
#include "GeoSemHelpers.h"
#include "Geometric/Plane.h"
#include "KeyFrame.h"
#include "Map.h"
#include "Semantic/Floor.h"
#include "Semantic/Passage.h"
#include "Semantic/Room.h"

#include "Thirdparty/g2o/g2o/types/sim3.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <vector>

namespace ORB_SLAM3
{
namespace test
{

/**
 * @brief   Builds a deterministic, exactly-on-plane rectangular grid point
 *          cloud spanning [-halfU, halfU] along axisU_World and
 *          [-halfV, halfV] along axisV_World, centered on
 *          centroid_World_m_in.
 *
 * @param   centroid_World_m_in Grid center, world frame, meters.
 * @param   axisU_World_in      Unit in-plane axis; caller ensures orthogonality
 *                               with axisV_World_in.
 * @param   axisV_World_in      Unit in-plane axis; caller ensures orthogonality
 *                               with axisU_World_in.
 * @param   halfU_m_in          Half-extent along axisU_World_in, meters.
 * @param   halfV_m_in          Half-extent along axisV_World_in, meters.
 * @param   stepsPerSide_in     Grid points per side (>= 2).
 *
 * @return  A populated, owned point cloud; never null.
 */
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
    makeGridCloud(const Eigen::Vector3d &centroid_World_m_in,
                  const Eigen::Vector3d &axisU_World_in,
                  const Eigen::Vector3d &axisV_World_in,
                  double                 halfU_m_in,
                  double                 halfV_m_in,
                  int                    stepsPerSide_in = 20);

/**
 * @brief   Constructs a wall Plane with a genuine, production-computed
 *          geometry snapshot: a flat grid cloud fed through the real
 *          Plane::updateSizeOfPlane() path (the same function wall-admission
 *          and ownership resolvers read from), rather than hand-setting
 *          finite-extent fields.
 *
 * The caller owns wall_inout's lifetime; this function only mutates it and
 * registers it as a map plane candidate is left to the caller
 * (Map::AddMapPlane), since some tests intentionally exercise unregistered
 * planes.
 *
 * @param   wall_inout          Plane to initialize; any prior state is
 *                               overwritten.
 * @param   id_in               Plane id.
 * @param   p_map_in            Owning map; non-owning, must outlive wall_inout.
 * @param   equation_World_in   Plane equation (nx, ny, nz, d), world frame.
 * @param   axisU_World_in      In-plane grid axis U, world frame, unit length.
 * @param   axisV_World_in      In-plane grid axis V, world frame, unit length.
 * @param   halfU_m_in          Half-extent along axisU_World_in, meters.
 * @param   halfV_m_in          Half-extent along axisV_World_in, meters.
 * @param   centroid_World_m_in Grid/plane centroid, world frame, meters.
 */
void makeWallPlane(
    Plane                 &wall_inout,
    int                    id_in,
    Map                   *p_map_in,
    const Eigen::Vector4d &equation_World_in,
    const Eigen::Vector3d &axisU_World_in,
    const Eigen::Vector3d &axisV_World_in,
    double                 halfU_m_in,
    double                 halfV_m_in,
    const Eigen::Vector3d &centroid_World_m_in = Eigen::Vector3d::Zero());

/**
 * @brief   Constructs a ground Plane from a flat, horizontal grid cloud and
 *          refits it through the real production path
 *          (GeoSemHelpers::refitMappedPlaneFromCloud), matching how a real
 *          ground plane is derived from segmentation evidence.
 *
 * @param   ground_inout      Plane to initialize; any prior state is
 *                             overwritten.
 * @param   id_in             Plane id.
 * @param   p_map_in          Owning map; non-owning, must outlive ground_inout.
 * @param   halfExtent_m_in   Half-extent of the square ground patch, meters.
 * @param   stepsPerSide_in   Grid points per side (>= 2).
 *
 * @return  Whether the production refit accepted the synthetic cloud (mirrors
 *          GeoSemHelpers::refitMappedPlaneFromCloud's own return contract).
 */
bool makeGroundPlane(Plane &ground_inout,
                     int    id_in,
                     Map   *p_map_in,
                     double halfExtent_m_in = 0.75,
                     int    stepsPerSide_in = 5);

/**
 * @brief   Constructs a minimal, valid Room bound to one wall face.
 *
 * @param   room_inout          Room to initialize; any prior state is
 *                               overwritten.
 * @param   id_in               Room id.
 * @param   p_map_in            Owning map; non-owning, must outlive room_inout.
 * @param   p_wall_in           Wall face to attach; may be null for a room
 *                               fixture that only needs a centroid.
 * @param   centroid_World_m_in Room centroid, world frame, meters.
 * @param   variant_in          Room semantic variant.
 */
void makeRoom(
    Room                  &room_inout,
    int                    id_in,
    Map                   *p_map_in,
    Plane                 *p_wall_in,
    const Eigen::Vector3d &centroid_World_m_in = Eigen::Vector3d::Zero(),
    Room::roomVariant      variant_in          = Room::roomVariant::ROOM);

/**
 * @brief   Constructs a minimal Passage with one known-side room and,
 *          optionally, a prospective or second confirmed far-side room.
 *
 * @param   passage_inout            Passage to initialize; any prior state is
 *                                    overwritten.
 * @param   id_in                    Passage id.
 * @param   p_map_in                 Owning map; non-owning, must outlive
 *                                    passage_inout.
 * @param   equation_World_in        Aperture plane equation, world frame.
 * @param   centroid_World_m_in      Aperture centroid, world frame, meters.
 * @param   p_knownSideRoom_in       Known/near-side room; non-owning, may be
 *                                    null.
 * @param   knownSideDirection_World_in Unit direction from the aperture
 *                                    toward the known-side room, world frame.
 * @param   p_farRoom_in             Prospective or confirmed far-side room;
 *                                    non-owning, may be null.
 * @param   passable_in              Whether the passage carries skeleton-
 *                                    crossing (Voxblox) evidence.
 * @param   width_m_in               Aperture width, meters.
 * @param   height_m_in              Aperture height, meters.
 */
void makePassage(Passage               &passage_inout,
                 int                    id_in,
                 Map                   *p_map_in,
                 const Eigen::Vector4d &equation_World_in,
                 const Eigen::Vector3d &centroid_World_m_in,
                 Room                  *p_knownSideRoom_in,
                 const Eigen::Vector3d &knownSideDirection_World_in,
                 Room                  *p_farRoom_in = nullptr,
                 bool                   passable_in  = true,
                 double                 width_m_in   = 1.0,
                 double                 height_m_in  = 2.0);

/**
 * @brief   Constructs a minimal Floor owning the given rooms.
 *
 * @param   floor_inout           Floor to initialize; any prior state is
 *                                 overwritten.
 * @param   id_in                 Floor id.
 * @param   p_map_in              Owning map; non-owning, must outlive
 *                                 floor_inout.
 * @param   rooms_in              Rooms this floor owns; non-owning pointers.
 * @param   centroidZ_World_m_in  Floor plane height, world frame, meters.
 */
void makeFloor(Floor                     &floor_inout,
               int                        id_in,
               Map                       *p_map_in,
               const std::vector<Room *> &rooms_in,
               double                     centroidZ_World_m_in = 0.0);

/**
 * @brief   Constructs a KeyFrame at the given world camera center with
 *          identity orientation, and registers it with p_map_in.
 *
 * @param   keyFrame_inout           KeyFrame to initialize; any prior state
 *                                    is overwritten.
 * @param   id_in                    KeyFrame id (KeyFrame::mnId).
 * @param   p_map_in                 Map to register the KeyFrame with; non-
 *                                    owning, may be null to skip registration.
 * @param   cameraCenter_World_m_in  Camera center, world frame, meters.
 */
void makeKeyFrameAt(KeyFrame              &keyFrame_inout,
                    unsigned long          id_in,
                    Map                   *p_map_in,
                    const Eigen::Vector3f &cameraCenter_World_m_in);

/**
 * @brief   Builds an arbitrary, non-trivial Sim3 old-world -> new-world
 *          transform (rotation + translation + non-unity scale), for
 *          AX-FRAME-01 equivariance tests: a transform that only permutes
 *          axes or is the identity can hide a missing-field bug that a
 *          generic rotation/scale would catch.
 *
 * @param   rotationAngle_rad_in       Rotation angle about rotationAxis_in.
 * @param   rotationAxis_in            Rotation axis; normalized internally.
 * @param   translation_World_m_in     Translation applied in the new-world
 *                                      frame, meters.
 * @param   scale_in                   Uniform scale; must be > 0.
 *
 * @return  The requested old-world -> new-world Sim3 transform.
 */
g2o::Sim3 makeNonTrivialSim3(
    double                 rotationAngle_rad_in = 0.4,
    const Eigen::Vector3d &rotationAxis_in = Eigen::Vector3d(0.2, 0.7, 0.3),
    const Eigen::Vector3d &translation_World_m_in = Eigen::Vector3d(1.5,
                                                                    -0.8,
                                                                    0.4),
    double                 scale_in               = 1.2);

} // namespace test
} // namespace ORB_SLAM3
