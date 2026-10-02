/*!
 * @file SemanticFixtures.cc
 * @brief Implementation of the deterministic semantic-fixture builders
 *        declared in SemanticFixtures.h (semantic-axiom-reliability-plan.md,
 *        P0.4).
 */

#include "SemanticFixtures.h"

#include <sophus/se3.hpp>

#include <algorithm>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace test
{

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
    makeGridCloud(const Eigen::Vector3d &centroid_world_m_in,
                  const Eigen::Vector3d &axisU_world_in,
                  const Eigen::Vector3d &axisV_world_in,
                  double                 halfU_m_in,
                  double                 halfV_m_in,
                  int                    stepsPerSide_in)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);

    const int steps = std::max(stepsPerSide_in, 2);
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
            const Eigen::Vector3d point_world =
                centroid_world_m_in + u * axisU_world_in + v * axisV_world_in;

            pcl::PointXYZRGBA pclPoint;
            pclPoint.x = static_cast<float>(point_world.x());
            pclPoint.y = static_cast<float>(point_world.y());
            pclPoint.z = static_cast<float>(point_world.z());
            cloud->push_back(pclPoint);
        }
    }
    return cloud;
}

void makeWallPlane(geometric::Plane      &wall_inout,
                   int                    id_in,
                   Map                   *p_map_in,
                   const Eigen::Vector4d &equation_world_in,
                   const Eigen::Vector3d &axisU_world_in,
                   const Eigen::Vector3d &axisV_world_in,
                   double                 halfU_m_in,
                   double                 halfV_m_in,
                   const Eigen::Vector3d &centroid_world_m_in)
{
    if (wall_inout.setId(id_in) != geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (wall_inout.setMap(p_map_in) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (wall_inout.setPlaneType(geometric::Plane::PlaneVariant::WALL) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPlaneType returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    /* Wall-admission/ownership gates compare getPlaneType() against
     * getExpectedPlaneType(), which is derived from semanticVotes rather than
     * settable directly -- cast a vote so the two agree, matching what real
     * wall classification does over time. */
    if (wall_inout.castWeightedVote(geometric::Plane::PlaneVariant::WALL,
                                    1.0) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: castWeightedVote returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (wall_inout.setGlobalEquation(g2o::Plane3D(equation_world_in)) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setGlobalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (wall_inout.setCentroid(centroid_world_m_in) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (wall_inout.setMapClouds(makeGridCloud(centroid_world_m_in,
                                              axisU_world_in,
                                              axisV_world_in,
                                              halfU_m_in,
                                              halfV_m_in)) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMapClouds returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (wall_inout.updateSizeOfPlane() !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateSizeOfPlane returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
}

bool makeGroundPlane(geometric::Plane &ground_inout,
                     int               id_in,
                     Map              *p_map_in,
                     double            halfExtent_m_in,
                     int               stepsPerSide_in)
{
    if (ground_inout.setId(id_in) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (ground_inout.setMap(p_map_in) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (ground_inout.setPlaneType(geometric::Plane::PlaneVariant::GROUND) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPlaneType returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (ground_inout.setGlobalEquation(
            g2o::Plane3D(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0))) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setGlobalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (ground_inout.setMapClouds(makeGridCloud(Eigen::Vector3d::Zero(),
                                                Eigen::Vector3d::UnitX(),
                                                Eigen::Vector3d::UnitY(),
                                                halfExtent_m_in,
                                                halfExtent_m_in,
                                                stepsPerSide_in)) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMapClouds returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool wasPlaneRefit{};
    if (GeoSemHelpers::refitMappedPlaneFromCloud(&ground_inout,
                                                 wasPlaneRefit) !=
        GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: refitMappedPlaneFromCloud returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    return wasPlaneRefit;
}

void makeRoom(semantic::Room             &room_inout,
              int                         id_in,
              Map                        *p_map_in,
              geometric::Plane           *p_wall_in,
              const Eigen::Vector3d      &centroid_world_m_in,
              semantic::Room::RoomVariant variant_in)
{
    if (room_inout.setId(id_in) !=
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (room_inout.setMap(p_map_in) !=
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (room_inout.setRoomVariant(variant_in) !=
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setRoomVariant returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (room_inout.setCentroid(centroid_world_m_in) !=
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_wall_in != nullptr)
    {
        if (room_inout.setWalls(p_wall_in) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }
}

void makePassage(semantic::Passage     &passage_inout,
                 int                    id_in,
                 Map                   *p_map_in,
                 const Eigen::Vector4d &equation_world_in,
                 const Eigen::Vector3d &centroid_world_m_in,
                 semantic::Room        *p_knownSideRoom_in,
                 const Eigen::Vector3d &knownSideDirection_world_in,
                 semantic::Room        *p_farRoom_in,
                 bool                   passable_in,
                 double                 width_m_in,
                 double                 height_m_in)
{
    if (passage_inout.setId(id_in) !=
        vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (passage_inout.setMap(p_map_in) !=
        vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (passage_inout.setPassable(passable_in) !=
        vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPassable returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (passage_inout.setWidth(width_m_in) !=
        vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setWidth returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (passage_inout.setHeight(height_m_in) !=
        vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setHeight returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (passage_inout.setCentroid(centroid_world_m_in) !=
        vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (passage_inout.setGlobalEquation(g2o::Plane3D(equation_world_in)) !=
        vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setGlobalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_knownSideRoom_in != nullptr)
    {
        if (passage_inout.setKnownSideRoom(p_knownSideRoom_in) !=
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setKnownSideRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (passage_inout.setKnownSideDirection(knownSideDirection_world_in) !=
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                        "%s: setKnownSideDirection rejected its input; "
                        "continuing as before.",
                        __func__);
        }
    }
    if (p_farRoom_in != nullptr)
    {
        if (passage_inout.setProspectiveRoom(p_farRoom_in) !=
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setProspectiveRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
}

void makeFloor(semantic::Floor                     &floor_inout,
               int                                  id_in,
               Map                                 *p_map_in,
               const std::vector<semantic::Room *> &rooms_in,
               double                               centroidZ_world_m_in)
{
    if (floor_inout.setId(id_in) !=
        vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (floor_inout.setMap(p_map_in) !=
        vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (floor_inout.setCentroid(
            Eigen::Vector3d(0.0, 0.0, centroidZ_world_m_in)) !=
        vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (floor_inout.setRooms(rooms_in) !=
        vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
}

void makeKeyFrameAt(KeyFrame              &keyFrame_inout,
                    unsigned long          id_in,
                    Map                   *p_map_in,
                    const Eigen::Vector3f &cameraCenter_world_m_in)
{
    keyFrame_inout.id = id_in;
    /* KeyFrame::SetPose takes T_camera_World (world -> camera); a camera
     * sitting at cameraCenter_world_m_in under identity orientation has
     * translation -cameraCenter_world_m_in in that convention. */
    if (keyFrame_inout.setPose(Sophus::SE3f(Eigen::Matrix3f::Identity(),
                                            -cameraCenter_world_m_in)) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_map_in != nullptr)
    {
        if (p_map_in->addKeyFrame(&keyFrame_inout) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addKeyFrame returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
    }
}

g2o::Sim3 makeNonTrivialSim3(double                 rotationAngle_rad_in,
                             const Eigen::Vector3d &rotationAxis_in,
                             const Eigen::Vector3d &translation_world_m_in,
                             double                 scale_in)
{
    const Eigen::Matrix3d rotation_oldWorldToNewWorld =
        Eigen::AngleAxisd(rotationAngle_rad_in, rotationAxis_in.normalized())
            .toRotationMatrix();
    return g2o::Sim3(rotation_oldWorldToNewWorld,
                     translation_world_m_in,
                     scale_in);
}

} // namespace test
} // namespace core
} // namespace vs_graphs
