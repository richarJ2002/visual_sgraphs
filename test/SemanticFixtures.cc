/**
 * @file SemanticFixtures.cc
 * @brief Implementation of the deterministic semantic-fixture builders
 *        declared in SemanticFixtures.h (semantic-axiom-reliability-plan.md,
 *        P0.4).
 */

#include "SemanticFixtures.h"

#include <sophus/se3.hpp>

#include <algorithm>

namespace ORB_SLAM3
{
namespace test
{

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
    makeGridCloud(const Eigen::Vector3d &centroid_World_m_in,
                  const Eigen::Vector3d &axisU_World_in,
                  const Eigen::Vector3d &axisV_World_in,
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
            const Eigen::Vector3d point_World =
                centroid_World_m_in + u * axisU_World_in + v * axisV_World_in;

            pcl::PointXYZRGBA pclPoint;
            pclPoint.x = static_cast<float>(point_World.x());
            pclPoint.y = static_cast<float>(point_World.y());
            pclPoint.z = static_cast<float>(point_World.z());
            cloud->push_back(pclPoint);
        }
    }
    return cloud;
}

void makeWallPlane(Plane                 &wall_inout,
                   int                    id_in,
                   Map                   *p_map_in,
                   const Eigen::Vector4d &equation_World_in,
                   const Eigen::Vector3d &axisU_World_in,
                   const Eigen::Vector3d &axisV_World_in,
                   double                 halfU_m_in,
                   double                 halfV_m_in,
                   const Eigen::Vector3d &centroid_World_m_in)
{
    wall_inout.setId(id_in);
    wall_inout.SetMap(p_map_in);
    wall_inout.setPlaneType(Plane::planeVariant::WALL);
    /* Wall-admission/ownership gates compare getPlaneType() against
     * getExpectedPlaneType(), which is derived from semanticVotes rather than
     * settable directly -- cast a vote so the two agree, matching what real
     * wall classification does over time. */
    wall_inout.castWeightedVote(Plane::planeVariant::WALL, 1.0);
    wall_inout.setGlobalEquation(g2o::Plane3D(equation_World_in));
    wall_inout.setCentroid(centroid_World_m_in);
    wall_inout.setMapClouds(makeGridCloud(centroid_World_m_in,
                                          axisU_World_in,
                                          axisV_World_in,
                                          halfU_m_in,
                                          halfV_m_in));
    wall_inout.updateSizeOfPlane();
}

bool makeGroundPlane(Plane &ground_inout,
                     int    id_in,
                     Map   *p_map_in,
                     double halfExtent_m_in,
                     int    stepsPerSide_in)
{
    ground_inout.setId(id_in);
    ground_inout.SetMap(p_map_in);
    ground_inout.setPlaneType(Plane::planeVariant::GROUND);
    ground_inout.setGlobalEquation(
        g2o::Plane3D(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0)));
    ground_inout.setMapClouds(makeGridCloud(Eigen::Vector3d::Zero(),
                                            Eigen::Vector3d::UnitX(),
                                            Eigen::Vector3d::UnitY(),
                                            halfExtent_m_in,
                                            halfExtent_m_in,
                                            stepsPerSide_in));
    return GeoSemHelpers::refitMappedPlaneFromCloud(&ground_inout);
}

void makeRoom(Room                  &room_inout,
              int                    id_in,
              Map                   *p_map_in,
              Plane                 *p_wall_in,
              const Eigen::Vector3d &centroid_World_m_in,
              Room::roomVariant      variant_in)
{
    room_inout.setId(id_in);
    room_inout.setMap(p_map_in);
    room_inout.setRoomVariant(variant_in);
    room_inout.setCentroid(centroid_World_m_in);
    if (p_wall_in != nullptr)
    {
        room_inout.setWalls(p_wall_in);
    }
}

void makePassage(Passage               &passage_inout,
                 int                    id_in,
                 Map                   *p_map_in,
                 const Eigen::Vector4d &equation_World_in,
                 const Eigen::Vector3d &centroid_World_m_in,
                 Room                  *p_knownSideRoom_in,
                 const Eigen::Vector3d &knownSideDirection_World_in,
                 Room                  *p_farRoom_in,
                 bool                   passable_in,
                 double                 width_m_in,
                 double                 height_m_in)
{
    passage_inout.setId(id_in);
    passage_inout.setMap(p_map_in);
    passage_inout.setPassable(passable_in);
    passage_inout.setWidth(width_m_in);
    passage_inout.setHeight(height_m_in);
    passage_inout.setCentroid(centroid_World_m_in);
    passage_inout.setGlobalEquation(g2o::Plane3D(equation_World_in));
    if (p_knownSideRoom_in != nullptr)
    {
        passage_inout.setKnownSideRoom(p_knownSideRoom_in);
        passage_inout.setKnownSideDirection(knownSideDirection_World_in);
    }
    if (p_farRoom_in != nullptr)
    {
        passage_inout.setProspectiveRoom(p_farRoom_in);
    }
}

void makeFloor(Floor                     &floor_inout,
               int                        id_in,
               Map                       *p_map_in,
               const std::vector<Room *> &rooms_in,
               double                     centroidZ_World_m_in)
{
    floor_inout.setId(id_in);
    floor_inout.setMap(p_map_in);
    floor_inout.setCentroid(Eigen::Vector3d(0.0, 0.0, centroidZ_World_m_in));
    floor_inout.setRooms(rooms_in);
}

void makeKeyFrameAt(KeyFrame              &keyFrame_inout,
                    unsigned long          id_in,
                    Map                   *p_map_in,
                    const Eigen::Vector3f &cameraCenter_World_m_in)
{
    keyFrame_inout.mnId = id_in;
    /* KeyFrame::SetPose takes T_camera_World (world -> camera); a camera
     * sitting at cameraCenter_World_m_in under identity orientation has
     * translation -cameraCenter_World_m_in in that convention. */
    keyFrame_inout.SetPose(
        Sophus::SE3f(Eigen::Matrix3f::Identity(), -cameraCenter_World_m_in));
    if (p_map_in != nullptr)
    {
        p_map_in->AddKeyFrame(&keyFrame_inout);
    }
}

g2o::Sim3 makeNonTrivialSim3(double                 rotationAngle_rad_in,
                             const Eigen::Vector3d &rotationAxis_in,
                             const Eigen::Vector3d &translation_World_m_in,
                             double                 scale_in)
{
    const Eigen::Matrix3d rotation_oldWorldToNewWorld =
        Eigen::AngleAxisd(rotationAngle_rad_in, rotationAxis_in.normalized())
            .toRotationMatrix();
    return g2o::Sim3(rotation_oldWorldToNewWorld,
                     translation_World_m_in,
                     scale_in);
}

} // namespace test
} // namespace ORB_SLAM3
