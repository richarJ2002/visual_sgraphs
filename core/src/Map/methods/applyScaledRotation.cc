/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "Map.h"

#include <algorithm>
#include <iterator>
#include <mutex>

namespace vs_graphs
{
namespace core
{

void Map::applyScaledRotation(const Sophus::SE3f &T_in,
                              const float         s_in,
                              const bool          isScaledVelocity_in)
{
    unique_lock<mutex> lock(mapMutex);

    // Body position (IMU) of first keyframe is fixed to (0,0,0)
    Sophus::SE3f    Tyw = T_in;
    Eigen::Matrix3f Ryw = Tyw.rotationMatrix();
    Eigen::Vector3f tyw = Tyw.translation();

    const g2o::Sim3 transform_oldWorldToNewWorld(Ryw.cast<double>(),
                                                 tyw.cast<double>(),
                                                 static_cast<double>(s_in));

    for (set<KeyFrame *>::iterator sit = keyFrames.begin();
         sit != keyFrames.end();
         sit++)
    {
        KeyFrame    *p_keyFrame = *sit;
        Sophus::SE3f Twc        = p_keyFrame->getPoseInverse();
        Twc.translation() *= s_in;
        Sophus::SE3f Tyc = Tyw * Twc;
        Sophus::SE3f Tcy = Tyc.inverse();
        p_keyFrame->setPose(Tcy);
        Eigen::Vector3f Vw = p_keyFrame->getVelocity();
        if (!isScaledVelocity_in)
            p_keyFrame->setVelocity(Ryw * Vw);
        else
            p_keyFrame->setVelocity(Ryw * Vw * s_in);
    }

    for (set<MapPoint *>::iterator sit = mapPoints.begin();
         sit != mapPoints.end();
         sit++)
    {
        MapPoint *p_mapPoint = *sit;
        p_mapPoint->setWorldPos(s_in * Ryw * p_mapPoint->getWorldPos() + tyw);
        p_mapPoint->updateNormalAndDepth();
    }

    for (geometric::Plane *p_plane : planes)
    {
        if (p_plane != nullptr && !p_plane->isBad())
        {
            p_plane->applyTransform(transform_oldWorldToNewWorld);
        }
    }

    for (semantic::Marker *p_marker : markers)
    {
        if (p_marker != nullptr)
        {
            if (p_marker->applyTransform(transform_oldWorldToNewWorld) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                // applyTransform cannot fail; continue as before.
            }
        }
    }

    for (vs_graphs::core::semantic::Passage *p_passage : passages)
    {
        if (p_passage != nullptr)
        {
            if (p_passage->applyTransform(transform_oldWorldToNewWorld) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // applyTransform cannot fail; continue as before.
            }
        }
    }

    for (semantic::Room *p_room : detectedRooms)
    {
        bool roomIsBad{};
        if ((p_room != nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // isBad cannot fail; continue as before.
        }
        if (p_room != nullptr && !roomIsBad)
        {
            if (p_room->applyTransform(transform_oldWorldToNewWorld) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // applyTransform cannot fail; continue as before.
            }
        }
    }

    for (semantic::Room *p_room : markerBasedRooms)
    {
        bool roomIsBad2{};
        if ((p_room != nullptr) &&
            p_room->isBad(roomIsBad2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // isBad cannot fail; continue as before.
        }
        if (p_room != nullptr && !roomIsBad2)
        {
            if (p_room->applyTransform(transform_oldWorldToNewWorld) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // applyTransform cannot fail; continue as before.
            }
        }
    }

    for (semantic::Floor *p_floor : floors)
    {
        if (p_floor != nullptr)
        {
            if (p_floor->applyTransform(transform_oldWorldToNewWorld) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                // applyTransform cannot fail; continue as before.
            }
        }
    }

    for (std::vector<Eigen::Vector3d> &cluster_world : skeletonClusterPoints)
    {
        for (Eigen::Vector3d &point_world_m : cluster_world)
        {
            point_world_m = transform_oldWorldToNewWorld.map(point_world_m);
        }
    }

    for (auto &skeletonEdge_world : skeletonEdges)
    {
        skeletonEdge_world.first =
            transform_oldWorldToNewWorld.map(skeletonEdge_world.first);
        skeletonEdge_world.second =
            transform_oldWorldToNewWorld.map(skeletonEdge_world.second);
    }

    mapChange++;
    worldFrameEpoch++;
}

} // namespace core
} // namespace vs_graphs
