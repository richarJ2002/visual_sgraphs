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

void Map::applyScaledRotation(const Sophus::SE3f &T,
                              const float         s,
                              const bool          bScaledVel)
{
    unique_lock<mutex> lock(mMutexMap);

    // Body position (IMU) of first keyframe is fixed to (0,0,0)
    Sophus::SE3f    Tyw = T;
    Eigen::Matrix3f Ryw = Tyw.rotationMatrix();
    Eigen::Vector3f tyw = Tyw.translation();

    const g2o::Sim3 transform_oldWorldToNewWorld(Ryw.cast<double>(),
                                                 tyw.cast<double>(),
                                                 static_cast<double>(s));

    for (set<KeyFrame *>::iterator sit = keyFrames.begin();
         sit != keyFrames.end();
         sit++)
    {
        KeyFrame    *pKF = *sit;
        Sophus::SE3f Twc = pKF->getPoseInverse();
        Twc.translation() *= s;
        Sophus::SE3f Tyc = Tyw * Twc;
        Sophus::SE3f Tcy = Tyc.inverse();
        pKF->setPose(Tcy);
        Eigen::Vector3f Vw = pKF->getVelocity();
        if (!bScaledVel)
            pKF->setVelocity(Ryw * Vw);
        else
            pKF->setVelocity(Ryw * Vw * s);
    }

    for (set<MapPoint *>::iterator sit = mapPoints.begin();
         sit != mapPoints.end();
         sit++)
    {
        MapPoint *pMP = *sit;
        pMP->setWorldPos(s * Ryw * pMP->getWorldPos() + tyw);
        pMP->updateNormalAndDepth();
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
            p_marker->applyTransform(transform_oldWorldToNewWorld);
        }
    }

    for (vs_graphs::core::semantic::Passage *p_passage : passages)
    {
        if (p_passage != nullptr)
        {
            p_passage->applyTransform(transform_oldWorldToNewWorld);
        }
    }

    for (semantic::Room *p_room : detectedRooms)
    {
        if (p_room != nullptr && !p_room->isBad())
        {
            p_room->applyTransform(transform_oldWorldToNewWorld);
        }
    }

    for (semantic::Room *p_room : markerBasedRooms)
    {
        if (p_room != nullptr && !p_room->isBad())
        {
            p_room->applyTransform(transform_oldWorldToNewWorld);
        }
    }

    for (semantic::Floor *p_floor : floors)
    {
        if (p_floor != nullptr)
        {
            p_floor->applyTransform(transform_oldWorldToNewWorld);
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
