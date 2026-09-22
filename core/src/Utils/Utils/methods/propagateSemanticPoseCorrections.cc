/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            propagateSemanticPoseCorrections.cc
 *
 * @brief           Implements
 *                  Utils::propagateSemanticPoseCorrections(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

void Utils::propagateSemanticPoseCorrections(
    Map                   *p_map_inout,
    const KeyFramePoseMap &keyFramePosesBefore_WorldToCamera_in,
    const KeyFramePoseMap &keyFramePosesAfter_WorldToCamera_in,
    const g2o::Sim3       &fallbackTransform_oldWorldToNewWorld_in)
{
    if (p_map_inout == nullptr)
    {
        return;
    }

    struct PoseCorrectionNode
    {
        KeyFrame       *p_keyFrame;
        g2o::Sim3       poseBefore_WorldToCamera;
        g2o::Sim3       poseAfter_WorldToCamera;
        g2o::Sim3       correction_oldWorldToNewWorld;
        Eigen::Vector3d cameraCenter_OldWorld_m;
    };

    const auto isFiniteSim3 = [](const g2o::Sim3 &transform_in)
    {
        return std::isfinite(transform_in.scale()) &&
               std::abs(transform_in.scale()) > 1e-12 &&
               transform_in.translation().allFinite() &&
               transform_in.rotation().coeffs().allFinite();
    };

    std::vector<PoseCorrectionNode> correctionNodes;
    correctionNodes.reserve(keyFramePosesBefore_WorldToCamera_in.size());

    for (const auto &[p_keyFrame, poseBefore_WorldToCamera] :
         keyFramePosesBefore_WorldToCamera_in)
    {
        if (p_keyFrame == nullptr || p_keyFrame->isBad() ||
            !isFiniteSim3(poseBefore_WorldToCamera))
        {
            continue;
        }

        const auto poseAfterIterator =
            keyFramePosesAfter_WorldToCamera_in.find(p_keyFrame);

        if (poseAfterIterator == keyFramePosesAfter_WorldToCamera_in.end() ||
            !isFiniteSim3(poseAfterIterator->second))
        {
            continue;
        }

        const g2o::Sim3 correction_oldWorldToNewWorld =
            poseAfterIterator->second.inverse() * poseBefore_WorldToCamera;

        const Eigen::Vector3d cameraCenter_OldWorld_m =
            poseBefore_WorldToCamera.inverse().map(Eigen::Vector3d::Zero());

        if (!isFiniteSim3(correction_oldWorldToNewWorld) ||
            !cameraCenter_OldWorld_m.allFinite())
        {
            continue;
        }

        correctionNodes.push_back({p_keyFrame,
                                   poseBefore_WorldToCamera,
                                   poseAfterIterator->second,
                                   correction_oldWorldToNewWorld,
                                   cameraCenter_OldWorld_m});
    }

    std::sort(correctionNodes.begin(),
              correctionNodes.end(),
              [](const PoseCorrectionNode &firstNode_in,
                 const PoseCorrectionNode &secondNode_in) {
                  return firstNode_in.p_keyFrame->mnId <
                         secondNode_in.p_keyFrame->mnId;
              });

    const auto findNodeForKeyFrame =
        [&correctionNodes](
            const KeyFrame *p_keyFrame_in) -> const PoseCorrectionNode *
    {
        if (p_keyFrame_in == nullptr)
        {
            return nullptr;
        }

        const auto nodeIterator =
            std::find_if(correctionNodes.begin(),
                         correctionNodes.end(),
                         [p_keyFrame_in](const PoseCorrectionNode &node_in)
                         { return node_in.p_keyFrame == p_keyFrame_in; });

        return nodeIterator == correctionNodes.end() ? nullptr
                                                     : &(*nodeIterator);
    };

    const auto findNearestNode =
        [&correctionNodes](const Eigen::Vector3d &point_OldWorld_m)
        -> const PoseCorrectionNode *
    {
        if (!point_OldWorld_m.allFinite())
        {
            return nullptr;
        }

        const PoseCorrectionNode *p_nearestNode = nullptr;
        double                    nearestSquaredDistance_m2 =
            std::numeric_limits<double>::infinity();

        for (const PoseCorrectionNode &node : correctionNodes)
        {
            const double squaredDistance_m2 =
                (node.cameraCenter_OldWorld_m - point_OldWorld_m).squaredNorm();

            if (squaredDistance_m2 < nearestSquaredDistance_m2)
            {
                nearestSquaredDistance_m2 = squaredDistance_m2;
                p_nearestNode             = &node;
            }
        }

        return p_nearestNode;
    };

    const auto selectCorrectionForPoint =
        [&findNearestNode, &fallbackTransform_oldWorldToNewWorld_in](
            const Eigen::Vector3d &point_OldWorld_m) -> const g2o::Sim3 &
    {
        const PoseCorrectionNode *p_nearestNode =
            findNearestNode(point_OldWorld_m);

        return p_nearestNode != nullptr
                   ? p_nearestNode->correction_oldWorldToNewWorld
                   : fallbackTransform_oldWorldToNewWorld_in;
    };

    std::map<geometric::Plane *, g2o::Sim3> planeCorrections_oldWorldToNewWorld;
    std::map<geometric::Plane *, Eigen::Vector3d> planeCentroids_OldWorld_m;

    for (geometric::Plane *p_plane : p_map_inout->getAllPlanes())
    {
        if (p_plane == nullptr || p_plane->isBad())
        {
            continue;
        }

        const Eigen::Vector3d planeCentroid_OldWorld_m = p_plane->getCentroid();

        planeCentroids_OldWorld_m.insert_or_assign(p_plane,
                                                   planeCentroid_OldWorld_m);

        const PoseCorrectionNode *p_selectedNode =
            findNodeForKeyFrame(p_plane->p_refKeyFrame);

        if (p_selectedNode == nullptr)
        {
            double nearestObserverSquaredDistance_m2 =
                std::numeric_limits<double>::infinity();

            for (const auto &[p_observingKeyFrame, observation] :
                 p_plane->getObservations())
            {
                (void)observation;

                const PoseCorrectionNode *p_observerNode =
                    findNodeForKeyFrame(p_observingKeyFrame);

                if (p_observerNode == nullptr)
                {
                    continue;
                }

                const double squaredDistance_m2 =
                    (p_observerNode->cameraCenter_OldWorld_m -
                     planeCentroid_OldWorld_m)
                        .squaredNorm();

                if (squaredDistance_m2 < nearestObserverSquaredDistance_m2)
                {
                    nearestObserverSquaredDistance_m2 = squaredDistance_m2;
                    p_selectedNode                    = p_observerNode;
                }
            }
        }

        const g2o::Sim3 &correction_oldWorldToNewWorld =
            p_selectedNode != nullptr
                ? p_selectedNode->correction_oldWorldToNewWorld
                : selectCorrectionForPoint(planeCentroid_OldWorld_m);

        p_plane->applyTransform(correction_oldWorldToNewWorld);
        planeCorrections_oldWorldToNewWorld.insert_or_assign(
            p_plane,
            correction_oldWorldToNewWorld);
    }

    std::map<semantic::Marker *, g2o::Sim3>
        markerCorrections_oldWorldToNewWorld;

    for (semantic::Marker *p_marker : p_map_inout->getAllMarkers())
    {
        if (p_marker == nullptr)
        {
            continue;
        }

        const Sophus::SE3f markerPose_MarkerToOldWorld =
            p_marker->getGlobalPose();

        const PoseCorrectionNode *p_selectedNode = nullptr;
        double                    smallestReconstructionError_m2 =
            std::numeric_limits<double>::infinity();

        for (const auto &[p_observingKeyFrame, markerPose_MarkerToCamera] :
             p_marker->getObservations())
        {
            const PoseCorrectionNode *p_observerNode =
                findNodeForKeyFrame(p_observingKeyFrame);

            if (p_observerNode == nullptr)
            {
                continue;
            }

            const Eigen::Vector3d reconstructedPosition_OldWorld_m =
                p_observerNode->poseBefore_WorldToCamera.inverse().map(
                    markerPose_MarkerToCamera.translation().cast<double>());

            const double reconstructionError_m2 =
                (reconstructedPosition_OldWorld_m -
                 markerPose_MarkerToOldWorld.translation().cast<double>())
                    .squaredNorm();

            if (reconstructionError_m2 < smallestReconstructionError_m2)
            {
                smallestReconstructionError_m2 = reconstructionError_m2;
                p_selectedNode                 = p_observerNode;
            }
        }

        const g2o::Sim3 &markerCorrection_oldWorldToNewWorld =
            p_selectedNode != nullptr
                ? p_selectedNode->correction_oldWorldToNewWorld
                : selectCorrectionForPoint(
                      markerPose_MarkerToOldWorld.translation().cast<double>());

        /*
         * Preserve the fused global marker estimate. Reconstructing it from a
         * single noisy observation would move the marker even for an identity
         * pose correction, and repeated GBA/remerge cycles would accumulate
         * that observation noise.
         */
        p_marker->applyTransform(markerCorrection_oldWorldToNewWorld);
        markerCorrections_oldWorldToNewWorld.insert_or_assign(
            p_marker,
            markerCorrection_oldWorldToNewWorld);
    }

    for (vs_graphs::core::semantic::Passage *p_passage :
         p_map_inout->getAllPassages())
    {
        if (p_passage == nullptr)
        {
            continue;
        }

        const Eigen::Vector3d passageCentroid_OldWorld_m =
            p_passage->getCentroid();

        const g2o::Sim3 *p_passageCorrection = nullptr;

        if (geometric::Plane *p_doorPlane = p_passage->getAssociateDoor();
            p_doorPlane != nullptr)
        {
            const auto correctionIterator =
                planeCorrections_oldWorldToNewWorld.find(p_doorPlane);

            if (correctionIterator != planeCorrections_oldWorldToNewWorld.end())
            {
                p_passageCorrection = &correctionIterator->second;
            }
        }

        if (p_passageCorrection == nullptr)
        {
            double closestWallSquaredDistance_m2 =
                std::numeric_limits<double>::infinity();

            for (geometric::Plane *p_wall : p_passage->getAssociateWalls())
            {
                const auto correctionIterator =
                    planeCorrections_oldWorldToNewWorld.find(p_wall);

                if (p_wall == nullptr ||
                    correctionIterator ==
                        planeCorrections_oldWorldToNewWorld.end())
                {
                    continue;
                }

                const auto centroidIterator =
                    planeCentroids_OldWorld_m.find(p_wall);

                if (centroidIterator == planeCentroids_OldWorld_m.end())
                {
                    continue;
                }

                const double squaredDistance_m2 =
                    (centroidIterator->second - passageCentroid_OldWorld_m)
                        .squaredNorm();

                if (squaredDistance_m2 < closestWallSquaredDistance_m2)
                {
                    closestWallSquaredDistance_m2 = squaredDistance_m2;
                    p_passageCorrection           = &correctionIterator->second;
                }
            }
        }

        p_passage->applyTransform(
            p_passageCorrection != nullptr
                ? *p_passageCorrection
                : selectCorrectionForPoint(passageCentroid_OldWorld_m));
    }

    std::map<semantic::Room *, g2o::Sim3> roomCorrections_oldWorldToNewWorld;
    std::map<semantic::Room *, Eigen::Vector3d> roomCentroids_OldWorld_m;
    std::set<semantic::Room *>                  correctedRooms;
    std::vector<semantic::Room *> rooms = p_map_inout->getAllDetectedMapRooms();
    const std::vector<semantic::Room *> markerRooms =
        p_map_inout->getAllMarkerBasedMapRooms();
    rooms.insert(rooms.end(), markerRooms.begin(), markerRooms.end());

    for (semantic::Room *p_room : rooms)
    {
        if (p_room == nullptr || p_room->isBad() ||
            !correctedRooms.insert(p_room).second)
        {
            continue;
        }

        const Eigen::Vector3d roomCentroid_OldWorld_m = p_room->getCentroid();
        roomCentroids_OldWorld_m.insert_or_assign(p_room,
                                                  roomCentroid_OldWorld_m);

        const g2o::Sim3  *p_roomCorrection = nullptr;
        semantic::Marker *p_metaMarker     = p_room->getMetaMarker();

        if (p_metaMarker != nullptr)
        {
            const auto markerCorrectionIterator =
                markerCorrections_oldWorldToNewWorld.find(p_metaMarker);

            if (markerCorrectionIterator !=
                markerCorrections_oldWorldToNewWorld.end())
            {
                p_roomCorrection = &markerCorrectionIterator->second;
            }
        }

        if (p_roomCorrection == nullptr)
        {
            double closestWallSquaredDistance_m2 =
                std::numeric_limits<double>::infinity();

            for (geometric::Plane *p_wall : p_room->getWalls())
            {
                const auto correctionIterator =
                    planeCorrections_oldWorldToNewWorld.find(p_wall);
                const auto centroidIterator =
                    planeCentroids_OldWorld_m.find(p_wall);

                if (correctionIterator ==
                        planeCorrections_oldWorldToNewWorld.end() ||
                    centroidIterator == planeCentroids_OldWorld_m.end())
                {
                    continue;
                }

                const double squaredDistance_m2 =
                    (centroidIterator->second - roomCentroid_OldWorld_m)
                        .squaredNorm();

                if (squaredDistance_m2 < closestWallSquaredDistance_m2)
                {
                    closestWallSquaredDistance_m2 = squaredDistance_m2;
                    p_roomCorrection              = &correctionIterator->second;
                }
            }
        }

        const g2o::Sim3 &roomCorrection_oldWorldToNewWorld =
            p_roomCorrection != nullptr
                ? *p_roomCorrection
                : selectCorrectionForPoint(roomCentroid_OldWorld_m);

        p_room->applyTransform(roomCorrection_oldWorldToNewWorld);
        roomCorrections_oldWorldToNewWorld.insert_or_assign(
            p_room,
            roomCorrection_oldWorldToNewWorld);
    }

    for (semantic::Floor *p_floor : p_map_inout->getAllFloors())
    {
        if (p_floor == nullptr)
        {
            continue;
        }

        const Eigen::Vector3d floorCentroid_OldWorld_m = p_floor->getCentroid();

        const g2o::Sim3 *p_floorCorrection = nullptr;
        double           closestRoomSquaredDistance_m2 =
            std::numeric_limits<double>::infinity();

        for (semantic::Room *p_room : p_floor->getRooms())
        {
            if (p_room == nullptr || p_room->isBad() ||
                p_room->getMap() != p_map_inout)
            {
                continue;
            }

            const auto correctionIterator =
                roomCorrections_oldWorldToNewWorld.find(p_room);
            const auto centroidIterator = roomCentroids_OldWorld_m.find(p_room);

            if (correctionIterator ==
                    roomCorrections_oldWorldToNewWorld.end() ||
                centroidIterator == roomCentroids_OldWorld_m.end())
            {
                continue;
            }

            const double squaredDistance_m2 =
                (centroidIterator->second - floorCentroid_OldWorld_m)
                    .squaredNorm();

            if (squaredDistance_m2 < closestRoomSquaredDistance_m2)
            {
                closestRoomSquaredDistance_m2 = squaredDistance_m2;
                p_floorCorrection             = &correctionIterator->second;
            }
        }

        p_floor->applyTransform(
            p_floorCorrection != nullptr
                ? *p_floorCorrection
                : selectCorrectionForPoint(floorCentroid_OldWorld_m));
    }

    auto skeletonClusters_OldWorld_m = p_map_inout->getSkeletonClusterPoints();

    for (std::vector<Eigen::Vector3d> &cluster_OldWorld_m :
         skeletonClusters_OldWorld_m)
    {
        if (cluster_OldWorld_m.empty())
        {
            continue;
        }

        Eigen::Vector3d clusterCentroid_OldWorld_m = Eigen::Vector3d::Zero();
        for (const Eigen::Vector3d &point_OldWorld_m : cluster_OldWorld_m)
        {
            clusterCentroid_OldWorld_m += point_OldWorld_m;
        }
        clusterCentroid_OldWorld_m /=
            static_cast<double>(cluster_OldWorld_m.size());

        const g2o::Sim3 &clusterCorrection_oldWorldToNewWorld =
            selectCorrectionForPoint(clusterCentroid_OldWorld_m);

        for (Eigen::Vector3d &point_OldWorld_m : cluster_OldWorld_m)
        {
            point_OldWorld_m =
                clusterCorrection_oldWorldToNewWorld.map(point_OldWorld_m);
        }
    }

    p_map_inout->setSkeletonClusterPoints(skeletonClusters_OldWorld_m);

    auto skeletonEdges_OldWorld_m = p_map_inout->getSkeletonEdges();

    for (auto &edge_OldWorld_m : skeletonEdges_OldWorld_m)
    {
        const Eigen::Vector3d firstEndpoint_OldWorld_m = edge_OldWorld_m.first;
        const Eigen::Vector3d secondEndpoint_OldWorld_m =
            edge_OldWorld_m.second;

        edge_OldWorld_m.first =
            selectCorrectionForPoint(firstEndpoint_OldWorld_m)
                .map(firstEndpoint_OldWorld_m);
        edge_OldWorld_m.second =
            selectCorrectionForPoint(secondEndpoint_OldWorld_m)
                .map(secondEndpoint_OldWorld_m);
    }

    p_map_inout->setSkeletonEdges(skeletonEdges_OldWorld_m);
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
