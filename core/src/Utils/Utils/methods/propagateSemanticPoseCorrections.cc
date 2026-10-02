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
#include <rclcpp/logging.hpp>
#include <set>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus Utils::propagateSemanticPoseCorrections(
    Map                   *p_map_inout,
    const KeyFramePoseMap &keyFramePosesBefore_worldToCamera_in,
    const KeyFramePoseMap &keyFramePosesAfter_worldToCamera_in,
    const g2o::Sim3       &fallbackTransform_oldWorldToNewWorld_in)
{
    if (p_map_inout == nullptr)
    {
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    struct PoseCorrectionNode
    {
        KeyFrame       *p_keyFrame;
        g2o::Sim3       poseBefore_worldToCamera;
        g2o::Sim3       poseAfter_worldToCamera;
        g2o::Sim3       poseCorrection_oldWorldToNewWorld;
        Eigen::Vector3d cameraCenter_oldWorld_m;
    };

    const auto isFiniteSim3 = [](const g2o::Sim3 &transform_in)
    {
        return std::isfinite(transform_in.scale()) &&
               std::abs(transform_in.scale()) > 1e-12 &&
               transform_in.translation().allFinite() &&
               transform_in.rotation().coeffs().allFinite();
    };

    std::vector<PoseCorrectionNode> correctionNodes;
    correctionNodes.reserve(keyFramePosesBefore_worldToCamera_in.size());

    for (const auto &[p_keyFrame, poseBefore_worldToCamera] :
         keyFramePosesBefore_worldToCamera_in)
    {
        bool keyFrameIsBad{};
        if (!(p_keyFrame == nullptr) &&
            p_keyFrame->isBad(keyFrameIsBad) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_keyFrame == nullptr || keyFrameIsBad ||
            !isFiniteSim3(poseBefore_worldToCamera))
        {
            continue;
        }

        const KeyFramePoseMap::const_iterator poseAfterIterator =
            keyFramePosesAfter_worldToCamera_in.find(p_keyFrame);

        if (poseAfterIterator == keyFramePosesAfter_worldToCamera_in.end() ||
            !isFiniteSim3(poseAfterIterator->second))
        {
            continue;
        }

        const g2o::Sim3 poseCorrection_oldWorldToNewWorld =
            poseAfterIterator->second.inverse() * poseBefore_worldToCamera;

        const Eigen::Vector3d cameraCenter_oldWorld_m =
            poseBefore_worldToCamera.inverse().map(Eigen::Vector3d::Zero());

        if (!isFiniteSim3(poseCorrection_oldWorldToNewWorld) ||
            !cameraCenter_oldWorld_m.allFinite())
        {
            continue;
        }

        correctionNodes.push_back({p_keyFrame,
                                   poseBefore_worldToCamera,
                                   poseAfterIterator->second,
                                   poseCorrection_oldWorldToNewWorld,
                                   cameraCenter_oldWorld_m});
    }

    std::sort(
        correctionNodes.begin(),
        correctionNodes.end(),
        [](const PoseCorrectionNode &firstNode_in,
           const PoseCorrectionNode &secondNode_in)
        { return firstNode_in.p_keyFrame->id < secondNode_in.p_keyFrame->id; });

    const auto findNodeForKeyFrame =
        [&correctionNodes](
            const KeyFrame *p_keyFrame_in) -> const PoseCorrectionNode *
    {
        if (p_keyFrame_in == nullptr)
        {
            return nullptr;
        }

        const std::vector<PoseCorrectionNode>::iterator nodeIterator =
            std::find_if(correctionNodes.begin(),
                         correctionNodes.end(),
                         [p_keyFrame_in](const PoseCorrectionNode &node_in)
                         { return node_in.p_keyFrame == p_keyFrame_in; });

        return nodeIterator == correctionNodes.end() ? nullptr
                                                     : &(*nodeIterator);
    };

    const auto findNearestNode =
        [&correctionNodes](const Eigen::Vector3d &semanticPoint_oldWorld_m)
        -> const PoseCorrectionNode *
    {
        if (!semanticPoint_oldWorld_m.allFinite())
        {
            return nullptr;
        }

        const PoseCorrectionNode *p_nearestNode = nullptr;
        double                    nearestSquaredDistance_m2 =
            std::numeric_limits<double>::infinity();

        for (const PoseCorrectionNode &node : correctionNodes)
        {
            const double squaredDistance_m2 =
                (node.cameraCenter_oldWorld_m - semanticPoint_oldWorld_m)
                    .squaredNorm();

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
            const Eigen::Vector3d &semanticPoint_oldWorld_m)
        -> const g2o::Sim3 &
    {
        const PoseCorrectionNode *p_nearestNode =
            findNearestNode(semanticPoint_oldWorld_m);

        return p_nearestNode != nullptr
                   ? p_nearestNode->poseCorrection_oldWorldToNewWorld
                   : fallbackTransform_oldWorldToNewWorld_in;
    };

    std::map<geometric::Plane *, g2o::Sim3> planeCorrections_oldWorldToNewWorld;
    std::map<geometric::Plane *, Eigen::Vector3d> planeCentroids_oldWorld_m;

    std::vector<geometric::Plane *> mapAllPlanes{};
    if (p_map_inout->getAllPlanes(mapAllPlanes) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (geometric::Plane *p_plane : mapAllPlanes)
    {
        bool planeIsBad{};
        if (!(p_plane == nullptr) &&
            p_plane->isBad(planeIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_plane == nullptr || planeIsBad)
        {
            continue;
        }

        Eigen::Vector3d planeCentroid_oldWorld_m{};
        if (p_plane->getCentroid(planeCentroid_oldWorld_m) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        planeCentroids_oldWorld_m.insert_or_assign(p_plane,
                                                   planeCentroid_oldWorld_m);

        const PoseCorrectionNode *p_selectedNode =
            findNodeForKeyFrame(p_plane->p_refKeyFrame);

        if (p_selectedNode == nullptr)
        {
            double nearestObserverSquaredDistance_m2 =
                std::numeric_limits<double>::infinity();

            std::map<core::KeyFrame *, geometric::Plane::Observation>
                planeGetObservations{};
            if (p_plane->getObservations(planeGetObservations) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getObservations returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (const auto &[p_observingKeyFrame, observation] :
                 planeGetObservations)
            {
                (void)observation;

                const PoseCorrectionNode *p_observerNode =
                    findNodeForKeyFrame(p_observingKeyFrame);

                if (p_observerNode == nullptr)
                {
                    continue;
                }

                const double squaredDistance_m2 =
                    (p_observerNode->cameraCenter_oldWorld_m -
                     planeCentroid_oldWorld_m)
                        .squaredNorm();

                if (squaredDistance_m2 < nearestObserverSquaredDistance_m2)
                {
                    nearestObserverSquaredDistance_m2 = squaredDistance_m2;
                    p_selectedNode                    = p_observerNode;
                }
            }
        }

        const g2o::Sim3 &poseCorrection_oldWorldToNewWorld =
            p_selectedNode != nullptr
                ? p_selectedNode->poseCorrection_oldWorldToNewWorld
                : selectCorrectionForPoint(planeCentroid_oldWorld_m);

        if (p_plane->applyTransform(poseCorrection_oldWorldToNewWorld) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: applyTransform returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        planeCorrections_oldWorldToNewWorld.insert_or_assign(
            p_plane,
            poseCorrection_oldWorldToNewWorld);
    }

    std::map<semantic::Marker *, g2o::Sim3>
        markerCorrections_oldWorldToNewWorld;

    std::vector<semantic::Marker *> mapAllMarkers{};
    if (p_map_inout->getAllMarkers(mapAllMarkers) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMarkers returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (semantic::Marker *p_marker : mapAllMarkers)
    {
        if (p_marker == nullptr)
        {
            continue;
        }

        Sophus::SE3f markerPose_markerToOldWorld{};
        if (p_marker->getGlobalPose(markerPose_markerToOldWorld) !=
            semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalPose returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        const PoseCorrectionNode *p_selectedNode = nullptr;
        double                    smallestReconstructionError_m2 =
            std::numeric_limits<double>::infinity();

        std::map<core::KeyFrame *, Sophus::SE3f> markerObservations{};
        if (p_marker->getObservations(markerObservations) !=
            semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getObservations returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (const auto &[p_observingKeyFrame, markerPose_markerToCamera] :
             markerObservations)
        {
            const PoseCorrectionNode *p_observerNode =
                findNodeForKeyFrame(p_observingKeyFrame);

            if (p_observerNode == nullptr)
            {
                continue;
            }

            const Eigen::Vector3d reconstructedPosition_oldWorld_m =
                p_observerNode->poseBefore_worldToCamera.inverse().map(
                    markerPose_markerToCamera.translation().cast<double>());

            const double reconstructionError_m2 =
                (reconstructedPosition_oldWorld_m -
                 markerPose_markerToOldWorld.translation().cast<double>())
                    .squaredNorm();

            if (reconstructionError_m2 < smallestReconstructionError_m2)
            {
                smallestReconstructionError_m2 = reconstructionError_m2;
                p_selectedNode                 = p_observerNode;
            }
        }

        const g2o::Sim3 &markerCorrection_oldWorldToNewWorld =
            p_selectedNode != nullptr
                ? p_selectedNode->poseCorrection_oldWorldToNewWorld
                : selectCorrectionForPoint(
                      markerPose_markerToOldWorld.translation().cast<double>());

        /*
         * Preserve the fused global marker estimate. Reconstructing it from a
         * single noisy observation would move the marker even for an identity
         * pose correction, and repeated GBA/remerge cycles would accumulate
         * that observation noise.
         */
        if (p_marker->applyTransform(markerCorrection_oldWorldToNewWorld) !=
            semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: applyTransform returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        markerCorrections_oldWorldToNewWorld.insert_or_assign(
            p_marker,
            markerCorrection_oldWorldToNewWorld);
    }

    std::vector<vs_graphs::core::semantic::Passage *> mapAllPassages{};
    if (p_map_inout->getAllPassages(mapAllPassages) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassages returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (vs_graphs::core::semantic::Passage *p_passage : mapAllPassages)
    {
        if (p_passage == nullptr)
        {
            continue;
        }

        Eigen::Vector3d passageCentroid_oldWorld_m{};
        if (p_passage->getCentroid(passageCentroid_oldWorld_m) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        const g2o::Sim3 *p_passageCorrection = nullptr;

        vs_graphs::core::geometric::Plane *p_passageAssociateDoor = nullptr;
        if (p_passage->getAssociateDoor(p_passageAssociateDoor) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAssociateDoor returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (geometric::Plane *p_doorPlane = p_passageAssociateDoor;
            p_doorPlane != nullptr)
        {
            const std::map<geometric::Plane *, g2o::Sim3>::iterator
                correctionIterator =
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

            std::vector<vs_graphs::core::geometric::Plane *>
                passageAssociateWalls{};
            if (p_passage->getAssociateWalls(passageAssociateWalls) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAssociateWalls returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (geometric::Plane *p_wall : passageAssociateWalls)
            {
                const std::map<geometric::Plane *, g2o::Sim3>::iterator
                    correctionIterator =
                        planeCorrections_oldWorldToNewWorld.find(p_wall);

                if (p_wall == nullptr ||
                    correctionIterator ==
                        planeCorrections_oldWorldToNewWorld.end())
                {
                    continue;
                }

                const std::map<geometric::Plane *, Eigen::Vector3d>::iterator
                    centroidIterator = planeCentroids_oldWorld_m.find(p_wall);

                if (centroidIterator == planeCentroids_oldWorld_m.end())
                {
                    continue;
                }

                const double squaredDistance_m2 =
                    (centroidIterator->second - passageCentroid_oldWorld_m)
                        .squaredNorm();

                if (squaredDistance_m2 < closestWallSquaredDistance_m2)
                {
                    closestWallSquaredDistance_m2 = squaredDistance_m2;
                    p_passageCorrection           = &correctionIterator->second;
                }
            }
        }

        if (p_passage->applyTransform(
                p_passageCorrection != nullptr
                    ? *p_passageCorrection
                    : selectCorrectionForPoint(passageCentroid_oldWorld_m)) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: applyTransform returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    std::map<semantic::Room *, g2o::Sim3> roomCorrections_oldWorldToNewWorld;
    std::map<semantic::Room *, Eigen::Vector3d> roomCentroids_oldWorld_m;
    std::set<semantic::Room *>                  correctedRooms;
    std::vector<semantic::Room *>               rooms{};
    if (p_map_inout->getAllDetectedMapRooms(rooms) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllDetectedMapRooms returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::Room *> markerRooms{};
    if (p_map_inout->getAllMarkerBasedMapRooms(markerRooms) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMarkerBasedMapRooms returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    rooms.insert(rooms.end(), markerRooms.begin(), markerRooms.end());

    for (semantic::Room *p_room : rooms)
    {
        bool roomIsBad{};
        if (!(p_room == nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room == nullptr || roomIsBad ||
            !correctedRooms.insert(p_room).second)
        {
            continue;
        }

        Eigen::Vector3d roomCentroid_oldWorld_m{};
        if (p_room->getCentroid(roomCentroid_oldWorld_m) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        roomCentroids_oldWorld_m.insert_or_assign(p_room,
                                                  roomCentroid_oldWorld_m);

        const g2o::Sim3  *p_roomCorrection = nullptr;
        semantic::Marker *p_metaMarker     = nullptr;
        if (p_room->getMetaMarker(p_metaMarker) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMetaMarker returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        if (p_metaMarker != nullptr)
        {
            const std::map<semantic::Marker *, g2o::Sim3>::iterator
                markerCorrectionIterator =
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

            std::vector<geometric::Plane *> roomWalls{};
            if (p_room->getWalls(roomWalls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWalls returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            for (geometric::Plane *p_wall : roomWalls)
            {
                const std::map<geometric::Plane *, g2o::Sim3>::iterator
                    correctionIterator =
                        planeCorrections_oldWorldToNewWorld.find(p_wall);
                const std::map<geometric::Plane *, Eigen::Vector3d>::iterator
                    centroidIterator = planeCentroids_oldWorld_m.find(p_wall);

                if (correctionIterator ==
                        planeCorrections_oldWorldToNewWorld.end() ||
                    centroidIterator == planeCentroids_oldWorld_m.end())
                {
                    continue;
                }

                const double squaredDistance_m2 =
                    (centroidIterator->second - roomCentroid_oldWorld_m)
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
                : selectCorrectionForPoint(roomCentroid_oldWorld_m);

        if (p_room->applyTransform(roomCorrection_oldWorldToNewWorld) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: applyTransform returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        roomCorrections_oldWorldToNewWorld.insert_or_assign(
            p_room,
            roomCorrection_oldWorldToNewWorld);
    }

    std::vector<semantic::Floor *> mapAllFloors{};
    if (p_map_inout->getAllFloors(mapAllFloors) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllFloors returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (semantic::Floor *p_floor : mapAllFloors)
    {
        if (p_floor == nullptr)
        {
            continue;
        }

        Eigen::Vector3d floorCentroid_oldWorld_m{};
        if (p_floor->getCentroid(floorCentroid_oldWorld_m) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        const g2o::Sim3 *p_floorCorrection = nullptr;
        double           closestRoomSquaredDistance_m2 =
            std::numeric_limits<double>::infinity();

        std::vector<vs_graphs::core::semantic::Room *> floorRooms{};
        if (p_floor->getRooms(floorRooms) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRooms returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        for (semantic::Room *p_room : floorRooms)
        {
            bool roomIsBad2{};
            if (!(p_room == nullptr) &&
                p_room->isBad(roomIsBad2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            core::Map *p_roomMap = nullptr;
            if (!(p_room == nullptr || roomIsBad2) &&
                p_room->getMap(p_roomMap) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room == nullptr || roomIsBad2 || p_roomMap != p_map_inout)
            {
                continue;
            }

            const std::map<semantic::Room *, g2o::Sim3>::iterator
                correctionIterator =
                    roomCorrections_oldWorldToNewWorld.find(p_room);
            const std::map<semantic::Room *, Eigen::Vector3d>::iterator
                centroidIterator = roomCentroids_oldWorld_m.find(p_room);

            if (correctionIterator ==
                    roomCorrections_oldWorldToNewWorld.end() ||
                centroidIterator == roomCentroids_oldWorld_m.end())
            {
                continue;
            }

            const double squaredDistance_m2 =
                (centroidIterator->second - floorCentroid_oldWorld_m)
                    .squaredNorm();

            if (squaredDistance_m2 < closestRoomSquaredDistance_m2)
            {
                closestRoomSquaredDistance_m2 = squaredDistance_m2;
                p_floorCorrection             = &correctionIterator->second;
            }
        }

        if (p_floor->applyTransform(
                p_floorCorrection != nullptr
                    ? *p_floorCorrection
                    : selectCorrectionForPoint(floorCentroid_oldWorld_m)) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: applyTransform returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    std::vector<std::vector<Eigen::Vector3d>> skeletonClusters_oldWorld_m{};
    if (p_map_inout->getSkeletonClusterPoints(skeletonClusters_oldWorld_m) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSkeletonClusterPoints returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    for (std::vector<Eigen::Vector3d> &skeletonCluster_oldWorld_m :
         skeletonClusters_oldWorld_m)
    {
        if (skeletonCluster_oldWorld_m.empty())
        {
            continue;
        }

        Eigen::Vector3d clusterCentroid_oldWorld_m = Eigen::Vector3d::Zero();
        for (const Eigen::Vector3d &semanticPoint_oldWorld_m :
             skeletonCluster_oldWorld_m)
        {
            clusterCentroid_oldWorld_m += semanticPoint_oldWorld_m;
        }
        clusterCentroid_oldWorld_m /=
            static_cast<double>(skeletonCluster_oldWorld_m.size());

        const g2o::Sim3 &clusterCorrection_oldWorldToNewWorld =
            selectCorrectionForPoint(clusterCentroid_oldWorld_m);

        for (Eigen::Vector3d &semanticPoint_oldWorld_m :
             skeletonCluster_oldWorld_m)
        {
            semanticPoint_oldWorld_m = clusterCorrection_oldWorldToNewWorld.map(
                semanticPoint_oldWorld_m);
        }
    }

    if (p_map_inout->setSkeletonClusterPoints(skeletonClusters_oldWorld_m) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setSkeletonClusterPoints returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        skeletonEdges_oldWorld_m{};
    if (p_map_inout->getSkeletonEdges(skeletonEdges_oldWorld_m) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSkeletonEdges returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    for (std::pair<Eigen::Vector3d, Eigen::Vector3d> &skeletonEdge_oldWorld_m :
         skeletonEdges_oldWorld_m)
    {
        const Eigen::Vector3d firstEndpoint_oldWorld_m =
            skeletonEdge_oldWorld_m.first;
        const Eigen::Vector3d secondEndpoint_oldWorld_m =
            skeletonEdge_oldWorld_m.second;

        skeletonEdge_oldWorld_m.first =
            selectCorrectionForPoint(firstEndpoint_oldWorld_m)
                .map(firstEndpoint_oldWorld_m);
        skeletonEdge_oldWorld_m.second =
            selectCorrectionForPoint(secondEndpoint_oldWorld_m)
                .map(secondEndpoint_oldWorld_m);
    }

    if (p_map_inout->setSkeletonEdges(skeletonEdges_oldWorld_m) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setSkeletonEdges returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
