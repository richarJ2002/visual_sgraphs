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
 * @file            reAssociateSemanticPlanes.cc
 *
 * @brief           Implements Utils::reAssociateSemanticPlanes(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"
#include "Utils/Utils/private_functions.h"

#include <algorithm>
#include <iostream>
#include <map>
#include <rclcpp/logging.hpp>
#include <tuple>

#include "GeoSemHelpers.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus Utils::reAssociateSemanticPlanes(Atlas *p_atlas_in)
{
    if (p_atlas_in == nullptr)
    {
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    types::SystemParams *p_systemParams = nullptr;
    if (types::SystemParams::getParams(p_systemParams) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    bool mergedPlaneInPass = true;

    while (mergedPlaneInPass)
    {
        mergedPlaneInPass = false;

        std::vector<geometric::Plane *> mappedPlanes{};
        if (p_atlas_in->getAllPlanes(mappedPlanes) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllPlanes returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        for (geometric::Plane *p_candidatePlane : mappedPlanes)
        {
            bool candidatePlaneIsBad{};
            if (!(p_candidatePlane == nullptr) &&
                p_candidatePlane->isBad(candidatePlaneIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            geometric::Plane::PlaneVariant candidatePlanePlaneType{};
            if (!(p_candidatePlane == nullptr || candidatePlaneIsBad) &&
                p_candidatePlane->getPlaneType(candidatePlanePlaneType) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPlaneType returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_candidatePlane == nullptr || candidatePlaneIsBad ||
                candidatePlanePlaneType ==
                    geometric::Plane::PlaneVariant::UNDEFINED)
            {
                continue;
            }

            std::vector<geometric::Plane *> compatiblePlanes;
            compatiblePlanes.reserve(mappedPlanes.size());

            for (geometric::Plane *p_otherPlane : mappedPlanes)
            {
                bool otherPlaneIsBad{};
                if (!(p_otherPlane == nullptr ||
                      p_otherPlane == p_candidatePlane) &&
                    p_otherPlane->isBad(otherPlaneIsBad) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                geometric::Plane::PlaneVariant otherPlanePlaneType{};
                if (!(p_otherPlane == nullptr ||
                      p_otherPlane == p_candidatePlane || otherPlaneIsBad) &&
                    p_otherPlane->getPlaneType(otherPlanePlaneType) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPlaneType returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                geometric::Plane::PlaneVariant candidatePlanePlaneType2{};
                if (!(p_otherPlane == nullptr ||
                      p_otherPlane == p_candidatePlane || otherPlaneIsBad) &&
                    p_candidatePlane->getPlaneType(candidatePlanePlaneType2) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPlaneType returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_otherPlane == nullptr ||
                    p_otherPlane == p_candidatePlane || otherPlaneIsBad ||
                    otherPlanePlaneType != candidatePlanePlaneType2)
                {
                    continue;
                }

                /*
                 * Never merge wall faces observed from opposite sides. Their
                 * equations can be nearly identical when wall thickness is
                 * below the association threshold, but they bound different
                 * rooms and require independent ownership.
                 */
                geometric::Plane::PlaneVariant candidatePlanePlaneType3{};
                if (p_candidatePlane->getPlaneType(candidatePlanePlaneType3) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPlaneType returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (candidatePlanePlaneType3 ==
                    geometric::Plane::PlaneVariant::WALL)
                {
                    geometric::Plane::GeometrySnapshot candidateGeometry{};
                    if (p_candidatePlane->getGeometrySnapshot(
                            candidateGeometry) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getGeometrySnapshot returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    geometric::Plane::GeometrySnapshot otherGeometry{};
                    if (p_otherPlane->getGeometrySnapshot(otherGeometry) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getGeometrySnapshot returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    Eigen::Vector4d candidateEquation_world =
                        candidateGeometry.planeEquation_world;
                    Eigen::Vector4d otherEquation_world =
                        otherGeometry.planeEquation_world;

                    const double candidateNormalNorm =
                        candidateEquation_world.head<3>().norm();
                    const double otherNormalNorm =
                        otherEquation_world.head<3>().norm();

                    if (candidateEquation_world.allFinite() &&
                        otherEquation_world.allFinite() &&
                        candidateNormalNorm >= 1e-8 && otherNormalNorm >= 1e-8)
                    {
                        candidateEquation_world /= candidateNormalNorm;
                        otherEquation_world /= otherNormalNorm;

                        if (candidateEquation_world.head<3>().dot(
                                otherEquation_world.head<3>()) < 0.0)
                        {
                            otherEquation_world *= -1.0;
                        }

                        ObservationSideEvidence candidateObservationSide{};
                        if (getMedianObservationSide_world_m(
                                p_candidatePlane,
                                candidateEquation_world,
                                candidateObservationSide) !=
                            UtilsStatus::UTILS_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getMedianObservationSide_world_m returned "
                                "a failure status although it cannot fail; "
                                "continuing as before.",
                                __func__);
                        }
                        ObservationSideEvidence otherObservationSide{};
                        if (getMedianObservationSide_world_m(
                                p_otherPlane,
                                otherEquation_world,
                                otherObservationSide) !=
                            UtilsStatus::UTILS_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getMedianObservationSide_world_m returned "
                                "a failure status although it cannot fail; "
                                "continuing as before.",
                                __func__);
                        }

                        if (candidateObservationSide.isAmbiguous ||
                            otherObservationSide.isAmbiguous)
                        {
                            continue;
                        }

                        if (candidateObservationSide.medianSignedDistance_m
                                .has_value() &&
                            otherObservationSide.medianSignedDistance_m
                                .has_value() &&
                            candidateObservationSide.medianSignedDistance_m
                                        .value() *
                                    otherObservationSide.medianSignedDistance_m
                                        .value() <
                                0.0)
                        {
                            continue;
                        }

                        /* Finite support is checked once by associatePlanes(),
                         * including nearest-neighbour partial overlap. */
                    }
                }

                compatiblePlanes.push_back(p_otherPlane);
            }

            if (compatiblePlanes.empty())
            {
                continue;
            }

            geometric::Plane::PlaneVariant candidatePlanePlaneType4{};
            if (p_candidatePlane->getPlaneType(candidatePlanePlaneType4) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPlaneType returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            const bool useWallExtensionDistance =
                candidatePlanePlaneType4 ==
                    geometric::Plane::PlaneVariant::WALL &&
                p_systemParams->semSeg.reassociate.wallExtension.enabled;

            const float maximumFiniteCloudDistance_m =
                useWallExtensionDistance
                    ? p_systemParams->semSeg.reassociate.wallExtension
                          .maximumInPlaneGap_m
                    : -1.0F;

            geometric::Plane::GeometrySnapshot candidateAssociationGeometry{};
            if (p_candidatePlane->getGeometrySnapshot(
                    candidateAssociationGeometry) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getGeometrySnapshot returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            int                            matchedPlaneId{};
            geometric::Plane::PlaneVariant candidatePlanePlaneType5{};
            if (p_candidatePlane->getPlaneType(candidatePlanePlaneType5) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPlaneType returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (associatePlanes(
                    compatiblePlanes,
                    g2o::Plane3D(
                        candidateAssociationGeometry.planeEquation_world),
                    candidateAssociationGeometry.supportCloud,
                    Eigen::Matrix4d::Identity(),
                    candidatePlanePlaneType5,
                    p_systemParams->semSeg.reassociate.associationThresh,
                    matchedPlaneId,
                    maximumFiniteCloudDistance_m) !=
                UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: associatePlanes returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (matchedPlaneId < 0)
            {
                continue;
            }

            const std::vector<geometric::Plane *>::iterator
                matchedPlaneIterator = std::find_if(
                    compatiblePlanes.begin(),
                    compatiblePlanes.end(),
                    [matchedPlaneId](const geometric::Plane *p_plane)
                    {
                        int planeGetId{};
                        if ((p_plane != nullptr) &&
                            p_plane->getId(planeGetId) !=
                                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getId returned a failure status "
                                         "although it "
                                         "cannot fail; continuing as before.",
                                         __func__);
                        }
                        return p_plane != nullptr &&
                               planeGetId == matchedPlaneId;
                    });

            if (matchedPlaneIterator == compatiblePlanes.end())
            {
                continue;
            }

            geometric::Plane *p_matchedPlane = *matchedPlaneIterator;

            geometric::Plane::GeometrySnapshot candidateGeometry{};
            if (p_candidatePlane->getGeometrySnapshot(candidateGeometry) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getGeometrySnapshot returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            geometric::Plane::GeometrySnapshot matchedGeometry{};
            if (p_matchedPlane->getGeometrySnapshot(matchedGeometry) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getGeometrySnapshot returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            std::size_t candidatePlaneGetObservationCount{};
            if (p_candidatePlane->getObservationCount(
                    candidatePlaneGetObservationCount) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getObservationCount returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            int candidatePlaneGetId{};
            if (p_candidatePlane->getId(candidatePlaneGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            const std::tuple<unsigned long, unsigned long, int>
                candidateEvidence =
                    std::make_tuple(candidatePlaneGetObservationCount,
                                    candidateGeometry.supportCloud != nullptr
                                        ? candidateGeometry.supportCloud->size()
                                        : 0U,
                                    -candidatePlaneGetId);

            std::size_t matchedPlaneGetObservationCount{};
            if (p_matchedPlane->getObservationCount(
                    matchedPlaneGetObservationCount) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getObservationCount returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            int matchedPlaneGetId{};
            if (p_matchedPlane->getId(matchedPlaneGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            const std::tuple<unsigned long, unsigned long, int>
                matchedEvidence =
                    std::make_tuple(matchedPlaneGetObservationCount,
                                    matchedGeometry.supportCloud != nullptr
                                        ? matchedGeometry.supportCloud->size()
                                        : 0U,
                                    -matchedPlaneGetId);

            geometric::Plane *p_retainedPlane =
                candidateEvidence >= matchedEvidence ? p_candidatePlane
                                                     : p_matchedPlane;

            geometric::Plane *p_retiredPlane =
                p_retainedPlane == p_candidatePlane ? p_matchedPlane
                                                    : p_candidatePlane;

            geometric::Plane::GeometrySnapshot retiredGeometry{};
            if (p_retiredPlane->getGeometrySnapshot(retiredGeometry) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getGeometrySnapshot returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_retiredCloudCopy(
                new pcl::PointCloud<pcl::PointXYZRGBA>);
            if (retiredGeometry.supportCloud != nullptr)
            {
                *p_retiredCloudCopy = *retiredGeometry.supportCloud;
                if (p_retainedPlane->setMapClouds(p_retiredCloudCopy) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setMapClouds returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
            }

            std::set<core::MapPoint *> retiredPlaneMapPoints{};
            if (p_retiredPlane->getMapPoints(retiredPlaneMapPoints) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMapPoints returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (MapPoint *p_mapPoint : retiredPlaneMapPoints)
            {
                bool mapPointIsBad{};
                if ((p_mapPoint != nullptr) &&
                    p_mapPoint->isBad(mapPointIsBad) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_mapPoint != nullptr && !mapPointIsBad)
                {
                    if (p_retainedPlane->setMapPoints(p_mapPoint) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: setMapPoints returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                }
            }

            std::map<KeyFrame *, geometric::Plane::Observation>
                retiredObservations{};
            if (p_retiredPlane->getObservations(retiredObservations) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getObservations returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            for (const auto &[p_keyFrame, observation] : retiredObservations)
            {
                bool keyFrameIsBad{};
                if (!(p_keyFrame == nullptr) &&
                    p_keyFrame->isBad(keyFrameIsBad) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_keyFrame == nullptr || keyFrameIsBad)
                {
                    continue;
                }

                if (p_retainedPlane->mergeObservation(p_keyFrame,
                                                      observation) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: mergeObservation returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }

            bool wasPlaneRefit{};
            if (GeoSemHelpers::refitMappedPlaneFromCloud(p_retainedPlane,
                                                         wasPlaneRefit) !=
                GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: refitMappedPlaneFromCloud returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            geometric::Plane::PlaneVariant retainedPlaneType{};
            if (p_retainedPlane->getPlaneType(retainedPlaneType) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPlaneType returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            std::vector<semantic::Room *> atlasAllRooms{};
            if (p_atlas_in->getAllRooms(atlasAllRooms) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllRooms returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (semantic::Room *p_room : atlasAllRooms)
            {
                bool roomIsBad{};
                if (!(p_room == nullptr) &&
                    p_room->isBad(roomIsBad) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_room == nullptr || roomIsBad)
                {
                    continue;
                }

                bool roomWasWallReplaced{};
                if (p_room->replaceWall(p_retiredPlane,
                                        p_retainedPlane,
                                        roomWasWallReplaced) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    roomWasWallReplaced = false;
                    RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                "%s: replaceWall rejected its input; "
                                "continuing as before.",
                                __func__);
                }
                bool roomWasGroundPlaneReplaced{};
                if (p_room->replaceGroundPlane(p_retiredPlane,
                                               p_retainedPlane,
                                               roomWasGroundPlaneReplaced) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    roomWasGroundPlaneReplaced = false;
                    RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                "%s: replaceGroundPlane rejected its input; "
                                "continuing as before.",
                                __func__);
                }
            }

            std::vector<vs_graphs::core::semantic::Passage *>
                atlasAllPassages{};
            if (p_atlas_in->getAllPassages(atlasAllPassages) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllPassages returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (vs_graphs::core::semantic::Passage *p_passage :
                 atlasAllPassages)
            {
                if (p_passage != nullptr)
                {
                    bool passageWasAssociationReplaced{};
                    if (p_passage->replacePlaneAssociation(
                            p_retiredPlane,
                            p_retainedPlane,
                            passageWasAssociationReplaced) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        passageWasAssociationReplaced = false;
                        RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                    "%s: replacePlaneAssociation rejected its "
                                    "input; continuing as before.",
                                    __func__);
                    }
                }
            }

            std::vector<KeyFrame *> atlasAllKeyFrames{};
            if (p_atlas_in->getAllKeyFrames(atlasAllKeyFrames) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllKeyFrames returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (KeyFrame *p_keyFrame : atlasAllKeyFrames)
            {
                bool keyFrameIsBad2{};
                if ((p_keyFrame != nullptr) &&
                    p_keyFrame->isBad(keyFrameIsBad2) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_keyFrame != nullptr && !keyFrameIsBad2)
                {
                    bool keyFrameWasReplaced{};
                    if (p_keyFrame->replaceMapPlane(p_retiredPlane,
                                                    p_retainedPlane,
                                                    keyFrameWasReplaced) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        keyFrameWasReplaced = false;
                        RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                    "%s: replaceMapPlane rejected its input; "
                                    "continuing as before.",
                                    __func__);
                    }
                }
            }

            Map *p_currentMap = nullptr;
            if (p_atlas_in->getCurrentMap(p_currentMap) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCurrentMap returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (p_currentMap != nullptr)
            {
                if (p_currentMap->eraseRoomWallPlane(p_retiredPlane) !=
                    MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: eraseRoomWallPlane returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }

                if (retainedPlaneType == geometric::Plane::PlaneVariant::WALL)
                {
                    if (p_currentMap->addRoomWallPlane(p_retainedPlane) !=
                        MapStatus::MAP_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: addRoomWallPlane returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                }
            }

            /*
             * Invalidate only after every graph edge points at the survivor,
             * then remove the retired hypothesis from the map container and
             * ID index so later merge passes cannot rediscover stale state.
             */
            if (p_retiredPlane->setBad() !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }

            if (p_currentMap != nullptr)
            {
                if (p_currentMap->eraseMapPlane(p_retiredPlane) !=
                    MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: eraseMapPlane returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
            }

            if (p_retiredPlane->setMap(nullptr) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }

            int retiredPlaneGetId{};
            if (p_retiredPlane->getId(retiredPlaneGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int retainedPlaneGetId{};
            if (p_retainedPlane->getId(retainedPlaneGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemanticMerge] Fused Plane#" << retiredPlaneGetId
                      << " into Plane#" << retainedPlaneGetId << '.'
                      << std::endl;

            mergedPlaneInPass = true;
            break;
        }
    }

    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
