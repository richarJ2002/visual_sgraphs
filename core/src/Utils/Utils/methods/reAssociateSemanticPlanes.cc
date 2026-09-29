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
        // getParams cannot fail; continue as before.
    }

    bool mergedPlaneInPass = true;

    while (mergedPlaneInPass)
    {
        mergedPlaneInPass = false;

        const std::vector<geometric::Plane *> mappedPlanes =
            p_atlas_in->getAllPlanes();

        for (geometric::Plane *p_candidatePlane : mappedPlanes)
        {
            bool candidatePlaneIsBad{};
            if (!(p_candidatePlane == nullptr) &&
                p_candidatePlane->isBad(candidatePlaneIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // isBad cannot fail; continue as before.
            }
            geometric::Plane::PlaneVariant candidatePlanePlaneType{};
            if (!(p_candidatePlane == nullptr || candidatePlaneIsBad) &&
                p_candidatePlane->getPlaneType(candidatePlanePlaneType) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getPlaneType cannot fail; continue as before.
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
                    // isBad cannot fail; continue as before.
                }
                geometric::Plane::PlaneVariant otherPlanePlaneType{};
                if (!(p_otherPlane == nullptr ||
                      p_otherPlane == p_candidatePlane || otherPlaneIsBad) &&
                    p_otherPlane->getPlaneType(otherPlanePlaneType) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // getPlaneType cannot fail; continue as before.
                }
                geometric::Plane::PlaneVariant candidatePlanePlaneType2{};
                if (!(p_otherPlane == nullptr ||
                      p_otherPlane == p_candidatePlane || otherPlaneIsBad) &&
                    p_candidatePlane->getPlaneType(candidatePlanePlaneType2) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // getPlaneType cannot fail; continue as before.
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
                    // getPlaneType cannot fail; continue as before.
                }
                if (candidatePlanePlaneType3 ==
                    geometric::Plane::PlaneVariant::WALL)
                {
                    geometric::Plane::GeometrySnapshot candidateGeometry{};
                    if (p_candidatePlane->getGeometrySnapshot(
                            candidateGeometry) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // getGeometrySnapshot cannot fail; continue as before.
                    }
                    geometric::Plane::GeometrySnapshot otherGeometry{};
                    if (p_otherPlane->getGeometrySnapshot(otherGeometry) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // getGeometrySnapshot cannot fail; continue as before.
                    }
                    Eigen::Vector4d candidateEquation_World =
                        candidateGeometry.equation_World;
                    Eigen::Vector4d otherEquation_World =
                        otherGeometry.equation_World;

                    const double candidateNormalNorm =
                        candidateEquation_World.head<3>().norm();
                    const double otherNormalNorm =
                        otherEquation_World.head<3>().norm();

                    if (candidateEquation_World.allFinite() &&
                        otherEquation_World.allFinite() &&
                        candidateNormalNorm >= 1e-8 && otherNormalNorm >= 1e-8)
                    {
                        candidateEquation_World /= candidateNormalNorm;
                        otherEquation_World /= otherNormalNorm;

                        if (candidateEquation_World.head<3>().dot(
                                otherEquation_World.head<3>()) < 0.0)
                        {
                            otherEquation_World *= -1.0;
                        }

                        ObservationSideEvidence candidateObservationSide{};
                        if (getMedianObservationSide_World_m(
                                p_candidatePlane,
                                candidateEquation_World,
                                candidateObservationSide) !=
                            UtilsStatus::UTILS_STATUS_SUCCESS)
                        {
                            // getMedianObservationSide_World_m cannot fail;
                            // continue as before.
                        }
                        ObservationSideEvidence otherObservationSide{};
                        if (getMedianObservationSide_World_m(
                                p_otherPlane,
                                otherEquation_World,
                                otherObservationSide) !=
                            UtilsStatus::UTILS_STATUS_SUCCESS)
                        {
                            // getMedianObservationSide_World_m cannot fail;
                            // continue as before.
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
                // getPlaneType cannot fail; continue as before.
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
                // getGeometrySnapshot cannot fail; continue as before.
            }
            int                            matchedPlaneId{};
            geometric::Plane::PlaneVariant candidatePlanePlaneType5{};
            if (p_candidatePlane->getPlaneType(candidatePlanePlaneType5) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getPlaneType cannot fail; continue as before.
            }
            if (associatePlanes(
                    compatiblePlanes,
                    g2o::Plane3D(candidateAssociationGeometry.equation_World),
                    candidateAssociationGeometry.supportCloud,
                    Eigen::Matrix4d::Identity(),
                    candidatePlanePlaneType5,
                    p_systemParams->semSeg.reassociate.associationThresh,
                    matchedPlaneId,
                    maximumFiniteCloudDistance_m) !=
                UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                // associatePlanes cannot fail; continue as before.
            }

            if (matchedPlaneId < 0)
            {
                continue;
            }

            const auto matchedPlaneIterator = std::find_if(
                compatiblePlanes.begin(),
                compatiblePlanes.end(),
                [matchedPlaneId](const geometric::Plane *p_plane)
                {
                    int planeGetId{};
                    if ((p_plane != nullptr) &&
                        p_plane->getId(planeGetId) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    return p_plane != nullptr && planeGetId == matchedPlaneId;
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
                // getGeometrySnapshot cannot fail; continue as before.
            }
            geometric::Plane::GeometrySnapshot matchedGeometry{};
            if (p_matchedPlane->getGeometrySnapshot(matchedGeometry) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getGeometrySnapshot cannot fail; continue as before.
            }
            std::size_t candidatePlaneGetObservationCount{};
            if (p_candidatePlane->getObservationCount(
                    candidatePlaneGetObservationCount) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getObservationCount cannot fail; continue as before.
            }
            int candidatePlaneGetId{};
            if (p_candidatePlane->getId(candidatePlaneGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            const auto candidateEvidence =
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
                // getObservationCount cannot fail; continue as before.
            }
            int matchedPlaneGetId{};
            if (p_matchedPlane->getId(matchedPlaneGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            const auto matchedEvidence =
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
                // getGeometrySnapshot cannot fail; continue as before.
            }
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_retiredCloudCopy(
                new pcl::PointCloud<pcl::PointXYZRGBA>);
            if (retiredGeometry.supportCloud != nullptr)
            {
                *p_retiredCloudCopy = *retiredGeometry.supportCloud;
                if (p_retainedPlane->setMapClouds(p_retiredCloudCopy) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // setMapClouds cannot fail; continue as before.
                }
            }

            std::set<core::MapPoint *> retiredPlaneMapPoints{};
            if (p_retiredPlane->getMapPoints(retiredPlaneMapPoints) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getMapPoints cannot fail; continue as before.
            }
            for (MapPoint *p_mapPoint : retiredPlaneMapPoints)
            {
                if (p_mapPoint != nullptr && !p_mapPoint->isBad())
                {
                    if (p_retainedPlane->setMapPoints(p_mapPoint) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // setMapPoints cannot fail; continue as before.
                    }
                }
            }

            std::map<KeyFrame *, geometric::Plane::Observation>
                retiredObservations{};
            if (p_retiredPlane->getObservations(retiredObservations) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getObservations cannot fail; continue as before.
            }

            for (const auto &[p_keyFrame, observation] : retiredObservations)
            {
                if (p_keyFrame == nullptr || p_keyFrame->isBad())
                {
                    continue;
                }

                if (p_retainedPlane->mergeObservation(p_keyFrame,
                                                      observation) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // mergeObservation cannot fail; continue as before.
                }
            }

            bool wasPlaneRefit{};
            if (GeoSemHelpers::refitMappedPlaneFromCloud(p_retainedPlane,
                                                         wasPlaneRefit) !=
                GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS)
            {
                // refitMappedPlaneFromCloud cannot fail; continue as before.
            }
            geometric::Plane::PlaneVariant retainedPlaneType{};
            if (p_retainedPlane->getPlaneType(retainedPlaneType) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getPlaneType cannot fail; continue as before.
            }

            for (semantic::Room *p_room : p_atlas_in->getAllRooms())
            {
                bool roomIsBad{};
                if (!(p_room == nullptr) &&
                    p_room->isBad(roomIsBad) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // isBad cannot fail; continue as before.
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
                    roomWasWallReplaced =
                        false; // rejected input reads as before
                }
                bool roomWasGroundPlaneReplaced{};
                if (p_room->replaceGroundPlane(p_retiredPlane,
                                               p_retainedPlane,
                                               roomWasGroundPlaneReplaced) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    roomWasGroundPlaneReplaced =
                        false; // rejected input reads as before
                }
            }

            for (vs_graphs::core::semantic::Passage *p_passage :
                 p_atlas_in->getAllPassages())
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
                        passageWasAssociationReplaced =
                            false; // rejected input reads as before
                    }
                }
            }

            for (KeyFrame *p_keyFrame : p_atlas_in->getAllKeyFrames())
            {
                if (p_keyFrame != nullptr && !p_keyFrame->isBad())
                {
                    p_keyFrame->replaceMapPlane(p_retiredPlane,
                                                p_retainedPlane);
                }
            }

            Map *p_currentMap = p_atlas_in->getCurrentMap();

            if (p_currentMap != nullptr)
            {
                p_currentMap->eraseRoomWallPlane(p_retiredPlane);

                if (retainedPlaneType == geometric::Plane::PlaneVariant::WALL)
                {
                    p_currentMap->addRoomWallPlane(p_retainedPlane);
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
                // setBad cannot fail; continue as before.
            }

            if (p_currentMap != nullptr)
            {
                p_currentMap->eraseMapPlane(p_retiredPlane);
            }

            if (p_retiredPlane->setMap(nullptr) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // setMap cannot fail; continue as before.
            }

            int retiredPlaneGetId{};
            if (p_retiredPlane->getId(retiredPlaneGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            int retainedPlaneGetId{};
            if (p_retainedPlane->getId(retainedPlaneGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            std::cout << "[SemanticMerge] Fused geometric::Plane#"
                      << retiredPlaneGetId << " into geometric::Plane#"
                      << retainedPlaneGetId << '.' << std::endl;

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
