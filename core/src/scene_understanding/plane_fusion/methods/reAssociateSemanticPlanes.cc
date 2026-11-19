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
 * @brief           Implements SemanticsManager::reAssociateSemanticPlanes(),
 *                  declared in
 * semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "configuration/types/types.h"
#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <algorithm>
#include <iostream>
#include <rclcpp/logging.hpp>
#include <tuple>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::reAssociateSemanticPlanes(atlas::Atlas *p_atlas_in)
{
    if (p_atlas_in == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const types::SystemParams *p_systemParams = p_sysParams;

    bool mergedPlaneInPass = true;

    while (mergedPlaneInPass)
    {
        mergedPlaneInPass = false;

        std::vector<geometric::Plane *> mappedPlanes{};
        if (p_atlas_in->getAllPlanes(mappedPlanes) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllPlanes returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* The map's plane set is ordered by address; the first fusable pair
         * wins, so visit the planes in id order. */
        std::sort(mappedPlanes.begin(),
                  mappedPlanes.end(),
                  semantic::isEntityIdLess<geometric::Plane>);

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

            /*!
             * Every other plane is a candidate; the matcher applies the strict
             * type rule and keeps wall faces seen from opposite sides apart.
             */
            std::vector<geometric::Plane *> otherPlanes;
            otherPlanes.reserve(mappedPlanes.size());

            for (geometric::Plane *p_otherPlane : mappedPlanes)
            {
                if (p_otherPlane != p_candidatePlane)
                {
                    otherPlanes.push_back(p_otherPlane);
                }
            }

            geometric::Plane::GeometrySnapshot candidateMatchGeometry{};
            if (p_candidatePlane->getGeometrySnapshot(candidateMatchGeometry) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getGeometrySnapshot returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            utils::utils::PlaneEvidence candidatePlaneEvidence{};
            candidatePlaneEvidence.planeEquation_world =
                candidateMatchGeometry.planeEquation_world;
            candidatePlaneEvidence.p_supportCloud_world_m =
                candidateMatchGeometry.supportCloud;
            candidatePlaneEvidence.planeLabel = candidatePlanePlaneType;

            if (candidatePlanePlaneType == geometric::Plane::PlaneVariant::WALL)
            {
                const Eigen::Vector4d candidateEquation_world =
                    candidateMatchGeometry.planeEquation_world /
                    candidateMatchGeometry.planeEquation_world.head<3>().norm();

                geometric::Plane::ObservationSideSnapshot observingCameras{};
                if (p_candidatePlane->getObservationSideSnapshot(
                        candidateEquation_world,
                        observingCameras) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getObservationSideSnapshot returned a "
                                 "failure status although it cannot fail; "
                                 "continuing as before.",
                                 __func__);
                }
                candidatePlaneEvidence.observerSide = observingCameras;
            }

            utils::utils::PlaneMatchSettings matchSettings{};
            matchSettings.typeRule =
                utils::utils::PlaneTypeRule::CONFIRMED_TYPE_STRICT;
            matchSettings.maximumAngle_rad = static_cast<double>(
                p_systemParams->semSeg.reassociate.associationThresh);
            matchSettings.maximumPlaneDistance_m = static_cast<double>(
                p_systemParams->seg.planeAssociation.distanceThresh);
            matchSettings.maximumCentroidDistance_m = static_cast<double>(
                p_systemParams->seg.planeAssociation.centroidThresh);
            matchSettings.maximumCloudGap_m =
                static_cast<double>(p_systemParams->seg.planeAssociation
                                        .clusterSeparation.tolerance);

            int matchedPlaneId{};
            if (utils::utils::Utils::findMatchingPlane(otherPlanes,
                                                       candidatePlaneEvidence,
                                                       matchSettings,
                                                       matchedPlaneId,
                                                       nullptr) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: findMatchingPlane returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (matchedPlaneId < 0)
            {
                continue;
            }

            /* The first other plane carrying the matched id */
            geometric::Plane *p_matchedPlane = nullptr;
            for (geometric::Plane *p_otherPlane : otherPlanes)
            {
                int otherPlaneId{};
                if ((p_otherPlane != nullptr) &&
                    p_otherPlane->getId(otherPlaneId) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_otherPlane != nullptr && otherPlaneId == matchedPlaneId)
                {
                    p_matchedPlane = p_otherPlane;
                    break;
                }
            }

            if (p_matchedPlane == nullptr)
            {
                continue;
            }

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

            /* Every holder of the retired plane (its own records, the rooms
             * and passages, the keyframes) is re-pointed in one place. */
            if (relinkRetiredPlane(p_atlas_in,
                                   p_retiredPlane,
                                   p_retainedPlane) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: relinkRetiredPlane returned a failure "
                             "status although it cannot fail; continuing as "
                             "before.",
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

            map::Map *p_currentMap = nullptr;
            if (p_atlas_in->getCurrentMap(p_currentMap) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCurrentMap returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (p_currentMap != nullptr)
            {
                if (p_currentMap->eraseRoomWallPlane(p_retiredPlane) !=
                    map::MapStatus::MAP_STATUS_SUCCESS)
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
                        map::MapStatus::MAP_STATUS_SUCCESS)
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
                    map::MapStatus::MAP_STATUS_SUCCESS)
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
            retiredPlanesUnfreedCount++;

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

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
