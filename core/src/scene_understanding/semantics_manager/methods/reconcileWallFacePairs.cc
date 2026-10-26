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
 * @file            reconcileWallFacePairs.cc
 *
 * @brief           Implements SemanticsManager::reconcileWallFacePairs(),
 *                  declared in SemanticsManager.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/semantics_manager/objects/TwinFaceCheckEnum.h"

#include "scene_understanding/semantics_manager/private_functions/private_functions.h"
#include "scene_understanding/semantic/value_order/value_order.h"

#include <algorithm>
#include <iostream>
#include <string>
#include <utility>
#include <rclcpp/logging.hpp>

#include "execution/profiling/profiling.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::reconcileWallFacePairs(void)
{
    PROFILE_FUNCTION();
    geometric::Plane *p_groundPlane = nullptr;
    if (p_atlas->getBiggestGroundPlane(p_groundPlane) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector3d groundNormal_world = Eigen::Vector3d::Zero();
    bool            groundPlaneIsBad{};
    if ((p_groundPlane != nullptr) &&
        p_groundPlane->isBad(groundPlaneIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_groundPlane != nullptr && !groundPlaneIsBad)
    {
        g2o::Plane3D groundPlaneGetGlobalEquation{};
        if (p_groundPlane->getGlobalEquation(groundPlaneGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector4d groundEq = groundPlaneGetGlobalEquation.coeffs();
        const double          groundNorm = groundEq.head<3>().norm();
        if (groundEq.allFinite() && groundNorm > 1e-8)
        {
            groundNormal_world = groundEq.head<3>() / groundNorm;
        }
    }

    const double minimumThickness_m =
        static_cast<double>(p_sysParams->semSeg.wallPairing.minimumThickness_m);
    const double maximumThickness_m =
        static_cast<double>(p_sysParams->semSeg.wallPairing.maximumThickness_m);
    const double minimumOverlapRatio = static_cast<double>(
        p_sysParams->semSeg.wallPairing.minimumOverlapRatio);

    std::vector<geometric::Plane *>                  wallPlanes;
    std::vector<vs_graphs::core::geometric::Plane *> atlasAllPlanes{};
    if (p_atlas->getAllPlanes(atlasAllPlanes) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (geometric::Plane *p_plane : atlasAllPlanes)
    {
        bool planeIsBad{};
        if ((p_plane != nullptr) &&
            p_plane->isBad(planeIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        geometric::Plane::PlaneVariant planeType{};
        if ((p_plane != nullptr && !planeIsBad) &&
            p_plane->getPlaneType(planeType) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPlaneType returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_plane != nullptr && !planeIsBad &&
            planeType == geometric::Plane::PlaneVariant::WALL)
        {
            wallPlanes.push_back(p_plane);
        }
    }
    /* Lock two Planes in ascending id order to avoid a lock-order hazard,
     * matching the convention already used for passages in
     * admitWallToRoom(). */
    std::sort(wallPlanes.begin(),
              wallPlanes.end(),
              semantic::isEntityIdLess<geometric::Plane>);

    map::Map *p_activeMap = nullptr;
    if (p_atlas->getCurrentMap(p_activeMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    unsigned long activeMapId{};
    if ((p_activeMap != nullptr) &&
        p_activeMap->getId(activeMapId) != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    const auto twinFaceCheckName = [](TwinFaceCheck check_in) -> const char *
    {
        switch (check_in)
        {
        case TwinFaceCheck::PLAUSIBLE:
            return "PLAUSIBLE";
        case TwinFaceCheck::INVALID_PLANE:
            return "INVALID_PLANE";
        case TwinFaceCheck::NOT_PARALLEL:
            return "NOT_PARALLEL";
        case TwinFaceCheck::THICKNESS_OUT_OF_RANGE:
            return "THICKNESS_OUT_OF_RANGE";
        case TwinFaceCheck::NO_OBSERVATION_ORIGIN:
            return "NO_OBSERVATION_ORIGIN";
        case TwinFaceCheck::OBSERVATION_SIDE_UNRESOLVED:
            return "OBSERVATION_SIDE_UNRESOLVED";
        case TwinFaceCheck::OBSERVED_FROM_SAME_SIDE:
            return "OBSERVED_FROM_SAME_SIDE";
        case TwinFaceCheck::NO_FOOTPRINT_OVERLAP:
            return "NO_FOOTPRINT_OVERLAP";
        case TwinFaceCheck::LOW_OVERLAP:
            return "LOW_OVERLAP";
        }
        return "UNKNOWN";
    };

    /* Emits the twin_face event when a pair's state or deciding check differs
     * from the one last logged for it. */
    const auto logTwinFaceState =
        [this, &twinFaceCheckName, p_activeMap, activeMapId](
            geometric::Plane *p_first_in,
            geometric::Plane *p_second_in,
            const char       *state_in,
            TwinFaceCheck     check_in,
            double            value_in)
    {
        int firstId{};
        if (p_first_in->getId(firstId) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        int secondId{};
        if (p_second_in->getId(secondId) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        const std::pair<int, int> pairKey{std::min(firstId, secondId),
                                          std::max(firstId, secondId)};
        const std::string         stateKey =
            std::string(state_in) + ":" + twinFaceCheckName(check_in);
        std::string &loggedState = loggedTwinFaceStates[pairKey];
        if (loggedState == stateKey)
        {
            return;
        }
        loggedState = stateKey;
        std::cout << "SG_PIPELINE {\"event\":\"twin_face\",\"map_id\":"
                  << (p_activeMap != nullptr
                          ? static_cast<long long>(activeMapId)
                          : -1)
                  << ",\"semantic_cycle\":" << pipelineSemanticCycle
                  << ",\"first_wall_id\":" << pairKey.first
                  << ",\"second_wall_id\":" << pairKey.second << ",\"state\":\""
                  << state_in << "\",\"check\":\""
                  << twinFaceCheckName(check_in) << "\",\"value\":" << value_in
                  << "}" << std::endl;
    };

    for (geometric::Plane *p_wall : wallPlanes)
    {
        geometric::Plane *p_existingTwin = nullptr;
        if (p_wall->getTwinFace(p_existingTwin) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getTwinFace returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_existingTwin != nullptr)
        {
            /* Cheap common case: re-validate rather than search again. A
             * lower-id plane already validated (and, if still plausible,
             * re-linked) this pair when it was itself visited. */
            int wallGetId{};
            if (p_wall->getId(wallGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int existingTwinGetId{};
            if (p_existingTwin->getId(existingTwinGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (wallGetId < existingTwinGetId)
            {
                continue;
            }

            bool          arePlausibleTwinWallFaces2{};
            TwinFaceCheck decidingCheck{};
            double        decidingValue{};
            if (arePlausibleTwinWallFaces(p_wall,
                                          p_existingTwin,
                                          minimumThickness_m,
                                          maximumThickness_m,
                                          minimumOverlapRatio,
                                          groundNormal_world,
                                          arePlausibleTwinWallFaces2,
                                          decidingCheck,
                                          decidingValue) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: arePlausibleTwinWallFaces returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (arePlausibleTwinWallFaces2)
            {
                continue;
            }

            /* Pairing is no longer plausible (e.g. one side drifted after a
             * refit) -- unlink both sides rather than leave a stale
             * one-directional pointer. */
            logTwinFaceState(p_wall,
                             p_existingTwin,
                             "UNLINKED",
                             decidingCheck,
                             decidingValue);
            if (p_existingTwin->clearTwinFace() !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: clearTwinFace returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_wall->clearTwinFace() !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: clearTwinFace returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    for (std::size_t firstIndex = 0; firstIndex < wallPlanes.size();
         ++firstIndex)
    {
        geometric::Plane *p_first            = wallPlanes[firstIndex];
        geometric::Plane *p_firstGetTwinFace = nullptr;
        if (p_first->getTwinFace(p_firstGetTwinFace) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getTwinFace returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_firstGetTwinFace != nullptr)
        {
            continue;
        }

        geometric::Plane *p_bestMatch         = nullptr;
        double            bestOverlapRatio_m2 = -1.0;

        for (std::size_t secondIndex = firstIndex + 1;
             secondIndex < wallPlanes.size();
             ++secondIndex)
        {
            geometric::Plane *p_second            = wallPlanes[secondIndex];
            geometric::Plane *p_secondGetTwinFace = nullptr;
            if (p_second->getTwinFace(p_secondGetTwinFace) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getTwinFace returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_secondGetTwinFace != nullptr)
            {
                continue;
            }

            bool          arePlausibleTwinWallFaces3{};
            TwinFaceCheck decidingCheck2{};
            double        decidingValue2{};
            if (arePlausibleTwinWallFaces(p_first,
                                          p_second,
                                          minimumThickness_m,
                                          maximumThickness_m,
                                          minimumOverlapRatio,
                                          groundNormal_world,
                                          arePlausibleTwinWallFaces3,
                                          decidingCheck2,
                                          decidingValue2) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: arePlausibleTwinWallFaces returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (!arePlausibleTwinWallFaces3)
            {
                /* Only pairs that could be one wall's two faces are worth a
                 * record: parallel planes no thicker than a wall, or a pair
                 * that was linked or refused before. */
                int firstGetId2{};
                if (p_first->getId(firstGetId2) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int secondGetId{};
                if (p_second->getId(secondGetId) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                const bool isTwinCandidate =
                    decidingCheck2 != TwinFaceCheck::INVALID_PLANE &&
                    decidingCheck2 != TwinFaceCheck::NOT_PARALLEL &&
                    !(decidingCheck2 == TwinFaceCheck::THICKNESS_OUT_OF_RANGE &&
                      decidingValue2 > maximumThickness_m);
                if (isTwinCandidate ||
                    loggedTwinFaceStates.count(
                        {std::min(firstGetId2, secondGetId),
                         std::max(firstGetId2, secondGetId)}) > 0U)
                {
                    logTwinFaceState(p_first,
                                     p_second,
                                     "REFUSED",
                                     decidingCheck2,
                                     decidingValue2);
                }
                continue;
            }

            /* Prefer the most-overlapping plausible candidate when more
             * than one exists, using observation count as a simple,
             * deterministic tiebreaker proxy for "most overlap". */
            std::size_t secondGetObservationCount{};
            if (p_second->getObservationCount(secondGetObservationCount) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getObservationCount returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            const double candidateScore =
                static_cast<double>(secondGetObservationCount);
            if (p_bestMatch == nullptr || candidateScore > bestOverlapRatio_m2)
            {
                p_bestMatch         = p_second;
                bestOverlapRatio_m2 = candidateScore;
            }
        }

        if (p_bestMatch != nullptr)
        {
            if (p_first->setTwinFace(p_bestMatch) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setTwinFace returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_bestMatch->setTwinFace(p_first) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setTwinFace returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            int firstGetId{};
            if (p_first->getId(firstGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int bestMatchGetId{};
            if (p_bestMatch->getId(bestMatchGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            logTwinFaceState(p_first,
                             p_bestMatch,
                             "LINKED",
                             TwinFaceCheck::PLAUSIBLE,
                             0.0);
            std::cout << "[SemMgr] Linked Wall#" << firstGetId << " and Wall#"
                      << bestMatchGetId
                      << " as opposite faces of one physical wall."
                      << std::endl;
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
