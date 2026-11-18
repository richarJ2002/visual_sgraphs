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
 * @file            reAssociatePassages.cc
 *
 * @brief           Implements Utils::reAssociatePassages(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "configuration/types/objects/SystemParamsStatusEnum.h"
#include "configuration/types/types.h"
#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/private_functions/private_functions.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/KnownSideProvenanceStatusEnum.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/logging.hpp>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus Utils::reAssociatePassages(atlas::Atlas *p_atlas_in)
{
    if (p_atlas_in == nullptr)
    {
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    map::Map *p_activeMap = nullptr;
    if (p_atlas_in->getCurrentMap(p_activeMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    if (p_activeMap == nullptr)
    {
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    SemanticsManager *p_semanticsManager = nullptr;
    unsigned long     activeMapId{};
    if ((p_atlas_in->getSemanticsManager(p_semanticsManager) !=
         atlas::AtlasStatus::ATLAS_STATUS_SUCCESS) ||
        (p_activeMap->getId(activeMapId) != map::MapStatus::MAP_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSemanticsManager/getId returned a failure status "
                     "although they cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::Passage *> passages{};
    if (p_semanticsManager->getMapPassagesInIdOrder(activeMapId, passages) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: the semantics manager has no scene graph for map %lu; "
            "continuing without its passages.",
            __func__,
            activeMapId);
    }
    std::vector<semantic::Room *> activeRooms{};
    if (p_semanticsManager->getMapRoomsInIdOrder(activeMapId, activeRooms) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: the semantics manager has no scene graph for map %lu; "
            "continuing without its rooms.",
            __func__,
            activeMapId);
    }
    const std::unordered_set<semantic::Room *> activeRoomSet(
        activeRooms.begin(),
        activeRooms.end());

    std::sort(passages.begin(),
              passages.end(),
              semantic::isEntityIdLess<semantic::Passage>);

    types::SystemParams *p_params = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    const types::SystemParams::SemSeg::PassageDetection &passageParameters =
        p_params->semSeg.passageDetection;

    /*
     * Use the same geometrically constrained identity gate as online passage
     * tracking. Coplanarity and normal checks below prevent this distance from
     * collapsing openings on unrelated walls after a map merge.
     */
    const double maximumCentroidDistance_m =
        static_cast<double>(passageParameters.duplicatePassageDistance_m);
    const double minimumNormalAlignment =
        static_cast<double>(passageParameters.duplicateNormalAlignment);
    constexpr double maximumSupportingPlaneSeparation_m = 0.30;

    std::unordered_set<semantic::Passage *> retiredPassages;

    for (std::size_t retainedIndex = 0U; retainedIndex < passages.size();
         retainedIndex++)
    {
        semantic::Passage *p_retainedPassage = passages[retainedIndex];

        if (p_retainedPassage == nullptr ||
            retiredPassages.count(p_retainedPassage) > 0U)
        {
            continue;
        }

        g2o::Plane3D retainedPassageGlobalEquation{};
        if (p_retainedPassage->getGlobalEquation(
                retainedPassageGlobalEquation) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector4d retainedEquation =
            retainedPassageGlobalEquation.coeffs();
        const double retainedNormalNorm = retainedEquation.head<3>().norm();

        if (!retainedEquation.allFinite() || retainedNormalNorm < 1e-8)
        {
            continue;
        }

        retainedEquation /= retainedNormalNorm;

        for (std::size_t candidateIndex = retainedIndex + 1U;
             candidateIndex < passages.size();
             candidateIndex++)
        {
            semantic::Passage *p_candidatePassage = passages[candidateIndex];

            if (p_candidatePassage == nullptr ||
                retiredPassages.count(p_candidatePassage) > 0U)
            {
                continue;
            }

            Eigen::Vector3d retainedCentroid_world_m{};
            if (p_retainedPassage->getCentroid(retainedCentroid_world_m) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector3d candidateCentroid_world_m{};
            if (p_candidatePassage->getCentroid(candidateCentroid_world_m) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (!retainedCentroid_world_m.allFinite() ||
                !candidateCentroid_world_m.allFinite() ||
                (candidateCentroid_world_m - retainedCentroid_world_m).norm() >
                    maximumCentroidDistance_m)
            {
                continue;
            }

            g2o::Plane3D candidatePassageGlobalEquation{};
            if (p_candidatePassage->getGlobalEquation(
                    candidatePassageGlobalEquation) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getGlobalEquation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector4d candidateEquation =
                candidatePassageGlobalEquation.coeffs();
            const double candidateNormalNorm =
                candidateEquation.head<3>().norm();

            if (!candidateEquation.allFinite() || candidateNormalNorm < 1e-8)
            {
                continue;
            }

            candidateEquation /= candidateNormalNorm;

            if (std::abs(retainedEquation.head<3>().dot(
                    candidateEquation.head<3>())) < minimumNormalAlignment)
            {
                continue;
            }

            const double retainedPlaneResidual_m = std::abs(
                retainedEquation.head<3>().dot(candidateCentroid_world_m) +
                retainedEquation(3));
            const double candidatePlaneResidual_m = std::abs(
                candidateEquation.head<3>().dot(retainedCentroid_world_m) +
                candidateEquation(3));

            if (retainedPlaneResidual_m > maximumSupportingPlaneSeparation_m ||
                candidatePlaneResidual_m > maximumSupportingPlaneSeparation_m)
            {
                continue;
            }

            vs_graphs::core::semantic::Room *p_retainedPassageProspectiveRoom =
                nullptr;
            if (p_retainedPassage->getProspectiveRoom(
                    p_retainedPassageProspectiveRoom) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            semantic::Room *p_retainedHandle = nullptr;
            if (resolveLiveRoomHandle(activeRoomSet,
                                      p_retainedPassageProspectiveRoom,
                                      p_retainedHandle) !=
                UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: resolveLiveRoomHandle returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            vs_graphs::core::semantic::Room *p_candidatePassageProspectiveRoom =
                nullptr;
            if (p_candidatePassage->getProspectiveRoom(
                    p_candidatePassageProspectiveRoom) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            semantic::Room *p_candidateHandle = nullptr;
            if (resolveLiveRoomHandle(activeRoomSet,
                                      p_candidatePassageProspectiveRoom,
                                      p_candidateHandle) !=
                UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: resolveLiveRoomHandle returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            semantic::Passage::KnownSideProvenance
                candidatePassageKnownSideProvenance{};
            if (p_candidatePassage->getKnownSideProvenance(
                    candidatePassageKnownSideProvenance) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getKnownSideProvenance returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (p_retainedPassage->mergeKnownSideProvenance(
                    candidatePassageKnownSideProvenance) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: mergeKnownSideProvenance returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            semantic::Passage::KnownSideProvenance knownSide{};
            if (p_retainedPassage->getKnownSideProvenance(knownSide) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getKnownSideProvenance returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            semantic::Room *p_survivingHandle = p_retainedHandle;

            if (p_survivingHandle == nullptr)
            {
                p_survivingHandle = p_candidateHandle;
            }
            else if (p_candidateHandle != nullptr &&
                     p_candidateHandle != p_survivingHandle)
            {
                bool retainedIsFarSide{};
                if (isRoomOnProvenFarSide(knownSide,
                                          retainedCentroid_world_m,
                                          p_retainedHandle,
                                          retainedIsFarSide) !=
                    UtilsStatus::UTILS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: isRoomOnProvenFarSide returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                bool candidateIsFarSide{};
                if (isRoomOnProvenFarSide(knownSide,
                                          retainedCentroid_world_m,
                                          p_candidateHandle,
                                          candidateIsFarSide) !=
                    UtilsStatus::UTILS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: isRoomOnProvenFarSide returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (candidateIsFarSide && !retainedIsFarSide)
                {
                    p_survivingHandle = p_candidateHandle;
                    int retainedPassageId{};
                    int retainedHandleId{};
                    int candidateHandleId{};
                    if (p_retainedPassage->getId(retainedPassageId) !=
                            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
                        p_retainedHandle->getId(retainedHandleId) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
                        p_candidateHandle->getId(candidateHandleId) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: getId returned a failure status "
                                     "although it cannot fail; continuing as "
                                     "before.",
                                     __func__);
                    }
                    RCLCPP_INFO(rclcpp::get_logger("vs_graphs"),
                                "[GeoSemHelper] Passage#%d far-side handle "
                                "Room#%d replaced by Room#%d: passages merged, "
                                "only the absorbed passage's room is proven "
                                "far side.",
                                retainedPassageId,
                                retainedHandleId,
                                candidateHandleId);
                }
            }

            if (p_retainedPassage->setProspectiveRoom(p_survivingHandle) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_survivingHandle != nullptr)
            {
                if (p_survivingHandle->addPassageAssociation(
                        p_retainedPassage) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: addPassageAssociation returned a failure "
                                 "status although it "
                                 "cannot fail; continuing as before.",
                                 __func__);
                }
            }

            /* Centroid, size, passability, traversals, walls, door */
            if (fuseDuplicatePassageAttributes(p_retainedPassage,
                                               p_candidatePassage,
                                               retainedEquation,
                                               retainedCentroid_world_m,
                                               candidateCentroid_world_m) !=
                UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: fuseDuplicatePassageAttributes returned a "
                             "failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            std::vector<semantic::Room *> activeMapAllRooms{};
            if (p_semanticsManager->getMapRoomsInIdOrder(activeMapId,
                                                         activeMapAllRooms) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: the semantics manager has no scene graph for map %lu; "
                    "continuing without its rooms.",
                    __func__,
                    activeMapId);
            }
            for (semantic::Room *p_room : activeMapAllRooms)
            {
                bool roomIsBad{};
                if ((p_room != nullptr) &&
                    p_room->isBad(roomIsBad) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_room != nullptr && !roomIsBad)
                {
                    bool roomWasAssociationReplaced{};
                    if (p_room->replacePassageAssociation(
                            p_candidatePassage,
                            p_retainedPassage,
                            roomWasAssociationReplaced) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        roomWasAssociationReplaced = false;
                        RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                    "%s: replacePassageAssociation rejected "
                                    "its input; continuing as before.",
                                    __func__);
                    }
                }
            }

            if (p_semanticsManager->erasePassage(activeMapId,
                                                 p_candidatePassage) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: erasePassage found no scene graph for map %lu; "
                    "continuing as before.",
                    __func__,
                    activeMapId);
            }
            if (p_candidatePassage->setProspectiveRoom(nullptr) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_candidatePassage->setMap(nullptr) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            retiredPassages.insert(p_candidatePassage);
        }
    }

    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} /* namespace utils */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */
