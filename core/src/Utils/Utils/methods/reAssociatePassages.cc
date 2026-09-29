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

#include "Semantic/ValueOrder.h"
#include "Utils/Utils/objects/Utils.h"

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

UtilsStatus Utils::reAssociatePassages(Atlas *p_atlas_in)
{
    if (p_atlas_in == nullptr)
    {
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    Map *p_activeMap = p_atlas_in->getCurrentMap();

    if (p_activeMap == nullptr)
    {
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    std::vector<semantic::Passage *> passages{};
    if (p_activeMap->getAllPassages(passages) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassages returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::Room *> activeRooms{};
    if (p_activeMap->getAllRooms(activeRooms) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    const std::unordered_set<semantic::Room *> activeRoomSet(
        activeRooms.begin(),
        activeRooms.end());

    const auto liveRoomHandle =
        [&activeRoomSet](semantic::Room *p_room) -> semantic::Room *
    {
        bool roomIsBad{};
        if ((p_room != nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        return p_room != nullptr && !roomIsBad &&
                       activeRoomSet.count(p_room) > 0U
                   ? p_room
                   : nullptr;
    };

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

            Eigen::Vector3d retainedCentroid_World_m{};
            if (p_retainedPassage->getCentroid(retainedCentroid_World_m) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector3d candidateCentroid_World_m{};
            if (p_candidatePassage->getCentroid(candidateCentroid_World_m) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (!retainedCentroid_World_m.allFinite() ||
                !candidateCentroid_World_m.allFinite() ||
                (candidateCentroid_World_m - retainedCentroid_World_m).norm() >
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
                retainedEquation.head<3>().dot(candidateCentroid_World_m) +
                retainedEquation(3));
            const double candidatePlaneResidual_m = std::abs(
                candidateEquation.head<3>().dot(retainedCentroid_World_m) +
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
            semantic::Room *p_retainedHandle =
                liveRoomHandle(p_retainedPassageProspectiveRoom);
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
            semantic::Room *p_candidateHandle =
                liveRoomHandle(p_candidatePassageProspectiveRoom);

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

            const auto isProvenFarSide =
                [&knownSide, &retainedCentroid_World_m](semantic::Room *p_room)
            {
                Eigen::Vector3d roomCentroid{};
                bool            knownSideHasDirection{};
                if ((p_room != nullptr) &&
                    knownSide.hasDirection(knownSideHasDirection) !=
                        semantic::KnownSideProvenanceStatus::
                            KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: hasDirection returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if ((p_room != nullptr && knownSideHasDirection) &&
                    p_room->getCentroid(roomCentroid) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getCentroid returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                bool knownSideHasDirection2{};
                if ((p_room != nullptr) &&
                    knownSide.hasDirection(knownSideHasDirection2) !=
                        semantic::KnownSideProvenanceStatus::
                            KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: hasDirection returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                return p_room != nullptr && knownSideHasDirection2 &&
                       knownSide.direction_World.dot(
                           roomCentroid - retainedCentroid_World_m) < -0.20;
            };

            semantic::Room *p_survivingHandle = p_retainedHandle;

            if (p_survivingHandle == nullptr)
            {
                p_survivingHandle = p_candidateHandle;
            }
            else if (p_candidateHandle != nullptr &&
                     p_candidateHandle != p_survivingHandle)
            {
                const bool retainedIsFarSide =
                    isProvenFarSide(p_retainedHandle);
                const bool candidateIsFarSide =
                    isProvenFarSide(p_candidateHandle);
                if (candidateIsFarSide && !retainedIsFarSide)
                {
                    p_survivingHandle = p_candidateHandle;
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
                if (p_survivingHandle->setDoorways(p_retainedPassage) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setDoorways returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
            }

            Eigen::Vector3d fusedCentroid_World_m =
                0.5 * (retainedCentroid_World_m + candidateCentroid_World_m);
            const double fusedCentroidResidual_m =
                retainedEquation.head<3>().dot(fusedCentroid_World_m) +
                retainedEquation(3);
            fusedCentroid_World_m -=
                fusedCentroidResidual_m * retainedEquation.head<3>();

            if (p_retainedPassage->setCentroid(fusedCentroid_World_m) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            double retainedPassageWidth{};
            if (p_retainedPassage->getWidth(retainedPassageWidth) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWidth returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            double candidatePassageWidth{};
            if (p_candidatePassage->getWidth(candidatePassageWidth) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWidth returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_retainedPassage->setWidth(
                    std::max(retainedPassageWidth, candidatePassageWidth)) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setWidth returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            double retainedPassageHeight{};
            if (p_retainedPassage->getHeight(retainedPassageHeight) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getHeight returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            double candidatePassageHeight{};
            if (p_candidatePassage->getHeight(candidatePassageHeight) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getHeight returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_retainedPassage->setHeight(
                    std::max(retainedPassageHeight, candidatePassageHeight)) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setHeight returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            bool retainedPassageIsPassable{};
            if (p_retainedPassage->isPassable(retainedPassageIsPassable) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isPassable returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            bool candidatePassageIsPassable{};
            if (!(retainedPassageIsPassable) &&
                p_candidatePassage->isPassable(candidatePassageIsPassable) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isPassable returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_retainedPassage->setPassable(retainedPassageIsPassable ||
                                               candidatePassageIsPassable) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setPassable returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            std::size_t retainedTraversalCount{};
            if (p_retainedPassage->getTraversalObservationCount(
                    retainedTraversalCount) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getTraversalObservationCount returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
            std::size_t candidateTraversalCount{};
            if (p_candidatePassage->getTraversalObservationCount(
                    candidateTraversalCount) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getTraversalObservationCount returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
            const std::size_t maximumTraversalCount =
                std::numeric_limits<std::size_t>::max();

            if (p_retainedPassage->setTraversalObservationCount(
                    candidateTraversalCount >
                            maximumTraversalCount - retainedTraversalCount
                        ? maximumTraversalCount
                        : retainedTraversalCount + candidateTraversalCount) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: setTraversalObservationCount returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }

            std::vector<vs_graphs::core::geometric::Plane *>
                candidatePassageAssociateWalls{};
            if (p_candidatePassage->getAssociateWalls(
                    candidatePassageAssociateWalls) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAssociateWalls returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (geometric::Plane *p_supportingWall :
                 candidatePassageAssociateWalls)
            {
                if (p_retainedPassage->addAssociateWall(p_supportingWall) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: addAssociateWall returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }

            vs_graphs::core::geometric::Plane *p_retainedPassageAssociateDoor =
                nullptr;
            if (p_retainedPassage->getAssociateDoor(
                    p_retainedPassageAssociateDoor) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAssociateDoor returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            vs_graphs::core::geometric::Plane *p_candidatePassageAssociateDoor =
                nullptr;
            if ((p_retainedPassageAssociateDoor == nullptr) &&
                p_candidatePassage->getAssociateDoor(
                    p_candidatePassageAssociateDoor) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAssociateDoor returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_retainedPassageAssociateDoor == nullptr &&
                p_candidatePassageAssociateDoor != nullptr)
            {
                vs_graphs::core::geometric::Plane
                    *p_candidatePassageAssociateDoor2 = nullptr;
                if (p_candidatePassage->getAssociateDoor(
                        p_candidatePassageAssociateDoor2) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getAssociateDoor returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_retainedPassage->setAssociateDoor(
                        p_candidatePassageAssociateDoor2) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setAssociateDoor returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }

            std::vector<semantic::Room *> activeMapAllRooms{};
            if (p_activeMap->getAllRooms(activeMapAllRooms) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllRooms returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
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

            std::vector<KeyFrame *> activeMapAllKeyFrames{};
            if (p_activeMap->getAllKeyFrames(activeMapAllKeyFrames) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllKeyFrames returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (KeyFrame *p_keyFrame : activeMapAllKeyFrames)
            {
                bool keyFrameIsBad{};
                if ((p_keyFrame != nullptr) &&
                    p_keyFrame->isBad(keyFrameIsBad) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_keyFrame != nullptr && !keyFrameIsBad)
                {
                    bool keyFrameWasReplaced{};
                    if (p_keyFrame->replaceMapPassage(p_candidatePassage,
                                                      p_retainedPassage,
                                                      keyFrameWasReplaced) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        keyFrameWasReplaced = false;
                        RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                    "%s: replaceMapPassage rejected its input; "
                                    "continuing as before.",
                                    __func__);
                    }
                }
            }

            if (p_activeMap->eraseMapPassage(p_candidatePassage) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseMapPassage returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
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

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
