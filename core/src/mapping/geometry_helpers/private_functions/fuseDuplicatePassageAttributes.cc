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
 * @file            fuseDuplicatePassageAttributes.cc
 *
 * @brief           Implements fuseDuplicatePassageAttributes(), declared in
 *                  geometry_helpers/private_functions/private_functions.h.
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

#include "mapping/geometry_helpers/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus fuseDuplicatePassageAttributes(
    semantic::Passage     *p_retainedPassage_inout,
    semantic::Passage     *p_candidatePassage_in,
    const Eigen::Vector4d &retainedEquation_in,
    const Eigen::Vector3d &retainedCentroid_world_m_in,
    const Eigen::Vector3d &candidateCentroid_world_m_in)
{
    Eigen::Vector3d fusedCentroid_world_m =
        0.5 * (retainedCentroid_world_m_in + candidateCentroid_world_m_in);
    const double fusedCentroidResidual_m =
        retainedEquation_in.head<3>().dot(fusedCentroid_world_m) +
        retainedEquation_in(3);
    fusedCentroid_world_m -=
        fusedCentroidResidual_m * retainedEquation_in.head<3>();

    if (p_retainedPassage_inout->setCentroid(fusedCentroid_world_m) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCentroid returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    double retainedPassageWidth{};
    if (p_retainedPassage_inout->getWidth(retainedPassageWidth) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWidth returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    double candidatePassageWidth{};
    if (p_candidatePassage_in->getWidth(candidatePassageWidth) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWidth returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_retainedPassage_inout->setWidth(
            std::max(retainedPassageWidth, candidatePassageWidth)) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setWidth returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    double retainedPassageHeight{};
    if (p_retainedPassage_inout->getHeight(retainedPassageHeight) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getHeight returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    double candidatePassageHeight{};
    if (p_candidatePassage_in->getHeight(candidatePassageHeight) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getHeight returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_retainedPassage_inout->setHeight(
            std::max(retainedPassageHeight, candidatePassageHeight)) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setHeight returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    bool retainedPassageIsPassable{};
    if (p_retainedPassage_inout->isPassable(retainedPassageIsPassable) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isPassable returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    bool candidatePassageIsPassable{};
    if (!(retainedPassageIsPassable) &&
        p_candidatePassage_in->isPassable(candidatePassageIsPassable) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isPassable returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_retainedPassage_inout->setPassable(retainedPassageIsPassable ||
                                             candidatePassageIsPassable) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPassable returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    std::size_t retainedTraversalCount{};
    if (p_retainedPassage_inout->getTraversalObservationCount(
            retainedTraversalCount) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getTraversalObservationCount returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    std::size_t candidateTraversalCount{};
    if (p_candidatePassage_in->getTraversalObservationCount(
            candidateTraversalCount) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getTraversalObservationCount returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    const std::size_t maximumTraversalCount =
        std::numeric_limits<std::size_t>::max();

    if (p_retainedPassage_inout->setTraversalObservationCount(
            candidateTraversalCount >
                    maximumTraversalCount - retainedTraversalCount
                ? maximumTraversalCount
                : retainedTraversalCount + candidateTraversalCount) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setTraversalObservationCount returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    std::vector<vs_graphs::core::geometric::Plane *>
        candidatePassageAssociateWalls{};
    if (p_candidatePassage_in->getAssociateWalls(
            candidatePassageAssociateWalls) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAssociateWalls returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    for (geometric::Plane *p_supportingWall : candidatePassageAssociateWalls)
    {
        if (p_retainedPassage_inout->addAssociateWall(p_supportingWall) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addAssociateWall returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    vs_graphs::core::geometric::Plane *p_retainedPassageAssociateDoor = nullptr;
    if (p_retainedPassage_inout->getAssociateDoor(
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
        p_candidatePassage_in->getAssociateDoor(
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
        vs_graphs::core::geometric::Plane *p_candidatePassageAssociateDoor2 =
            nullptr;
        if (p_candidatePassage_in->getAssociateDoor(
                p_candidatePassageAssociateDoor2) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAssociateDoor returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_retainedPassage_inout->setAssociateDoor(
                p_candidatePassageAssociateDoor2) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setAssociateDoor returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} /* namespace utils */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */
