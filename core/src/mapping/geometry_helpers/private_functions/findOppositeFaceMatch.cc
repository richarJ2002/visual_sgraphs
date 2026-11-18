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
 * @file            findOppositeFaceMatch.cc
 *
 * @brief           Implements findOppositeFaceMatch(), declared in
 *                  geometry_helpers/private_functions/private_functions.h.
 */

#include "configuration/types/objects/SystemParamsStatusEnum.h"
#include "configuration/types/types.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/private_functions/private_functions.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <variant>

#include <pcl/common/point_tests.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus findOppositeFaceMatch(
    const std::vector<geometric::Plane *> &mappedPlanes_in,
    const PlaneEvidence                   &observedPlane_in,
    const PlaneMatchSettings              &matchSettings_in,
    int                                   &matchedPlaneId_out,
    std::vector<PlaneMatchVerdict>        *p_candidateVerdicts_out)
{
    /* Most observations wins; the earliest candidate wins a tie. */
    matchedPlaneId_out = -1;

    std::size_t bestObservationCount = 0U;

    for (geometric::Plane *p_mappedPlane : mappedPlanes_in)
    {
        PlaneMatchVerdict candidateVerdict{};
        if (checkOppositeFacePair(observedPlane_in,
                                  p_mappedPlane,
                                  matchSettings_in,
                                  candidateVerdict) !=
            UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: checkOppositeFacePair returned a failure "
                         "status although it cannot fail; continuing as "
                         "before.",
                         __func__);
        }
        if (p_candidateVerdicts_out != nullptr)
        {
            p_candidateVerdicts_out->push_back(candidateVerdict);
        }
        if (candidateVerdict.decidingCheck != TwinFaceCheck::PLAUSIBLE)
        {
            continue;
        }

        std::size_t observationCount{};
        if (p_mappedPlane->getObservationCount(observationCount) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getObservationCount returned a failure "
                         "status although it cannot fail; continuing as "
                         "before.",
                         __func__);
        }
        if (matchedPlaneId_out < 0 || observationCount > bestObservationCount)
        {
            int mappedPlaneId{};
            if (p_mappedPlane->getId(mappedPlaneId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status "
                             "although it cannot fail; continuing as "
                             "before.",
                             __func__);
            }
            matchedPlaneId_out   = mappedPlaneId;
            bestObservationCount = observationCount;
        }
    }
    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} /* namespace utils */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */
