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
 * @file            checkObserverSideAgreement.cc
 *
 * @brief           Implements checkObserverSideAgreement(), declared in
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

UtilsStatus checkObserverSideAgreement(
    geometric::Plane                                *p_mappedPlane_in,
    const Eigen::Vector4d                           &mappedEquation_in,
    const Eigen::Vector3d                           *p_cameraCentre_world_m_in,
    bool                                             hasCameraCentre_in,
    const geometric::Plane::ObservationSideSnapshot *p_observingCameras_in,
    double  minimumReliableSideDistance_m_in,
    bool   &isOppositeSide_out,
    double &observerSide_m_out)
{
    isOppositeSide_out = false;
    observerSide_m_out = 0.0;

    ObservationSideEvidence mappedObservationSide{};
    if (getMedianObservationSide_world_m(p_mappedPlane_in,
                                         mappedEquation_in,
                                         mappedObservationSide) !=
        UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMedianObservationSide_world_m returned a "
                     "failure status although it cannot fail; "
                     "continuing as before.",
                     __func__);
    }

    std::optional<double> givenObservationSide_m{};
    bool                  givenSideIsAmbiguous = false;

    if (hasCameraCentre_in)
    {
        const double cameraSide_m =
            mappedEquation_in.head<3>().dot(*p_cameraCentre_world_m_in) +
            mappedEquation_in(3);

        if (std::abs(cameraSide_m) >= minimumReliableSideDistance_m_in)
        {
            givenObservationSide_m = cameraSide_m;
        }
    }
    else
    {
        givenObservationSide_m = p_observingCameras_in->medianSignedDistance_m;
        givenSideIsAmbiguous =
            p_observingCameras_in->face ==
            geometric::Plane::ObservationSideSnapshot::Face::AMBIGUOUS;
    }

    if (givenSideIsAmbiguous || mappedObservationSide.isAmbiguous)
    {
        isOppositeSide_out = true;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    if (givenObservationSide_m.has_value() &&
        mappedObservationSide.medianSignedDistance_m.has_value() &&
        givenObservationSide_m.value() *
                mappedObservationSide.medianSignedDistance_m.value() <
            0.0)
    {
        isOppositeSide_out = true;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    observerSide_m_out = mappedObservationSide.medianSignedDistance_m.value_or(
        givenObservationSide_m.value_or(0.0));

    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} /* namespace utils */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */
