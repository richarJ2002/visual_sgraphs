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
 * @file            passageOpening.cc
 *
 * @brief           Implements crossesPassablePassageOpening(), declared
 *                  in Utils/Utils/private_functions.h.
 */

#include "Utils/Utils/private_functions.h"

#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus crossesPassablePassageOpening(
    const Eigen::Vector3d              &segmentStart_world_m_in,
    const Eigen::Vector3d              &segmentEnd_world_m_in,
    vs_graphs::core::semantic::Passage *p_passage_in,
    const Eigen::Vector3d              &groundNormal_world_in,
    const double                        openingMargin_m_in,
    const double                        minimumSideDistance_m_in,
    bool                               &crossesOpening_out)
{
    bool passage_inIsPassable{};
    if (!(p_passage_in == nullptr) &&
        p_passage_in->isPassable(passage_inIsPassable) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isPassable returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_passage_in == nullptr || !passage_inIsPassable ||
        !segmentStart_world_m_in.allFinite() ||
        !segmentEnd_world_m_in.allFinite())
    {
        crossesOpening_out = false;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    g2o::Plane3D passage_inGlobalEquation{};
    if (p_passage_in->getGlobalEquation(passage_inGlobalEquation) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector4d passageEquation_world = passage_inGlobalEquation.coeffs();
    const double    passageNormalNorm = passageEquation_world.head<3>().norm();

    if (!passageEquation_world.allFinite() || passageNormalNorm < 1e-8)
    {
        crossesOpening_out = false;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    passageEquation_world /= passageNormalNorm;
    const Eigen::Vector3d passageNormal_world = passageEquation_world.head<3>();
    const double          startSide_m =
        passageNormal_world.dot(segmentStart_world_m_in) +
        passageEquation_world(3);
    const double endSide_m = passageNormal_world.dot(segmentEnd_world_m_in) +
                             passageEquation_world(3);

    if (startSide_m * endSide_m >= 0.0 ||
        std::abs(startSide_m) < minimumSideDistance_m_in ||
        std::abs(endSide_m) < minimumSideDistance_m_in)
    {
        crossesOpening_out = false;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    const double interpolation = startSide_m / (startSide_m - endSide_m);

    if (!std::isfinite(interpolation) || interpolation < 0.0 ||
        interpolation > 1.0)
    {
        crossesOpening_out = false;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    const Eigen::Vector3d planeIntersection_world_m =
        segmentStart_world_m_in +
        interpolation * (segmentEnd_world_m_in - segmentStart_world_m_in);
    Eigen::Vector3d passageCentroid_world_m{};
    if (p_passage_in->getCentroid(passageCentroid_world_m) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    if (!passageCentroid_world_m.allFinite())
    {
        crossesOpening_out = false;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    Eigen::Vector3d apertureOffset_world_m =
        planeIntersection_world_m - passageCentroid_world_m;
    apertureOffset_world_m -=
        apertureOffset_world_m.dot(passageNormal_world) * passageNormal_world;

    const double verticalOffset_m =
        std::abs(apertureOffset_world_m.dot(groundNormal_world_in));
    const Eigen::Vector3d horizontalOffset_world_m =
        apertureOffset_world_m -
        apertureOffset_world_m.dot(groundNormal_world_in) *
            groundNormal_world_in;
    const double horizontalOffset_m = horizontalOffset_world_m.norm();

    double passage_inWidth{};
    if (p_passage_in->getWidth(passage_inWidth) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWidth returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    double passage_inHeight{};
    if ((horizontalOffset_m <= 0.5 * passage_inWidth + openingMargin_m_in) &&
        p_passage_in->getHeight(passage_inHeight) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getHeight returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    crossesOpening_out =
        horizontalOffset_m <= 0.5 * passage_inWidth + openingMargin_m_in &&
        verticalOffset_m <= 0.5 * passage_inHeight + openingMargin_m_in;
    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
