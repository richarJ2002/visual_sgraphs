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
 * @file            segmentCrossesPassageOpening.cc
 *
 * @brief           Implements segmentCrossesPassageOpening(), declared in
 *                  SemanticsManager/private_functions.h.
 */

#include "SemanticsManager.h"

#include "private_functions.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief Tests whether a segment crosses a passage aperture.
 *
 * @param[in] segmentStart_World_m_in First endpoint in the active map frame.
 * @param[in] segmentEnd_World_m_in Second endpoint in the active map frame.
 * @param[in] p_passage_in Passage defining the finite aperture.
 * @param[in] groundNormal_World_in Unit ground normal in the active map frame.
 * @param[in] openingMargin_m_in Aperture expansion used for noisy geometry.
 * @param[in] minimumSideDistance_m_in Required endpoint distance from plane.
 * @param[out] crossesPassageOpening_out True when the segment crosses inside
 *              the finite opening.
 * @param[in] requirePassable_in True when the passage must already be passable
 *              before the geometric test may fire. Far-side wall routing may
 *              pass false so the aperture geometry alone drives the decision
 *              even while the passage is still being confirmed.
 * @return SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
SemanticsManagerStatus
    segmentCrossesPassageOpening(const Eigen::Vector3d &segmentStart_World_m_in,
                                 const Eigen::Vector3d &segmentEnd_World_m_in,
                                 semantic::Passage     *p_passage_in,
                                 const Eigen::Vector3d &groundNormal_World_in,
                                 const double           openingMargin_m_in,
                                 const double minimumSideDistance_m_in,
                                 bool        &crossesPassageOpening_out,
                                 const bool   requirePassable_in)
{
    bool passage_inIsPassable{};
    if (!(p_passage_in == nullptr) && (requirePassable_in) &&
        p_passage_in->isPassable(passage_inIsPassable) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isPassable returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_passage_in == nullptr ||
        (requirePassable_in && !passage_inIsPassable))
    {
        crossesPassageOpening_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
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
    Eigen::Vector3d passage_inCentroid{};
    if (p_passage_in->getCentroid(passage_inCentroid) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
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
    if (p_passage_in->getHeight(passage_inHeight) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getHeight returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool crossesAperture{};
    if (segmentCrossesAperture(segmentStart_World_m_in,
                               segmentEnd_World_m_in,
                               passage_inGlobalEquation.coeffs(),
                               passage_inCentroid,
                               passage_inWidth,
                               passage_inHeight,
                               groundNormal_World_in,
                               openingMargin_m_in,
                               minimumSideDistance_m_in,
                               crossesAperture) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: segmentCrossesAperture returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    crossesPassageOpening_out = crossesAperture;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
