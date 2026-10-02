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
 * @file            segmentCrossesOpenPassageEvidence.cc
 *
 * @brief           Implements segmentCrossesOpenPassageEvidence(), declared in
 *                  SemanticsManager/private_functions.h.
 */

#include "SemanticsManager.h"

#include "private_functions.h"

#include <algorithm>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief       Same aperture-crossing test as segmentCrossesPassageOpening,
 *              but against a still-unconfirmed OpenPassageEvidence
 *              hypothesis instead of a confirmed Passage.
 *
 *              Passage confirmation requires several genuinely independent
 *              Voxblox skeleton snapshots (minimumConfirmationSnapshots,
 *              config-gated to guard against double-counting one latched ROS
 *              message -- see the skeleton-fingerprint check in
 *              detectDoorsAndDoorways()) and therefore real elapsed
 *              exploration time. Until that confirmation completes, no
 *              Passage object exists for mpAtlas->GetAllPassages() to
 *              return, so any far-side-routing check that only consults
 *              confirmed passages is blind for that entire window -- a wall
 *              genuinely on the far side of a real, already-evidenced
 *              opening falls through to ordinary admission and gets bound
 *              to the WRONG (near) room, exactly the corruption far-side
 *              routing exists to prevent. Using the same aperture geometry
 *              math against the pending evidence (its supporting wall's
 *              plane stands in for the eventual passage plane, its
 *              openingRadius_m/heightSpan_m for the eventual width/height --
 *              the same derivation createMapPassage() itself uses once
 *              confirmed) closes that window without weakening the
 *              confirmation gate itself: the passage still is not created,
 *              only wall ADMISSION becomes conservative while its identity
 *              is still ambiguous.
 */
SemanticsManagerStatus segmentCrossesOpenPassageEvidence(
    const Eigen::Vector3d &segmentStart_world_m_in,
    const Eigen::Vector3d &segmentEnd_world_m_in,
    geometric::Plane      *p_evidenceSupportingWall_in,
    const Eigen::Vector3d &evidenceCentroid_world_m_in,
    const double           evidenceOpeningRadius_m_in,
    const double           evidenceHeightSpan_m_in,
    const Eigen::Vector3d &groundNormal_world_in,
    const double           openingMargin_m_in,
    const double           minimumSideDistance_m_in,
    bool                  &crossesOpenPassageEvidence_out)
{
    bool evidenceSupportingWallIsBad{};
    if (!(p_evidenceSupportingWall_in == nullptr) &&
        p_evidenceSupportingWall_in->isBad(evidenceSupportingWallIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_evidenceSupportingWall_in == nullptr || evidenceSupportingWallIsBad ||
        evidenceOpeningRadius_m_in <= 0.0)
    {
        crossesOpenPassageEvidence_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    constexpr double defaultOpenPassageHeight_m = 2.0;

    g2o::Plane3D evidenceSupportingWallGetGlobalEquation{};
    if (p_evidenceSupportingWall_in->getGlobalEquation(
            evidenceSupportingWallGetGlobalEquation) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    bool crossesAperture{};
    if (segmentCrossesAperture(
            segmentStart_world_m_in,
            segmentEnd_world_m_in,
            evidenceSupportingWallGetGlobalEquation.coeffs(),
            evidenceCentroid_world_m_in,
            2.0 * evidenceOpeningRadius_m_in,
            std::max(evidenceHeightSpan_m_in, defaultOpenPassageHeight_m),
            groundNormal_world_in,
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
    crossesOpenPassageEvidence_out = crossesAperture;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
