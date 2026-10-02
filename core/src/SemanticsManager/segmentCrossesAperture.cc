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
 * @file            segmentCrossesAperture.cc
 *
 * @brief           Implements segmentCrossesAperture(), declared in
 *                  SemanticsManager/private_functions.h.
 */

#include "SemanticsManager.h"

#include "private_functions.h"

#include <cmath>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief       Core aperture-crossing math shared by both a confirmed
 *              Passage and a still-unconfirmed OpenPassageEvidence
 *              hypothesis (see segmentCrossesOpenPassageEvidence below) --
 *              the two differ only in where the plane equation, centroid,
 *              and opening size come from, never in how the crossing test
 *              itself works.
 *
 * @param[in]   segmentStart_world_m_in     First endpoint, in metres.
 * @param[in]   segmentEnd_world_m_in       Second endpoint, in metres.
 * @param[in]   apertureEquation_world_in   Plane of the aperture.
 * @param[in]   apertureCentroid_world_m_in Centre of the aperture, in metres.
 * @param[in]   apertureWidth_m_in          Width of the opening, in metres.
 * @param[in]   apertureHeight_m_in         Height of the opening, in metres.
 * @param[in]   groundNormal_world_in       Unit ground normal.
 * @param[in]   openingMargin_m_in          Aperture expansion used for noisy
 *                                          geometry, in metres.
 * @param[in]   minimumSideDistance_m_in    Required endpoint distance from the
 *                                          plane, in metres.
 * @param[out]  crossesAperture_out         True when the segment crosses
 *                                          inside the opening.
 * @return      SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
SemanticsManagerStatus
    segmentCrossesAperture(const Eigen::Vector3d &segmentStart_world_m_in,
                           const Eigen::Vector3d &segmentEnd_world_m_in,
                           const Eigen::Vector4d &apertureEquation_world_in,
                           const Eigen::Vector3d &apertureCentroid_world_m_in,
                           const double           apertureWidth_m_in,
                           const double           apertureHeight_m_in,
                           const Eigen::Vector3d &groundNormal_world_in,
                           const double           openingMargin_m_in,
                           const double           minimumSideDistance_m_in,
                           bool                  &crossesAperture_out)
{
    if (!segmentStart_world_m_in.allFinite() ||
        !segmentEnd_world_m_in.allFinite() ||
        !apertureCentroid_world_m_in.allFinite())
    {
        crossesAperture_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    Eigen::Vector4d apertureEquation_world = apertureEquation_world_in;
    const double apertureNormalNorm = apertureEquation_world.head<3>().norm();

    if (!apertureEquation_world.allFinite() || apertureNormalNorm < 1e-8)
    {
        crossesAperture_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    apertureEquation_world /= apertureNormalNorm;
    const Eigen::Vector3d apertureNormal_world =
        apertureEquation_world.head<3>();
    const double startSide_m =
        apertureNormal_world.dot(segmentStart_world_m_in) +
        apertureEquation_world(3);
    const double endSide_m = apertureNormal_world.dot(segmentEnd_world_m_in) +
                             apertureEquation_world(3);

    if (startSide_m * endSide_m >= 0.0 ||
        std::abs(startSide_m) < minimumSideDistance_m_in ||
        std::abs(endSide_m) < minimumSideDistance_m_in)
    {
        crossesAperture_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const double interpolation = startSide_m / (startSide_m - endSide_m);

    if (!std::isfinite(interpolation) || interpolation < 0.0 ||
        interpolation > 1.0)
    {
        crossesAperture_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const Eigen::Vector3d intersection_world_m =
        segmentStart_world_m_in +
        interpolation * (segmentEnd_world_m_in - segmentStart_world_m_in);

    Eigen::Vector3d apertureOffset_world_m =
        intersection_world_m - apertureCentroid_world_m_in;
    apertureOffset_world_m -=
        apertureOffset_world_m.dot(apertureNormal_world) * apertureNormal_world;

    const double verticalOffset_m =
        std::abs(apertureOffset_world_m.dot(groundNormal_world_in));
    const Eigen::Vector3d horizontalOffset_world_m =
        apertureOffset_world_m -
        apertureOffset_world_m.dot(groundNormal_world_in) *
            groundNormal_world_in;
    const double horizontalOffset_m = horizontalOffset_world_m.norm();

    crossesAperture_out =
        horizontalOffset_m <= 0.5 * apertureWidth_m_in + openingMargin_m_in &&
        verticalOffset_m <= 0.5 * apertureHeight_m_in + openingMargin_m_in;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
