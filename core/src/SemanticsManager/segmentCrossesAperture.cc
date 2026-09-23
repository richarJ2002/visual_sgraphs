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

#include "SemanticsManager.h"

#include "private_functions.h"

#include <cmath>

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
 * @param[in] requirePassable_in True when the passage must already be passable
 *              before the geometric test may fire. Far-side wall routing may
 *              pass false so the aperture geometry alone drives the decision
 *              even while the passage is still being confirmed.
 * @return True when the segment crosses inside the finite opening.
 */
/*!
 * @brief       Core aperture-crossing math shared by both a confirmed
 *              Passage and a still-unconfirmed OpenPassageEvidence
 *              hypothesis (see segmentCrossesOpenPassageEvidence below) --
 *              the two differ only in where the plane equation, centroid,
 *              and opening size come from, never in how the crossing test
 *              itself works.
 */
bool segmentCrossesAperture(const Eigen::Vector3d &segmentStart_World_m_in,
                            const Eigen::Vector3d &segmentEnd_World_m_in,
                            const Eigen::Vector4d &apertureEquation_World_in,
                            const Eigen::Vector3d &apertureCentroid_World_m_in,
                            const double           apertureWidth_m_in,
                            const double           apertureHeight_m_in,
                            const Eigen::Vector3d &groundNormal_World_in,
                            const double           openingMargin_m_in,
                            const double           minimumSideDistance_m_in)
{
    if (!segmentStart_World_m_in.allFinite() ||
        !segmentEnd_World_m_in.allFinite() ||
        !apertureCentroid_World_m_in.allFinite())
    {
        return false;
    }

    Eigen::Vector4d apertureEquation_World = apertureEquation_World_in;
    const double apertureNormalNorm = apertureEquation_World.head<3>().norm();

    if (!apertureEquation_World.allFinite() || apertureNormalNorm < 1e-8)
    {
        return false;
    }

    apertureEquation_World /= apertureNormalNorm;
    const Eigen::Vector3d apertureNormal_World =
        apertureEquation_World.head<3>();
    const double startSide_m =
        apertureNormal_World.dot(segmentStart_World_m_in) +
        apertureEquation_World(3);
    const double endSide_m = apertureNormal_World.dot(segmentEnd_World_m_in) +
                             apertureEquation_World(3);

    if (startSide_m * endSide_m >= 0.0 ||
        std::abs(startSide_m) < minimumSideDistance_m_in ||
        std::abs(endSide_m) < minimumSideDistance_m_in)
    {
        return false;
    }

    const double interpolation = startSide_m / (startSide_m - endSide_m);

    if (!std::isfinite(interpolation) || interpolation < 0.0 ||
        interpolation > 1.0)
    {
        return false;
    }

    const Eigen::Vector3d intersection_World_m =
        segmentStart_World_m_in +
        interpolation * (segmentEnd_World_m_in - segmentStart_World_m_in);

    Eigen::Vector3d apertureOffset_World_m =
        intersection_World_m - apertureCentroid_World_m_in;
    apertureOffset_World_m -=
        apertureOffset_World_m.dot(apertureNormal_World) * apertureNormal_World;

    const double verticalOffset_m =
        std::abs(apertureOffset_World_m.dot(groundNormal_World_in));
    const Eigen::Vector3d horizontalOffset_World_m =
        apertureOffset_World_m -
        apertureOffset_World_m.dot(groundNormal_World_in) *
            groundNormal_World_in;
    const double horizontalOffset_m = horizontalOffset_World_m.norm();

    return horizontalOffset_m <=
               0.5 * apertureWidth_m_in + openingMargin_m_in &&
           verticalOffset_m <= 0.5 * apertureHeight_m_in + openingMargin_m_in;
}

} // namespace core
} // namespace vs_graphs
