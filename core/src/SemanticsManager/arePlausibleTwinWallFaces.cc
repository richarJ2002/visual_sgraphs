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

#include <algorithm>
#include <cmath>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief Decides whether two WALL Planes are plausibly the two opposite
 *        faces of the same physical wall (axiom (e)): parallel, a plausible
 *        wall thickness apart, observed from opposite exterior sides, and
 *        overlapping in-plane footprint.
 */
bool arePlausibleTwinWallFaces(geometric::Plane      *p_first_in,
                               geometric::Plane      *p_second_in,
                               double                 minimumThickness_m_in,
                               double                 maximumThickness_m_in,
                               double                 minimumOverlapRatio_in,
                               const Eigen::Vector3d &groundNormal_World_in)
{
    if (p_first_in == nullptr || p_first_in->isBad() ||
        p_second_in == nullptr || p_second_in->isBad() ||
        p_first_in == p_second_in)
    {
        return false;
    }

    if (!utils::utils::Utils::arePlanesParallel(p_first_in, p_second_in))
    {
        return false;
    }

    Eigen::Vector4d equation1   = p_first_in->getGlobalEquation().coeffs();
    Eigen::Vector4d equation2   = p_second_in->getGlobalEquation().coeffs();
    const double    normalNorm1 = equation1.head<3>().norm();
    const double    normalNorm2 = equation2.head<3>().norm();

    if (!equation1.allFinite() || !equation2.allFinite() ||
        normalNorm1 < 1e-8 || normalNorm2 < 1e-8)
    {
        return false;
    }

    equation1 /= normalNorm1;
    equation2 /= normalNorm2;

    /* Align equation2's sign to equation1's before comparing offsets --
     * plane-equation sign is arbitrary. */
    Eigen::Vector4d alignedEquation2 = equation2;
    if (equation1.head<3>().dot(equation2.head<3>()) < 0.0)
    {
        alignedEquation2 *= -1.0;
    }
    const double separation_m = std::abs(equation1(3) - alignedEquation2(3));

    if (separation_m < minimumThickness_m_in ||
        separation_m > maximumThickness_m_in)
    {
        return false;
    }

    /* Opposite-exterior-side check, generalising isWallFaceForeignToRoom's
     * side-sign math from plane-to-room to plane-to-plane: a wall's two
     * faces are observed from cameras standing on opposite exterior sides,
     * so each face's observation origin must resolve to opposite sides of
     * the OTHER face's equation. */
    const std::optional<Eigen::Vector3d> origin1 =
        p_first_in->getObservationOrigin_World();
    const std::optional<Eigen::Vector3d> origin2 =
        p_second_in->getObservationOrigin_World();

    if (!origin1.has_value() || !origin1->allFinite() || !origin2.has_value() ||
        !origin2->allFinite())
    {
        /* Planes created before the stamp existed carry no face identity;
         * make no claim rather than a wrong one. */
        return false;
    }

    constexpr double minimumResolvableSide_m = 0.10;
    const double     side1AtOrigin1 =
        equation1.head<3>().dot(origin1.value()) + equation1(3);
    const double side1AtOrigin2 =
        equation1.head<3>().dot(origin2.value()) + equation1(3);
    const double side2AtOrigin1 =
        equation2.head<3>().dot(origin1.value()) + equation2(3);
    const double side2AtOrigin2 =
        equation2.head<3>().dot(origin2.value()) + equation2(3);

    if (!std::isfinite(side1AtOrigin1) || !std::isfinite(side1AtOrigin2) ||
        !std::isfinite(side2AtOrigin1) || !std::isfinite(side2AtOrigin2) ||
        std::abs(side1AtOrigin1) < minimumResolvableSide_m ||
        std::abs(side1AtOrigin2) < minimumResolvableSide_m ||
        std::abs(side2AtOrigin1) < minimumResolvableSide_m ||
        std::abs(side2AtOrigin2) < minimumResolvableSide_m)
    {
        return false;
    }

    const bool oppositeAcrossPlane1 = (side1AtOrigin1 * side1AtOrigin2) < 0.0;
    const bool oppositeAcrossPlane2 = (side2AtOrigin1 * side2AtOrigin2) < 0.0;

    if (!oppositeAcrossPlane1 || !oppositeAcrossPlane2)
    {
        return false;
    }

    /* In-plane footprint overlap, projected onto one shared ground-anchored
     * tangent frame so the two planes' (possibly differently canonicalised)
     * own local U/V axes don't have to agree. */
    const double    groundNormalNorm = groundNormal_World_in.norm();
    Eigen::Vector3d axisU_World      = Eigen::Vector3d::Zero();
    Eigen::Vector3d axisV_World      = Eigen::Vector3d::Zero();
    if (std::isfinite(groundNormalNorm) && groundNormalNorm > 1e-8)
    {
        const Eigen::Vector3d unitGroundNormal_World =
            groundNormal_World_in / groundNormalNorm;
        const Eigen::Vector3d horizontalCandidate_World =
            unitGroundNormal_World.cross(equation1.head<3>());
        const double horizontalNorm = horizontalCandidate_World.norm();
        if (std::isfinite(horizontalNorm) && horizontalNorm > 1e-3)
        {
            axisU_World = horizontalCandidate_World / horizontalNorm;
            axisV_World = axisU_World.cross(equation1.head<3>()).normalized();
        }
    }
    if (axisU_World.squaredNorm() < 0.5 || axisV_World.squaredNorm() < 0.5)
    {
        axisU_World = equation1.head<3>().unitOrthogonal().normalized();
        axisV_World = equation1.head<3>().cross(axisU_World).normalized();
    }

    double minU1 = 0.0, maxU1 = 0.0, minV1 = 0.0, maxV1 = 0.0;
    double minU2 = 0.0, maxU2 = 0.0, minV2 = 0.0, maxV2 = 0.0;
    if (!projectPlaneFootprintOntoSharedAxes(p_first_in,
                                             axisU_World,
                                             axisV_World,
                                             minU1,
                                             maxU1,
                                             minV1,
                                             maxV1) ||
        !projectPlaneFootprintOntoSharedAxes(p_second_in,
                                             axisU_World,
                                             axisV_World,
                                             minU2,
                                             maxU2,
                                             minV2,
                                             maxV2))
    {
        return false;
    }

    const double overlapU_m = std::min(maxU1, maxU2) - std::max(minU1, minU2);
    const double overlapV_m = std::min(maxV1, maxV2) - std::max(minV1, minV2);

    if (overlapU_m <= 0.0 || overlapV_m <= 0.0)
    {
        return false;
    }

    const double overlapArea_m2 = overlapU_m * overlapV_m;
    const double area1_m2       = (maxU1 - minU1) * (maxV1 - minV1);
    const double area2_m2       = (maxU2 - minU2) * (maxV2 - minV2);
    const double smallerArea_m2 = std::min(area1_m2, area2_m2);

    if (!std::isfinite(smallerArea_m2) || smallerArea_m2 < 1e-6)
    {
        return false;
    }

    return (overlapArea_m2 / smallerArea_m2) >= minimumOverlapRatio_in;
}

} // namespace core
} // namespace vs_graphs
