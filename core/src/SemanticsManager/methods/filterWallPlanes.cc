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

namespace vs_graphs
{
namespace core
{

void SemanticsManager::filterWallPlanes(void)
{
    /* Iterate through all the planes and filter the walls */
    for (const auto &plane : p_atlas->getAllPlanes())
    {
        /* Skip planes which are not classed as walls */
        if (plane->getExpectedPlaneType() ==
            vs_graphs::core::geometric::Plane::PlaneVariant::WALL)
        {
            /*!
             * Wall validation based on the mPlanePoseMat only works if the
             * ground plane is set. Needs the correction matrix: mPlanePoseMat.
             */
            Eigen::Vector3f transformedPlaneCoefficients =
                transformPlaneEqToGroundReference(
                    plane->getGlobalEquation().coeffs());

            /*!
             * If the transformed plane is vertical based on absolute value,
             * then assign semantic, otherwise ignore threshold should be
             * leniently set (ideally with correct ground plane reference, this
             * value should be close to 0.00)
             */
            if (abs(transformedPlaneCoefficients(1)) >
                p_sysParams->semSeg.maxTiltWall)
            {
                plane->resetPlaneSemantics();
            }
        }
    }
}

} // namespace core
} // namespace vs_graphs
