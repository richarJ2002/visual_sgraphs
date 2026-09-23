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

namespace vs_graphs
{
namespace core
{

bool segmentCrossesPassageOpening(
    const Eigen::Vector3d &segmentStart_World_m_in,
    const Eigen::Vector3d &segmentEnd_World_m_in,
    semantic::Passage     *p_passage_in,
    const Eigen::Vector3d &groundNormal_World_in,
    const double           openingMargin_m_in,
    const double           minimumSideDistance_m_in,
    const bool             requirePassable_in)
{
    if (p_passage_in == nullptr ||
        (requirePassable_in && !p_passage_in->isPassable()))
    {
        return false;
    }

    return segmentCrossesAperture(segmentStart_World_m_in,
                                  segmentEnd_World_m_in,
                                  p_passage_in->getGlobalEquation().coeffs(),
                                  p_passage_in->getCentroid(),
                                  p_passage_in->getWidth(),
                                  p_passage_in->getHeight(),
                                  groundNormal_World_in,
                                  openingMargin_m_in,
                                  minimumSideDistance_m_in);
}

} // namespace core
} // namespace vs_graphs
