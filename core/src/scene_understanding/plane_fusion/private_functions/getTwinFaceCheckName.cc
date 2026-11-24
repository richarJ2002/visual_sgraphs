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
 * @file            getTwinFaceCheckName.cc
 *
 * @brief           Implements getTwinFaceCheckName(), declared in
 *                  plane_fusion/private_functions/private_functions.h.
 */

#include "scene_understanding/plane_fusion/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

const char *getTwinFaceCheckName(utils::utils::TwinFaceCheck twinFaceCheck_in)
{
    switch (twinFaceCheck_in)
    {
    case utils::utils::TwinFaceCheck::PLAUSIBLE:
        return "PLAUSIBLE";
    case utils::utils::TwinFaceCheck::INVALID_PLANE:
        return "INVALID_PLANE";
    case utils::utils::TwinFaceCheck::NOT_PARALLEL:
        return "NOT_PARALLEL";
    case utils::utils::TwinFaceCheck::THICKNESS_OUT_OF_RANGE:
        return "THICKNESS_OUT_OF_RANGE";
    case utils::utils::TwinFaceCheck::NO_OBSERVATION_ORIGIN:
        return "NO_OBSERVATION_ORIGIN";
    case utils::utils::TwinFaceCheck::OBSERVATION_SIDE_UNRESOLVED:
        return "OBSERVATION_SIDE_UNRESOLVED";
    case utils::utils::TwinFaceCheck::OBSERVED_FROM_SAME_SIDE:
        return "OBSERVED_FROM_SAME_SIDE";
    case utils::utils::TwinFaceCheck::NO_FOOTPRINT_OVERLAP:
        return "NO_FOOTPRINT_OVERLAP";
    case utils::utils::TwinFaceCheck::LOW_OVERLAP:
        return "LOW_OVERLAP";
    }
    return "UNKNOWN";
}

} /* namespace core */
} /* namespace vs_graphs */
