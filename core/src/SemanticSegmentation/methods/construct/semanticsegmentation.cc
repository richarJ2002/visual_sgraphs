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

#include "SemanticSegmentation.h"

namespace vs_graphs
{
namespace core
{

SemanticSegmentation::SemanticSegmentation(Atlas *p_atlas_in)
{
    /* Store atlas object address */
    p_atlas = p_atlas_in;

    /* Get the system parameters */
    types::SystemParams *p_params = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        // getParams cannot fail; continue as before.
    }
    p_sysParams = p_params;

    /* Set the booleans according to the mode of operation */
    isGeometricSegmentationRunning =
        !(p_sysParams->general.modeOfOperation ==
          types::SystemParams::General::ModeOfOperation::SEM);
}

} // namespace core
} // namespace vs_graphs
