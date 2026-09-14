/**
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
 * @file            serializeFiniteAwareDouble.cc
 *
 * @brief           Implements serializeFiniteAwareDouble(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticDiagnostics/private_functions.h"

#include <cmath>

namespace ORB_SLAM3
{
namespace semantic
{

nlohmann::json serializeFiniteAwareDouble(double value_in)
{
    if (std::isnan(value_in))
    {
        return "NaN";
    }
    if (std::isinf(value_in))
    {
        return value_in > 0.0 ? "Infinity" : "-Infinity";
    }
    return value_in;
}

} // namespace semantic
} // namespace ORB_SLAM3
