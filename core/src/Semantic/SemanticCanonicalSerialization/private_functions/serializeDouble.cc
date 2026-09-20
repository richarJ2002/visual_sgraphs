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
 * @file            serializeDouble.cc
 *
 * @brief           Implements serializeDouble(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

#include <cmath>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

nlohmann::json serializeDouble(double value_in)
{
    if (std::isnan(value_in))
    {
        return nlohmann::json("NaN");
    }
    if (std::isinf(value_in))
    {
        return nlohmann::json(value_in > 0.0 ? "Infinity" : "-Infinity");
    }
    /* Every finite value, including signed zero: nlohmann::json's own
     * numeric formatter already preserves -0.0's sign in its textual dump
     * (verified directly against this vendored version) and is
     * locale-independent by construction (it never calls a locale-
     * sensitive C function such as snprintf for this). */
    return nlohmann::json(value_in);
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
