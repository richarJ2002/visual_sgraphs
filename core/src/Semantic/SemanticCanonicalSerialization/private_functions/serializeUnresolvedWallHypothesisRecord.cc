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
 * @file            serializeUnresolvedWallHypothesisRecord.cc
 *
 * @brief           Implements serializeUnresolvedWallHypothesisRecord(),
 *                  declared in private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

nlohmann::json serializeUnresolvedWallHypothesisRecord(
    const UnresolvedWallHypothesisRecord &value_in)
{
    nlohmann::json json;
    json["wallRef"]          = serializeRawPlaneRef(value_in.wallRef);
    json["unresolvedCycles"] = value_in.unresolvedCycles;
    json["cloudPointCount"]  = value_in.cloudPointCount;
    json["observationCount"] = value_in.observationCount;
    return json;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
