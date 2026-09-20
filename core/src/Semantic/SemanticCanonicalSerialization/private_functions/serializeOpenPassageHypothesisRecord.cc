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
 * @file            serializeOpenPassageHypothesisRecord.cc
 *
 * @brief           Implements serializeOpenPassageHypothesisRecord(),
 *                  declared in private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

nlohmann::json serializeOpenPassageHypothesisRecord(
    const OpenPassageHypothesisRecord &value_in,
    bool                               includeGeometry_in)
{
    nlohmann::json json;
    json["supportingWallRef"] =
        serializeRawPlaneRef(value_in.supportingWallRef);
    json["confirmationCount"] = value_in.confirmationCount;
    json["missedUpdateCount"] = value_in.missedUpdateCount;
    json["lastConfirmedSkeletonFingerprint"] =
        value_in.lastConfirmedSkeletonFingerprint;

    if (includeGeometry_in)
    {
        json["centroid_World_m"] = serializeVector3d(value_in.centroid_World_m);
        json["openingRadius_m"]  = serializeDouble(value_in.openingRadius_m);
        json["heightSpan_m"]     = serializeDouble(value_in.heightSpan_m);
    }

    return json;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
