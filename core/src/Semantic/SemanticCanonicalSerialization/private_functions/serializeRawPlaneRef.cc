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
 * @file            serializeRawPlaneRef.cc
 *
 * @brief           Implements serializeRawPlaneRef(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

nlohmann::json serializeRawPlaneRef(const RawPlaneRef &value_in)
{
    nlohmann::json json;
    if (value_in.mapId.has_value())
    {
        json["mapId"] = *value_in.mapId;
    }
    json["planeId"]    = value_in.planeId;
    json["isLive"]     = value_in.isLive;
    json["planeType"]  = static_cast<int>(value_in.planeType);
    json["reason"]     = static_cast<unsigned int>(value_in.reason);
    json["reasonName"] = unavailableReasonName(value_in.reason);
    if (value_in.wallKey.has_value())
    {
        json["wallKey"] = serializeEntityKey(*value_in.wallKey);
    }
    return json;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
