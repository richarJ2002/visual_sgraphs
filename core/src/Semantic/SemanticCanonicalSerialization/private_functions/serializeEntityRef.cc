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
 * @file            serializeEntityRef.cc
 *
 * @brief           Implements serializeEntityRef(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

nlohmann::json serializeEntityRef(const EntityRef &value_in)
{
    nlohmann::json json;
    if (value_in.key.has_value())
    {
        json["key"] = serializeEntityKey(*value_in.key);
    }
    json["reason"]     = static_cast<unsigned int>(value_in.reason);
    json["reasonName"] = unavailableReasonName(value_in.reason);
    if (value_in.localId.has_value())
    {
        json["localId"] = *value_in.localId;
    }
    if (value_in.isLive.has_value())
    {
        json["isLive"] = *value_in.isLive;
    }
    json["livenessUnavailableReason"] =
        static_cast<unsigned int>(value_in.livenessUnavailableReason);
    json["livenessUnavailableReasonName"] =
        unavailableReasonName(value_in.livenessUnavailableReason);
    return json;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
