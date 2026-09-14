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
 * @file            unavailableReasonName.cc
 *
 * @brief           Implements unavailableReasonName(), declared in
 *                  EnumNames.h.
 */

#include "Semantic/SemanticAxiomEvaluator/EnumNames.h"

namespace ORB_SLAM3
{
namespace semantic
{

std::string unavailableReasonName(UnavailableReason reason_in)
{
    switch (reason_in)
    {
    case UnavailableReason::NONE:
        return "NONE";
    case UnavailableReason::NULL_REFERENCE:
        return "NULL_REFERENCE";
    case UnavailableReason::ENTITY_HAS_NO_MAP:
        return "ENTITY_HAS_NO_MAP";
    case UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA:
        return "NOT_TRACKED_BY_CURRENT_SCHEMA";
    case UnavailableReason::NOT_EXPOSED_BY_CURRENT_API:
        return "NOT_EXPOSED_BY_CURRENT_API";
    case UnavailableReason::NOT_CAPTURED_IN_FOUNDATION_SLICE:
        return "NOT_CAPTURED_IN_FOUNDATION_SLICE";
    }
    return "UNKNOWN_UNAVAILABLE_REASON";
}

} // namespace semantic
} // namespace ORB_SLAM3
