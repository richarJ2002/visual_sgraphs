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
 * @file            missingProofOwnerName.cc
 *
 * @brief           Implements missingProofOwnerName(), declared in
 *                  EnumNames.h.
 */

#include "Semantic/SemanticAxiomEvaluator/EnumNames.h"

namespace ORB_SLAM3
{
namespace semantic
{

std::string missingProofOwnerName(MissingProofOwner owner_in)
{
    switch (owner_in)
    {
    case MissingProofOwner::NONE:
        return "NONE";
    case MissingProofOwner::PHASE_2:
        return "PHASE_2";
    case MissingProofOwner::PHASE_3:
        return "PHASE_3";
    case MissingProofOwner::PHASE_4:
        return "PHASE_4";
    case MissingProofOwner::PHASE_4_OR_5:
        return "PHASE_4_OR_5";
    case MissingProofOwner::PHASE_5:
        return "PHASE_5";
    case MissingProofOwner::PHASE_6:
        return "PHASE_6";
    case MissingProofOwner::PHASE_7:
        return "PHASE_7";
    case MissingProofOwner::PHASE_8:
        return "PHASE_8";
    case MissingProofOwner::SCOPE_DECISION_REQUIRED:
        return "SCOPE_DECISION_REQUIRED";
    }
    return "UNKNOWN_MISSING_PROOF_OWNER";
}

} // namespace semantic
} // namespace ORB_SLAM3
