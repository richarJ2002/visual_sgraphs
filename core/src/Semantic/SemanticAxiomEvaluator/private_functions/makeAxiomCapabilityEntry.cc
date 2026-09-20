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
 * @file            makeAxiomCapabilityEntry.cc
 *
 * @brief           Implements makeAxiomCapabilityEntry(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

AxiomCapabilityEntry makeAxiomCapabilityEntry(AxiomCode         axiomCode_in,
                                              CapabilityLevel   capability_in,
                                              MissingProofOwner owner_in)
{
    AxiomCapabilityEntry entry;
    entry.axiomCode      = axiomCode_in;
    entry.classification = axiomClassFor(axiomCode_in);
    entry.capability     = capability_in;
    entry.owner          = owner_in;
    return entry;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
