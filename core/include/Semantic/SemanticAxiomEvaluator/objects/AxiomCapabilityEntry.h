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
 * @file         AxiomCapabilityEntry.h
 *
 * @brief        Declares one row of the fixed,
 *               snapshot-independent axiom capability/ownership
 *               table (computeAxiomCapabilityTable()).
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_AXIOM_CAPABILITY_ENTRY_H
#define SEMANTIC_AXIOM_EVALUATOR_AXIOM_CAPABILITY_ENTRY_H

#include "Semantic/SemanticAxiomEvaluator/objects/AxiomClass.h"
#include "Semantic/SemanticAxiomEvaluator/objects/AxiomCode.h"
#include "Semantic/SemanticAxiomEvaluator/objects/CapabilityLevel.h"
#include "Semantic/SemanticAxiomEvaluator/objects/MissingProofOwner.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief        One axiom code's fixed capability classification
 *               and, when not FULL, the evidence area that owns
 *               supplying the missing proof. Every field is a
 *               property of this evaluator's current implementation
 *               against the current SemanticGraphSnapshot schema,
 *               not of any one evaluated snapshot.
 */
struct AxiomCapabilityEntry
{
  public:
    /*! @brief Which of the sixteen axiom codes this row describes. */
    AxiomCode axiomCode{AxiomCode::AX_FRAME_01};

    /*!
     * @brief        Fixed "Class" column value for axiomCode.
     */
    AxiomClass classification{AxiomClass::HARD};

    /*! @brief How completely this axiom code can be proven from the
     *  current schema. */
    CapabilityLevel capability{CapabilityLevel::FULL};

    /*!
     * @brief        MissingProofOwner::NONE when capability == FULL;
     *               otherwise the evidence area (or
     *               SCOPE_DECISION_REQUIRED) that owns the missing
     *               evidence.
     */
    MissingProofOwner owner{MissingProofOwner::NONE};
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_AXIOM_EVALUATOR_AXIOM_CAPABILITY_ENTRY_H
