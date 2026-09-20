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
 * @file         AxiomClass.h
 *
 * @brief        Declares the severity/classification distinguishing
 *               a hard contradiction from a derived aggregate,
 *               mirroring the catalogue's "Class" column.
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_AXIOM_CLASS_H
#define SEMANTIC_AXIOM_EVALUATOR_AXIOM_CLASS_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief        Severity classification carried by every Finding and
 *               by each axiom code's fixed catalogue entry
 *               (axiomClassFor()).
 *
 *               A HARD axiom is a direct contradiction of the
 *               semantic model (fifteen of the sixteen codes are
 *               "hard"). A DERIVED axiom (AX-COMP-01 only)
 *               aggregates other axioms' outcomes rather than
 *               checking a contradiction of its own; a DERIVED
 *               FAIL/UNKNOWN reports that its inputs were not all
 *               PASS, not a newly discovered independent
 *               contradiction.
 */
enum class AxiomClass : std::uint8_t
{
    /*! @brief A direct semantic contradiction. */
    HARD = 0U,

    /*! @brief An aggregate over other axioms' outcomes. */
    DERIVED = 1U
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_AXIOM_EVALUATOR_AXIOM_CLASS_H
