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
 * @file            AxiomResult.h
 *
 * @brief           Declares the tri-state outcome of one axiom evaluation.
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_AXIOM_RESULT_H
#define SEMANTIC_AXIOM_EVALUATOR_AXIOM_RESULT_H

#include <cstdint>

namespace ORB_SLAM3
{
namespace semantic
{
/*!
 * @brief       Tri-state outcome of one axiom evaluation, at either finding
 *              or aggregate granularity.
 *
 *              Aggregation precedence is always FAIL > UNKNOWN > PASS
 *              (aggregateFindings()): a single FAIL anywhere makes the whole
 *              aggregate FAIL; otherwise a single UNKNOWN makes it UNKNOWN;
 *              only unanimous PASS (or a vacuously empty finding set for an
 *              axiom whose contract does not require a witness) yields PASS.
 *              Missing evidence must never be reported as PASS -- see each
 *              per-axiom evaluator's own Doxygen for when it emits UNKNOWN
 *              instead of guessing.
 */
enum class AxiomResult : std::uint8_t
{
    /*! @brief The observable clause holds; positive proof was available. */
    PASS = 0U,

    /*! @brief An observable contradiction was found. */
    FAIL = 1U,

    /*! @brief Required proof is unavailable; no contradiction was found
     *  either. Never upgraded to PASS merely because nothing looked wrong. */
    UNKNOWN = 2U
};

} // namespace semantic
} // namespace ORB_SLAM3

#endif // SEMANTIC_AXIOM_EVALUATOR_AXIOM_RESULT_H
