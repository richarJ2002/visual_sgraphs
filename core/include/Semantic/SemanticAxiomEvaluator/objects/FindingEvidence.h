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
 * @file            FindingEvidence.h
 *
 * @brief           Declares the bounded, typed evidence payload attached to
 *                  one Finding.
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_FINDING_EVIDENCE_H
#define SEMANTIC_AXIOM_EVALUATOR_FINDING_EVIDENCE_H

#include <cstddef>
#include <optional>

namespace ORB_SLAM3
{
namespace semantic
{
/*!
 * @brief       Bounded, typed evidence carried by one Finding, interpreted
 *              per its ReasonCode.
 *
 *              Deliberately a small fixed-shape value, not free prose, a
 *              wall-clock timestamp, a pointer address, or an unstable hash:
 *              a count and/or a numeric value, both optional and populated
 *              only when the owning evaluator function's Doxygen documents
 *              their meaning for that specific ReasonCode. Entity identity
 *              beyond a Finding's own \c involvedKeys never belongs here --
 *              see Finding.h.
 */
struct FindingEvidence
{
  public:
    /*! @brief A bounded count relevant to the reason (e.g. observed owner
     *  count, wall/corner count); meaning is reason-specific. */
    std::optional<std::size_t> observedCount{};

    /*! @brief A second bounded count relevant to the reason (e.g. an
     *  expected/reference count to compare observedCount against);
     *  meaning is reason-specific. */
    std::optional<std::size_t> expectedCount{};

    /*! @brief A bounded numeric value relevant to the reason (e.g. a
     *  computed distance or angle); meaning is reason-specific. */
    std::optional<double> numericValue{};
};

} // namespace semantic
} // namespace ORB_SLAM3

#endif // SEMANTIC_AXIOM_EVALUATOR_FINDING_EVIDENCE_H
