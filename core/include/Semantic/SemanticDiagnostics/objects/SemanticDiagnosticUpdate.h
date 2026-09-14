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
 * @file            SemanticDiagnosticUpdate.h
 *
 * @brief           Declares the pure result of one
 *                   buildSemanticDiagnosticUpdate() call: whether to emit,
 *                   and the exact bounded JSON to print if so. Carries no
 *                   I/O of its own -- the caller alone decides how/whether
 *                   to print it (SG_AXIOM/SG_VIOLATION prefixes, stream
 *                   choice), keeping this module free of logging concerns.
 */

#ifndef SEMANTIC_DIAGNOSTICS_UPDATE_H
#define SEMANTIC_DIAGNOSTICS_UPDATE_H

#include <cstdint>
#include <vector>

#include "Thirdparty/nlohmann/json.hpp"

namespace ORB_SLAM3
{
namespace semantic
{
/*!
 * @brief       Pure output of one buildSemanticDiagnosticUpdate() call.
 */
struct SemanticDiagnosticUpdate
{
  public:
    /*! @brief True when this cycle's state changed (a "summary" eventType)
     *  or the heartbeat is due (a "heartbeat" eventType); false when
     *  neither condition holds and the caller must print nothing. */
    bool emit{false};

    /*! @brief The complete SG_AXIOM summary object; only meaningful when
     *  \c emit is true. */
    nlohmann::json summary;

    /*! @brief One SG_VIOLATION detail object per emitted appeared/changed/
     *  resolved FAIL-finding transition, already capped at
     *  kMaxViolationDetailsPerCycle and in deterministic order; only
     *  meaningful when \c emit is true. Never contains a PASS or UNKNOWN
     *  finding -- see buildSemanticDiagnosticUpdate.cc. */
    std::vector<nlohmann::json> violationDetails;
};

} // namespace semantic
} // namespace ORB_SLAM3

#endif // SEMANTIC_DIAGNOSTICS_UPDATE_H
