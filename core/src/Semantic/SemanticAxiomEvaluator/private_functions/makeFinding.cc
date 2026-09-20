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
 * @file            makeFinding.cc
 *
 * @brief           Implements makeFinding(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

#include <algorithm>
#include <string>
#include <utility>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

Finding makeFinding(AxiomCode              axiomCode_in,
                    AxiomResult            result_in,
                    ReasonCode             reasonCode_in,
                    std::vector<EntityKey> involvedKeys_in,
                    FindingEvidence        evidence_in)
{
    std::sort(involvedKeys_in.begin(), involvedKeys_in.end());
    involvedKeys_in.erase(
        std::unique(involvedKeys_in.begin(), involvedKeys_in.end()),
        involvedKeys_in.end());

    /* id is built purely from the numeric enumerator values of axiomCode_in
     * and reasonCode_in plus the now-sorted, deduplicated involvedKeys_in
     * -- never a name string (avoiding any dependency on enumerator
     * spelling), a hash (avoiding any dependency on a specific hash
     * algorithm remaining stable), a pointer address, wall-clock time, or
     * unordered-iteration artifact. Two Findings with identical axiomCode,
     * reasonCode, and involvedKeys always produce byte-identical ids. */
    std::string id = std::to_string(static_cast<unsigned int>(axiomCode_in)) +
                     '#' +
                     std::to_string(static_cast<unsigned int>(reasonCode_in));
    for (const EntityKey &key : involvedKeys_in)
    {
        id += '#';
        id += std::to_string(static_cast<unsigned int>(key.kind));
        id += '.';
        id += std::to_string(key.mapId);
        id += '.';
        id += std::to_string(key.entityId);
    }

    Finding finding;
    finding.id             = std::move(id);
    finding.axiomCode      = axiomCode_in;
    finding.result         = result_in;
    finding.classification = axiomClassFor(axiomCode_in);
    finding.reasonCode     = reasonCode_in;
    finding.involvedKeys   = std::move(involvedKeys_in);
    finding.evidence       = evidence_in;
    return finding;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
