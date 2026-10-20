/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            MergeAttemptStateStruct.h
 *
 * @brief           Declares the MergeAttemptState per-old-map merge
 * bookkeeping.
 */

#ifndef VS_GRAPHS_CORE_SRC_ATLAS_OBJECTS_MERGE_ATTEMPT_STATE_STRUCT_H
#define VS_GRAPHS_CORE_SRC_ATLAS_OBJECTS_MERGE_ATTEMPT_STATE_STRUCT_H

#include <chrono>
#include <cstddef>

namespace vs_graphs
{
namespace core
{
namespace atlas
{
/*!
 * @brief           Per-old-map bookkeeping for continuous consecutive-map
 *                  matching.
 *
 *                  Lives in the Atlas (not in either map) because it describes
 *                  a map PAIR relationship across semantic cycles: when the
 *                  pair was last attempted and what the content looked like
 *                  then, so unchanged pairs are not re-attempted every cycle.
 */
struct MergeAttemptState
{
    /*!
     * @brief           Steady-clock time of the latest merge attempt for this
     *                  old map. Only meaningful once hasEverAttempted is true.
     */
    std::chrono::steady_clock::time_point lastAttemptTime{};
    /*!
     * @brief           Content hash (see consecutiveContentHash) of the old and
     *                  current map at the latest attempt; an unchanged hash
     *                  lets the scheduler skip the pair.
     */
    std::size_t                           contentHashAtLastAttempt{0U};
    /*!
     * @brief           True once an attempt has been recorded for this old map;
     *                  false means no cooldown or hash check applies yet.
     */
    bool                                  hasEverAttempted{false};
};

} /* namespace atlas */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_ATLAS_OBJECTS_MERGE_ATTEMPT_STATE_STRUCT_H */
