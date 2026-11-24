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
 * @file            collectAbsorbedSceneGraph.cc
 *
 * @brief           Implements collectAbsorbedSceneGraph(), declared in
 *                  graph/merging/private_functions.h.
 */

#include "scene_understanding/semantics_manager/graph/merging/private_functions/private_functions.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void collectAbsorbedSceneGraph(SemanticsManager *p_semanticsManager_in,
                               unsigned long     absorbedMapId_in,
                               AbsorbedSceneGraphContents &absorbedContents_out)
{
    absorbedContents_out = AbsorbedSceneGraphContents{};

    /* Copy the passages, detected rooms, candidate (marker-based) rooms and
     * floors the semantics manager lists for the absorbed map. */
    if ((p_semanticsManager_in->getMapPassagesInIdOrder(
             absorbedMapId_in,
             absorbedContents_out.passages) !=
         SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS) ||
        (p_semanticsManager_in->getMapDetectedRoomsInIdOrder(
             absorbedMapId_in,
             absorbedContents_out.detectedRooms) !=
         SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS) ||
        (p_semanticsManager_in->getMapCandidateRoomsInIdOrder(
             absorbedMapId_in,
             absorbedContents_out.candidateRooms) !=
         SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS) ||
        (p_semanticsManager_in->getMapFloorsInIdOrder(
             absorbedMapId_in,
             absorbedContents_out.floors) !=
         SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: the semantics manager has no scene graph for map %lu; "
            "continuing without its semantic objects.",
            __func__,
            absorbedMapId_in);
    }
}

} /* namespace core */
} /* namespace vs_graphs */
