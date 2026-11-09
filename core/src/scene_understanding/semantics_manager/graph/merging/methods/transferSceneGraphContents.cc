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
 * @file            transferSceneGraphContents.cc
 *
 * @brief           Implements SemanticsManager::transferSceneGraphContents(),
 *                  declared in
 * semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <rclcpp/logging.hpp>

#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/semantic/semantic_verify/objects/SemanticVerifyStatusEnum.h"
#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"
#include "scene_understanding/semantics_manager/graph/merging/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::transferSceneGraphContents(
    map::Map                   *p_survivingMap_inout,
    map::Map                   *p_absorbedMap_inout,
    bool                        skipBadCandidateRooms_in,
    AbsorbedSceneGraphContents &absorbedContents_out)
{
    absorbedContents_out = AbsorbedSceneGraphContents{};
    unsigned long survivingMapId{};
    unsigned long absorbedMapId{};
    if ((p_survivingMap_inout->getId(survivingMapId) !=
         map::MapStatus::MAP_STATUS_SUCCESS) ||
        (p_absorbedMap_inout->getId(absorbedMapId) !=
         map::MapStatus::MAP_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Both graphs are edited in place, locked exclusive in ascending map-id
     * order. */
    GraphLockScope graphScope{};
    if (lockGraphs({survivingMapId, absorbedMapId},
                   GraphLockMode::GRAPH_LOCK_MODE_EXCLUSIVE,
                   graphScope) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND;
    }

    /* Passages and floors are paired by geometry with the merge thresholds. */
    semantic::SemanticVerifyConfig configuration{};
    if (semantic::SemanticVerify::configFromSystemParams(configuration) !=
        semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: configFromSystemParams returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    collectAbsorbedSceneGraph(this, absorbedMapId, absorbedContents_out);
    transferPassages(this,
                     p_survivingMap_inout,
                     p_absorbedMap_inout,
                     absorbedContents_out.passages,
                     absorbedContents_out.detectedRooms,
                     absorbedContents_out.candidateRooms,
                     configuration);
    transferRooms(this,
                  p_survivingMap_inout,
                  p_absorbedMap_inout,
                  absorbedContents_out.detectedRooms,
                  absorbedContents_out.candidateRooms,
                  skipBadCandidateRooms_in);
    transferFloors(this,
                   p_survivingMap_inout,
                   p_absorbedMap_inout,
                   absorbedContents_out.floors,
                   configuration);
    indexAbsorbedRoomWalls(this, p_survivingMap_inout);
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
