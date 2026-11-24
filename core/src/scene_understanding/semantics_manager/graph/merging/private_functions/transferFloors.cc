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
 * @file            transferFloors.cc
 *
 * @brief           Implements transferFloors(), declared in
 *                  graph/merging/private_functions.h.
 */

#include "scene_understanding/semantics_manager/graph/merging/private_functions/private_functions.h"

#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"

#include <optional>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void transferFloors(SemanticsManager *p_semanticsManager_inout,
                    map::Map         *p_currentMap_inout,
                    map::Map         *p_mergeMap_inout,
                    const std::vector<semantic::Floor *> &floors_in,
                    const semantic::SemanticVerifyConfig &configuration_in)
{
    unsigned long currentMapId{};
    unsigned long mergeMapId{};
    if ((p_currentMap_inout->getId(currentMapId) !=
         map::MapStatus::MAP_STATUS_SUCCESS) ||
        (p_mergeMap_inout->getId(mergeMapId) !=
         map::MapStatus::MAP_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    for (semantic::Floor *p_floor : floors_in)
    {
        if (p_floor == nullptr)
        {
            continue;
        }

        semantic::Floor               *p_retainedFloor = nullptr;
        std::vector<semantic::Floor *> currentMapAllFloors{};
        if (p_semanticsManager_inout->getMapFloorsInIdOrder(
                currentMapId,
                currentMapAllFloors) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: the semantics manager has no scene graph for map %lu; "
                "continuing without its floors.",
                __func__,
                currentMapId);
        }
        /* A retained floor is a coplanar one, never one with the same id. */
        std::optional<semantic::Floor::PlaneIdentity> floorIdentity{};
        if (p_floor->getPlaneIdentity(floorIdentity) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPlaneIdentity returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (semantic::Floor *p_existingFloor : currentMapAllFloors)
        {
            if (p_existingFloor == nullptr || !floorIdentity.has_value())
            {
                continue;
            }
            std::optional<semantic::Floor::PlaneIdentity> existingIdentity{};
            if (p_existingFloor->getPlaneIdentity(existingIdentity) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPlaneIdentity returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            double normalAngle_deg{};
            double planeOffset_m{};
            bool   isCoplanar{};
            if (existingIdentity.has_value() &&
                semantic::Floor::planeIdentitiesMatch(
                    *existingIdentity,
                    *floorIdentity,
                    configuration_in.coplanarAngle_deg,
                    configuration_in.floorMatchTolerance_m,
                    normalAngle_deg,
                    planeOffset_m,
                    isCoplanar) != semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: planeIdentitiesMatch returned a failure "
                             "status although it cannot fail; continuing as "
                             "before.",
                             __func__);
            }
            if (existingIdentity.has_value() && isCoplanar)
            {
                p_retainedFloor = p_existingFloor;
                break;
            }
        }
        if (p_retainedFloor != nullptr)
        {
            /* The duplicate storey leaves the tree and is freed by a later
             * sweep. */
            if (p_semanticsManager_inout->eraseFloor(mergeMapId, p_floor) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseFloor found no scene graph for map %lu; "
                             "continuing as before.",
                             __func__,
                             mergeMapId);
            }
            mergeFloorEvidenceAndRooms(p_retainedFloor, p_floor);
            continue;
        }
        if (p_floor->setMap(p_currentMap_inout) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_semanticsManager_inout->moveFloor(mergeMapId,
                                                currentMapId,
                                                p_floor) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: moveFloor found no scene graph for maps %lu and "
                         "%lu or no such storey; continuing as before.",
                         __func__,
                         mergeMapId,
                         currentMapId);
        }
    }

    collapseMergedFloors(p_semanticsManager_inout, p_currentMap_inout);
}

} /* namespace core */
} /* namespace vs_graphs */
