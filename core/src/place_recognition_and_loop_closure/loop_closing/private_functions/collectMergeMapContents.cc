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
 * @file            collectMergeMapContents.cc
 *
 * @brief           Implements collectMergeMapContents(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/scene_graph/SceneGraph/SceneGraph.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void collectMergeMapContents(
    atlas::Atlas                   *p_atlas_in,
    map::Map                       *p_mergeMap_in,
    loop_closing::MergeMapContents &mergeMapContents_out)
{
    mergeMapContents_out = loop_closing::MergeMapContents{};

    /* Copy all planes currently owned by the merge map */
    if (p_mergeMap_in->getAllPlanes(mergeMapContents_out.planes) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Copy all keyframes currently owned by the merge map */
    if (p_mergeMap_in->getAllKeyFrames(mergeMapContents_out.keyFrames) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /* Copy all map points currently owned by the merge map */
    if (p_mergeMap_in->getAllMapPoints(mergeMapContents_out.mapPoints) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMapPoints returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    scene_graph::SceneGraph *p_sceneGraph = nullptr;
    unsigned long            mergeMapId{};
    if ((p_atlas_in->getSceneGraph(p_sceneGraph) !=
         atlas::AtlasStatus::ATLAS_STATUS_SUCCESS) ||
        (p_mergeMap_in->getId(mergeMapId) !=
         map::MapStatus::MAP_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSceneGraph/getId returned a failure status "
                     "although they cannot fail; continuing as before.",
                     __func__);
    }

    /* Copy the passages, detected rooms, candidate (marker-based) rooms and
     * floors the scene graph lists for the merge map. */
    if ((p_sceneGraph->getMapPassagesInIdOrder(mergeMapId,
                                               mergeMapContents_out.passages) !=
         scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS) ||
        (p_sceneGraph->getMapDetectedRoomsInIdOrder(
             mergeMapId,
             mergeMapContents_out.detectedRooms) !=
         scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS) ||
        (p_sceneGraph->getMapCandidateRoomsInIdOrder(
             mergeMapId,
             mergeMapContents_out.markerBasedRooms) !=
         scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS) ||
        (p_sceneGraph->getMapFloorsInIdOrder(mergeMapId,
                                             mergeMapContents_out.floors) !=
         scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: the scene graph has no scene map for map %lu; "
                     "continuing without its semantic objects.",
                     __func__,
                     mergeMapId);
    }
}

} /* namespace core */
} /* namespace vs_graphs */
