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
 * @file            globalBundleAdjustment.cc
 *
 * @brief           Implements Optimizer::globalBundleAdjustment(), declared in
 *                  Optimizer.h.
 */

#include "map_state/keyframe/keyframe.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "map_state/map_point/map_point.h"
#include "mapping/geometric/Plane/Plane.h"
#include "optimization/optimizer/objects/OptimizerClass.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"
#include <map>
#include <rclcpp/logging.hpp>

#include "execution/profiling/profiling.h"

namespace vs_graphs
{
namespace core
{
namespace optimizer
{

OptimizerStatus Optimizer::globalBundleAdjustment(
    map::Map               *p_map_in,
    SemanticsManager       *p_semanticsManager_in,
    int                     iterationCount_in,
    bool                   *p_stopFlag_inout,
    const unsigned long     loopKeyFrameId_in,
    const bool              useRobustKernel_in,
    const std::atomic_bool *p_stopRequested_in)
{
    PROFILE_FUNCTION();
    std::vector<RoomSnapshot> allRooms{};
    unsigned long             mapId{};
    /* The rooms are copied out as plain data under the shared graph lock, so
     * the long optimisation below holds no lock and a cycle is never delayed.
     */
    if ((p_map_in->getId(mapId) != map::MapStatus::MAP_STATUS_SUCCESS) ||
        (p_semanticsManager_in == nullptr) ||
        (p_semanticsManager_in->getMapRoomSnapshots(
             mapId,
             GraphLockMode::GRAPH_LOCK_MODE_SHARED,
             allRooms) !=
         SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: the semantics manager has no scene graph for this map; "
            "continuing without rooms.",
            __func__);
    }
    std::vector<vs_graphs::core::geometric::Plane *> allPlanes{};
    if (p_map_in->getAllPlanes(allPlanes) != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<vs_graphs::core::map_point::MapPoint *> allMapPoints{};
    if (p_map_in->getAllMapPoints(allMapPoints) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMapPoints returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<vs_graphs::core::keyframe::KeyFrame *> allKeyFrames{};
    if (p_map_in->getAllKeyFrames(allKeyFrames) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    std::map<int, Eigen::Vector3d> optimizedRoomCentroids{};
    if (bundleAdjustment(allKeyFrames,
                         allMapPoints,
                         allPlanes,
                         allRooms,
                         optimizedRoomCentroids,
                         iterationCount_in,
                         p_stopFlag_inout,
                         loopKeyFrameId_in,
                         useRobustKernel_in,
                         p_stopRequested_in) !=
        OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: bundleAdjustment returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    for (const std::pair<const int, Eigen::Vector3d> &roomCentroidEntry :
         optimizedRoomCentroids)
    {
        if (p_semanticsManager_in->setRoomCentroid(mapId,
                                                   roomCentroidEntry.first,
                                                   roomCentroidEntry.second) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setRoomCentroid found no scene graph for the "
                         "map; continuing as before.",
                         __func__);
        }
    }

    return OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
}

} /* namespace optimizer */
} /* namespace core */
} /* namespace vs_graphs */
