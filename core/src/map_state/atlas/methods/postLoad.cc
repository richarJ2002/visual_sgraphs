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
 * @file            postLoad.cc
 *
 * @brief           Implements Atlas::postLoad(), declared in
 * atlas/objects/AtlasClass.h.
 */

#include "map_state/atlas/objects/AtlasClass.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "map_state/map_point/map_point.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"
#include "sensor_models/camera_models/geometric_camera/geometric_camera.h"
#include "sensor_models/camera_models/geometric_camera/objects/GeometricCameraStatusEnum.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace atlas
{

AtlasStatus Atlas::postLoad()
{
    std::map<unsigned int, camera_models::geometric_camera::GeometricCamera *>
        camerasById;
    for (camera_models::geometric_camera::GeometricCamera *p_camera : cameras)
    {
        unsigned int cameraId{};
        if (p_camera->getId(cameraId) !=
            camera_models::geometric_camera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        camerasById[cameraId] = p_camera;
    }

    /* The loaded maps replace maps: the old scene maps go, and each loaded
     * map gets an empty scene map once it has re-linked below (while no
     * semantics manager is attached, Atlas::setSemanticsManager() registers
     * them). Rooms,
     * floors and passages are not part of a saved map, so there is nothing
     * to restore into it. */
    for (map::Map *p_map : maps)
    {
        unsigned long replacedMapId{};
        if ((p_map->getId(replacedMapId) !=
             map::MapStatus::MAP_STATUS_SUCCESS) ||
            ((p_semanticsManager != nullptr) &&
             (p_semanticsManager->eraseSceneMap(replacedMapId) !=
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)))
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId/eraseSceneMap returned a failure status "
                         "although they cannot fail; continuing as before.",
                         __func__);
        }
    }
    maps.clear();
    unsigned long int keyFrameCount = 0, mapPointCount = 0;
    for (map::Map *p_map : backupMaps)
    {
        maps.insert(p_map);
        if (p_map->postLoad(p_keyFrameDatabase, p_orbVocabulary, camerasById) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: postLoad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if ((p_semanticsManager != nullptr) &&
            (p_semanticsManager->addSceneMap(*p_map) !=
             SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS))
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addSceneMap returned a failure "
                         "status although it cannot fail; continuing as "
                         "before.",
                         __func__);
        }
        std::vector<keyframe::KeyFrame *> mapAllKeyFrames{};
        if (p_map->getAllKeyFrames(mapAllKeyFrames) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        keyFrameCount += mapAllKeyFrames.size();
        std::vector<map_point::MapPoint *> mapAllMapPoints{};
        if (p_map->getAllMapPoints(mapAllMapPoints) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllMapPoints returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        mapPointCount += mapAllMapPoints.size();
    }
    backupMaps.clear();

    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} /* namespace atlas */
} /* namespace core */
} /* namespace vs_graphs */
