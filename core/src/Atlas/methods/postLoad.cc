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
 * @brief           Implements Atlas::postLoad(), declared in Atlas.h.
 */

#include "Atlas.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

AtlasStatus Atlas::postLoad()
{
    std::map<unsigned int, camera_models::geometriccamera::GeometricCamera *>
        camerasById;
    for (camera_models::geometriccamera::GeometricCamera *p_camera : cameras)
    {
        unsigned int cameraId{};
        if (p_camera->getId(cameraId) !=
            camera_models::geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        camerasById[cameraId] = p_camera;
    }

    maps.clear();
    unsigned long int keyFrameCount = 0, mapPointCount = 0;
    for (Map *p_map : backupMaps)
    {
        maps.insert(p_map);
        if (p_map->postLoad(p_keyFrameDatabase, p_orbVocabulary, camerasById) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: postLoad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<KeyFrame *> mapAllKeyFrames{};
        if (p_map->getAllKeyFrames(mapAllKeyFrames) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        keyFrameCount += mapAllKeyFrames.size();
        std::vector<MapPoint *> mapAllMapPoints{};
        if (p_map->getAllMapPoints(mapAllMapPoints) !=
            MapStatus::MAP_STATUS_SUCCESS)
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

} // namespace core
} // namespace vs_graphs
