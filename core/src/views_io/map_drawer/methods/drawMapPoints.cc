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
 * @file            drawMapPoints.cc
 *
 * @brief           Implements MapDrawer::drawMapPoints(), declared in
 *                  MapDrawer.h.
 */

#include "map_state/keyframe/keyframe.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "map_state/map_point/objects/MapPointStatusEnum.h"
#include "map_state/map_point/map_point.h"
#include "views_io/map_drawer/objects/MapDrawerClass.h"
#include <mutex>
#include <pangolin/pangolin.h>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace map_drawer
{

MapDrawerStatus MapDrawer::drawMapPoints() const
{
    map::Map *p_activeMap = nullptr;
    if (p_atlas->getCurrentMap(p_activeMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (!p_activeMap)
        return MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS;

    std::vector<map_point::MapPoint *> mapPoints{};
    if (p_activeMap->getAllMapPoints(mapPoints) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMapPoints returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<map_point::MapPoint *> vpRefMPs{};
    if (p_activeMap->getReferenceMapPoints(vpRefMPs) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getReferenceMapPoints returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    std::set<map_point::MapPoint *> referenceMapPoints(vpRefMPs.begin(), vpRefMPs.end());

    if (mapPoints.empty())
        return MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS;

    glPointSize(pointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 0.0);

    for (size_t mapPointIndex = 0, iend = mapPoints.size();
         mapPointIndex < iend;
         mapPointIndex++)
    {
        bool isBad2{};
        if (mapPoints[mapPointIndex]->isBad(isBad2) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (isBad2 || referenceMapPoints.count(mapPoints[mapPointIndex]))
            continue;
        Eigen::Matrix<float, 3, 1> position{};
        if (mapPoints[mapPointIndex]->getWorldPos(position) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        glVertex3f(position(0), position(1), position(2));
    }
    glEnd();

    glPointSize(pointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);

    for (std::set<map_point::MapPoint *>::iterator sit  = referenceMapPoints.begin(),
                                        send = referenceMapPoints.end();
         sit != send;
         sit++)
    {
        bool isBad3{};
        if ((*sit)->isBad(isBad3) != map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (isBad3)
            continue;
        Eigen::Matrix<float, 3, 1> position{};
        if ((*sit)->getWorldPos(position) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        glVertex3f(position(0), position(1), position(2));
    }

    glEnd();

    return MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS;
}

} /* namespace map_drawer */
} /* namespace core */
} /* namespace vs_graphs */
