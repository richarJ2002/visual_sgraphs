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
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "System.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SystemStatus System::saveMapPointsAsPCD(const string &filename_in,
                                        bool         &isSaved_out)
{
    try
    {
        // make a pointcloud out of all map points
        pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<MapPoint *> mapPoints{};
        Map                    *p_atlasCurrentMap = nullptr;
        if (p_atlas->getCurrentMap(p_atlasCurrentMap) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_atlasCurrentMap->getAllMapPoints(mapPoints) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllMapPoints returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (size_t mapPointIndex = 0; mapPointIndex < mapPoints.size();
             mapPointIndex++)
        {
            MapPoint *p_mapPoint = mapPoints[mapPointIndex];
            bool      mapPointIsBad{};
            if (p_mapPoint->isBad(mapPointIsBad) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (mapPointIsBad)
                continue;

            Eigen::Vector3f mapPointWorldPos{};
            if (p_mapPoint->getWorldPos(mapPointWorldPos) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector3d P3Dw = mapPointWorldPos.cast<double>();
            pcl::PointXYZ   point;
            point.x = P3Dw.x();
            point.y = P3Dw.y();
            point.z = P3Dw.z();
            p_cloud->push_back(point);
        }

        // save the pointcloud
        pcl::io::savePCDFileBinary(filename_in + ".pcd", *p_cloud);

        isSaved_out = true;
        return SystemStatus::SYSTEM_STATUS_SUCCESS;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        isSaved_out = false;
        return SystemStatus::SYSTEM_STATUS_SUCCESS;
    }
    catch (...)
    {
        std::cerr << "Unknows exeption" << std::endl;
        isSaved_out = false;
        return SystemStatus::SYSTEM_STATUS_SUCCESS;
    }
}

} // namespace core
} // namespace vs_graphs
