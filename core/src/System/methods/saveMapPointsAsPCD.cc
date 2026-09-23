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

namespace vs_graphs
{
namespace core
{

bool System::saveMapPointsAsPCD(const string &filename)
{
    try
    {
        // make a pointcloud out of all map points
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        vector<MapPoint *> vpMPs = p_atlas->getCurrentMap()->getAllMapPoints();
        for (size_t i = 0; i < vpMPs.size(); i++)
        {
            MapPoint *pMP = vpMPs[i];
            if (pMP->isBad())
                continue;

            Eigen::Vector3d P3Dw = pMP->getWorldPos().cast<double>();
            pcl::PointXYZ   point;
            point.x = P3Dw.x();
            point.y = P3Dw.y();
            point.z = P3Dw.z();
            cloud->push_back(point);
        }

        // save the pointcloud
        pcl::io::savePCDFileBinary(filename + ".pcd", *cloud);

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return false;
    }
    catch (...)
    {
        std::cerr << "Unknows exeption" << std::endl;
        return false;
    }
}

} // namespace core
} // namespace vs_graphs
