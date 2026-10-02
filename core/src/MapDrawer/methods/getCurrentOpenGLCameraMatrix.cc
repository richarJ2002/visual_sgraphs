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
 * @file            getCurrentOpenGLCameraMatrix.cc
 *
 * @brief           Implements MapDrawer::getCurrentOpenGLCameraMatrix(),
 *                  declared in MapDrawer.h.
 */

#include "KeyFrame.h"
#include "MapDrawer.h"
#include "MapPoint.h"
#include <mutex>
#include <pangolin/pangolin.h>

namespace vs_graphs
{
namespace core
{

MapDrawerStatus
    MapDrawer::getCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M_in,
                                            pangolin::OpenGlMatrix &MOw_inout)
{
    Eigen::Matrix4f cameraPose_cameraToWorld;
    {
        std::unique_lock<std::mutex> lock(cameraMutex);
        cameraPose_cameraToWorld = cameraPose.matrix();
    }

    for (int i = 0; i < 4; i++)
    {
        M_in.m[4 * i]     = cameraPose_cameraToWorld(0, i);
        M_in.m[4 * i + 1] = cameraPose_cameraToWorld(1, i);
        M_in.m[4 * i + 2] = cameraPose_cameraToWorld(2, i);
        M_in.m[4 * i + 3] = cameraPose_cameraToWorld(3, i);
    }

    MOw_inout.SetIdentity();
    MOw_inout.m[12] = cameraPose_cameraToWorld(0, 3);
    MOw_inout.m[13] = cameraPose_cameraToWorld(1, 3);
    MOw_inout.m[14] = cameraPose_cameraToWorld(2, 3);

    return MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
