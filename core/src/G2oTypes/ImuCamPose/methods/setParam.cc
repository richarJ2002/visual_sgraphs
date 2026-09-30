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
 * @file            setParam.cc
 *
 * @brief           Implements ImuCamPose::setParam(), declared in G2oTypes.h.
 */

#include "G2oTypes.h"
#include "ImuTypes.h"
#include "Utils/Converter/objects/Converter.h"

namespace vs_graphs
{
namespace core
{

ImuCamPoseStatus
    ImuCamPose::setParam(const std::vector<Eigen::Matrix3d> &Rcw_in,
                         const std::vector<Eigen::Vector3d> &tcw_in,
                         const std::vector<Eigen::Matrix3d> &Rbc_in,
                         const std::vector<Eigen::Vector3d> &tbc_in,
                         const double &baselineFocalProduct_in)
{
    Rbc                   = Rbc_in;
    tbc                   = tbc_in;
    Rcw                   = Rcw_in;
    tcw                   = tcw_in;
    const int cameraCount = Rbc.size();
    Rcb.resize(cameraCount);
    tcb.resize(cameraCount);

    for (std::size_t cameraIndex = 0; cameraIndex < tcb.size(); cameraIndex++)
    {
        Rcb[cameraIndex] = Rbc[cameraIndex].transpose();
        tcb[cameraIndex] = -Rcb[cameraIndex] * tbc[cameraIndex];
    }
    Rwb = Rcw[0].transpose() * Rcb[0];
    twb = Rcw[0].transpose() * (tcb[0] - tcw[0]);

    bf = baselineFocalProduct_in;

    return ImuCamPoseStatus::IMU_CAM_POSE_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
