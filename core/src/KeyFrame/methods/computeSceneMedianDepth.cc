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

#include "KeyFrame.h"

#include "ImuTypes.h"
#include "MapPoint.h"
#include "Utils/Converter/objects/Converter.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

KeyFrameStatus KeyFrame::computeSceneMedianDepth(const int q_in,
                                                 float    &sceneMedianDepth_out)
{
    if (keyPointCount == 0)
    {
        sceneMedianDepth_out = -1.0;
        return KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS;
    }

    vector<MapPoint *> keyFrameMapPoints;
    Eigen::Matrix3f    Rcw;
    Eigen::Vector3f    tcw;
    {
        unique_lock<mutex> lock(featuresMutex);
        unique_lock<mutex> lock2(poseMutex);
        keyFrameMapPoints = mapPoints;
        tcw               = poseTcw.translation();
        Rcw               = rotationRcw;
    }

    vector<float> mapPointDepths;
    mapPointDepths.reserve(keyPointCount);
    Eigen::Matrix<float, 1, 3> Rcw2 = Rcw.row(2);
    float                      zcw  = tcw(2);
    for (int keyPointIndex = 0; keyPointIndex < keyPointCount; keyPointIndex++)
    {
        if (mapPoints[keyPointIndex])
        {
            MapPoint       *p_mapPoint = mapPoints[keyPointIndex];
            Eigen::Vector3f x3Dw{};
            if (p_mapPoint->getWorldPos(x3Dw) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            float z = Rcw2.dot(x3Dw) + zcw;
            mapPointDepths.push_back(z);
        }
    }

    sort(mapPointDepths.begin(), mapPointDepths.end());

    sceneMedianDepth_out = mapPointDepths[(mapPointDepths.size() - 1) / q_in];
    return KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
