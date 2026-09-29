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
#include "Utils/Converter/objects/Converter.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

KeyFrameStatus KeyFrame::unprojectStereo(int              index_in,
                                         Eigen::Vector3f &x3D_out,
                                         bool            &isUnprojected_out)
{
    const float z = depths[index_in];
    if (z > 0)
    {
        const float     u = keyPoints[index_in].pt.x;
        const float     v = keyPoints[index_in].pt.y;
        const float     x = (u - cx) * z * invfx;
        const float     y = (v - cy) * z * invfy;
        Eigen::Vector3f x3Dc(x, y, z);

        unique_lock<mutex> lock(poseMutex);
        x3D_out           = rotationRwc * x3Dc + twc.translation();
        isUnprojected_out = true;
        return KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS;
    }
    else
    {
        isUnprojected_out = false;
        return KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS;
    }
}

} // namespace core
} // namespace vs_graphs
