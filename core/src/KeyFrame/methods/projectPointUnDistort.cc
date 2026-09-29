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

KeyFrameStatus KeyFrame::projectPointUnDistort(MapPoint    *p_mapPoint_in,
                                               cv::Point2f &keyPoint_out,
                                               float       &u_out,
                                               float       &v_out,
                                               bool        &isProjected_out)
{

    // 3D in absolute coordinates
    Eigen::Vector3f P{};
    if (p_mapPoint_in->getWorldPos(P) !=
        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWorldPos returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    // 3D in camera coordinates
    Eigen::Vector3f Pc  = rotationRcw * P + poseTcw.translation();
    float          &PcX = Pc(0);
    float          &PcY = Pc(1);
    float          &PcZ = Pc(2);

    // Check positive depth
    if (PcZ < 0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        isProjected_out = false;
        return KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS;
    }

    // Project in image and check it is not outside
    const float invz = 1.0f / PcZ;
    u_out            = fx * PcX * invz + cx;
    v_out            = fy * PcY * invz + cy;

    if (u_out < gridMinX || u_out > gridMaxX)
    {
        isProjected_out = false;
        return KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS;
    }
    if (v_out < gridMinY || v_out > gridMaxY)
    {
        isProjected_out = false;
        return KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS;
    }

    keyPoint_out = cv::Point2f(u_out, v_out);

    isProjected_out = true;
    return KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
