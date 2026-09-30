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

#include "Frame.h"

#include "CameraModels/GeometricCamera/objects/GeometricCamera.h"
#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"
#include "CameraModels/Pinhole/objects/Pinhole.h"
#include "G2oTypes.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "StereoMatchOutlierRejection.h"
#include "Utils/Converter/objects/Converter.h"

#include <rclcpp/logging.hpp>
#include <thread>

namespace vs_graphs
{
namespace core
{

FrameStatus Frame::projectPointDistort(MapPoint    *p_mapPoint_in,
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
    const Eigen::Vector3f Pc  = rotationRcw * P + translationTcw;
    const float          &PcX = Pc(0);
    const float          &PcY = Pc(1);
    const float          &PcZ = Pc(2);

    // Check positive depth
    if (PcZ < 0.0f)
    {
        std::cout << "Negative depth: " << PcZ << std::endl;
        isProjected_out = false;
        return FrameStatus::FRAME_STATUS_SUCCESS;
    }

    // Project in image and check it is not outside
    const float invz = 1.0f / PcZ;
    u_out            = fx * PcX * invz + cx;
    v_out            = fy * PcY * invz + cy;

    if (u_out < gridMinX || u_out > gridMaxX)
    {
        isProjected_out = false;
        return FrameStatus::FRAME_STATUS_SUCCESS;
    }
    if (v_out < gridMinY || v_out > gridMaxY)
    {
        isProjected_out = false;
        return FrameStatus::FRAME_STATUS_SUCCESS;
    }

    float distortedU, distort;

    float x  = (u_out - cx) * invfx;
    float y  = (v_out - cy) * invfy;
    float r2 = x * x + y * y;
    float k1 = distortionCoefficients.at<float>(0);
    float k2 = distortionCoefficients.at<float>(1);
    float p1 = distortionCoefficients.at<float>(2);
    float p2 = distortionCoefficients.at<float>(3);
    float k3 = 0;
    if (distortionCoefficients.total() == 5)
    {
        k3 = distortionCoefficients.at<float>(4);
    }

    // Radial distorsion
    float distortedX = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
    float distortedY = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

    // Tangential distorsion
    distortedX = distortedX + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
    distortedY = distortedY + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

    distortedU = distortedX * fx + cx;
    distort    = distortedY * fy + cy;

    u_out = distortedU;
    v_out = distort;

    keyPoint_out = cv::Point2f(u_out, v_out);

    isProjected_out = true;
    return FrameStatus::FRAME_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
