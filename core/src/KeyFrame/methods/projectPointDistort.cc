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

bool KeyFrame::projectPointDistort(MapPoint    *p_mapPoint_in,
                                   cv::Point2f &keyPoint_out,
                                   float       &u_out,
                                   float       &v_out)
{

    // 3D in absolute coordinates
    Eigen::Vector3f P = p_mapPoint_in->getWorldPos();

    // 3D in camera coordinates
    Eigen::Vector3f Pc  = rotationRcw * P + poseTcw.translation();
    float          &PcX = Pc(0);
    float          &PcY = Pc(1);
    float          &PcZ = Pc(2);

    // Check positive depth
    if (PcZ < 0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // Project in image and check it is not outside
    float invz = 1.0f / PcZ;
    u_out      = fx * PcX * invz + cx;
    v_out      = fy * PcY * invz + cy;

    // cout << "c";

    if (u_out < gridMinX || u_out > gridMaxX)
        return false;
    if (v_out < gridMinY || v_out > gridMaxY)
        return false;

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

    float distortedU = distortedX * fx + cx;
    float distort    = distortedY * fy + cy;

    u_out = distortedU;
    v_out = distort;

    keyPoint_out = cv::Point2f(u_out, v_out);

    return true;
}

} // namespace core
} // namespace vs_graphs
