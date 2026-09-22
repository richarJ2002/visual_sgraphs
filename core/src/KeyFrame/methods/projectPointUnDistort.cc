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

bool KeyFrame::projectPointUnDistort(MapPoint    *pMP,
                                     cv::Point2f &kp,
                                     float       &u,
                                     float       &v)
{

    // 3D in absolute coordinates
    Eigen::Vector3f P = pMP->getWorldPos();

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
    const float invz = 1.0f / PcZ;
    u                = fx * PcX * invz + cx;
    v                = fy * PcY * invz + cy;

    if (u < gridMinX || u > gridMaxX)
        return false;
    if (v < gridMinY || v > gridMaxY)
        return false;

    kp = cv::Point2f(u, v);

    return true;
}

} // namespace core
} // namespace vs_graphs
