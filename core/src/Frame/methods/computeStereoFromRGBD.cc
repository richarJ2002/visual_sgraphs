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
 * @file            computeStereoFromRGBD.cc
 *
 * @brief           Implements Frame::computeStereoFromRGBD(), declared in
 *                  Frame.h.
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

#include <thread>

namespace vs_graphs
{
namespace core
{

FrameStatus Frame::computeStereoFromRGBD(const cv::Mat &imageDepth_in)
{
    uRight = std::vector<float>(keyPointCount, -1);
    depths = std::vector<float>(keyPointCount, -1);

    for (int keyPointIndex = 0; keyPointIndex < keyPointCount; keyPointIndex++)
    {
        const cv::KeyPoint &keyPoint  = keyPoints[keyPointIndex];
        const cv::KeyPoint &keyPointU = keyPointsUndistorted[keyPointIndex];

        const float &v = keyPoint.pt.y;
        const float &u = keyPoint.pt.x;

        const float d = imageDepth_in.at<float>(v, u);

        if (d > 0)
        {
            depths[keyPointIndex] = d;
            uRight[keyPointIndex] = keyPointU.pt.x - mbf / d;
        }
    }

    return FrameStatus::FRAME_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
