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
 * @file            undistortKeyPoints.cc
 *
 * @brief           Implements Frame::undistortKeyPoints(), declared in Frame.h.
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

#include <opencv2/calib3d.hpp>
#include <thread>

namespace vs_graphs
{
namespace core
{

FrameStatus Frame::undistortKeyPoints()
{
    if (distortionCoefficients.at<float>(0) == 0.0)
    {
        keyPointsUndistorted = keyPoints;
        return FrameStatus::FRAME_STATUS_SUCCESS;
    }

    // Fill matrix with points
    cv::Mat matrix(keyPointCount, 2, CV_32F);

    for (int keyPointIndex = 0; keyPointIndex < keyPointCount; keyPointIndex++)
    {
        matrix.at<float>(keyPointIndex, 0) = keyPoints[keyPointIndex].pt.x;
        matrix.at<float>(keyPointIndex, 1) = keyPoints[keyPointIndex].pt.y;
    }

    // Undistort points
    matrix = matrix.reshape(2);
    cv::undistortPoints(
        matrix,
        matrix,
        static_cast<camera_models::pinhole::Pinhole *>(p_camera)->toK(),
        distortionCoefficients,
        cv::Mat(),
        calibrationMatrix);
    matrix = matrix.reshape(1);

    // Fill undistorted keypoint vector
    keyPointsUndistorted.resize(keyPointCount);
    for (int keyPointIndex = 0; keyPointIndex < keyPointCount; keyPointIndex++)
    {
        cv::KeyPoint keyPoint = keyPoints[keyPointIndex];
        keyPoint.pt.x         = matrix.at<float>(keyPointIndex, 0);
        keyPoint.pt.y         = matrix.at<float>(keyPointIndex, 1);
        keyPointsUndistorted[keyPointIndex] = keyPoint;
    }

    return FrameStatus::FRAME_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
