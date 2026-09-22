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

#include <thread>

namespace vs_graphs
{
namespace core
{

void Frame::computeImageBounds(const cv::Mat &imLeft)
{
    if (distortionCoefficients.at<float>(0) != 0.0)
    {
        cv::Mat mat(4, 2, CV_32F);
        mat.at<float>(0, 0) = 0.0;
        mat.at<float>(0, 1) = 0.0;
        mat.at<float>(1, 0) = imLeft.cols;
        mat.at<float>(1, 1) = 0.0;
        mat.at<float>(2, 0) = 0.0;
        mat.at<float>(2, 1) = imLeft.rows;
        mat.at<float>(3, 0) = imLeft.cols;
        mat.at<float>(3, 1) = imLeft.rows;

        mat = mat.reshape(2);
        cv::undistortPoints(
            mat,
            mat,
            static_cast<camera_models::pinhole::Pinhole *>(p_camera)->toK(),
            distortionCoefficients,
            cv::Mat(),
            calibrationMatrix);
        mat = mat.reshape(1);

        // Undistort corners
        gridMinX = min(mat.at<float>(0, 0), mat.at<float>(2, 0));
        gridMaxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0));
        gridMinY = min(mat.at<float>(0, 1), mat.at<float>(1, 1));
        gridMaxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1));
    }
    else
    {
        gridMinX = 0.0f;
        gridMaxX = imLeft.cols;
        gridMinY = 0.0f;
        gridMaxY = imLeft.rows;
    }
}

} // namespace core
} // namespace vs_graphs
