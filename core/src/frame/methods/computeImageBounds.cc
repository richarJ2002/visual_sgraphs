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
 * @file            computeImageBounds.cc
 *
 * @brief           Implements Frame::computeImageBounds(), declared in
 * frame/objects/FrameClass.h.
 */

#include "frame/objects/FrameClass.h"

#include "camera_models/geometric_camera/geometric_camera.h"
#include "camera_models/kannala_brandt8/kannala_brandt8.h"
#include "camera_models/pinhole/pinhole.h"

#include "keyframe/keyframe.h"
#include "map_point/map_point.h"
#include "orb_extractor/orb_extractor.h"
#include "orb_matcher/orb_matcher.h"
#include "stereo_match_outlier_rejection/stereo_match_outlier_rejection.h"
#include "utils/converter/converter.h"

#include <opencv2/calib3d.hpp>
#include <thread>

namespace vs_graphs
{
namespace core
{
namespace frame
{

FrameStatus Frame::computeImageBounds(const cv::Mat &imageLeft_in)
{
    if (distortionCoefficients.at<float>(0) != 0.0)
    {
        cv::Mat matrix(4, 2, CV_32F);
        matrix.at<float>(0, 0) = 0.0;
        matrix.at<float>(0, 1) = 0.0;
        matrix.at<float>(1, 0) = imageLeft_in.cols;
        matrix.at<float>(1, 1) = 0.0;
        matrix.at<float>(2, 0) = 0.0;
        matrix.at<float>(2, 1) = imageLeft_in.rows;
        matrix.at<float>(3, 0) = imageLeft_in.cols;
        matrix.at<float>(3, 1) = imageLeft_in.rows;

        matrix = matrix.reshape(2);
        cv::undistortPoints(
            matrix,
            matrix,
            static_cast<camera_models::pinhole::Pinhole *>(p_camera)->toK(),
            distortionCoefficients,
            cv::Mat(),
            calibrationMatrix);
        matrix = matrix.reshape(1);

        /* Undistort corners */
        gridMinX = std::min(matrix.at<float>(0, 0), matrix.at<float>(2, 0));
        gridMaxX = std::max(matrix.at<float>(1, 0), matrix.at<float>(3, 0));
        gridMinY = std::min(matrix.at<float>(0, 1), matrix.at<float>(1, 1));
        gridMaxY = std::max(matrix.at<float>(2, 1), matrix.at<float>(3, 1));
    }
    else
    {
        gridMinX = 0.0f;
        gridMaxX = imageLeft_in.cols;
        gridMinY = 0.0f;
        gridMaxY = imageLeft_in.rows;
    }

    return FrameStatus::FRAME_STATUS_SUCCESS;
}

} /* namespace frame */
} /* namespace core */
} /* namespace vs_graphs */
