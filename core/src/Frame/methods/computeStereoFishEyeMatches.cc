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

FrameStatus Frame::computeStereoFishEyeMatches()
{
    // Speed it up by matching keypoints in the lapping area
    std::vector<cv::KeyPoint> stereoLeft(keyPoints.begin() + monoLeft,
                                         keyPoints.end());
    std::vector<cv::KeyPoint> stereoRight(keyPointsRight.begin() + monoRight,
                                          keyPointsRight.end());

    cv::Mat stereoDescriptorLeft =
        descriptors.rowRange(monoLeft, descriptors.rows);
    cv::Mat stereoDescriptorRight =
        descriptorsRight.rowRange(monoRight, descriptorsRight.rows);

    leftToRightMatches = std::vector<int>(leftKeyPointCount, -1);
    rightToLeftMatches = std::vector<int>(rightKeyPointCount, -1);
    depths             = std::vector<float>(leftKeyPointCount, -1.0f);
    uRight             = std::vector<float>(leftKeyPointCount, -1);
    stereoPoints3D     = std::vector<Eigen::Vector3f>(leftKeyPointCount);
    closeMapPointCount = 0;

    // Perform a brute force between Keypoint in the left and right image
    std::vector<std::vector<cv::DMatch>> matches;

    bfMatcher.knnMatch(stereoDescriptorLeft, stereoDescriptorRight, matches, 2);

    int matchCount        = 0;
    int descriptorMatches = 0;

    // Check matches using Lowe's ratio
    for (std::vector<std::vector<cv::DMatch>>::iterator matchIt =
             matches.begin();
         matchIt != matches.end();
         ++matchIt)
    {
        if ((*matchIt).size() >= 2 &&
            (*matchIt)[0].distance < (*matchIt)[1].distance * 0.7)
        {
            // For every good match, check parallax and reprojection error to
            // discard spurious matches
            Eigen::Vector3f p3D;
            descriptorMatches++;
            float
                sigma1 = levelSigmaSquared
                    [keyPoints[(*matchIt)[0].queryIdx + monoLeft].octave],
                sigma2 = levelSigmaSquared
                    [keyPointsRight[(*matchIt)[0].trainIdx + monoRight].octave];
            float depth{};
            if (static_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(
                    p_camera)
                    ->triangulateMatches(
                        p_camera2,
                        keyPoints[(*matchIt)[0].queryIdx + monoLeft],
                        keyPointsRight[(*matchIt)[0].trainIdx + monoRight],
                        rotationRlr,
                        translationTlr,
                        sigma1,
                        sigma2,
                        p3D,
                        depth) !=
                camera_models::kannalabrandt8::KannalaBrandt8Status::
                    KANNALA_BRANDT8_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: triangulateMatches returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (depth > 0.0001f)
            {
                leftToRightMatches[(*matchIt)[0].queryIdx + monoLeft] =
                    (*matchIt)[0].trainIdx + monoRight;
                rightToLeftMatches[(*matchIt)[0].trainIdx + monoRight] =
                    (*matchIt)[0].queryIdx + monoLeft;
                stereoPoints3D[(*matchIt)[0].queryIdx + monoLeft] = p3D;
                depths[(*matchIt)[0].queryIdx + monoLeft]         = depth;
                matchCount++;
            }
        }
    }

    return FrameStatus::FRAME_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
