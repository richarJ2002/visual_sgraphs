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

void Frame::computeStereoFishEyeMatches()
{
    // Speed it up by matching keypoints in the lapping area
    vector<cv::KeyPoint> stereoLeft(keyPoints.begin() + monoLeft,
                                    keyPoints.end());
    vector<cv::KeyPoint> stereoRight(keyPointsRight.begin() + monoRight,
                                     keyPointsRight.end());

    cv::Mat stereoDescLeft = descriptors.rowRange(monoLeft, descriptors.rows);
    cv::Mat stereoDescRight =
        descriptorsRight.rowRange(monoRight, descriptorsRight.rows);

    leftToRightMatches = vector<int>(Nleft, -1);
    rightToLeftMatches = vector<int>(Nright, -1);
    depths             = vector<float>(Nleft, -1.0f);
    uRight             = vector<float>(Nleft, -1);
    stereoPoints3D     = vector<Eigen::Vector3f>(Nleft);
    closeMapPointCount = 0;

    // Perform a brute force between Keypoint in the left and right image
    vector<vector<cv::DMatch>> matches;

    bfMatcher.knnMatch(stereoDescLeft, stereoDescRight, matches, 2);

    int nMatches    = 0;
    int descMatches = 0;

    // Check matches using Lowe's ratio
    for (vector<vector<cv::DMatch>>::iterator it = matches.begin();
         it != matches.end();
         ++it)
    {
        if ((*it).size() >= 2 && (*it)[0].distance < (*it)[1].distance * 0.7)
        {
            // For every good match, check parallax and reprojection error to
            // discard spurious matches
            Eigen::Vector3f p3D;
            descMatches++;
            float sigma1 =
                      levelSigmaSquared[keyPoints[(*it)[0].queryIdx + monoLeft]
                                            .octave],
                  sigma2 = levelSigmaSquared
                      [keyPointsRight[(*it)[0].trainIdx + monoRight].octave];
            float depth =
                static_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(
                    p_camera)
                    ->triangulateMatches(
                        p_camera2,
                        keyPoints[(*it)[0].queryIdx + monoLeft],
                        keyPointsRight[(*it)[0].trainIdx + monoRight],
                        rotationRlr,
                        translationTlr,
                        sigma1,
                        sigma2,
                        p3D);
            if (depth > 0.0001f)
            {
                leftToRightMatches[(*it)[0].queryIdx + monoLeft] =
                    (*it)[0].trainIdx + monoRight;
                rightToLeftMatches[(*it)[0].trainIdx + monoRight] =
                    (*it)[0].queryIdx + monoLeft;
                stereoPoints3D[(*it)[0].queryIdx + monoLeft] = p3D;
                depths[(*it)[0].queryIdx + monoLeft]         = depth;
                nMatches++;
            }
        }
    }
}

} // namespace core
} // namespace vs_graphs
