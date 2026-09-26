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

/* NOTE: out-of-line to break the Frame<->KeyFrame/MapPoint include cycle. */

Frame::Frame(const cv::Mat                                   &imColor,
             const cv::Mat                                   &imGray,
             const cv::Mat                                   &imDepth,
             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr    &pointcloud,
             const double                                    &timeStamp,
             ORBextractor                                    *extractor,
             ORBVocabulary                                   *voc,
             cv::Mat                                         &K,
             cv::Mat                                         &distCoef,
             const float                                     &bf,
             const float                                     &thDepth,
             camera_models::geometriccamera::GeometricCamera *pCamera,
             Frame                                           *pPrevF,
             const IMU::Calib                                &ImuCalib,
             const std::vector<semantic::Marker *>            markers) :
    p_poseImuConstraint(nullptr),
    poseAvailable(false),
    velocityAvailable(false),
    p_orbVocabulary(voc),
    p_orbExtractorLeft(extractor),
    p_orbExtractorRight(static_cast<ORBextractor *>(nullptr)),
    timeStamp(timeStamp),
    calibrationMatrix(K.clone()),
    calibrationMatrixEigen(utils::converter::Converter::toMatrix3f(K)),
    distortionCoefficients(distCoef.clone()),
    mbf(bf),
    depthThreshold(thDepth),
    imuCalibration(ImuCalib),
    p_imuPreintegrated(nullptr),
    p_previousFrame(pPrevF),
    p_imuPreintegratedFrame(nullptr),
    p_referenceKeyFrame(static_cast<KeyFrame *>(nullptr)),
    isFrameSet(false),
    imuPreintegrated(false),
    p_camera(pCamera),
    p_camera2(nullptr)
{
    // Setting the color image for Semantic Segmentation
    colorImg = imColor.clone();

    // Frame ID
    mnId = nNextId++;

    // Get the scale level info from the ORB extractor
    scaleLevelCount      = p_orbExtractorLeft->getLevelCount();
    scaleFactor          = p_orbExtractorLeft->getScaleFactor();
    logScaleFactor       = log(scaleFactor);
    scaleFactors         = p_orbExtractorLeft->getScaleFactors();
    invScaleFactors      = p_orbExtractorLeft->getInverseScaleFactors();
    levelSigmaSquared    = p_orbExtractorLeft->getScaleSigmaSquares();
    invLevelSigmaSquared = p_orbExtractorLeft->getInverseScaleSigmaSquares();

    // ORB extraction
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartExtORB =
        std::chrono::steady_clock::now();
#endif
    extractOrbFeatures(0, imGray, 0, 0);
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB =
        std::chrono::steady_clock::now();

    orbExtractionTime =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndExtORB - time_StartExtORB)
            .count();
#endif

    N = keyPoints.size();
    if (keyPoints.empty())
        return;

    undistortKeyPoints();

    computeStereoFromRGBD(imDepth);

    // Initialize MapPoints
    mapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(nullptr));

    // Initialize MapMarkers
    mapMarkers = markers;

    // Initialize PointCloud
    pointClouds = pointcloud;

    projectedPoints.clear();
    matchedPoints.clear();

    outlierFlags = vector<bool>(N, false);

    // This is done only for the first Frame (or after a change in the
    // calibration)
    if (initialComputationsDone)
    {
        computeImageBounds(imGray);

        gridElementWidthInverse = static_cast<float>(FRAME_GRID_COLS) /
                                  static_cast<float>(gridMaxX - gridMinX);
        gridElementHeightInverse = static_cast<float>(FRAME_GRID_ROWS) /
                                   static_cast<float>(gridMaxY - gridMinY);

        fx    = K.at<float>(0, 0);
        fy    = K.at<float>(1, 1);
        cx    = K.at<float>(0, 2);
        cy    = K.at<float>(1, 2);
        invfx = 1.0f / fx;
        invfy = 1.0f / fy;

        initialComputationsDone = false;
    }

    mb = mbf / fx;

    if (pPrevF)
    {
        if (pPrevF->hasVelocity())
            setVelocity(pPrevF->getVelocity());
        else
            velocityVw.setZero();
    }

    // Set no stereo fisheye information
    Nleft              = -1;
    Nright             = -1;
    monoLeft           = -1;
    monoRight          = -1;
    leftToRightMatches = vector<int>(0);
    rightToLeftMatches = vector<int>(0);
    stereoPoints3D     = vector<Eigen::Vector3f>(0);

    assignFeaturesToGrid();
}

} // namespace core
} // namespace vs_graphs
