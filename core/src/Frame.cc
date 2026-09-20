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
 * @file         Frame.cc
 *
 * @brief        Implements Frame declared in Frame.h.
 */

#include "Frame.h"

#include "Converter.h"
#include "G2oTypes.h"
#include "GeometricCamera.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "StereoMatchOutlierRejection.h"

#include <include/CameraModels/KannalaBrandt8.h>
#include <include/CameraModels/Pinhole.h>
#include <thread>

namespace vs_graphs
{
namespace core
{

long unsigned int Frame::nNextId                 = 0;
bool              Frame::initialComputationsDone = true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::gridMinX, Frame::gridMinY, Frame::gridMaxX, Frame::gridMaxY;
float Frame::gridElementWidthInverse, Frame::gridElementHeightInverse;

// For stereo fisheye matching
cv::BFMatcher Frame::bfMatcher = cv::BFMatcher(cv::NORM_HAMMING);

Frame::Frame() :
    p_poseImuConstraint(nullptr),
    p_imuPreintegrated(nullptr),
    p_previousFrame(nullptr),
    p_imuPreintegratedFrame(nullptr),
    p_referenceKeyFrame(static_cast<KeyFrame *>(nullptr)),
    isFrameSet(false),
    imuPreintegrated(false),
    poseAvailable(false),
    velocityAvailable(false)
{
#ifdef REGISTER_TIMES
    stereoMatchTime   = 0;
    orbExtractionTime = 0;
#endif
}

// Copy Constructor
Frame::Frame(const Frame &frame) :
    p_poseImuConstraint(frame.p_poseImuConstraint),
    p_orbVocabulary(frame.p_orbVocabulary),
    p_orbExtractorLeft(frame.p_orbExtractorLeft),
    p_orbExtractorRight(frame.p_orbExtractorRight),
    timeStamp(frame.timeStamp),
    calibrationMatrix(frame.calibrationMatrix.clone()),
    calibrationMatrixEigen(Converter::toMatrix3f(frame.calibrationMatrix)),
    distortionCoefficients(frame.distortionCoefficients.clone()),
    mbf(frame.mbf),
    mb(frame.mb),
    depthThreshold(frame.depthThreshold),
    N(frame.N),
    keyPoints(frame.keyPoints),
    keyPointsRight(frame.keyPointsRight),
    keyPointsUndistorted(frame.keyPointsUndistorted),
    uRight(frame.uRight),
    depths(frame.depths),
    bowVector(frame.bowVector),
    featureVector(frame.featureVector),
    descriptors(frame.descriptors.clone()),
    descriptorsRight(frame.descriptorsRight.clone()),
    mapPoints(frame.mapPoints),
    outlierFlags(frame.outlierFlags),
    imuCalibration(frame.imuCalibration),
    closeMapPointCount(frame.closeMapPointCount),
    p_imuPreintegrated(frame.p_imuPreintegrated),
    p_imuPreintegratedFrame(frame.p_imuPreintegratedFrame),
    imuBias(frame.imuBias),
    mnId(frame.mnId),
    p_referenceKeyFrame(frame.p_referenceKeyFrame),
    scaleLevelCount(frame.scaleLevelCount),
    scaleFactor(frame.scaleFactor),
    logScaleFactor(frame.logScaleFactor),
    scaleFactors(frame.scaleFactors),
    invScaleFactors(frame.invScaleFactors),
    fileName(frame.fileName),
    datasetId(frame.datasetId),
    levelSigmaSquared(frame.levelSigmaSquared),
    invLevelSigmaSquared(frame.invLevelSigmaSquared),
    p_previousFrame(frame.p_previousFrame),
    p_lastKeyFrame(frame.p_lastKeyFrame),
    isFrameSet(frame.isFrameSet),
    imuPreintegrated(frame.imuPreintegrated),
    p_imuMutex(frame.p_imuMutex),
    p_camera(frame.p_camera),
    p_camera2(frame.p_camera2),
    Nleft(frame.Nleft),
    Nright(frame.Nright),
    monoLeft(frame.monoLeft),
    monoRight(frame.monoRight),
    leftToRightMatches(frame.leftToRightMatches),
    rightToLeftMatches(frame.rightToLeftMatches),
    stereoPoints3D(frame.stereoPoints3D),
    poseTlr(frame.poseTlr),
    rotationRlr(frame.rotationRlr),
    translationTlr(frame.translationTlr),
    poseTrl(frame.poseTrl),
    poseTcw(frame.poseTcw),
    poseAvailable(false),
    velocityAvailable(false)
{
    for (int i = 0; i < FRAME_GRID_COLS; i++)
        for (int j = 0; j < FRAME_GRID_ROWS; j++)
        {
            grid[i][j] = frame.grid[i][j];
            if (frame.Nleft > 0)
            {
                gridRight[i][j] = frame.gridRight[i][j];
            }
        }

    if (frame.poseAvailable)
        setPose(frame.getPose());

    if (frame.hasVelocity())
    {
        setVelocity(frame.getVelocity());
    }

    projectedPoints = frame.projectedPoints;
    matchedPoints   = frame.matchedPoints;

#ifdef REGISTER_TIMES
    stereoMatchTime   = frame.stereoMatchTime;
    orbExtractionTime = frame.orbExtractionTime;
#endif
}

// Stereo Frames Processing #1
Frame::Frame(const cv::Mat                        &imColor,
             const cv::Mat                        &imLeft,
             const cv::Mat                        &imRight,
             const double                         &timeStamp,
             ORBextractor                         *extractorLeft,
             ORBextractor                         *extractorRight,
             ORBVocabulary                        *voc,
             cv::Mat                              &K,
             cv::Mat                              &distCoef,
             const float                          &bf,
             const float                          &thDepth,
             camera_models::GeometricCamera       *pCamera,
             Frame                                *pPrevF,
             const IMU::Calib                     &ImuCalib,
             const std::vector<semantic::Marker *> markers) :
    p_poseImuConstraint(nullptr),
    p_orbVocabulary(voc),
    p_orbExtractorLeft(extractorLeft),
    p_orbExtractorRight(extractorRight),
    timeStamp(timeStamp),
    calibrationMatrix(K.clone()),
    calibrationMatrixEigen(Converter::toMatrix3f(K)),
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
    p_camera2(nullptr),
    poseAvailable(false),
    velocityAvailable(false)
{
    // Setting the color image for Semantic Segmentation
    colorImg = imColor.clone();

    // Frame ID
    mnId = nNextId++;

    // Scale Level Info
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
    thread threadLeft(&Frame::extractOrbFeatures, this, 0, imLeft, 0, 0);
    thread threadRight(&Frame::extractOrbFeatures, this, 1, imRight, 0, 0);
    threadLeft.join();
    threadRight.join();
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

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartStereoMatches =
        std::chrono::steady_clock::now();
#endif
    computeStereoMatches();
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndStereoMatches =
        std::chrono::steady_clock::now();

    stereoMatchTime =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndStereoMatches - time_StartStereoMatches)
            .count();
#endif

    // Initialize MapPoints
    mapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(nullptr));

    // Initialize MapMarkers
    mapMarkers = markers;

    projectedPoints.clear();
    matchedPoints.clear();

    outlierFlags = vector<bool>(N, false);

    // This is done only for the first Frame (or after a change in the
    // calibration)
    if (initialComputationsDone)
    {
        computeImageBounds(imLeft);

        gridElementWidthInverse =
            static_cast<float>(FRAME_GRID_COLS) / (gridMaxX - gridMinX);
        gridElementHeightInverse =
            static_cast<float>(FRAME_GRID_ROWS) / (gridMaxY - gridMinY);

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
    }
    else
    {
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

// Stereo Frames Processing #2
Frame::Frame(const cv::Mat                        &imColor,
             const cv::Mat                        &imLeft,
             const cv::Mat                        &imRight,
             const double                         &timeStamp,
             ORBextractor                         *extractorLeft,
             ORBextractor                         *extractorRight,
             ORBVocabulary                        *voc,
             cv::Mat                              &K,
             cv::Mat                              &distCoef,
             const float                          &bf,
             const float                          &thDepth,
             camera_models::GeometricCamera       *pCamera,
             camera_models::GeometricCamera       *pCamera2,
             Sophus::SE3f                         &Tlr,
             Frame                                *pPrevF,
             const IMU::Calib                     &ImuCalib,
             const std::vector<semantic::Marker *> markers) :
    p_poseImuConstraint(nullptr),
    p_orbVocabulary(voc),
    p_orbExtractorLeft(extractorLeft),
    p_orbExtractorRight(extractorRight),
    timeStamp(timeStamp),
    calibrationMatrix(K.clone()),
    calibrationMatrixEigen(Converter::toMatrix3f(K)),
    distortionCoefficients(distCoef.clone()),
    mbf(bf),
    depthThreshold(thDepth),
    imuCalibration(ImuCalib),
    p_imuPreintegrated(nullptr),
    p_previousFrame(pPrevF),
    p_imuPreintegratedFrame(nullptr),
    p_referenceKeyFrame(static_cast<KeyFrame *>(nullptr)),
    imuPreintegrated(false),
    p_camera(pCamera),
    p_camera2(pCamera2),
    poseAvailable(false),
    velocityAvailable(false)

{
    imgLeft  = imLeft.clone();
    imgRight = imRight.clone();

    // Setting the color image for Semantic Segmentation
    colorImg = imColor.clone();

    // Frame ID
    mnId = nNextId++;

    // Scale Level Info
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
    thread threadLeft(
        &Frame::extractOrbFeatures,
        this,
        0,
        imLeft,
        static_cast<camera_models::KannalaBrandt8 *>(p_camera)->lappingArea[0],
        static_cast<camera_models::KannalaBrandt8 *>(p_camera)->lappingArea[1]);
    thread threadRight(
        &Frame::extractOrbFeatures,
        this,
        1,
        imRight,
        static_cast<camera_models::KannalaBrandt8 *>(p_camera2)->lappingArea[0],
        static_cast<camera_models::KannalaBrandt8 *>(p_camera2)
            ->lappingArea[1]);
    threadLeft.join();
    threadRight.join();
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB =
        std::chrono::steady_clock::now();

    orbExtractionTime =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndExtORB - time_StartExtORB)
            .count();
#endif

    Nleft  = keyPoints.size();
    Nright = keyPointsRight.size();

    N = Nleft + Nright;
    if (N == 0)
        return;

    // This is done only for the first Frame (or after a change in the
    // calibration)
    if (initialComputationsDone)
    {
        computeImageBounds(imLeft);

        gridElementWidthInverse =
            static_cast<float>(FRAME_GRID_COLS) / (gridMaxX - gridMinX);
        gridElementHeightInverse =
            static_cast<float>(FRAME_GRID_ROWS) / (gridMaxY - gridMinY);

        fx    = K.at<float>(0, 0);
        fy    = K.at<float>(1, 1);
        cx    = K.at<float>(0, 2);
        cy    = K.at<float>(1, 2);
        invfx = 1.0f / fx;
        invfy = 1.0f / fy;

        initialComputationsDone = false;
    }

    mb = mbf / fx;

    // Sophus/Eigen
    poseTlr        = Tlr;
    poseTrl        = poseTlr.inverse();
    rotationRlr    = poseTlr.rotationMatrix();
    translationTlr = poseTlr.translation();

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartStereoMatches =
        std::chrono::steady_clock::now();
#endif
    computeStereoFishEyeMatches();
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndStereoMatches =
        std::chrono::steady_clock::now();

    stereoMatchTime =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndStereoMatches - time_StartStereoMatches)
            .count();
#endif

    // Put all descriptors in the same matrix
    cv::vconcat(descriptors, descriptorsRight, descriptors);

    // Initialize MapPoints
    mapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(nullptr));

    // Initialize MapMarkers
    mapMarkers = markers;

    outlierFlags = vector<bool>(N, false);

    assignFeaturesToGrid();

    undistortKeyPoints();
}

// RGB-D and RGBD-Inertial Frames Processing
Frame::Frame(const cv::Mat                                &imColor,
             const cv::Mat                                &imGray,
             const cv::Mat                                &imDepth,
             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud,
             const double                                 &timeStamp,
             ORBextractor                                 *extractor,
             ORBVocabulary                                *voc,
             cv::Mat                                      &K,
             cv::Mat                                      &distCoef,
             const float                                  &bf,
             const float                                  &thDepth,
             camera_models::GeometricCamera               *pCamera,
             Frame                                        *pPrevF,
             const IMU::Calib                             &ImuCalib,
             const std::vector<semantic::Marker *>         markers) :
    p_poseImuConstraint(nullptr),
    p_orbVocabulary(voc),
    p_orbExtractorLeft(extractor),
    p_orbExtractorRight(static_cast<ORBextractor *>(nullptr)),
    timeStamp(timeStamp),
    calibrationMatrix(K.clone()),
    calibrationMatrixEigen(Converter::toMatrix3f(K)),
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
    p_camera2(nullptr),
    poseAvailable(false),
    velocityAvailable(false)
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
        if (pPrevF->hasVelocity())
            setVelocity(pPrevF->getVelocity());
        else
            velocityVw.setZero();

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

// Monocular Frames Processing
Frame::Frame(const cv::Mat                        &imColor,
             const cv::Mat                        &imGray,
             const double                         &timeStamp,
             ORBextractor                         *extractor,
             ORBVocabulary                        *voc,
             camera_models::GeometricCamera       *pCamera,
             cv::Mat                              &distCoef,
             const float                          &bf,
             const float                          &thDepth,
             Frame                                *pPrevF,
             const IMU::Calib                     &ImuCalib,
             const std::vector<semantic::Marker *> markers) :
    p_poseImuConstraint(nullptr),
    p_orbVocabulary(voc),
    p_orbExtractorLeft(extractor),
    p_orbExtractorRight(static_cast<ORBextractor *>(nullptr)),
    timeStamp(timeStamp),
    calibrationMatrix(static_cast<camera_models::Pinhole *>(pCamera)->toK()),
    calibrationMatrixEigen(
        static_cast<camera_models::Pinhole *>(pCamera)->toK_()),
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
    p_camera2(nullptr),
    poseAvailable(false),
    velocityAvailable(false)
{
    // Setting the color image for Semantic Segmentation
    colorImg = imColor.clone();

    // Frame ID
    mnId = nNextId++;

    // Scale Level Info
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
    extractOrbFeatures(0, imGray, 0, 1000);
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

    // Set no stereo information
    closeMapPointCount = 0;
    depths             = vector<float>(N, -1);
    uRight             = vector<float>(N, -1);

    // Initialize MapPoints
    mapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(nullptr));

    // Initialize MapMarkers
    mapMarkers = markers;

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

        fx =
            static_cast<camera_models::Pinhole *>(p_camera)->toK().at<float>(0,
                                                                             0);
        fy =
            static_cast<camera_models::Pinhole *>(p_camera)->toK().at<float>(1,
                                                                             1);
        cx =
            static_cast<camera_models::Pinhole *>(p_camera)->toK().at<float>(0,
                                                                             2);
        cy =
            static_cast<camera_models::Pinhole *>(p_camera)->toK().at<float>(1,
                                                                             2);
        invfx = 1.0f / fx;
        invfy = 1.0f / fy;

        initialComputationsDone = false;
    }

    mb = mbf / fx;

    // Set no stereo fisheye information
    Nleft              = -1;
    Nright             = -1;
    monoLeft           = -1;
    monoRight          = -1;
    leftToRightMatches = vector<int>(0);
    rightToLeftMatches = vector<int>(0);
    stereoPoints3D     = vector<Eigen::Vector3f>(0);

    assignFeaturesToGrid();

    if (pPrevF)
    {
        if (pPrevF->hasVelocity())
        {
            setVelocity(pPrevF->getVelocity());
        }
    }
    else
    {
        velocityVw.setZero();
    }
}

void Frame::assignFeaturesToGrid()
{
    // Fill matrix with points
    const int nCells = FRAME_GRID_COLS * FRAME_GRID_ROWS;

    int nReserve = 0.5f * N / (nCells);

    for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
        for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
        {
            grid[i][j].reserve(nReserve);
            if (Nleft != -1)
            {
                gridRight[i][j].reserve(nReserve);
            }
        }

    for (int i = 0; i < N; i++)
    {
        const cv::KeyPoint &kp = (Nleft == -1) ? keyPointsUndistorted[i]
                                 : (i < Nleft) ? keyPoints[i]
                                               : keyPointsRight[i - Nleft];

        int nGridPosX, nGridPosY;
        if (isPositionInGrid(kp, nGridPosX, nGridPosY))
        {
            if (Nleft == -1 || i < Nleft)
                grid[nGridPosX][nGridPosY].push_back(i);
            else
                gridRight[nGridPosX][nGridPosY].push_back(i - Nleft);
        }
    }
}

void Frame::extractOrbFeatures(int            flag,
                               const cv::Mat &imageGray,
                               const int      x0,
                               const int      x1)
{
    vector<int> vLapping = {x0, x1};
    // Compute ORB based on the flag (0: left, 1: right)
    if (flag == 0)
        monoLeft = (*p_orbExtractorLeft)(imageGray,
                                         cv::Mat(),
                                         keyPoints,
                                         descriptors,
                                         vLapping);
    else
        monoRight = (*p_orbExtractorRight)(imageGray,
                                           cv::Mat(),
                                           keyPointsRight,
                                           descriptorsRight,
                                           vLapping);
}

bool Frame::isSet() const
{
    return isFrameSet;
}

void Frame::setPose(const Sophus::SE3<float> &Tcw)
{
    poseTcw = Tcw;

    updatePoseMatrices();
    isFrameSet    = true;
    poseAvailable = true;
}

void Frame::setNewBias(const IMU::Bias &b)
{
    imuBias = b;
    if (p_imuPreintegrated)
        p_imuPreintegrated->setNewBias(b);
}

void Frame::setVelocity(Eigen::Vector3f Vwb)
{
    velocityVw        = Vwb;
    velocityAvailable = true;
}

Eigen::Vector3f Frame::getVelocity() const
{
    return velocityVw;
}

void Frame::setImuPoseVelocity(const Eigen::Matrix3f &Rwb,
                               const Eigen::Vector3f &twb,
                               const Eigen::Vector3f &Vwb)
{
    velocityVw        = Vwb;
    velocityAvailable = true;

    Sophus::SE3f Twb(Rwb, twb);
    Sophus::SE3f Tbw = Twb.inverse();

    poseTcw = imuCalibration.mTcb * Tbw;

    updatePoseMatrices();
    isFrameSet    = true;
    poseAvailable = true;
}

void Frame::updatePoseMatrices()
{
    Sophus::SE3<float> Twc = poseTcw.inverse();
    rotationRwc            = Twc.rotationMatrix();
    centerOw               = Twc.translation();
    rotationRcw            = poseTcw.rotationMatrix();
    translationTcw         = poseTcw.translation();
}

Eigen::Matrix<float, 3, 1> Frame::getImuPosition() const
{
    return rotationRwc * imuCalibration.mTcb.translation() + centerOw;
}

Eigen::Matrix<float, 3, 3> Frame::getImuRotation()
{
    return rotationRwc * imuCalibration.mTcb.rotationMatrix();
}

Sophus::SE3<float> Frame::getImuPose()
{
    return poseTcw.inverse() * imuCalibration.mTcb;
}

Sophus::SE3f Frame::getRelativePoseTrl()
{
    return poseTrl;
}

Sophus::SE3f Frame::getRelativePoseTlr()
{
    return poseTlr;
}

Eigen::Matrix3f Frame::getRelativePoseTlrRotation()
{
    return poseTlr.rotationMatrix();
}

Eigen::Vector3f Frame::getRelativePoseTlrTranslation()
{
    return poseTlr.translation();
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    if (Nleft == -1)
    {
        pMP->trackInView = false;
        pMP->trackProjX  = -1;
        pMP->trackProjY  = -1;

        // 3D in absolute coordinates
        Eigen::Matrix<float, 3, 1> P = pMP->getWorldPos();

        // 3D in camera coordinates
        const Eigen::Matrix<float, 3, 1> Pc = rotationRcw * P + translationTcw;
        const float                      Pc_dist = Pc.norm();

        // Check positive depth
        const float &PcZ  = Pc(2);
        const float  invz = 1.0f / PcZ;
        if (PcZ < 0.0f)
            return false;

        const Eigen::Vector2f uv = p_camera->project(Pc);

        if (uv(0) < gridMinX || uv(0) > gridMaxX)
            return false;
        if (uv(1) < gridMinY || uv(1) > gridMaxY)
            return false;

        pMP->trackProjX = uv(0);
        pMP->trackProjY = uv(1);

        // Check distance is in the scale invariance region of the MapPoint
        const float           maxDistance = pMP->getMaxDistanceInvariance();
        const float           minDistance = pMP->getMinDistanceInvariance();
        const Eigen::Vector3f PO          = P - centerOw;
        const float           dist        = PO.norm();

        if (dist < minDistance || dist > maxDistance)
            return false;

        // Check viewing angle
        Eigen::Vector3f Pn = pMP->getNormal();

        const float viewCos = PO.dot(Pn) / dist;

        if (viewCos < viewingCosLimit)
            return false;

        // Predict scale in the image
        const int nPredictedLevel = pMP->predictScale(dist, this);

        // Data used by the tracking
        pMP->trackInView = true;
        pMP->trackProjX  = uv(0);
        pMP->trackProjXR = uv(0) - mbf * invz;

        pMP->trackDepth = Pc_dist;

        pMP->trackProjY      = uv(1);
        pMP->trackScaleLevel = nPredictedLevel;
        pMP->trackViewCos    = viewCos;

        return true;
    }
    else
    {
        pMP->trackInView      = false;
        pMP->trackInViewR     = false;
        pMP->trackScaleLevel  = -1;
        pMP->trackScaleLevelR = -1;

        pMP->trackInView  = isInFrustumChecks(pMP, viewingCosLimit);
        pMP->trackInViewR = isInFrustumChecks(pMP, viewingCosLimit, true);

        return pMP->trackInView || pMP->trackInViewR;
    }
}

bool Frame::projectPointDistort(MapPoint    *pMP,
                                cv::Point2f &kp,
                                float       &u,
                                float       &v)
{

    // 3D in absolute coordinates
    Eigen::Vector3f P = pMP->getWorldPos();

    // 3D in camera coordinates
    const Eigen::Vector3f Pc  = rotationRcw * P + translationTcw;
    const float          &PcX = Pc(0);
    const float          &PcY = Pc(1);
    const float          &PcZ = Pc(2);

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

    float u_distort, v_distort;

    float x  = (u - cx) * invfx;
    float y  = (v - cy) * invfy;
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
    float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
    float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

    // Tangential distorsion
    x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
    y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

    u_distort = x_distort * fx + cx;
    v_distort = y_distort * fy + cy;

    u = u_distort;
    v = v_distort;

    kp = cv::Point2f(u, v);

    return true;
}

Eigen::Vector3f Frame::inReferenceCoordinates(Eigen::Vector3f pCw)
{
    return rotationRcw * pCw + translationTcw;
}

vector<size_t> Frame::getFeaturesInArea(const float &x,
                                        const float &y,
                                        const float &r,
                                        const int    minLevel,
                                        const int    maxLevel,
                                        const bool   bRight) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    float factorX = r;
    float factorY = r;

    const int nMinCellX =
        max(0, (int)floor((x - gridMinX - factorX) * gridElementWidthInverse));
    if (nMinCellX >= FRAME_GRID_COLS)
    {
        return vIndices;
    }

    const int nMaxCellX =
        min((int)FRAME_GRID_COLS - 1,
            (int)ceil((x - gridMinX + factorX) * gridElementWidthInverse));
    if (nMaxCellX < 0)
    {
        return vIndices;
    }

    const int nMinCellY =
        max(0, (int)floor((y - gridMinY - factorY) * gridElementHeightInverse));
    if (nMinCellY >= FRAME_GRID_ROWS)
    {
        return vIndices;
    }

    const int nMaxCellY =
        min((int)FRAME_GRID_ROWS - 1,
            (int)ceil((y - gridMinY + factorY) * gridElementHeightInverse));
    if (nMaxCellY < 0)
    {
        return vIndices;
    }

    const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

    for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
    {
        for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
        {
            const vector<size_t> vCell =
                (!bRight) ? grid[ix][iy] : gridRight[ix][iy];
            if (vCell.empty())
                continue;

            for (size_t j = 0, jend = vCell.size(); j < jend; j++)
            {
                const cv::KeyPoint &kpUn =
                    (Nleft == -1) ? keyPointsUndistorted[vCell[j]]
                    : (!bRight)   ? keyPoints[vCell[j]]
                                  : keyPointsRight[vCell[j]];
                if (bCheckLevels)
                {
                    if (kpUn.octave < minLevel)
                        continue;
                    if (maxLevel >= 0)
                        if (kpUn.octave > maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x - x;
                const float disty = kpUn.pt.y - y;

                if (fabs(distx) < factorX && fabs(disty) < factorY)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::isPositionInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x - gridMinX) * gridElementWidthInverse);
    posY = round((kp.pt.y - gridMinY) * gridElementHeightInverse);

    // Keypoint's coordinates are undistorted, which could cause to go out of
    // the image
    if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 ||
        posY >= FRAME_GRID_ROWS)
        return false;

    return true;
}

void Frame::computeBagOfWords()
{
    if (bowVector.empty())
    {
        vector<cv::Mat> vCurrentDesc =
            Converter::toDescriptorVector(descriptors);
        p_orbVocabulary->transform(vCurrentDesc, bowVector, featureVector, 4);
    }
}

void Frame::undistortKeyPoints()
{
    if (distortionCoefficients.at<float>(0) == 0.0)
    {
        keyPointsUndistorted = keyPoints;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N, 2, CV_32F);

    for (int i = 0; i < N; i++)
    {
        mat.at<float>(i, 0) = keyPoints[i].pt.x;
        mat.at<float>(i, 1) = keyPoints[i].pt.y;
    }

    // Undistort points
    mat = mat.reshape(2);
    cv::undistortPoints(mat,
                        mat,
                        static_cast<camera_models::Pinhole *>(p_camera)->toK(),
                        distortionCoefficients,
                        cv::Mat(),
                        calibrationMatrix);
    mat = mat.reshape(1);

    // Fill undistorted keypoint vector
    keyPointsUndistorted.resize(N);
    for (int i = 0; i < N; i++)
    {
        cv::KeyPoint kp         = keyPoints[i];
        kp.pt.x                 = mat.at<float>(i, 0);
        kp.pt.y                 = mat.at<float>(i, 1);
        keyPointsUndistorted[i] = kp;
    }
}

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
            static_cast<camera_models::Pinhole *>(p_camera)->toK(),
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

void Frame::computeStereoMatches()
{
    uRight = vector<float>(N, -1.0f);
    depths = vector<float>(N, -1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH + ORBmatcher::TH_LOW) / 2;

    const int nRows = p_orbExtractorLeft->imagePyramid[0].rows;

    // Assign keypoints to row table
    vector<vector<size_t>> vRowIndices(nRows, vector<size_t>());

    for (int i = 0; i < nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = keyPointsRight.size();

    for (int iR = 0; iR < Nr; iR++)
    {
        const cv::KeyPoint &kp  = keyPointsRight[iR];
        const float        &kpY = kp.pt.y;
        const float         r = 2.0f * scaleFactors[keyPointsRight[iR].octave];
        const int           maxr = ceil(kpY + r);
        const int           minr = floor(kpY - r);

        for (int yi = minr; yi <= maxr; yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf / minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int>> vDistIdx;
    vDistIdx.reserve(N);

    for (int iL = 0; iL < N; iL++)
    {
        const cv::KeyPoint &kpL    = keyPoints[iL];
        const int          &levelL = kpL.octave;
        const float        &vL     = kpL.pt.y;
        const float        &uL     = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if (vCandidates.empty())
            continue;

        const float minU = uL - maxD;
        const float maxU = uL - minD;

        if (maxU < 0)
            continue;

        int    bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = descriptors.row(iL);

        // Compare descriptor to right keypoints
        for (size_t iC = 0; iC < vCandidates.size(); iC++)
        {
            const size_t        iR  = vCandidates[iC];
            const cv::KeyPoint &kpR = keyPointsRight[iR];

            if (kpR.octave < levelL - 1 || kpR.octave > levelL + 1)
                continue;

            const float &uR = kpR.pt.x;

            if (uR >= minU && uR <= maxU)
            {
                const cv::Mat &dR = descriptorsRight.row(iR);
                const int dist = ORBmatcher::computeDescriptorDistance(dL, dR);

                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if (bestDist < thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0         = keyPointsRight[bestIdxR].pt.x;
            const float scaleFactor = invScaleFactors[kpL.octave];
            const float scaleduL    = round(kpL.pt.x * scaleFactor);
            const float scaledvL    = round(kpL.pt.y * scaleFactor);
            const float scaleduR0   = round(uR0 * scaleFactor);

            // sliding window search
            const int w  = 5;
            cv::Mat   IL = p_orbExtractorLeft->imagePyramid[kpL.octave]
                             .rowRange(scaledvL - w, scaledvL + w + 1)
                             .colRange(scaleduL - w, scaleduL + w + 1);

            int           bestDist = INT_MAX;
            int           bestincR = 0;
            const int     L        = 5;
            vector<float> vDists;
            vDists.resize(2 * L + 1);

            const float iniu = scaleduR0 + L - w;
            const float endu = scaleduR0 + L + w + 1;
            if (iniu < 0 ||
                endu >= p_orbExtractorRight->imagePyramid[kpL.octave].cols)
                continue;

            for (int incR = -L; incR <= +L; incR++)
            {
                cv::Mat IR = p_orbExtractorRight->imagePyramid[kpL.octave]
                                 .rowRange(scaledvL - w, scaledvL + w + 1)
                                 .colRange(scaleduR0 + incR - w,
                                           scaleduR0 + incR + w + 1);

                float dist = cv::norm(IL, IR, cv::NORM_L1);
                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestincR = incR;
                }

                vDists[L + incR] = dist;
            }

            if (bestincR == -L || bestincR == L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L + bestincR - 1];
            const float dist2 = vDists[L + bestincR];
            const float dist3 = vDists[L + bestincR + 1];

            const float deltaR =
                (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));

            if (deltaR < -1 || deltaR > 1)
                continue;

            // Re-scaled coordinate
            float bestuR = scaleFactors[kpL.octave] *
                           ((float)scaleduR0 + (float)bestincR + deltaR);

            float disparity = (uL - bestuR);

            if (disparity >= minD && disparity < maxD)
            {
                if (disparity <= 0)
                {
                    disparity = 0.01;
                    bestuR    = uL - 0.01;
                }
                depths[iL] = mbf / disparity;
                uRight[iL] = bestuR;
                vDistIdx.push_back(pair<int, int>(bestDist, iL));
            }
        }
    }

    rejectOutlierStereoMatches(vDistIdx, uRight, depths);
}

void Frame::computeStereoFromRGBD(const cv::Mat &imDepth)
{
    uRight = vector<float>(N, -1);
    depths = vector<float>(N, -1);

    for (int i = 0; i < N; i++)
    {
        const cv::KeyPoint &kp  = keyPoints[i];
        const cv::KeyPoint &kpU = keyPointsUndistorted[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v, u);

        if (d > 0)
        {
            depths[i] = d;
            uRight[i] = kpU.pt.x - mbf / d;
        }
    }
}

bool Frame::unprojectStereo(const int &i, Eigen::Vector3f &x3D)
{
    const float z = depths[i];
    if (z > 0)
    {
        const float     u = keyPointsUndistorted[i].pt.x;
        const float     v = keyPointsUndistorted[i].pt.y;
        const float     x = (u - cx) * z * invfx;
        const float     y = (v - cy) * z * invfy;
        Eigen::Vector3f x3Dc(x, y, z);
        x3D = rotationRwc * x3Dc + centerOw;
        return true;
    }
    else
        return false;
}

bool Frame::isImuPreintegrated()
{
    unique_lock<std::mutex> lock(*p_imuMutex);
    return imuPreintegrated;
}

void Frame::setIntegrated()
{
    unique_lock<std::mutex> lock(*p_imuMutex);
    imuPreintegrated = true;
}

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
            float depth = static_cast<camera_models::KannalaBrandt8 *>(p_camera)
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

bool Frame::isInFrustumChecks(MapPoint *pMP, float viewingCosLimit, bool bRight)
{
    // 3D in absolute coordinates
    Eigen::Vector3f P = pMP->getWorldPos();

    Eigen::Matrix3f mR;
    Eigen::Vector3f mt, twc;
    if (bRight)
    {
        Eigen::Matrix3f Rrl = poseTrl.rotationMatrix();
        Eigen::Vector3f trl = poseTrl.translation();
        mR                  = Rrl * rotationRcw;
        mt                  = Rrl * translationTcw + trl;
        twc                 = rotationRwc * poseTlr.translation() + centerOw;
    }
    else
    {
        mR  = rotationRcw;
        mt  = translationTcw;
        twc = centerOw;
    }

    // 3D in camera coordinates
    Eigen::Vector3f Pc      = mR * P + mt;
    const float     Pc_dist = Pc.norm();
    const float    &PcZ     = Pc(2);

    // Check positive depth
    if (PcZ < 0.0f)
        return false;

    // Project in image and check it is not outside
    Eigen::Vector2f uv;
    if (bRight)
        uv = p_camera2->project(Pc);
    else
        uv = p_camera->project(Pc);

    if (uv(0) < gridMinX || uv(0) > gridMaxX)
        return false;
    if (uv(1) < gridMinY || uv(1) > gridMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float           maxDistance = pMP->getMaxDistanceInvariance();
    const float           minDistance = pMP->getMinDistanceInvariance();
    const Eigen::Vector3f PO          = P - twc;
    const float           dist        = PO.norm();

    if (dist < minDistance || dist > maxDistance)
        return false;

    // Check viewing angle
    Eigen::Vector3f Pn = pMP->getNormal();

    const float viewCos = PO.dot(Pn) / dist;

    if (viewCos < viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->predictScale(dist, this);

    if (bRight)
    {
        pMP->trackProjXR      = uv(0);
        pMP->trackProjYR      = uv(1);
        pMP->trackScaleLevelR = nPredictedLevel;
        pMP->trackViewCosR    = viewCos;
        pMP->trackDepthR      = Pc_dist;
    }
    else
    {
        pMP->trackProjX      = uv(0);
        pMP->trackProjY      = uv(1);
        pMP->trackScaleLevel = nPredictedLevel;
        pMP->trackViewCos    = viewCos;
        pMP->trackDepth      = Pc_dist;
    }

    return true;
}

Eigen::Vector3f Frame::unprojectStereoFishEye(const int &i)
{
    return rotationRwc * stereoPoints3D[i] + centerOw;
}

} // namespace core
} // namespace vs_graphs
