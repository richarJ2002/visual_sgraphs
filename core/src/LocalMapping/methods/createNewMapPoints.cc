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
 * @file            createNewMapPoints.cc
 *
 * @brief           Implements LocalMapping::createNewMapPoints(), declared in
 *                  LocalMapping.h.
 */

#include "LocalMapping.h"

#include "GeometricTools.h"
#include "ORBmatcher.h"
#include "Tracking.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

LocalMappingStatus LocalMapping::createNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    int neighborKeyFrameCount = 10;
    // For stereo inertial case
    if (isMonocular)
        neighborKeyFrameCount = 30;
    std::vector<KeyFrame *> neighborKeyFrames{};
    if (p_currentKeyFrame->getBestCovisibilityKeyFrames(neighborKeyFrameCount,
                                                        neighborKeyFrames) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBestCovisibilityKeyFrames returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    if (isInertial)
    {
        KeyFrame *p_walkKeyFrame         = p_currentKeyFrame;
        int       temporalChainStepCount = 0;
        // nn is the fixed covisibility budget set above (10, or 30 when
        // monocular), so it is always positive here.
        while ((neighborKeyFrames.size() <=
                static_cast<std::size_t>(neighborKeyFrameCount)) &&
               (p_walkKeyFrame->p_prevKF) &&
               (temporalChainStepCount++ < neighborKeyFrameCount))
        {
            std::vector<KeyFrame *>::iterator neighborKeyFrameIt =
                std::find(neighborKeyFrames.begin(),
                          neighborKeyFrames.end(),
                          p_walkKeyFrame->p_prevKF);
            if (neighborKeyFrameIt == neighborKeyFrames.end())
                neighborKeyFrames.push_back(p_walkKeyFrame->p_prevKF);
            p_walkKeyFrame = p_walkKeyFrame->p_prevKF;
        }
    }

    float matchNnRatio = 0.6f;

    ORBmatcher matcher(matchNnRatio, false);

    Sophus::SE3<float> sophTcw1{};
    if (p_currentKeyFrame->getPose(sophTcw1) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Eigen::Matrix<float, 3, 4> cameraPose_worldToCamera1 = sophTcw1.matrix3x4();
    Eigen::Matrix<float, 3, 3> cameraRotation_worldToCamera1 =
        cameraPose_worldToCamera1.block<3, 3>(0, 0);
    Eigen::Matrix<float, 3, 3> cameraRotation_camera1ToWorld =
        cameraRotation_worldToCamera1.transpose();
    Eigen::Vector3f cameraTranslation_worldToCamera1 = sophTcw1.translation();
    Eigen::Vector3f cameraCenter1_world{};
    if (p_currentKeyFrame->getCameraCenter(cameraCenter1_world) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCameraCenter returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    const float &focalLengthX1    = p_currentKeyFrame->fx;
    const float &focalLengthY1    = p_currentKeyFrame->fy;
    const float &principalPointX1 = p_currentKeyFrame->cx;
    const float &principalPointY1 = p_currentKeyFrame->cy;

    const float ratioFactor      = 1.5f * p_currentKeyFrame->scaleFactor;
    int         stereoPointCount = 0;
    int         stereoGoodProjectionCount = 0;
    int         stereoAttemptCount        = 0;
    int         totalStereoPointCount     = 0;
    // Search matches with epipolar restriction and triangulate
    for (size_t neighborKeyFrameIndex = 0;
         neighborKeyFrameIndex < neighborKeyFrames.size();
         neighborKeyFrameIndex++)
    {
        bool hasNewKeyFrames{};
        if ((neighborKeyFrameIndex > 0) &&
            checkNewKeyFrames(hasNewKeyFrames) !=
                LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: checkNewKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (neighborKeyFrameIndex > 0 && hasNewKeyFrames)
            return LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS;

        KeyFrame *p_neighborKeyFrame = neighborKeyFrames[neighborKeyFrameIndex];

        camera_models::geometriccamera::GeometricCamera
            *p_camera1 = p_currentKeyFrame->p_camera,
            *p_camera2 = p_neighborKeyFrame->p_camera;

        // Check first that baseline is not too short
        Eigen::Vector3f cameraCenter2_world{};
        if (p_neighborKeyFrame->getCameraCenter(cameraCenter2_world) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCameraCenter returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f baselineVector =
            cameraCenter2_world - cameraCenter1_world;
        const float baseline = baselineVector.norm();

        if (!isMonocular)
        {
            if (baseline < p_neighborKeyFrame->mb)
                continue;
        }
        else
        {
            float medianDepthKeyFrame2{};
            if (p_neighborKeyFrame->computeSceneMedianDepth(
                    2,
                    medianDepthKeyFrame2) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: computeSceneMedianDepth returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            const float ratioBaselineDepth = baseline / medianDepthKeyFrame2;

            if (ratioBaselineDepth < 0.01)
                continue;
        }

        // Search matches that fullfil epipolar constraint
        std::vector<std::pair<size_t, size_t>> matchedKeyPointIndices;
        Map                                   *p_currentKeyFrameMap = nullptr;
        if ((isInertial && p_tracker->state == Tracking::RECENTLY_LOST) &&
            p_currentKeyFrame->getMap(p_currentKeyFrameMap) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        bool inertialBA2{};
        if ((isInertial && p_tracker->state == Tracking::RECENTLY_LOST) &&
            p_currentKeyFrameMap->getInertialBA2(inertialBA2) !=
                MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getInertialBA2 returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        bool isCoarseSearch = isInertial &&
                              p_tracker->state == Tracking::RECENTLY_LOST &&
                              inertialBA2;

        int matcherForTriangulation{};
        if (matcher.searchForTriangulation(p_currentKeyFrame,
                                           p_neighborKeyFrame,
                                           matchedKeyPointIndices,
                                           false,
                                           matcherForTriangulation,
                                           isCoarseSearch) !=
            ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: searchForTriangulation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        Sophus::SE3<float> sophTcw2{};
        if (p_neighborKeyFrame->getPose(sophTcw2) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Matrix<float, 3, 4> cameraPose_worldToCamera2 =
            sophTcw2.matrix3x4();
        Eigen::Matrix<float, 3, 3> cameraRotation_worldToCamera2 =
            cameraPose_worldToCamera2.block<3, 3>(0, 0);
        Eigen::Matrix<float, 3, 3> cameraRotation_camera2ToWorld =
            cameraRotation_worldToCamera2.transpose();
        Eigen::Vector3f cameraTranslation_worldToCamera2 =
            sophTcw2.translation();

        const float &focalLengthX2    = p_neighborKeyFrame->fx;
        const float &focalLengthY2    = p_neighborKeyFrame->fy;
        const float &principalPointX2 = p_neighborKeyFrame->cx;
        const float &principalPointY2 = p_neighborKeyFrame->cy;

        // Triangulate each match
        const int matchCount = matchedKeyPointIndices.size();
        for (int matchIndex = 0; matchIndex < matchCount; matchIndex++)
        {
            const int &keyPointIndex1 =
                matchedKeyPointIndices[matchIndex].first;
            const int &keyPointIndex2 =
                matchedKeyPointIndices[matchIndex].second;

            const cv::KeyPoint &keyPoint1 =
                (p_currentKeyFrame->leftKeyPointCount == -1)
                    ? p_currentKeyFrame->keyPointsUndistorted[keyPointIndex1]
                : (keyPointIndex1 < p_currentKeyFrame->leftKeyPointCount)
                    ? p_currentKeyFrame->keyPoints[keyPointIndex1]
                    : p_currentKeyFrame->keyPointsRight
                          [keyPointIndex1 -
                           p_currentKeyFrame->leftKeyPointCount];
            const float keyPointRightU1 =
                p_currentKeyFrame->uRight[keyPointIndex1];
            bool hasStereoMatch1 =
                (!p_currentKeyFrame->p_camera2 && keyPointRightU1 >= 0);
            const bool isKeyPoint1FromRightCamera =
                (p_currentKeyFrame->leftKeyPointCount == -1 ||
                 keyPointIndex1 < p_currentKeyFrame->leftKeyPointCount)
                    ? false
                    : true;

            const cv::KeyPoint &keyPoint2 =
                (p_neighborKeyFrame->leftKeyPointCount == -1)
                    ? p_neighborKeyFrame->keyPointsUndistorted[keyPointIndex2]
                : (keyPointIndex2 < p_neighborKeyFrame->leftKeyPointCount)
                    ? p_neighborKeyFrame->keyPoints[keyPointIndex2]
                    : p_neighborKeyFrame->keyPointsRight
                          [keyPointIndex2 -
                           p_neighborKeyFrame->leftKeyPointCount];

            const float keyPointRightU2 =
                p_neighborKeyFrame->uRight[keyPointIndex2];
            bool hasStereoMatch2 =
                (!p_neighborKeyFrame->p_camera2 && keyPointRightU2 >= 0);
            const bool isKeyPoint2FromRightCamera =
                (p_neighborKeyFrame->leftKeyPointCount == -1 ||
                 keyPointIndex2 < p_neighborKeyFrame->leftKeyPointCount)
                    ? false
                    : true;

            if (p_currentKeyFrame->p_camera2 && p_neighborKeyFrame->p_camera2)
            {
                if (isKeyPoint1FromRightCamera && isKeyPoint2FromRightCamera)
                {
                    Sophus::SE3<float> currentKeyFrameRightPose{};
                    if (p_currentKeyFrame->getRightPose(
                            currentKeyFrameRightPose) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getRightPose returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    sophTcw1 = currentKeyFrameRightPose;
                    Eigen::Vector3f currentKeyFrameRightCameraCenter{};
                    if (p_currentKeyFrame->getRightCameraCenter(
                            currentKeyFrameRightCameraCenter) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: getRightCameraCenter returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }
                    cameraCenter1_world = currentKeyFrameRightCameraCenter;

                    Sophus::SE3<float> neighborKeyFrameRightPose{};
                    if (p_neighborKeyFrame->getRightPose(
                            neighborKeyFrameRightPose) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getRightPose returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    sophTcw2 = neighborKeyFrameRightPose;
                    Eigen::Vector3f neighborKeyFrameRightCameraCenter{};
                    if (p_neighborKeyFrame->getRightCameraCenter(
                            neighborKeyFrameRightCameraCenter) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: getRightCameraCenter returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }
                    cameraCenter2_world = neighborKeyFrameRightCameraCenter;

                    p_camera1 = p_currentKeyFrame->p_camera2;
                    p_camera2 = p_neighborKeyFrame->p_camera2;
                }
                else if (isKeyPoint1FromRightCamera &&
                         !isKeyPoint2FromRightCamera)
                {
                    Sophus::SE3<float> currentKeyFrameRightPose2{};
                    if (p_currentKeyFrame->getRightPose(
                            currentKeyFrameRightPose2) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getRightPose returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    sophTcw1 = currentKeyFrameRightPose2;
                    Eigen::Vector3f currentKeyFrameRightCameraCenter2{};
                    if (p_currentKeyFrame->getRightCameraCenter(
                            currentKeyFrameRightCameraCenter2) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: getRightCameraCenter returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }
                    cameraCenter1_world = currentKeyFrameRightCameraCenter2;

                    Sophus::SE3f neighborKeyFramePose{};
                    if (p_neighborKeyFrame->getPose(neighborKeyFramePose) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getPose returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    sophTcw2 = neighborKeyFramePose;
                    Eigen::Vector3f neighborKeyFrameCameraCenter{};
                    if (p_neighborKeyFrame->getCameraCenter(
                            neighborKeyFrameCameraCenter) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getCameraCenter returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    cameraCenter2_world = neighborKeyFrameCameraCenter;

                    p_camera1 = p_currentKeyFrame->p_camera2;
                    p_camera2 = p_neighborKeyFrame->p_camera;
                }
                else if (!isKeyPoint1FromRightCamera &&
                         isKeyPoint2FromRightCamera)
                {
                    Sophus::SE3f currentKeyFramePose{};
                    if (p_currentKeyFrame->getPose(currentKeyFramePose) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getPose returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    sophTcw1 = currentKeyFramePose;
                    Eigen::Vector3f currentKeyFrameCameraCenter{};
                    if (p_currentKeyFrame->getCameraCenter(
                            currentKeyFrameCameraCenter) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getCameraCenter returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    cameraCenter1_world = currentKeyFrameCameraCenter;

                    Sophus::SE3<float> neighborKeyFrameRightPose2{};
                    if (p_neighborKeyFrame->getRightPose(
                            neighborKeyFrameRightPose2) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getRightPose returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    sophTcw2 = neighborKeyFrameRightPose2;
                    Eigen::Vector3f neighborKeyFrameRightCameraCenter2{};
                    if (p_neighborKeyFrame->getRightCameraCenter(
                            neighborKeyFrameRightCameraCenter2) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: getRightCameraCenter returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }
                    cameraCenter2_world = neighborKeyFrameRightCameraCenter2;

                    p_camera1 = p_currentKeyFrame->p_camera;
                    p_camera2 = p_neighborKeyFrame->p_camera2;
                }
                else
                {
                    Sophus::SE3f currentKeyFramePose2{};
                    if (p_currentKeyFrame->getPose(currentKeyFramePose2) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getPose returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    sophTcw1 = currentKeyFramePose2;
                    Eigen::Vector3f currentKeyFrameCameraCenter2{};
                    if (p_currentKeyFrame->getCameraCenter(
                            currentKeyFrameCameraCenter2) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getCameraCenter returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    cameraCenter1_world = currentKeyFrameCameraCenter2;

                    Sophus::SE3f neighborKeyFramePose2{};
                    if (p_neighborKeyFrame->getPose(neighborKeyFramePose2) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getPose returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    sophTcw2 = neighborKeyFramePose2;
                    Eigen::Vector3f neighborKeyFrameCameraCenter2{};
                    if (p_neighborKeyFrame->getCameraCenter(
                            neighborKeyFrameCameraCenter2) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getCameraCenter returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    cameraCenter2_world = neighborKeyFrameCameraCenter2;

                    p_camera1 = p_currentKeyFrame->p_camera;
                    p_camera2 = p_neighborKeyFrame->p_camera;
                }
                cameraPose_worldToCamera1 = sophTcw1.matrix3x4();
                cameraRotation_worldToCamera1 =
                    cameraPose_worldToCamera1.block<3, 3>(0, 0);
                cameraRotation_camera1ToWorld =
                    cameraRotation_worldToCamera1.transpose();
                cameraTranslation_worldToCamera1 = sophTcw1.translation();

                cameraPose_worldToCamera2 = sophTcw2.matrix3x4();
                cameraRotation_worldToCamera2 =
                    cameraPose_worldToCamera2.block<3, 3>(0, 0);
                cameraRotation_camera2ToWorld =
                    cameraRotation_worldToCamera2.transpose();
                cameraTranslation_worldToCamera2 = sophTcw2.translation();
            }

            // Check parallax between rays
            Eigen::Vector3f unprojectedRay1 =
                p_camera1->unprojectEig(keyPoint1.pt);
            Eigen::Vector3f unprojectedRay2 =
                p_camera2->unprojectEig(keyPoint2.pt);

            Eigen::Vector3f worldViewingRay1 =
                cameraRotation_camera1ToWorld * unprojectedRay1;
            Eigen::Vector3f worldViewingRay2 =
                cameraRotation_camera2ToWorld * unprojectedRay2;
            const float cosParallaxRays =
                worldViewingRay1.dot(worldViewingRay2) /
                (worldViewingRay1.norm() * worldViewingRay2.norm());

            float cosParallaxStereo  = cosParallaxRays + 1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if (hasStereoMatch1)
                cosParallaxStereo1 = std::cos(
                    2 * std::atan2(p_currentKeyFrame->mb / 2,
                                   p_currentKeyFrame->depths[keyPointIndex1]));
            else if (hasStereoMatch2)
                cosParallaxStereo2 = std::cos(
                    2 * std::atan2(p_neighborKeyFrame->mb / 2,
                                   p_neighborKeyFrame->depths[keyPointIndex2]));

            if (hasStereoMatch1 || hasStereoMatch2)
                totalStereoPointCount++;

            cosParallaxStereo =
                std::min(cosParallaxStereo1, cosParallaxStereo2);

            Eigen::Vector3f triangulatedPoint;

            bool wasTriangulationSuccessful = false;
            bool isStereoTriangulatedPoint  = false;
            if (cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0 &&
                (hasStereoMatch1 || hasStereoMatch2 ||
                 (cosParallaxRays < 0.9996 && isInertial) ||
                 (cosParallaxRays < 0.9998 && !isInertial)))
            {
                wasTriangulationSuccessful =
                    (GeometricTools::triangulate(unprojectedRay1,
                                                 unprojectedRay2,
                                                 cameraPose_worldToCamera1,
                                                 cameraPose_worldToCamera2,
                                                 triangulatedPoint) ==
                     GeometricToolsStatus::GEOMETRIC_TOOLS_STATUS_SUCCESS);
                if (!wasTriangulationSuccessful)
                    continue;
            }
            else if (hasStereoMatch1 && cosParallaxStereo1 < cosParallaxStereo2)
            {
                stereoAttemptCount++;
                isStereoTriangulatedPoint = true;
                bool currentKeyFrameIsUnprojected{};
                if (p_currentKeyFrame->unprojectStereo(
                        keyPointIndex1,
                        triangulatedPoint,
                        currentKeyFrameIsUnprojected) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: unprojectStereo returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                wasTriangulationSuccessful = currentKeyFrameIsUnprojected;
            }
            else if (hasStereoMatch2 && cosParallaxStereo2 < cosParallaxStereo1)
            {
                stereoAttemptCount++;
                isStereoTriangulatedPoint = true;
                bool neighborKeyFrameIsUnprojected{};
                if (p_neighborKeyFrame->unprojectStereo(
                        keyPointIndex2,
                        triangulatedPoint,
                        neighborKeyFrameIsUnprojected) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: unprojectStereo returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                wasTriangulationSuccessful = neighborKeyFrameIsUnprojected;
            }
            else
            {
                continue; // No stereo and very low parallax
            }

            if (wasTriangulationSuccessful && isStereoTriangulatedPoint)
                stereoGoodProjectionCount++;

            if (!wasTriangulationSuccessful)
                continue;

            // Check triangulation in front of cameras
            float cameraFrameZ1 =
                cameraRotation_worldToCamera1.row(2).dot(triangulatedPoint) +
                cameraTranslation_worldToCamera1(2);
            if (cameraFrameZ1 <= 0)
                continue;

            float cameraFrameZ2 =
                cameraRotation_worldToCamera2.row(2).dot(triangulatedPoint) +
                cameraTranslation_worldToCamera2(2);
            if (cameraFrameZ2 <= 0)
                continue;

            // Check reprojection error in first keyframe
            const float &levelSigmaSquared1 =
                p_currentKeyFrame->levelSigmaSquared[keyPoint1.octave];
            const float cameraFrameX1 =
                cameraRotation_worldToCamera1.row(0).dot(triangulatedPoint) +
                cameraTranslation_worldToCamera1(0);
            const float cameraFrameY1 =
                cameraRotation_worldToCamera1.row(1).dot(triangulatedPoint) +
                cameraTranslation_worldToCamera1(1);
            const float inverseDepth1 = 1.0 / cameraFrameZ1;

            if (!hasStereoMatch1)
            {
                cv::Point2f projectedPixel1 = p_camera1->project(
                    cv::Point3f(cameraFrameX1, cameraFrameY1, cameraFrameZ1));
                float reprojectionErrorX1 = projectedPixel1.x - keyPoint1.pt.x;
                float reprojectionErrorY1 = projectedPixel1.y - keyPoint1.pt.y;

                if ((reprojectionErrorX1 * reprojectionErrorX1 +
                     reprojectionErrorY1 * reprojectionErrorY1) >
                    5.991 * levelSigmaSquared1)
                    continue;
            }
            else
            {
                float pixelU1 = focalLengthX1 * cameraFrameX1 * inverseDepth1 +
                                principalPointX1;
                float pixelU1_r =
                    pixelU1 - p_currentKeyFrame->mbf * inverseDepth1;
                float pixelV1 = focalLengthY1 * cameraFrameY1 * inverseDepth1 +
                                principalPointY1;
                float reprojectionErrorX1   = pixelU1 - keyPoint1.pt.x;
                float reprojectionErrorY1   = pixelV1 - keyPoint1.pt.y;
                float reprojectionErrorX1_r = pixelU1_r - keyPointRightU1;
                if ((reprojectionErrorX1 * reprojectionErrorX1 +
                     reprojectionErrorY1 * reprojectionErrorY1 +
                     reprojectionErrorX1_r * reprojectionErrorX1_r) >
                    7.8 * levelSigmaSquared1)
                    continue;
            }

            // Check reprojection error in second keyframe
            const float levelSigmaSquared2 =
                p_neighborKeyFrame->levelSigmaSquared[keyPoint2.octave];
            const float cameraFrameX2 =
                cameraRotation_worldToCamera2.row(0).dot(triangulatedPoint) +
                cameraTranslation_worldToCamera2(0);
            const float cameraFrameY2 =
                cameraRotation_worldToCamera2.row(1).dot(triangulatedPoint) +
                cameraTranslation_worldToCamera2(1);
            const float inverseDepth2 = 1.0 / cameraFrameZ2;
            if (!hasStereoMatch2)
            {
                cv::Point2f projectedPixel2 = p_camera2->project(
                    cv::Point3f(cameraFrameX2, cameraFrameY2, cameraFrameZ2));
                float reprojectionErrorX2 = projectedPixel2.x - keyPoint2.pt.x;
                float reprojectionErrorY2 = projectedPixel2.y - keyPoint2.pt.y;
                if ((reprojectionErrorX2 * reprojectionErrorX2 +
                     reprojectionErrorY2 * reprojectionErrorY2) >
                    5.991 * levelSigmaSquared2)
                    continue;
            }
            else
            {
                float pixelU2 = focalLengthX2 * cameraFrameX2 * inverseDepth2 +
                                principalPointX2;
                float pixelU2_r =
                    pixelU2 - p_currentKeyFrame->mbf * inverseDepth2;
                float pixelV2 = focalLengthY2 * cameraFrameY2 * inverseDepth2 +
                                principalPointY2;
                float reprojectionErrorX2   = pixelU2 - keyPoint2.pt.x;
                float reprojectionErrorY2   = pixelV2 - keyPoint2.pt.y;
                float reprojectionErrorX2_r = pixelU2_r - keyPointRightU2;
                if ((reprojectionErrorX2 * reprojectionErrorX2 +
                     reprojectionErrorY2 * reprojectionErrorY2 +
                     reprojectionErrorX2_r * reprojectionErrorX2_r) >
                    7.8 * levelSigmaSquared2)
                    continue;
            }

            // Check scale consistency
            Eigen::Vector3f pointViewVector1 =
                triangulatedPoint - cameraCenter1_world;
            float pointDistance1 = pointViewVector1.norm();

            Eigen::Vector3f pointViewVector2 =
                triangulatedPoint - cameraCenter2_world;
            float pointDistance2 = pointViewVector2.norm();

            if (pointDistance1 == 0 || pointDistance2 == 0)
                continue;

            if (shouldSkipFarPoints &&
                (pointDistance1 >= farPointsThreshold ||
                 pointDistance2 >= farPointsThreshold)) // MODIFICATION
                continue;

            const float ratioDistance = pointDistance2 / pointDistance1;
            const float ratioOctave =
                p_currentKeyFrame->scaleFactors[keyPoint1.octave] /
                p_neighborKeyFrame->scaleFactors[keyPoint2.octave];

            if (ratioDistance * ratioFactor < ratioOctave ||
                ratioDistance > ratioOctave * ratioFactor)
                continue;

            // Triangulation is succesfull
            Map *p_atlasCurrentMap = nullptr;
            if (p_atlas->getCurrentMap(p_atlasCurrentMap) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCurrentMap returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            MapPoint *p_mapPoint = new MapPoint(triangulatedPoint,
                                                p_currentKeyFrame,
                                                p_atlasCurrentMap);
            if (isStereoTriangulatedPoint)
                stereoPointCount++;

            if (p_mapPoint->addObservation(p_currentKeyFrame, keyPointIndex1) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addObservation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_mapPoint->addObservation(p_neighborKeyFrame,
                                           keyPointIndex2) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addObservation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (p_currentKeyFrame->addMapPoint(p_mapPoint, keyPointIndex1) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapPoint returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_neighborKeyFrame->addMapPoint(p_mapPoint, keyPointIndex2) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapPoint returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (p_mapPoint->computeDistinctiveDescriptors() !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: computeDistinctiveDescriptors returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }

            if (p_mapPoint->updateNormalAndDepth() !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: updateNormalAndDepth returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            if (p_atlas->addMapPoint(p_mapPoint) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapPoint returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            recentAddedMapPoints.push_back(p_mapPoint);
        }
    }

    return LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
