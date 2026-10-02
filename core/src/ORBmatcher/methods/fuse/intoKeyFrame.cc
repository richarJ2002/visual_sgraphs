/*!
 * This file is part of ORB-SLAM3.
 * Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details:
 * https://www.gnu.org/licenses/
 */

/*!
 * @file            intoKeyFrame.cc
 *
 * @brief           Implements ORBmatcher::fuse() (intoKeyFrame), declared in
 *                  ORBmatcher.h.
 */

#include "ORBmatcher.h"

#include <limits.h>

#include <opencv2/core/core.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include <cstdint>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

ORBmatcherStatus ORBmatcher::fuse(KeyFrame *p_keyframe_inout,
                                  const std::vector<MapPoint *> &mapPoints_in,
                                  int                           &fusedCount_out,
                                  const float                    threshold_in,
                                  const bool                     right_in)
{
    camera_models::geometriccamera::GeometricCamera *p_camera;
    Sophus::SE3f                                     poseWorldToCamera;
    Eigen::Vector3f                                  cameraCenter_World;

    if (right_in)
    {
        Sophus::SE3<float> keyframeRightPose{};
        if (p_keyframe_inout->getRightPose(keyframeRightPose) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRightPose returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        poseWorldToCamera = keyframeRightPose;
        Eigen::Vector3f keyframeRightCameraCenter{};
        if (p_keyframe_inout->getRightCameraCenter(keyframeRightCameraCenter) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRightCameraCenter returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        cameraCenter_World = keyframeRightCameraCenter;
        p_camera           = p_keyframe_inout->p_camera2;
    }
    else
    {
        Sophus::SE3f keyframePose{};
        if (p_keyframe_inout->getPose(keyframePose) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        poseWorldToCamera = keyframePose;
        Eigen::Vector3f keyframeCameraCenter{};
        if (p_keyframe_inout->getCameraCenter(keyframeCameraCenter) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCameraCenter returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        cameraCenter_World = keyframeCameraCenter;
        p_camera           = p_keyframe_inout->p_camera;
    }

    int          fusedCount    = 0;
    const float &bf            = p_keyframe_inout->mbf;
    const int    mapPointCount = mapPoints_in.size();

    // For debbuging
    int notMapPointCount = 0, badCount = 0, isinKeyFrameCount = 0,
        negdepthCount = 0, notinimCount = 0, distanceCount = 0, normalCount = 0,
        notidxCount = 0, thcheckCount = 0;
    for (int mapPointIndex = 0; mapPointIndex < mapPointCount; mapPointIndex++)
    {
        MapPoint *p_mapPoint = mapPoints_in[mapPointIndex];

        if (!p_mapPoint)
        {
            notMapPointCount++;
            continue;
        }

        bool mapPointIsBad{};
        if (p_mapPoint->isBad(mapPointIsBad) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (mapPointIsBad)
        {
            badCount++;
            continue;
        }
        else
        {
            bool mapPointIsInKeyFrame{};
            if (p_mapPoint->isInKeyFrame(p_keyframe_inout,
                                         mapPointIsInKeyFrame) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isInKeyFrame returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (mapPointIsInKeyFrame)
            {
                isinKeyFrameCount++;
                continue;
            }
        }

        Eigen::Vector3f p3Dw{};
        if (p_mapPoint->getWorldPos(p3Dw) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f p3Dc = poseWorldToCamera * p3Dw;

        // Depth must be positive
        if (p3Dc(2) < 0.0f)
        {
            negdepthCount++;
            continue;
        }

        const float invz = 1 / p3Dc(2);

        const Eigen::Vector2f uv = p_camera->project(p3Dc);

        // Point must be inside the image
        bool keyframeIsInImage{};
        if (p_keyframe_inout->isInImage(uv(0), uv(1), keyframeIsInImage) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isInImage returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!keyframeIsInImage)
        {
            notinimCount++;
            continue;
        }

        const float ur = uv(0) - bf * invz;

        float maximumDistance{};
        if (p_mapPoint->getMaxDistanceInvariance(maximumDistance) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getMaxDistanceInvariance returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        float minimumDistance{};
        if (p_mapPoint->getMinDistanceInvariance(minimumDistance) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getMinDistanceInvariance returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        Eigen::Vector3f PO         = p3Dw - cameraCenter_World;
        const float     distance3d = PO.norm();

        // Depth must be inside the scale pyramid of the image
        if (distance3d < minimumDistance || distance3d > maximumDistance)
        {
            distanceCount++;
            continue;
        }

        // Viewing angle must be less than 60 deg
        Eigen::Vector3f Pn{};
        if (p_mapPoint->getNormal(Pn) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getNormal returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        if (PO.dot(Pn) < 0.5 * distance3d)
        {
            normalCount++;
            continue;
        }

        int predictedLevelCount{};
        if (p_mapPoint->predictScale(distance3d,
                                     p_keyframe_inout,
                                     predictedLevelCount) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: predictScale returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        // Search in a radius
        const float radius =
            threshold_in * p_keyframe_inout->scaleFactors[predictedLevelCount];

        std::vector<size_t> indices{};
        if (p_keyframe_inout
                ->getFeaturesInArea(uv(0), uv(1), radius, indices, right_in) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getFeaturesInArea returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (indices.empty())
        {
            notidxCount++;
            continue;
        }

        // Match to the most similar keypoint in the radius

        cv::Mat mapPointDescriptor{};
        if (p_mapPoint->getDescriptor(mapPointDescriptor) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getDescriptor returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        int bestDistance = 256;
        int bestIndex    = -1;
        for (std::vector<size_t>::const_iterator vit  = indices.begin(),
                                                 vend = indices.end();
             vit != vend;
             vit++)
        {
            size_t              featureIndex = *vit;
            const cv::KeyPoint &keyPoint =
                (p_keyframe_inout->leftKeyPointCount == -1)
                    ? p_keyframe_inout->keyPointsUndistorted[featureIndex]
                : (!right_in) ? p_keyframe_inout->keyPoints[featureIndex]
                              : p_keyframe_inout->keyPointsRight[featureIndex];

            const int &keyPointLevel = keyPoint.octave;

            if (keyPointLevel < predictedLevelCount - 1 ||
                keyPointLevel > predictedLevelCount)
                continue;

            if (p_keyframe_inout->uRight[featureIndex] >= 0)
            {
                // Check reprojection error in stereo
                const float &kpx = keyPoint.pt.x;
                const float &kpy = keyPoint.pt.y;
                const float &kpr = p_keyframe_inout->uRight[featureIndex];
                const float  ex  = uv(0) - kpx;
                const float  ey  = uv(1) - kpy;
                const float  er  = ur - kpr;
                const float  e2  = ex * ex + ey * ey + er * er;

                if (e2 * p_keyframe_inout->invLevelSigmaSquared[keyPointLevel] >
                    7.8)
                    continue;
            }
            else
            {
                const float &kpx = keyPoint.pt.x;
                const float &kpy = keyPoint.pt.y;
                const float  ex  = uv(0) - kpx;
                const float  ey  = uv(1) - kpy;
                const float  e2  = ex * ex + ey * ey;

                if (e2 * p_keyframe_inout->invLevelSigmaSquared[keyPointLevel] >
                    5.99)
                    continue;
            }

            if (right_in)
                featureIndex += p_keyframe_inout->leftKeyPointCount;

            const cv::Mat &keyFrameDescriptor =
                p_keyframe_inout->descriptors.row(featureIndex);

            int distance{};
            if (computeDescriptorDistance(mapPointDescriptor,
                                          keyFrameDescriptor,
                                          distance) !=
                ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: computeDescriptorDistance returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            if (distance < bestDistance)
            {
                bestDistance = distance;
                bestIndex    = featureIndex;
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        if (bestDistance <= TH_LOW)
        {
            MapPoint *p_keyFrameMapPoint = nullptr;
            if (p_keyframe_inout->getMapPoint(bestIndex, p_keyFrameMapPoint) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMapPoint returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_keyFrameMapPoint)
            {
                bool keyFrameMapPointIsBad{};
                if (p_keyFrameMapPoint->isBad(keyFrameMapPointIsBad) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (!keyFrameMapPointIsBad)
                {
                    int keyFrameMapPointObservationCount{};
                    if (p_keyFrameMapPoint->getObservationCount(
                            keyFrameMapPointObservationCount) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getObservationCount returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    int mapPointObservationCount{};
                    if (p_mapPoint->getObservationCount(
                            mapPointObservationCount) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getObservationCount returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (keyFrameMapPointObservationCount >
                        mapPointObservationCount)
                    {
                        if (p_mapPoint->replace(p_keyFrameMapPoint) !=
                            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: replace returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                    }
                    else
                    {
                        if (p_keyFrameMapPoint->replace(p_mapPoint) !=
                            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: replace returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                    }
                }
            }
            else
            {
                if (p_mapPoint->addObservation(p_keyframe_inout, bestIndex) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: addObservation returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_keyframe_inout->addMapPoint(p_mapPoint, bestIndex) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: addMapPoint returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
            }
            fusedCount++;
        }
        else
            thcheckCount++;
    }

    fusedCount_out = fusedCount;
    return ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
