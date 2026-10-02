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
 * @file            searchForTriangulation.cc
 *
 * @brief           Implements ORBmatcher::searchForTriangulation(), declared in
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

ORBmatcherStatus ORBmatcher::searchForTriangulation(
    KeyFrame                               *p_keyframe1_in,
    KeyFrame                               *p_keyframe2_in,
    std::vector<std::pair<size_t, size_t>> &matchedPairs_out,
    const bool                              stereoOnly_in,
    int                                    &forTriangulation_out,
    const bool                              coarse_in)
{
    const DBoW2::FeatureVector &featureVector1 = p_keyframe1_in->featureVector;
    const DBoW2::FeatureVector &featureVector2 = p_keyframe2_in->featureVector;

    // Compute epipole in second image
    Sophus::SE3f T1w{};
    if (p_keyframe1_in->getPose(T1w) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Sophus::SE3f T2w{};
    if (p_keyframe2_in->getPose(T2w) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Sophus::SE3f Tw2{};
    if (p_keyframe2_in->getPoseInverse(Tw2) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPoseInverse returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    } // for convenience
    Eigen::Vector3f Cw{};
    if (p_keyframe1_in->getCameraCenter(Cw) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCameraCenter returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector3f C2 = T2w * Cw;

    Eigen::Vector2f ep = p_keyframe2_in->p_camera->project(C2);
    Sophus::SE3f    T12;
    Sophus::SE3f    relativePose_leftCamera2ToLeftCamera1,
        relativePose_rightCamera2ToLeftCamera1,
        relativePose_leftCamera2ToRightCamera1,
        relativePose_rightCamera2ToRightCamera1;
    Eigen::Matrix3f R12; // for fastest computation
    Eigen::Vector3f t12; // for fastest computation

    camera_models::geometriccamera::GeometricCamera
        *p_camera1 = p_keyframe1_in->p_camera,
        *p_camera2 = p_keyframe2_in->p_camera;

    if (!p_keyframe1_in->p_camera2 && !p_keyframe2_in->p_camera2)
    {
        T12 = T1w * Tw2;
        R12 = T12.rotationMatrix();
        t12 = T12.translation();
    }
    else
    {
        Sophus::SE3f Tr1w{};
        if (p_keyframe1_in->getRightPose(Tr1w) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRightPose returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        Sophus::SE3f rightCameraPose_rightCamera2ToWorld{};
        if (p_keyframe2_in->getRightPoseInverse(
                rightCameraPose_rightCamera2ToWorld) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRightPoseInverse returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        relativePose_leftCamera2ToLeftCamera1 = T1w * Tw2;
        relativePose_rightCamera2ToLeftCamera1 =
            T1w * rightCameraPose_rightCamera2ToWorld;
        relativePose_leftCamera2ToRightCamera1 = Tr1w * Tw2;
        relativePose_rightCamera2ToRightCamera1 =
            Tr1w * rightCameraPose_rightCamera2ToWorld;
    }

    Eigen::Matrix3f relativeRotation_leftCamera2ToLeftCamera1 =
                        relativePose_leftCamera2ToLeftCamera1.rotationMatrix(),
                    relativeRotation_rightCamera2ToLeftCamera1 =
                        relativePose_rightCamera2ToLeftCamera1.rotationMatrix(),
                    relativeRotation_leftCamera2ToRightCamera1 =
                        relativePose_leftCamera2ToRightCamera1.rotationMatrix(),
                    relativeRotation_rightCamera2ToRightCamera1 =
                        relativePose_rightCamera2ToRightCamera1
                            .rotationMatrix();
    Eigen::Vector3f relativeTranslation_leftCamera2ToLeftCamera1 =
                        relativePose_leftCamera2ToLeftCamera1.translation(),
                    relativeTranslation_rightCamera2ToLeftCamera1 =
                        relativePose_rightCamera2ToLeftCamera1.translation(),
                    relativeTranslation_leftCamera2ToRightCamera1 =
                        relativePose_leftCamera2ToRightCamera1.translation(),
                    relativeTranslation_rightCamera2ToRightCamera1 =
                        relativePose_rightCamera2ToRightCamera1.translation();

    // Find matches between not tracked keypoints
    // Matching speed-up by ORB Vocabulary
    // Compare only ORB that share the same node
    int               nmatches = 0;
    std::vector<bool> matched2Flags(p_keyframe2_in->keyPointCount, false);
    std::vector<int>  matches12(p_keyframe1_in->keyPointCount, -1);

    std::vector<int> rotHist[HISTO_LENGTH];
    for (int histogramBinIndex = 0; histogramBinIndex < HISTO_LENGTH;
         histogramBinIndex++)
        rotHist[histogramBinIndex].reserve(500);

    const float factor = 1.0f / HISTO_LENGTH;

    DBoW2::FeatureVector::const_iterator firstFeatureIt =
        featureVector1.begin();
    DBoW2::FeatureVector::const_iterator secondFeatureIt =
        featureVector2.begin();
    DBoW2::FeatureVector::const_iterator firstFeatureEnd = featureVector1.end();
    DBoW2::FeatureVector::const_iterator secondFeatureEnd =
        featureVector2.end();

    while (firstFeatureIt != firstFeatureEnd &&
           secondFeatureIt != secondFeatureEnd)
    {
        if (firstFeatureIt->first == secondFeatureIt->first)
        {
            for (size_t i1 = 0, iend1 = firstFeatureIt->second.size();
                 i1 < iend1;
                 i1++)
            {
                const size_t index1 = firstFeatureIt->second[i1];

                MapPoint *p_mapPoint1 = nullptr;
                if (p_keyframe1_in->getMapPoint(index1, p_mapPoint1) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMapPoint returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }

                // If there is already a MapPoint skip
                if (p_mapPoint1)
                {
                    continue;
                }

                const bool isStereo1 = (!p_keyframe1_in->p_camera2 &&
                                        p_keyframe1_in->uRight[index1] >= 0);

                if (stereoOnly_in)
                    if (!isStereo1)
                        continue;

                const cv::KeyPoint &keyPoint1 =
                    (p_keyframe1_in->leftKeyPointCount == -1)
                        ? p_keyframe1_in->keyPointsUndistorted[index1]
                    : (index1 <
                       static_cast<size_t>(p_keyframe1_in->leftKeyPointCount))
                        ? p_keyframe1_in->keyPoints[index1]
                        : p_keyframe1_in->keyPointsRight
                              [index1 - p_keyframe1_in->leftKeyPointCount];

                const bool isRightCamera1 =
                    (p_keyframe1_in->leftKeyPointCount == -1 ||
                     index1 <
                         static_cast<size_t>(p_keyframe1_in->leftKeyPointCount))
                        ? false
                        : true;

                const cv::Mat &d1 = p_keyframe1_in->descriptors.row(index1);

                int bestDistance = TH_LOW;
                int bestIndex2   = -1;

                for (size_t i2 = 0, iend2 = secondFeatureIt->second.size();
                     i2 < iend2;
                     i2++)
                {
                    size_t index2 = secondFeatureIt->second[i2];

                    MapPoint *p_mapPoint2 = nullptr;
                    if (p_keyframe2_in->getMapPoint(index2, p_mapPoint2) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getMapPoint returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }

                    // If we have already matched or there is a MapPoint skip
                    if (matched2Flags[index2] || p_mapPoint2)
                        continue;

                    const bool isStereo2 =
                        (!p_keyframe2_in->p_camera2 &&
                         p_keyframe2_in->uRight[index2] >= 0);

                    if (stereoOnly_in)
                        if (!isStereo2)
                            continue;

                    const cv::Mat &d2 = p_keyframe2_in->descriptors.row(index2);

                    int distance{};
                    if (computeDescriptorDistance(d1, d2, distance) !=
                        ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: computeDescriptorDistance returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }

                    if (distance > TH_LOW || distance > bestDistance)
                        continue;

                    const cv::KeyPoint &keyPoint2 =
                        (p_keyframe2_in->leftKeyPointCount == -1)
                            ? p_keyframe2_in->keyPointsUndistorted[index2]
                        : (index2 < static_cast<size_t>(
                                        p_keyframe2_in->leftKeyPointCount))
                            ? p_keyframe2_in->keyPoints[index2]
                            : p_keyframe2_in->keyPointsRight
                                  [index2 - p_keyframe2_in->leftKeyPointCount];
                    const bool isRightCamera2 =
                        (p_keyframe2_in->leftKeyPointCount == -1 ||
                         index2 < static_cast<size_t>(
                                      p_keyframe2_in->leftKeyPointCount))
                            ? false
                            : true;

                    if (!isStereo1 && !isStereo2 && !p_keyframe1_in->p_camera2)
                    {
                        const float distex = ep(0) - keyPoint2.pt.x;
                        const float distey = ep(1) - keyPoint2.pt.y;
                        if (distex * distex + distey * distey <
                            100 *
                                p_keyframe2_in->scaleFactors[keyPoint2.octave])
                        {
                            continue;
                        }
                    }

                    if (p_keyframe1_in->p_camera2 && p_keyframe2_in->p_camera2)
                    {
                        if (isRightCamera1 && isRightCamera2)
                        {
                            R12 = relativeRotation_rightCamera2ToRightCamera1;
                            t12 =
                                relativeTranslation_rightCamera2ToRightCamera1;
                            T12 = relativePose_rightCamera2ToRightCamera1;

                            p_camera1 = p_keyframe1_in->p_camera2;
                            p_camera2 = p_keyframe2_in->p_camera2;
                        }
                        else if (isRightCamera1 && !isRightCamera2)
                        {
                            R12 = relativeRotation_leftCamera2ToRightCamera1;
                            t12 = relativeTranslation_leftCamera2ToRightCamera1;
                            T12 = relativePose_leftCamera2ToRightCamera1;

                            p_camera1 = p_keyframe1_in->p_camera2;
                            p_camera2 = p_keyframe2_in->p_camera;
                        }
                        else if (!isRightCamera1 && isRightCamera2)
                        {
                            R12 = relativeRotation_rightCamera2ToLeftCamera1;
                            t12 = relativeTranslation_rightCamera2ToLeftCamera1;
                            T12 = relativePose_rightCamera2ToLeftCamera1;

                            p_camera1 = p_keyframe1_in->p_camera;
                            p_camera2 = p_keyframe2_in->p_camera2;
                        }
                        else
                        {
                            R12 = relativeRotation_leftCamera2ToLeftCamera1;
                            t12 = relativeTranslation_leftCamera2ToLeftCamera1;
                            T12 = relativePose_leftCamera2ToLeftCamera1;

                            p_camera1 = p_keyframe1_in->p_camera;
                            p_camera2 = p_keyframe2_in->p_camera;
                        }
                    }

                    if (coarse_in ||
                        p_camera1->epipolarConstrain(
                            p_camera2,
                            keyPoint1,
                            keyPoint2,
                            R12,
                            t12,
                            p_keyframe1_in->levelSigmaSquared[keyPoint1.octave],
                            p_keyframe2_in->levelSigmaSquared
                                [keyPoint2.octave])) // MODIFICATION_2
                    {
                        bestIndex2   = index2;
                        bestDistance = distance;
                    }
                }

                if (bestIndex2 >= 0)
                {
                    const cv::KeyPoint &keyPoint2 =
                        (p_keyframe2_in->leftKeyPointCount == -1)
                            ? p_keyframe2_in->keyPointsUndistorted[bestIndex2]
                        : (bestIndex2 < p_keyframe2_in->leftKeyPointCount)
                            ? p_keyframe2_in->keyPoints[bestIndex2]
                            : p_keyframe2_in->keyPointsRight
                                  [bestIndex2 -
                                   p_keyframe2_in->leftKeyPointCount];
                    matches12[index1] = bestIndex2;
                    nmatches++;

                    if (shouldCheckOrientation)
                    {
                        float rot = keyPoint1.angle - keyPoint2.angle;
                        if (rot < 0.0)
                            rot += 360.0f;
                        int bin = std::round(rot * factor);
                        if (bin == HISTO_LENGTH)
                            bin = 0;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(index1);
                    }
                }
            }

            firstFeatureIt++;
            secondFeatureIt++;
        }
        else if (firstFeatureIt->first < secondFeatureIt->first)
        {
            firstFeatureIt = featureVector1.lower_bound(secondFeatureIt->first);
        }
        else
        {
            secondFeatureIt = featureVector2.lower_bound(firstFeatureIt->first);
        }
    }

    if (shouldCheckOrientation)
    {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        if (computeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3) !=
            ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: computeThreeMaxima returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        for (int histogramBinIndex = 0; histogramBinIndex < HISTO_LENGTH;
             histogramBinIndex++)
        {
            if (histogramBinIndex == ind1 || histogramBinIndex == ind2 ||
                histogramBinIndex == ind3)
                continue;
            for (size_t binEntryIndex = 0,
                        jend          = rotHist[histogramBinIndex].size();
                 binEntryIndex < jend;
                 binEntryIndex++)
            {
                matches12[rotHist[histogramBinIndex][binEntryIndex]] = -1;
                nmatches--;
            }
        }
    }

    matchedPairs_out.clear();
    matchedPairs_out.reserve(nmatches);

    for (size_t histogramBinIndex = 0, iend = matches12.size();
         histogramBinIndex < iend;
         histogramBinIndex++)
    {
        if (matches12[histogramBinIndex] < 0)
            continue;
        matchedPairs_out.push_back(
            std::make_pair(histogramBinIndex, matches12[histogramBinIndex]));
    }

    forTriangulation_out = nmatches;
    return ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
