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

#include "ORBmatcher.h"

#include <limits.h>

#include <opencv2/core/core.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include <rclcpp/logging.hpp>
#include <stdint-gcc.h>

namespace vs_graphs
{
namespace core
{

int ORBmatcher::searchForTriangulation(
    KeyFrame                     *pKF1,
    KeyFrame                     *pKF2,
    vector<pair<size_t, size_t>> &vMatchedPairs,
    const bool                    bOnlyStereo,
    const bool                    bCoarse)
{
    const DBoW2::FeatureVector &featureVector1 = pKF1->featureVector;
    const DBoW2::FeatureVector &featureVector2 = pKF2->featureVector;

    // Compute epipole in second image
    Sophus::SE3f T1w{};
    if (pKF1->getPose(T1w) != KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Sophus::SE3f T2w{};
    if (pKF2->getPose(T2w) != KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Sophus::SE3f Tw2{};
    if (pKF2->getPoseInverse(Tw2) != KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPoseInverse returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    } // for convenience
    Eigen::Vector3f Cw{};
    if (pKF1->getCameraCenter(Cw) != KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCameraCenter returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector3f C2 = T2w * Cw;

    Eigen::Vector2f ep = pKF2->p_camera->project(C2);
    Sophus::SE3f    T12;
    Sophus::SE3f    Tll, Tlr, Trl, Trr;
    Eigen::Matrix3f R12; // for fastest computation
    Eigen::Vector3f t12; // for fastest computation

    camera_models::geometriccamera::GeometricCamera *p_camera1 = pKF1->p_camera,
                                                    *p_camera2 = pKF2->p_camera;

    if (!pKF1->p_camera2 && !pKF2->p_camera2)
    {
        T12 = T1w * Tw2;
        R12 = T12.rotationMatrix();
        t12 = T12.translation();
    }
    else
    {
        Sophus::SE3f Tr1w{};
        if (pKF1->getRightPose(Tr1w) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRightPose returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        Sophus::SE3f Twr2{};
        if (pKF2->getRightPoseInverse(Twr2) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRightPoseInverse returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Tll = T1w * Tw2;
        Tlr = T1w * Twr2;
        Trl = Tr1w * Tw2;
        Trr = Tr1w * Twr2;
    }

    Eigen::Matrix3f Rll = Tll.rotationMatrix(), Rlr = Tlr.rotationMatrix(),
                    Rrl = Trl.rotationMatrix(), Rrr = Trr.rotationMatrix();
    Eigen::Vector3f tll = Tll.translation(), tlr = Tlr.translation(),
                    trl = Trl.translation(), trr = Trr.translation();

    // Find matches between not tracked keypoints
    // Matching speed-up by ORB Vocabulary
    // Compare only ORB that share the same node
    int          nmatches = 0;
    vector<bool> matched2Flags(pKF2->keyPointCount, false);
    vector<int>  matches12(pKF1->keyPointCount, -1);

    vector<int> rotHist[HISTO_LENGTH];
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
                if (pKF1->getMapPoint(index1, p_mapPoint1) !=
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

                const bool isStereo1 =
                    (!pKF1->p_camera2 && pKF1->uRight[index1] >= 0);

                if (bOnlyStereo)
                    if (!isStereo1)
                        continue;

                const cv::KeyPoint &keyPoint1 =
                    (pKF1->leftKeyPointCount == -1)
                        ? pKF1->keyPointsUndistorted[index1]
                    : (index1 < static_cast<size_t>(pKF1->leftKeyPointCount))
                        ? pKF1->keyPoints[index1]
                        : pKF1->keyPointsRight[index1 -
                                               pKF1->leftKeyPointCount];

                const bool isRightCamera1 =
                    (pKF1->leftKeyPointCount == -1 ||
                     index1 < static_cast<size_t>(pKF1->leftKeyPointCount))
                        ? false
                        : true;

                const cv::Mat &d1 = pKF1->descriptors.row(index1);

                int bestDistance = TH_LOW;
                int bestIndex2   = -1;

                for (size_t i2 = 0, iend2 = secondFeatureIt->second.size();
                     i2 < iend2;
                     i2++)
                {
                    size_t index2 = secondFeatureIt->second[i2];

                    MapPoint *p_mapPoint2 = nullptr;
                    if (pKF2->getMapPoint(index2, p_mapPoint2) !=
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
                        (!pKF2->p_camera2 && pKF2->uRight[index2] >= 0);

                    if (bOnlyStereo)
                        if (!isStereo2)
                            continue;

                    const cv::Mat &d2 = pKF2->descriptors.row(index2);

                    const int distance = computeDescriptorDistance(d1, d2);

                    if (distance > TH_LOW || distance > bestDistance)
                        continue;

                    const cv::KeyPoint &keyPoint2 =
                        (pKF2->leftKeyPointCount == -1)
                            ? pKF2->keyPointsUndistorted[index2]
                        : (index2 <
                           static_cast<size_t>(pKF2->leftKeyPointCount))
                            ? pKF2->keyPoints[index2]
                            : pKF2->keyPointsRight[index2 -
                                                   pKF2->leftKeyPointCount];
                    const bool isRightCamera2 =
                        (pKF2->leftKeyPointCount == -1 ||
                         index2 < static_cast<size_t>(pKF2->leftKeyPointCount))
                            ? false
                            : true;

                    if (!isStereo1 && !isStereo2 && !pKF1->p_camera2)
                    {
                        const float distex = ep(0) - keyPoint2.pt.x;
                        const float distey = ep(1) - keyPoint2.pt.y;
                        if (distex * distex + distey * distey <
                            100 * pKF2->scaleFactors[keyPoint2.octave])
                        {
                            continue;
                        }
                    }

                    if (pKF1->p_camera2 && pKF2->p_camera2)
                    {
                        if (isRightCamera1 && isRightCamera2)
                        {
                            R12 = Rrr;
                            t12 = trr;
                            T12 = Trr;

                            p_camera1 = pKF1->p_camera2;
                            p_camera2 = pKF2->p_camera2;
                        }
                        else if (isRightCamera1 && !isRightCamera2)
                        {
                            R12 = Rrl;
                            t12 = trl;
                            T12 = Trl;

                            p_camera1 = pKF1->p_camera2;
                            p_camera2 = pKF2->p_camera;
                        }
                        else if (!isRightCamera1 && isRightCamera2)
                        {
                            R12 = Rlr;
                            t12 = tlr;
                            T12 = Tlr;

                            p_camera1 = pKF1->p_camera;
                            p_camera2 = pKF2->p_camera2;
                        }
                        else
                        {
                            R12 = Rll;
                            t12 = tll;
                            T12 = Tll;

                            p_camera1 = pKF1->p_camera;
                            p_camera2 = pKF2->p_camera;
                        }
                    }

                    if (bCoarse ||
                        p_camera1->epipolarConstrain(
                            p_camera2,
                            keyPoint1,
                            keyPoint2,
                            R12,
                            t12,
                            pKF1->levelSigmaSquared[keyPoint1.octave],
                            pKF2->levelSigmaSquared
                                [keyPoint2.octave])) // MODIFICATION_2
                    {
                        bestIndex2   = index2;
                        bestDistance = distance;
                    }
                }

                if (bestIndex2 >= 0)
                {
                    const cv::KeyPoint &keyPoint2 =
                        (pKF2->leftKeyPointCount == -1)
                            ? pKF2->keyPointsUndistorted[bestIndex2]
                        : (bestIndex2 < pKF2->leftKeyPointCount)
                            ? pKF2->keyPoints[bestIndex2]
                            : pKF2->keyPointsRight[bestIndex2 -
                                                   pKF2->leftKeyPointCount];
                    matches12[index1] = bestIndex2;
                    nmatches++;

                    if (shouldCheckOrientation)
                    {
                        float rot = keyPoint1.angle - keyPoint2.angle;
                        if (rot < 0.0)
                            rot += 360.0f;
                        int bin = round(rot * factor);
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

        computeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

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

    vMatchedPairs.clear();
    vMatchedPairs.reserve(nmatches);

    for (size_t histogramBinIndex = 0, iend = matches12.size();
         histogramBinIndex < iend;
         histogramBinIndex++)
    {
        if (matches12[histogramBinIndex] < 0)
            continue;
        vMatchedPairs.push_back(
            make_pair(histogramBinIndex, matches12[histogramBinIndex]));
    }

    return nmatches;
}

} // namespace core
} // namespace vs_graphs
