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

#include "TwoViewReconstruction.h"

#include "GeometricTools.h"
#include "Utils/Converter/objects/Converter.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

#include <rclcpp/logging.hpp>
#include <thread>

using namespace std;
namespace vs_graphs
{
namespace core
{

TwoViewReconstructionStatus
    TwoViewReconstruction::findHomography(vector<bool> &matchesInliersFlags_out,
                                          float        &score_inout,
                                          Eigen::Matrix3f &H21_out)
{
    // Number of putative matches
    const int N = matches12.size();

    // Normalize coordinates
    vector<cv::Point2f> normalizedPoints1, normalizedPoints2;
    Eigen::Matrix3f     T1, T2;
    if (normalize(keys1, normalizedPoints1, T1) !=
        TwoViewReconstructionStatus::TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: normalize returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (normalize(keys2, normalizedPoints2, T2) !=
        TwoViewReconstructionStatus::TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: normalize returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Matrix3f T2inv = T2.inverse();

    // Best Results variables
    score_inout             = 0.0;
    matchesInliersFlags_out = vector<bool>(N, false);

    // Iteration variables
    vector<cv::Point2f> sampledPoints1(8);
    vector<cv::Point2f> sampledPoints2(8);
    Eigen::Matrix3f     H21i, H12i;
    vector<bool>        currentInliersFlags(N, false);
    float               currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for (int iterationIndex = 0; iterationIndex < maxIterations;
         iterationIndex++)
    {
        // Select a minimum set
        for (size_t setPointIndex = 0; setPointIndex < 8; setPointIndex++)
        {
            int matchIndex = sets[iterationIndex][setPointIndex];

            sampledPoints1[setPointIndex] =
                normalizedPoints1[matches12[matchIndex].first];
            sampledPoints2[setPointIndex] =
                normalizedPoints2[matches12[matchIndex].second];
        }

        Eigen::Matrix3f Hn{};
        if (computeH21(sampledPoints1, sampledPoints2, Hn) !=
            TwoViewReconstructionStatus::TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: computeH21 returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        H21i = T2inv * Hn * T1;
        H12i = H21i.inverse();

        float score2{};
        if (checkHomography(H21i, H12i, currentInliersFlags, sigma, score2) !=
            TwoViewReconstructionStatus::TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: checkHomography returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        currentScore = score2;

        if (currentScore > score_inout)
        {
            H21_out                 = H21i;
            matchesInliersFlags_out = currentInliersFlags;
            score_inout             = currentScore;
        }
    }

    return TwoViewReconstructionStatus::TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
