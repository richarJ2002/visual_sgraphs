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
 * @file            reconstruct.cc
 *
 * @brief           Implements TwoViewReconstruction::reconstruct(), declared in
 *                  TwoViewReconstruction.h.
 */

#include "TwoViewReconstruction.h"

#include "GeometricTools.h"
#include "Utils/Converter/objects/Converter.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

#include <rclcpp/logging.hpp>
#include <thread>

namespace vs_graphs
{
namespace core
{

TwoViewReconstructionStatus TwoViewReconstruction::reconstruct(
    const std::vector<cv::KeyPoint> &keys1_in,
    const std::vector<cv::KeyPoint> &keys2_in,
    const std::vector<int>          &matches12_in,
    Sophus::SE3f                    &T21_inout,
    std::vector<cv::Point3f>        &vP3D_inout,
    std::vector<bool>               &triangulatedFlags_inout,
    bool                            &isReconstructed_out)
{
    keys1.clear();
    keys2.clear();

    keys1 = keys1_in;
    keys2 = keys2_in;

    // Fill structures with current keypoints and matches with reference frame
    // Reference Frame: 1, Current Frame: 2
    matches12.clear();
    matches12.reserve(keys2.size());
    matchedFlags1.resize(keys1.size());
    for (size_t matchIndex = 0, iend = matches12_in.size(); matchIndex < iend;
         matchIndex++)
    {
        if (matches12_in[matchIndex] >= 0)
        {
            matches12.push_back(
                std::make_pair(matchIndex, matches12_in[matchIndex]));
            matchedFlags1[matchIndex] = true;
        }
        else
            matchedFlags1[matchIndex] = false;
    }

    const int N = matches12.size();

    // Indices for minimum set selection
    std::vector<size_t> allIndices;
    allIndices.reserve(N);
    std::vector<size_t> availableIndices;

    for (int matchIndex = 0; matchIndex < N; matchIndex++)
    {
        allIndices.push_back(matchIndex);
    }

    // Generate sets of 8 points for each RANSAC iteration
    sets = std::vector<std::vector<size_t>>(maxIterations,
                                            std::vector<size_t>(8, 0));

    DUtils::Random::SeedRandOnce(0);

    for (int iterationIndex = 0; iterationIndex < maxIterations;
         iterationIndex++)
    {
        availableIndices = allIndices;

        // Select a minimum set
        for (size_t setPointIndex = 0; setPointIndex < 8; setPointIndex++)
        {
            int randi =
                DUtils::Random::RandomInt(0, availableIndices.size() - 1);
            int sampledIndex = availableIndices[randi];

            sets[iterationIndex][setPointIndex] = sampledIndex;

            availableIndices[randi] = availableIndices.back();
            availableIndices.pop_back();
        }
    }

    // Launch threads to compute in parallel a fundamental matrix and a
    // homography
    std::vector<bool> matchesInliersHFlags, matchesInliersFFlags;
    float             SH, SF;
    Eigen::Matrix3f   H, F;

    std::thread threadH(&TwoViewReconstruction::findHomography,
                        this,
                        std::ref(matchesInliersHFlags),
                        std::ref(SH),
                        std::ref(H));
    std::thread threadF(&TwoViewReconstruction::findFundamental,
                        this,
                        std::ref(matchesInliersFFlags),
                        std::ref(SF),
                        std::ref(F));

    // Wait until both threads have finished
    threadH.join();
    threadF.join();

    // Compute ratio of scores
    if (SH + SF == 0.f)
    {
        isReconstructed_out = false;
        return TwoViewReconstructionStatus::
            TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS;
    }
    float RH = SH / (SH + SF);

    float minimumParallax = 1.0;

    // Try to reconstruct from homography or fundamental depending on the ratio
    // (0.40-0.45)
    if (RH > 0.50) // if(RH>0.40)
    {
        // cout << "Initialization from Homography" << endl;
        bool isReconstructed{};
        if (reconstructH(matchesInliersHFlags,
                         H,
                         calibrationMatrix,
                         T21_inout,
                         vP3D_inout,
                         triangulatedFlags_inout,
                         minimumParallax,
                         50,
                         isReconstructed) !=
            TwoViewReconstructionStatus::TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: reconstructH returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        isReconstructed_out = isReconstructed;
        return TwoViewReconstructionStatus::
            TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS;
    }
    else // if(pF_HF>0.6)
    {
        // cout << "Initialization from Fundamental" << endl;
        bool isReconstructed2{};
        if (reconstructF(matchesInliersFFlags,
                         F,
                         calibrationMatrix,
                         T21_inout,
                         vP3D_inout,
                         triangulatedFlags_inout,
                         minimumParallax,
                         50,
                         isReconstructed2) !=
            TwoViewReconstructionStatus::TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: reconstructF returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        isReconstructed_out = isReconstructed2;
        return TwoViewReconstructionStatus::
            TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS;
    }
}

} // namespace core
} // namespace vs_graphs
