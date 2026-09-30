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

namespace vs_graphs
{
namespace core
{

TwoViewReconstructionStatus TwoViewReconstruction::reconstructF(
    std::vector<bool>        &matchesInliersFlags_inout,
    Eigen::Matrix3f          &F21_in,
    Eigen::Matrix3f          &K_in,
    Sophus::SE3f             &T21_out,
    std::vector<cv::Point3f> &vP3D_out,
    std::vector<bool>        &triangulatedFlags_out,
    float                     minimumParallax_in,
    int                       minimumTriangulated_in,
    bool                     &isReconstructed_out)
{
    int N = 0;
    for (size_t matchIndex = 0, iend = matchesInliersFlags_inout.size();
         matchIndex < iend;
         matchIndex++)
        if (matchesInliersFlags_inout[matchIndex])
            N++;

    // Compute Essential Matrix from Fundamental Matrix
    Eigen::Matrix3f E21 = K_in.transpose() * F21_in * K_in;

    Eigen::Matrix3f R1, R2;
    Eigen::Vector3f t;

    // Recover the 4 motion hypotheses
    if (decomposeE(E21, R1, R2, t) !=
        TwoViewReconstructionStatus::TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: decomposeE returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    Eigen::Vector3f t1 = t;
    Eigen::Vector3f t2 = -t;

    // Reconstruct with the 4 hyphoteses and check
    std::vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
    std::vector<bool>        triangulated1Flags, triangulated2Flags,
        triangulated3Flags, triangulated4Flags;
    float parallax1, parallax2, parallax3, parallax4;

    int good1Count{};
    if (checkRT(R1,
                t1,
                keys1,
                keys2,
                matches12,
                matchesInliersFlags_inout,
                K_in,
                vP3D1,
                4.0 * sigmaSquared,
                triangulated1Flags,
                parallax1,
                good1Count) !=
        TwoViewReconstructionStatus::TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: checkRT returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    int good2Count{};
    if (checkRT(R2,
                t1,
                keys1,
                keys2,
                matches12,
                matchesInliersFlags_inout,
                K_in,
                vP3D2,
                4.0 * sigmaSquared,
                triangulated2Flags,
                parallax2,
                good2Count) !=
        TwoViewReconstructionStatus::TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: checkRT returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    int good3Count{};
    if (checkRT(R1,
                t2,
                keys1,
                keys2,
                matches12,
                matchesInliersFlags_inout,
                K_in,
                vP3D3,
                4.0 * sigmaSquared,
                triangulated3Flags,
                parallax3,
                good3Count) !=
        TwoViewReconstructionStatus::TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: checkRT returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    int good4Count{};
    if (checkRT(R2,
                t2,
                keys1,
                keys2,
                matches12,
                matchesInliersFlags_inout,
                K_in,
                vP3D4,
                4.0 * sigmaSquared,
                triangulated4Flags,
                parallax4,
                good4Count) !=
        TwoViewReconstructionStatus::TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: checkRT returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    int maximumGood =
        std::max(good1Count,
                 std::max(good2Count, std::max(good3Count, good4Count)));

    int minimumGoodCount =
        std::max(static_cast<int>(0.9 * N), minimumTriangulated_in);

    int nsimilar = 0;
    if (good1Count > 0.7 * maximumGood)
        nsimilar++;
    if (good2Count > 0.7 * maximumGood)
        nsimilar++;
    if (good3Count > 0.7 * maximumGood)
        nsimilar++;
    if (good4Count > 0.7 * maximumGood)
        nsimilar++;

    // If there is not a clear winner or not enough triangulated points reject
    // initialization
    if (maximumGood < minimumGoodCount || nsimilar > 1)
    {
        isReconstructed_out = false;
        return TwoViewReconstructionStatus::
            TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS;
    }

    // If best reconstruction has enough parallax initialize
    if (maximumGood == good1Count)
    {
        if (parallax1 > minimumParallax_in)
        {
            vP3D_out              = vP3D1;
            triangulatedFlags_out = triangulated1Flags;

            T21_out             = Sophus::SE3f(R1, t1);
            isReconstructed_out = true;
            return TwoViewReconstructionStatus::
                TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS;
        }
    }
    else if (maximumGood == good2Count)
    {
        if (parallax2 > minimumParallax_in)
        {
            vP3D_out              = vP3D2;
            triangulatedFlags_out = triangulated2Flags;

            T21_out             = Sophus::SE3f(R2, t1);
            isReconstructed_out = true;
            return TwoViewReconstructionStatus::
                TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS;
        }
    }
    else if (maximumGood == good3Count)
    {
        if (parallax3 > minimumParallax_in)
        {
            vP3D_out              = vP3D3;
            triangulatedFlags_out = triangulated3Flags;

            T21_out             = Sophus::SE3f(R1, t2);
            isReconstructed_out = true;
            return TwoViewReconstructionStatus::
                TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS;
        }
    }
    else if (maximumGood == good4Count)
    {
        if (parallax4 > minimumParallax_in)
        {
            vP3D_out              = vP3D4;
            triangulatedFlags_out = triangulated4Flags;

            T21_out             = Sophus::SE3f(R2, t2);
            isReconstructed_out = true;
            return TwoViewReconstructionStatus::
                TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS;
        }
    }

    isReconstructed_out = false;
    return TwoViewReconstructionStatus::TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
