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
 * @file            iterate.cc
 *
 * @brief           Implements Sim3Solver::iterate() overloads, declared in
 *                  Sim3Solver.h.
 */

#include "d_utils/random/random.h"
#include "keyframe/keyframe.h"
#include "orb_matcher/orb_matcher.h"
#include "sim3_solver/objects/Sim3SolverClass.h"
#include <cmath>
#include <opencv2/core/core.hpp>
#include <rclcpp/logging.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace sim3_solver
{

Sim3SolverStatus Sim3Solver::iterate(int   iterationCount_in,
                                     bool &areIterationsExhausted_out,
                                     std::vector<bool> &inliersFlags_out,
                                     int               &inlierCount_out,
                                     Eigen::Matrix4f   &transform_out)
{
    areIterationsExhausted_out = false;
    inliersFlags_out           = std::vector<bool>(firstMatchCount, false);
    inlierCount_out            = 0;

    if (correspondenceCount < ransacMinInliers)
    {
        areIterationsExhausted_out = true;
        transform_out              = Eigen::Matrix4f::Identity();
        return Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS;
    }

    std::vector<size_t> availableIndices;

    Eigen::Matrix3f P3Dc1i;
    Eigen::Matrix3f P3Dc2i;

    int currentIterationCount = 0;
    while (iterationCount < ransacMaxIterations &&
           currentIterationCount < iterationCount_in)
    {
        currentIterationCount++;
        iterationCount++;

        availableIndices = allIndices;

        /* Get min set of points */
        for (short keyPointIndex = 0; keyPointIndex < 3; ++keyPointIndex)
        {
            int randi = d_utils::random::Random::generateUniformInteger(
                0,
                availableIndices.size() - 1);

            int sampledIndex = availableIndices[randi];

            P3Dc1i.col(keyPointIndex) = points3Dc1[sampledIndex];
            P3Dc2i.col(keyPointIndex) = points3Dc2[sampledIndex];

            availableIndices[randi] = availableIndices.back();
            availableIndices.pop_back();
        }

        if (computeSim3(P3Dc1i, P3Dc2i) !=
            Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: computeSim3 returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        if (checkInliers() != Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: checkInliers returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        if (inlierCount >= bestInlierCount)
        {
            bestInlierFlags = inlierFlags;
            bestInlierCount = inlierCount;
            mBestT12        = mT12i;
            bestRotation    = mR12i;
            bestTranslation = mt12i;
            bestScale       = ms12i;

            if (inlierCount > ransacMinInliers)
            {
                inlierCount_out = inlierCount;
                for (int keyPointIndex = 0; keyPointIndex < correspondenceCount;
                     keyPointIndex++)
                    if (inlierFlags[keyPointIndex])
                        inliersFlags_out[indices1[keyPointIndex]] = true;
                transform_out = mBestT12;
                return Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS;
            }
        }
    }

    if (iterationCount >= ransacMaxIterations)
        areIterationsExhausted_out = true;

    transform_out = Eigen::Matrix4f::Identity();
    return Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS;
}

Sim3SolverStatus Sim3Solver::iterate(int   iterationCount_in,
                                     bool &areIterationsExhausted_out,
                                     std::vector<bool> &inliersFlags_out,
                                     int               &inlierCount_out,
                                     bool              &hasConverged_out,
                                     Eigen::Matrix4f   &transform_out)
{
    areIterationsExhausted_out = false;
    hasConverged_out           = false;
    inliersFlags_out           = std::vector<bool>(firstMatchCount, false);
    inlierCount_out            = 0;

    if (correspondenceCount < ransacMinInliers)
    {
        areIterationsExhausted_out = true;
        transform_out              = Eigen::Matrix4f::Identity();
        return Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS;
    }

    std::vector<size_t> availableIndices;

    Eigen::Matrix3f P3Dc1i;
    Eigen::Matrix3f P3Dc2i;

    int currentIterationCount = 0;

    /* Identity unless a round of this call reaches the best inlier count. */
    Eigen::Matrix4f bestSim3 = Eigen::Matrix4f::Identity();

    while (iterationCount < ransacMaxIterations &&
           currentIterationCount < iterationCount_in)
    {
        currentIterationCount++;
        iterationCount++;

        availableIndices = allIndices;

        /* Get min set of points */
        for (short keyPointIndex = 0; keyPointIndex < 3; ++keyPointIndex)
        {
            int randi = d_utils::random::Random::generateUniformInteger(
                0,
                availableIndices.size() - 1);

            int sampledIndex = availableIndices[randi];

            P3Dc1i.col(keyPointIndex) = points3Dc1[sampledIndex];
            P3Dc2i.col(keyPointIndex) = points3Dc2[sampledIndex];

            availableIndices[randi] = availableIndices.back();
            availableIndices.pop_back();
        }

        if (computeSim3(P3Dc1i, P3Dc2i) !=
            Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: computeSim3 returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        if (checkInliers() != Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: checkInliers returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        if (inlierCount >= bestInlierCount)
        {
            bestInlierFlags = inlierFlags;
            bestInlierCount = inlierCount;
            mBestT12        = mT12i;
            bestRotation    = mR12i;
            bestTranslation = mt12i;
            bestScale       = ms12i;

            if (inlierCount > ransacMinInliers)
            {
                inlierCount_out = inlierCount;
                for (int keyPointIndex = 0; keyPointIndex < correspondenceCount;
                     keyPointIndex++)
                    if (inlierFlags[keyPointIndex])
                        inliersFlags_out[indices1[keyPointIndex]] = true;
                hasConverged_out = true;
                transform_out    = mBestT12;
                return Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS;
            }
            else
            {
                bestSim3 = mBestT12;
            }
        }
    }

    if (iterationCount >= ransacMaxIterations)
        areIterationsExhausted_out = true;

    transform_out = bestSim3;
    return Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS;
}

} /* namespace sim3_solver */
} /* namespace core */
} /* namespace vs_graphs */
