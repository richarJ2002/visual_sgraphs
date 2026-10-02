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
 * @file            refine.cc
 *
 * @brief           Implements MLPnPsolver::refine(), declared in MLPnPsolver.h.
 */

/******************************************************************************
 * Author:   Steffen Urban                                              *
 * Contact:  urbste@gmail.com                                          *
 * License:  Copyright (c) 2016 Steffen Urban, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 ******************************************************************************/

#include "MLPnPsolver.h"

#include <Eigen/Sparse>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

MLPnPsolverStatus MLPnPsolver::refine(bool &isRefined_out)
{
    std::vector<int> indices;
    indices.reserve(bestInlierFlags.size());

    for (size_t bestInlierFlagIndex = 0;
         bestInlierFlagIndex < bestInlierFlags.size();
         bestInlierFlagIndex++)
    {
        if (bestInlierFlags[bestInlierFlagIndex])
        {
            indices.push_back(bestInlierFlagIndex);
        }
    }

    // Bearing vectors and 3D points used for this ransac iteration
    BearingVectors   bearingVecs;
    Points3          p3DS;
    std::vector<int> indexes;

    for (size_t bestInlierFlagIndex = 0; bestInlierFlagIndex < indices.size();
         bestInlierFlagIndex++)
    {
        int featureIndex = indices[bestInlierFlagIndex];

        bearingVecs.push_back(bearingVectors[featureIndex]);
        p3DS.push_back(points3Dw[featureIndex]);
        indexes.push_back(bestInlierFlagIndex);
    }

    // By the moment, we are using MLPnP without covariance info
    Covariance3Matrices covs(1);

    // Result
    TransformationMatrix result;

    // Compute camera pose
    if (computePose(bearingVecs, p3DS, covs, indexes, result) !=
        MLPnPsolverStatus::MLPN_PSOLVER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computePose returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    // Check inliers
    if (checkInliers() != MLPnPsolverStatus::MLPN_PSOLVER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: checkInliers returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    refinedInlierCount = inlierCount;
    refinedInlierFlags = inlierFlags;

    if (inlierCount > ransacMinInliers)
    {
        cv::Mat rotation_worldToCamera(3, 3, CV_64F, mRi);
        cv::Mat translation_worldToCamera(3, 1, CV_64F, mti);
        rotation_worldToCamera.convertTo(rotation_worldToCamera, CV_32F);
        translation_worldToCamera.convertTo(translation_worldToCamera, CV_32F);
        mRefinedTcw.setIdentity();

        Eigen::Matrix<float, 3, 3> matrix3f{};
        if (utils::converter::Converter::toMatrix3f(rotation_worldToCamera,
                                                    matrix3f) !=
            utils::converter::ConverterStatus::CONVERTER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: toMatrix3f returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        mRefinedTcw.block<3, 3>(0, 0) = matrix3f;
        Eigen::Matrix<float, 3, 1> vector3f{};
        if (utils::converter::Converter::toVector3f(translation_worldToCamera,
                                                    vector3f) !=
            utils::converter::ConverterStatus::CONVERTER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: toVector3f returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        mRefinedTcw.block<3, 1>(0, 3) = vector3f;

        Eigen::Matrix<double, 3, 3, Eigen::RowMajor>
                        rotationEigen_worldToCamera(mRi[0]);
        Eigen::Vector3d translationEigen_worldToCamera(mti);

        isRefined_out = true;
        return MLPnPsolverStatus::MLPN_PSOLVER_STATUS_SUCCESS;
    }
    isRefined_out = false;
    return MLPnPsolverStatus::MLPN_PSOLVER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
