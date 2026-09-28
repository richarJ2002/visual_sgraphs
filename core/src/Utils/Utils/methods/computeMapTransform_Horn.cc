/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            computeMapTransform_Horn.cc
 *
 * @brief           Implements Utils::computeMapTransform_Horn(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

#include <iostream>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus Utils::computeMapTransform_Horn(
    const std::vector<Eigen::Vector3d> &normalsA_in,
    const std::vector<Eigen::Vector3d> &centroidsA_in,
    const std::vector<Eigen::Vector3d> &normalsB_in,
    const std::vector<Eigen::Vector3d> &centroidsB_in,
    Eigen::Isometry3d                  &mapTransform_Horn_out)
{
    if (normalsA_in.size() != normalsB_in.size() || normalsA_in.size() < 3 ||
        centroidsA_in.size() != normalsA_in.size() ||
        centroidsB_in.size() != normalsB_in.size())
    {
        std::cout << "[MapMerge] computeMapTransform_Horn: insufficient or "
                     "mismatched correspondences."
                  << std::endl;
        mapTransform_Horn_out = Eigen::Isometry3d::Identity();
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    Eigen::Vector3d centroidA = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroidB = Eigen::Vector3d::Zero();

    for (std::size_t index = 0; index < normalsA_in.size(); ++index)
    {
        if (!normalsA_in[index].allFinite() ||
            !normalsB_in[index].allFinite() ||
            !centroidsA_in[index].allFinite() ||
            !centroidsB_in[index].allFinite() ||
            normalsA_in[index].squaredNorm() < 1e-12 ||
            normalsB_in[index].squaredNorm() < 1e-12)
        {
            std::cout << "[MapMerge] computeMapTransform_Horn: invalid "
                         "correspondence data."
                      << std::endl;
            mapTransform_Horn_out = Eigen::Isometry3d::Identity();
            return UtilsStatus::UTILS_STATUS_SUCCESS;
        }

        centroidA += centroidsA_in[index];
        centroidB += centroidsB_in[index];
    }

    centroidA /= static_cast<double>(normalsA_in.size());
    centroidB /= static_cast<double>(normalsB_in.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();

    for (std::size_t index = 0; index < normalsA_in.size(); ++index)
    {
        const Eigen::Vector3d normalA = normalsA_in[index].normalized();
        const Eigen::Vector3d normalB = normalsB_in[index].normalized();
        covariance += normalA * normalB.transpose();
    }

    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance,
                                                Eigen::ComputeFullU |
                                                    Eigen::ComputeFullV);

    Eigen::Matrix3d signCorrection = Eigen::Matrix3d::Identity();
    if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0.0)
    {
        signCorrection(2, 2) = -1.0;
    }

    Eigen::Isometry3d transformBFromA = Eigen::Isometry3d::Identity();
    transformBFromA.linear() =
        svd.matrixV() * signCorrection * svd.matrixU().transpose();
    transformBFromA.translation() =
        centroidB - transformBFromA.linear() * centroidA;

    mapTransform_Horn_out = transformBFromA;
    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
