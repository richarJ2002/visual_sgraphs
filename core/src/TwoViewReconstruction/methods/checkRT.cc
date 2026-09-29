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
    TwoViewReconstruction::checkRT(const Eigen::Matrix3f      &R_in,
                                   const Eigen::Vector3f      &t_in,
                                   const vector<cv::KeyPoint> &keys1_in,
                                   const vector<cv::KeyPoint> &keys2_in,
                                   const vector<Match>        &matches12_in,
                                   vector<bool> &matchesInliersFlags_in,
                                   const Eigen::Matrix3f &K_in,
                                   vector<cv::Point3f>   &vP3D_inout,
                                   float                  threshold2_in,
                                   vector<bool>          &goodFlags_out,
                                   float                 &parallax_out,
                                   int                   &goodPointCount_out)
{
    // Calibration parameters
    const float fx = K_in(0, 0);
    const float fy = K_in(1, 1);
    const float cx = K_in(0, 2);
    const float cy = K_in(1, 2);

    goodFlags_out = vector<bool>(keys1_in.size(), false);
    vP3D_inout.resize(keys1_in.size());

    vector<float> cosParallaxes;
    cosParallaxes.reserve(keys1_in.size());

    // Camera 1 Projection Matrix K[I|0]
    Eigen::Matrix<float, 3, 4> P1;
    P1.setZero();
    P1.block<3, 3>(0, 0) = K_in;

    Eigen::Vector3f O1;
    O1.setZero();

    // Camera 2 Projection Matrix K[R|t]
    Eigen::Matrix<float, 3, 4> P2;
    P2.block<3, 3>(0, 0) = R_in;
    P2.block<3, 1>(0, 3) = t_in;
    P2                   = K_in * P2;

    Eigen::Vector3f O2 = -R_in.transpose() * t_in;

    int goodCount = 0;

    for (size_t matchIndex = 0, iend = matches12_in.size(); matchIndex < iend;
         matchIndex++)
    {
        if (!matchesInliersFlags_in[matchIndex])
            continue;

        const cv::KeyPoint &keyPoint1 =
            keys1_in[matches12_in[matchIndex].first];
        const cv::KeyPoint &keyPoint2 =
            keys2_in[matches12_in[matchIndex].second];

        Eigen::Vector3f p3dC1;
        Eigen::Vector3f homogeneousPoint1(keyPoint1.pt.x, keyPoint1.pt.y, 1);
        Eigen::Vector3f homogeneousPoint2(keyPoint2.pt.x, keyPoint2.pt.y, 1);

        if (GeometricTools::triangulate(homogeneousPoint1,
                                        homogeneousPoint2,
                                        P1,
                                        P2,
                                        p3dC1) !=
            GeometricToolsStatus::GEOMETRIC_TOOLS_STATUS_SUCCESS)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("vs_graphs"),
                "%s: triangulate rejected its input; continuing as before.",
                __func__);
        }

        if (!isfinite(p3dC1(0)) || !isfinite(p3dC1(1)) || !isfinite(p3dC1(2)))
        {
            goodFlags_out[matches12_in[matchIndex].first] = false;
            continue;
        }

        // Check parallax
        Eigen::Vector3f normal1   = p3dC1 - O1;
        float           distance1 = normal1.norm();

        Eigen::Vector3f normal2   = p3dC1 - O2;
        float           distance2 = normal2.norm();

        float cosParallax = normal1.dot(normal2) / (distance1 * distance2);

        // Check depth in front of first camera (only if enough parallax, as
        // "infinite" points can easily go to negative depth)
        if (p3dC1(2) <= 0 && cosParallax < 0.99998)
            continue;

        // Check depth in front of second camera (only if enough parallax, as
        // "infinite" points can easily go to negative depth)
        Eigen::Vector3f p3dC2 = R_in * p3dC1 + t_in;

        if (p3dC2(2) <= 0 && cosParallax < 0.99998)
            continue;

        // Check reprojection error in first image
        float image1X, image1Y;
        float invZ1 = 1.0 / p3dC1(2);
        image1X     = fx * p3dC1(0) * invZ1 + cx;
        image1Y     = fy * p3dC1(1) * invZ1 + cy;

        float squareError1 =
            (image1X - keyPoint1.pt.x) * (image1X - keyPoint1.pt.x) +
            (image1Y - keyPoint1.pt.y) * (image1Y - keyPoint1.pt.y);

        if (squareError1 > threshold2_in)
            continue;

        // Check reprojection error in second image
        float image2X, image2Y;
        float invZ2 = 1.0 / p3dC2(2);
        image2X     = fx * p3dC2(0) * invZ2 + cx;
        image2Y     = fy * p3dC2(1) * invZ2 + cy;

        float squareError2 =
            (image2X - keyPoint2.pt.x) * (image2X - keyPoint2.pt.x) +
            (image2Y - keyPoint2.pt.y) * (image2Y - keyPoint2.pt.y);

        if (squareError2 > threshold2_in)
            continue;

        cosParallaxes.push_back(cosParallax);
        vP3D_inout[matches12_in[matchIndex].first] =
            cv::Point3f(p3dC1(0), p3dC1(1), p3dC1(2));
        goodCount++;

        if (cosParallax < 0.99998)
            goodFlags_out[matches12_in[matchIndex].first] = true;
    }

    if (goodCount > 0)
    {
        sort(cosParallaxes.begin(), cosParallaxes.end());

        size_t parallaxIndex = min(50, int(cosParallaxes.size() - 1));
        parallax_out         = acos(cosParallaxes[parallaxIndex]) * 180 / CV_PI;
    }
    else
        parallax_out = 0;

    goodPointCount_out = goodCount;
    return TwoViewReconstructionStatus::TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
