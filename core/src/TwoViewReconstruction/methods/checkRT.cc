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

#include <thread>

using namespace std;
namespace vs_graphs
{
namespace core
{

int TwoViewReconstruction::checkRT(const Eigen::Matrix3f      &R,
                                   const Eigen::Vector3f      &t,
                                   const vector<cv::KeyPoint> &vKeys1,
                                   const vector<cv::KeyPoint> &vKeys2,
                                   const vector<Match>        &vMatches12,
                                   vector<bool>               &vbMatchesInliers,
                                   const Eigen::Matrix3f      &K,
                                   vector<cv::Point3f>        &vP3D,
                                   float                       th2,
                                   vector<bool>               &vbGood,
                                   float                      &parallax)
{
    // Calibration parameters
    const float fx = K(0, 0);
    const float fy = K(1, 1);
    const float cx = K(0, 2);
    const float cy = K(1, 2);

    vbGood = vector<bool>(vKeys1.size(), false);
    vP3D.resize(vKeys1.size());

    vector<float> vCosParallax;
    vCosParallax.reserve(vKeys1.size());

    // Camera 1 Projection Matrix K[I|0]
    Eigen::Matrix<float, 3, 4> P1;
    P1.setZero();
    P1.block<3, 3>(0, 0) = K;

    Eigen::Vector3f O1;
    O1.setZero();

    // Camera 2 Projection Matrix K[R|t]
    Eigen::Matrix<float, 3, 4> P2;
    P2.block<3, 3>(0, 0) = R;
    P2.block<3, 1>(0, 3) = t;
    P2                   = K * P2;

    Eigen::Vector3f O2 = -R.transpose() * t;

    int nGood = 0;

    for (size_t i = 0, iend = vMatches12.size(); i < iend; i++)
    {
        if (!vbMatchesInliers[i])
            continue;

        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];

        Eigen::Vector3f p3dC1;
        Eigen::Vector3f x_p1(kp1.pt.x, kp1.pt.y, 1);
        Eigen::Vector3f x_p2(kp2.pt.x, kp2.pt.y, 1);

        GeometricTools::triangulate(x_p1, x_p2, P1, P2, p3dC1);

        if (!isfinite(p3dC1(0)) || !isfinite(p3dC1(1)) || !isfinite(p3dC1(2)))
        {
            vbGood[vMatches12[i].first] = false;
            continue;
        }

        // Check parallax
        Eigen::Vector3f normal1 = p3dC1 - O1;
        float           dist1   = normal1.norm();

        Eigen::Vector3f normal2 = p3dC1 - O2;
        float           dist2   = normal2.norm();

        float cosParallax = normal1.dot(normal2) / (dist1 * dist2);

        // Check depth in front of first camera (only if enough parallax, as
        // "infinite" points can easily go to negative depth)
        if (p3dC1(2) <= 0 && cosParallax < 0.99998)
            continue;

        // Check depth in front of second camera (only if enough parallax, as
        // "infinite" points can easily go to negative depth)
        Eigen::Vector3f p3dC2 = R * p3dC1 + t;

        if (p3dC2(2) <= 0 && cosParallax < 0.99998)
            continue;

        // Check reprojection error in first image
        float im1x, im1y;
        float invZ1 = 1.0 / p3dC1(2);
        im1x        = fx * p3dC1(0) * invZ1 + cx;
        im1y        = fy * p3dC1(1) * invZ1 + cy;

        float squareError1 = (im1x - kp1.pt.x) * (im1x - kp1.pt.x) +
                             (im1y - kp1.pt.y) * (im1y - kp1.pt.y);

        if (squareError1 > th2)
            continue;

        // Check reprojection error in second image
        float im2x, im2y;
        float invZ2 = 1.0 / p3dC2(2);
        im2x        = fx * p3dC2(0) * invZ2 + cx;
        im2y        = fy * p3dC2(1) * invZ2 + cy;

        float squareError2 = (im2x - kp2.pt.x) * (im2x - kp2.pt.x) +
                             (im2y - kp2.pt.y) * (im2y - kp2.pt.y);

        if (squareError2 > th2)
            continue;

        vCosParallax.push_back(cosParallax);
        vP3D[vMatches12[i].first] = cv::Point3f(p3dC1(0), p3dC1(1), p3dC1(2));
        nGood++;

        if (cosParallax < 0.99998)
            vbGood[vMatches12[i].first] = true;
    }

    if (nGood > 0)
    {
        sort(vCosParallax.begin(), vCosParallax.end());

        size_t idx = min(50, int(vCosParallax.size() - 1));
        parallax   = acos(vCosParallax[idx]) * 180 / CV_PI;
    }
    else
        parallax = 0;

    return nGood;
}

} // namespace core
} // namespace vs_graphs
