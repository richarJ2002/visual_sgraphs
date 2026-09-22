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

bool TwoViewReconstruction::reconstructF(vector<bool>        &vbMatchesInliers,
                                         Eigen::Matrix3f     &F21,
                                         Eigen::Matrix3f     &K,
                                         Sophus::SE3f        &T21,
                                         vector<cv::Point3f> &vP3D,
                                         vector<bool>        &vbTriangulated,
                                         float                minParallax,
                                         int                  minTriangulated)
{
    int N = 0;
    for (size_t i = 0, iend = vbMatchesInliers.size(); i < iend; i++)
        if (vbMatchesInliers[i])
            N++;

    // Compute Essential Matrix from Fundamental Matrix
    Eigen::Matrix3f E21 = K.transpose() * F21 * K;

    Eigen::Matrix3f R1, R2;
    Eigen::Vector3f t;

    // Recover the 4 motion hypotheses
    decomposeE(E21, R1, R2, t);

    Eigen::Vector3f t1 = t;
    Eigen::Vector3f t2 = -t;

    // Reconstruct with the 4 hyphoteses and check
    vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
    vector<bool>        vbTriangulated1, vbTriangulated2, vbTriangulated3,
        vbTriangulated4;
    float parallax1, parallax2, parallax3, parallax4;

    int nGood1 = checkRT(R1,
                         t1,
                         keys1,
                         keys2,
                         matches12,
                         vbMatchesInliers,
                         K,
                         vP3D1,
                         4.0 * sigmaSquared,
                         vbTriangulated1,
                         parallax1);
    int nGood2 = checkRT(R2,
                         t1,
                         keys1,
                         keys2,
                         matches12,
                         vbMatchesInliers,
                         K,
                         vP3D2,
                         4.0 * sigmaSquared,
                         vbTriangulated2,
                         parallax2);
    int nGood3 = checkRT(R1,
                         t2,
                         keys1,
                         keys2,
                         matches12,
                         vbMatchesInliers,
                         K,
                         vP3D3,
                         4.0 * sigmaSquared,
                         vbTriangulated3,
                         parallax3);
    int nGood4 = checkRT(R2,
                         t2,
                         keys1,
                         keys2,
                         matches12,
                         vbMatchesInliers,
                         K,
                         vP3D4,
                         4.0 * sigmaSquared,
                         vbTriangulated4,
                         parallax4);

    int maxGood = max(nGood1, max(nGood2, max(nGood3, nGood4)));

    int nMinGood = max(static_cast<int>(0.9 * N), minTriangulated);

    int nsimilar = 0;
    if (nGood1 > 0.7 * maxGood)
        nsimilar++;
    if (nGood2 > 0.7 * maxGood)
        nsimilar++;
    if (nGood3 > 0.7 * maxGood)
        nsimilar++;
    if (nGood4 > 0.7 * maxGood)
        nsimilar++;

    // If there is not a clear winner or not enough triangulated points reject
    // initialization
    if (maxGood < nMinGood || nsimilar > 1)
    {
        return false;
    }

    // If best reconstruction has enough parallax initialize
    if (maxGood == nGood1)
    {
        if (parallax1 > minParallax)
        {
            vP3D           = vP3D1;
            vbTriangulated = vbTriangulated1;

            T21 = Sophus::SE3f(R1, t1);
            return true;
        }
    }
    else if (maxGood == nGood2)
    {
        if (parallax2 > minParallax)
        {
            vP3D           = vP3D2;
            vbTriangulated = vbTriangulated2;

            T21 = Sophus::SE3f(R2, t1);
            return true;
        }
    }
    else if (maxGood == nGood3)
    {
        if (parallax3 > minParallax)
        {
            vP3D           = vP3D3;
            vbTriangulated = vbTriangulated3;

            T21 = Sophus::SE3f(R1, t2);
            return true;
        }
    }
    else if (maxGood == nGood4)
    {
        if (parallax4 > minParallax)
        {
            vP3D           = vP3D4;
            vbTriangulated = vbTriangulated4;

            T21 = Sophus::SE3f(R2, t2);
            return true;
        }
    }

    return false;
}

} // namespace core
} // namespace vs_graphs
