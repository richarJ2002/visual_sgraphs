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

bool TwoViewReconstruction::reconstructH(
    vector<bool>        &matchesInliersFlags_inout,
    Eigen::Matrix3f     &H21_in,
    Eigen::Matrix3f     &K_in,
    Sophus::SE3f        &T21_out,
    vector<cv::Point3f> &vP3D_inout,
    vector<bool>        &triangulatedFlags_out,
    float                minimumParallax_in,
    int                  minimumTriangulated_in)
{
    int N = 0;
    for (size_t i = 0, iend = matchesInliersFlags_inout.size(); i < iend; i++)
        if (matchesInliersFlags_inout[i])
            N++;

    // We recover 8 motion hypotheses using the method of Faugeras et al.
    // Motion and structure from motion in a piecewise planar environment.
    // International Journal of Pattern Recognition and Artificial Intelligence,
    // 1988
    Eigen::Matrix3f invK = K_in.inverse();
    Eigen::Matrix3f A    = invK * H21_in * K_in;

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(A,
                                          Eigen::ComputeFullU |
                                              Eigen::ComputeFullV);
    Eigen::Matrix3f                   U  = svd.matrixU();
    Eigen::Matrix3f                   V  = svd.matrixV();
    Eigen::Matrix3f                   Vt = V.transpose();
    Eigen::Vector3f                   w  = svd.singularValues();

    float s = U.determinant() * Vt.determinant();

    float d1 = w(0);
    float d2 = w(1);
    float d3 = w(2);

    if (d1 / d2 < 1.00001 || d2 / d3 < 1.00001)
    {
        return false;
    }

    vector<Eigen::Matrix3f> vR;
    vector<Eigen::Vector3f> vt, vn;
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);

    // n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
    float aux1 = sqrt((d1 * d1 - d2 * d2) / (d1 * d1 - d3 * d3));
    float aux3 = sqrt((d2 * d2 - d3 * d3) / (d1 * d1 - d3 * d3));
    float x1[] = {aux1, aux1, -aux1, -aux1};
    float x3[] = {aux3, -aux3, aux3, -aux3};

    // case d'=d2
    float auxStheta =
        sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 + d3) * d2);

    float ctheta   = (d2 * d2 + d1 * d3) / ((d1 + d3) * d2);
    float stheta[] = {auxStheta, -auxStheta, -auxStheta, auxStheta};

    for (int i = 0; i < 4; i++)
    {
        Eigen::Matrix3f Rp;
        Rp.setZero();
        Rp(0, 0) = ctheta;
        Rp(0, 2) = -stheta[i];
        Rp(1, 1) = 1.f;
        Rp(2, 0) = stheta[i];
        Rp(2, 2) = ctheta;

        Eigen::Matrix3f R = s * U * Rp * Vt;
        vR.push_back(R);

        Eigen::Vector3f tp;
        tp(0) = x1[i];
        tp(1) = 0;
        tp(2) = -x3[i];
        tp *= d1 - d3;

        Eigen::Vector3f t = U * tp;
        vt.push_back(t / t.norm());

        Eigen::Vector3f np;
        np(0) = x1[i];
        np(1) = 0;
        np(2) = x3[i];

        Eigen::Vector3f n = V * np;
        if (n(2) < 0)
            n = -n;
        vn.push_back(n);
    }

    // case d'=-d2
    float auxSphi =
        sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 - d3) * d2);

    float cphi   = (d1 * d3 - d2 * d2) / ((d1 - d3) * d2);
    float sphi[] = {auxSphi, -auxSphi, -auxSphi, auxSphi};

    for (int i = 0; i < 4; i++)
    {
        Eigen::Matrix3f Rp;
        Rp.setZero();
        Rp(0, 0) = cphi;
        Rp(0, 2) = sphi[i];
        Rp(1, 1) = -1;
        Rp(2, 0) = sphi[i];
        Rp(2, 2) = -cphi;

        Eigen::Matrix3f R = s * U * Rp * Vt;
        vR.push_back(R);

        Eigen::Vector3f tp;
        tp(0) = x1[i];
        tp(1) = 0;
        tp(2) = x3[i];
        tp *= d1 + d3;

        Eigen::Vector3f t = U * tp;
        vt.push_back(t / t.norm());

        Eigen::Vector3f np;
        np(0) = x1[i];
        np(1) = 0;
        np(2) = x3[i];

        Eigen::Vector3f n = V * np;
        if (n(2) < 0)
            n = -n;
        vn.push_back(n);
    }

    int                 bestGood          = 0;
    int                 secondBestGood    = 0;
    int                 bestSolutionIndex = -1;
    float               bestParallax      = -1;
    vector<cv::Point3f> bestP3d;
    vector<bool>        bestTriangulated;

    // Instead of applying the visibility constraints proposed in the Faugeras'
    // paper (which could fail for points seen with low parallax) We reconstruct
    // all hypotheses and check in terms of triangulated points and parallax
    for (size_t i = 0; i < 8; i++)
    {
        float               parallaxi;
        vector<cv::Point3f> vP3Di;
        vector<bool>        triangulatediFlags;
        int                 goodCount = checkRT(vR[i],
                                vt[i],
                                keys1,
                                keys2,
                                matches12,
                                matchesInliersFlags_inout,
                                K_in,
                                vP3Di,
                                4.0 * sigmaSquared,
                                triangulatediFlags,
                                parallaxi);

        if (goodCount > bestGood)
        {
            secondBestGood    = bestGood;
            bestGood          = goodCount;
            bestSolutionIndex = i;
            bestParallax      = parallaxi;
            bestP3d           = vP3Di;
            bestTriangulated  = triangulatediFlags;
        }
        else if (goodCount > secondBestGood)
        {
            secondBestGood = goodCount;
        }
    }

    if (secondBestGood < 0.75 * bestGood &&
        bestParallax >= minimumParallax_in &&
        bestGood > minimumTriangulated_in && bestGood > 0.9 * N)
    {
        T21_out = Sophus::SE3f(vR[bestSolutionIndex], vt[bestSolutionIndex]);

        // Publish the winning hypothesis' structure, matching the output
        // contract reconstructF() honours: the monocular initializer reads
        // vP3D straight after Reconstruct() returns, so leaving it untouched
        // on this branch would hand it stale or empty map points.
        vP3D_inout            = bestP3d;
        triangulatedFlags_out = bestTriangulated;

        return true;
    }

    return false;
}

} // namespace core
} // namespace vs_graphs
