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

#include "Sim3Solver.h"

#include <cmath>
#include <opencv2/core/core.hpp>
#include <vector>

#include "KeyFrame.h"
#include "ORBmatcher.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

namespace vs_graphs
{
namespace core
{

Sim3Solver::Sim3Solver(KeyFrame                 *pKF1,
                       KeyFrame                 *pKF2,
                       const vector<MapPoint *> &vpMatched12,
                       const bool                bFixScale,
                       vector<KeyFrame *>        vpKeyFrameMatchedMP) :
    iterationCount(0),
    bestInlierCount(0),
    fixScale(bFixScale),
    pCamera1(pKF1->p_camera),
    pCamera2(pKF2->p_camera)
{
    bool bDifferentKFs = false;
    if (vpKeyFrameMatchedMP.empty())
    {
        bDifferentKFs       = true;
        vpKeyFrameMatchedMP = vector<KeyFrame *>(vpMatched12.size(), pKF2);
    }

    p_keyFrame1 = pKF1;
    p_keyFrame2 = pKF2;

    vector<MapPoint *> vpKeyFrameMP1 = pKF1->getMapPointMatches();

    mN1 = vpMatched12.size();

    mapPoints1.reserve(mN1);
    mapPoints2.reserve(mN1);
    mapPointMatches12 = vpMatched12;
    indices1.reserve(mN1);
    points3Dc1.reserve(mN1);
    points3Dc2.reserve(mN1);

    Eigen::Matrix3f Rcw1 = pKF1->getRotation();
    Eigen::Vector3f tcw1 = pKF1->getTranslation();
    Eigen::Matrix3f Rcw2 = pKF2->getRotation();
    Eigen::Vector3f tcw2 = pKF2->getTranslation();

    allIndices.reserve(mN1);

    size_t idx = 0;

    KeyFrame *pKFm = pKF2; // Default variable
    for (int i1 = 0; i1 < mN1; i1++)
    {
        if (vpMatched12[i1])
        {
            MapPoint *pMP1 = vpKeyFrameMP1[i1];
            MapPoint *pMP2 = vpMatched12[i1];

            if (!pMP1)
                continue;

            if (pMP1->isBad() || pMP2->isBad())
                continue;

            if (bDifferentKFs)
                pKFm = vpKeyFrameMatchedMP[i1];

            int indexKF1 = get<0>(pMP1->getIndexInKeyFrame(pKF1));
            int indexKF2 = get<0>(pMP2->getIndexInKeyFrame(pKFm));

            if (indexKF1 < 0 || indexKF2 < 0)
                continue;

            const cv::KeyPoint &kp1 = pKF1->keyPointsUndistorted[indexKF1];
            const cv::KeyPoint &kp2 = pKFm->keyPointsUndistorted[indexKF2];

            const float sigmaSquare1 = pKF1->levelSigmaSquared[kp1.octave];
            const float sigmaSquare2 = pKFm->levelSigmaSquared[kp2.octave];

            maxError1.push_back(9.210 * sigmaSquare1);
            maxError2.push_back(9.210 * sigmaSquare2);

            mapPoints1.push_back(pMP1);
            mapPoints2.push_back(pMP2);
            indices1.push_back(i1);

            Eigen::Vector3f X3D1w = pMP1->getWorldPos();
            points3Dc1.push_back(Rcw1 * X3D1w + tcw1);

            Eigen::Vector3f X3D2w = pMP2->getWorldPos();
            points3Dc2.push_back(Rcw2 * X3D2w + tcw2);

            allIndices.push_back(idx);
            idx++;
        }
    }

    fromCameraToImage(points3Dc1, points1im1, pCamera1);
    fromCameraToImage(points3Dc2, points2im2, pCamera2);

    setRansacParameters();
}

void Sim3Solver::setRansacParameters(double probability,
                                     int    minInliers,
                                     int    maxIterations)
{
    ransacProb          = probability;
    ransacMinInliers    = minInliers;
    ransacMaxIterations = maxIterations;

    N = mapPoints1.size(); // number of correspondences

    inlierFlags.resize(N);

    // Adjust Parameters according to number of correspondences
    float epsilon = (float)ransacMinInliers / N;

    // Set RANSAC iterations according to probability, epsilon, and max
    // iterations
    int nIterations;

    if (ransacMinInliers == N)
        nIterations = 1;
    else
        nIterations = ceil(log(1 - ransacProb) / log(1 - pow(epsilon, 3)));

    ransacMaxIterations = max(1, min(nIterations, ransacMaxIterations));

    iterationCount = 0;
}

Eigen::Matrix4f Sim3Solver::iterate(int           nIterations,
                                    bool         &bNoMore,
                                    vector<bool> &vbInliers,
                                    int          &nInliers)
{
    bNoMore   = false;
    vbInliers = vector<bool>(mN1, false);
    nInliers  = 0;

    if (N < ransacMinInliers)
    {
        bNoMore = true;
        return Eigen::Matrix4f::Identity();
    }

    vector<size_t> vAvailableIndices;

    Eigen::Matrix3f P3Dc1i;
    Eigen::Matrix3f P3Dc2i;

    int nCurrentIterations = 0;
    while (iterationCount < ransacMaxIterations &&
           nCurrentIterations < nIterations)
    {
        nCurrentIterations++;
        iterationCount++;

        vAvailableIndices = allIndices;

        // Get min set of points
        for (short i = 0; i < 3; ++i)
        {
            int randi =
                DUtils::Random::RandomInt(0, vAvailableIndices.size() - 1);

            int idx = vAvailableIndices[randi];

            P3Dc1i.col(i) = points3Dc1[idx];
            P3Dc2i.col(i) = points3Dc2[idx];

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        computeSim3(P3Dc1i, P3Dc2i);

        checkInliers();

        if (inlierCount >= bestInlierCount)
        {
            bestInlierFlags  = inlierFlags;
            bestInlierCount  = inlierCount;
            mBestT12         = mT12i;
            mBestRotation    = mR12i;
            mBestTranslation = mt12i;
            mBestScale       = ms12i;

            if (inlierCount > ransacMinInliers)
            {
                nInliers = inlierCount;
                for (int i = 0; i < N; i++)
                    if (inlierFlags[i])
                        vbInliers[indices1[i]] = true;
                return mBestT12;
            }
        }
    }

    if (iterationCount >= ransacMaxIterations)
        bNoMore = true;

    return Eigen::Matrix4f::Identity();
}

Eigen::Matrix4f Sim3Solver::iterate(int           nIterations,
                                    bool         &bNoMore,
                                    vector<bool> &vbInliers,
                                    int          &nInliers,
                                    bool         &bConverge)
{
    bNoMore   = false;
    bConverge = false;
    vbInliers = vector<bool>(mN1, false);
    nInliers  = 0;

    if (N < ransacMinInliers)
    {
        bNoMore = true;
        return Eigen::Matrix4f::Identity();
    }

    vector<size_t> vAvailableIndices;

    Eigen::Matrix3f P3Dc1i;
    Eigen::Matrix3f P3Dc2i;

    int nCurrentIterations = 0;

    Eigen::Matrix4f bestSim3;

    while (iterationCount < ransacMaxIterations &&
           nCurrentIterations < nIterations)
    {
        nCurrentIterations++;
        iterationCount++;

        vAvailableIndices = allIndices;

        // Get min set of points
        for (short i = 0; i < 3; ++i)
        {
            int randi =
                DUtils::Random::RandomInt(0, vAvailableIndices.size() - 1);

            int idx = vAvailableIndices[randi];

            P3Dc1i.col(i) = points3Dc1[idx];
            P3Dc2i.col(i) = points3Dc2[idx];

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        computeSim3(P3Dc1i, P3Dc2i);

        checkInliers();

        if (inlierCount >= bestInlierCount)
        {
            bestInlierFlags  = inlierFlags;
            bestInlierCount  = inlierCount;
            mBestT12         = mT12i;
            mBestRotation    = mR12i;
            mBestTranslation = mt12i;
            mBestScale       = ms12i;

            if (inlierCount > ransacMinInliers)
            {
                nInliers = inlierCount;
                for (int i = 0; i < N; i++)
                    if (inlierFlags[i])
                        vbInliers[indices1[i]] = true;
                bConverge = true;
                return mBestT12;
            }
            else
            {
                bestSim3 = mBestT12;
            }
        }
    }

    if (iterationCount >= ransacMaxIterations)
        bNoMore = true;

    return bestSim3;
}

Eigen::Matrix4f Sim3Solver::find(vector<bool> &vbInliers12, int &nInliers)
{
    bool bFlag;
    return iterate(ransacMaxIterations, bFlag, vbInliers12, nInliers);
}

void Sim3Solver::computeCentroid(Eigen::Matrix3f &P,
                                 Eigen::Matrix3f &Pr,
                                 Eigen::Vector3f &C)
{
    C = P.rowwise().sum();
    C = C / P.cols();
    for (int i = 0; i < P.cols(); i++)
        Pr.col(i) = P.col(i) - C;
}

void Sim3Solver::computeSim3(Eigen::Matrix3f &P1, Eigen::Matrix3f &P2)
{
    // Custom implementation of:
    // Horn 1987, Closed-form solution of absolute orientataion using unit
    // quaternions

    // Step 1: Centroid and relative coordinates

    Eigen::Matrix3f Pr1; // Relative coordinates to centroid (set 1)
    Eigen::Matrix3f Pr2; // Relative coordinates to centroid (set 2)
    Eigen::Vector3f O1;  // Centroid of P1
    Eigen::Vector3f O2;  // Centroid of P2

    computeCentroid(P1, Pr1, O1);
    computeCentroid(P2, Pr2, O2);

    // Step 2: Compute M matrix

    Eigen::Matrix3f M = Pr2 * Pr1.transpose();

    // Step 3: Compute N matrix
    double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

    Eigen::Matrix4f N;

    N11 = M(0, 0) + M(1, 1) + M(2, 2);
    N12 = M(1, 2) - M(2, 1);
    N13 = M(2, 0) - M(0, 2);
    N14 = M(0, 1) - M(1, 0);
    N22 = M(0, 0) - M(1, 1) - M(2, 2);
    N23 = M(0, 1) + M(1, 0);
    N24 = M(2, 0) + M(0, 2);
    N33 = -M(0, 0) + M(1, 1) - M(2, 2);
    N34 = M(1, 2) + M(2, 1);
    N44 = -M(0, 0) - M(1, 1) + M(2, 2);

    N << N11, N12, N13, N14, N12, N22, N23, N24, N13, N23, N33, N34, N14, N24,
        N34, N44;

    // Step 4: Eigenvector of the highest eigenvalue
    Eigen::EigenSolver<Eigen::Matrix4f> eigSolver;
    eigSolver.compute(N);

    Eigen::Vector4f eval = eigSolver.eigenvalues().real();
    Eigen::Matrix4f evec =
        eigSolver.eigenvectors()
            .real(); // evec[0] is the quaternion of the desired rotation

    int maxIndex; // should be zero
    eval.maxCoeff(&maxIndex);

    Eigen::Vector3f vec = evec.block<3, 1>(
        1,
        maxIndex); // extract imaginary part of the quaternion (sin*axis)

    // Rotation angle. sin is the norm of the imaginary part, cos is the real
    // part
    double ang = atan2(vec.norm(), evec(0, maxIndex));

    vec = 2 * ang * vec /
          vec.norm(); // Angle-axis representation. quaternion angle is the half
    mR12i = Sophus::SO3f::exp(vec).matrix();

    // Step 5: Rotate set 2
    Eigen::Matrix3f P3 = mR12i * Pr2;

    // Step 6: Scale

    if (!fixScale)
    {
        double cvnom = Converter::toCvMat(Pr1).dot(Converter::toCvMat(P3));
        double nom   = (Pr1.array() * P3.array()).sum();
        if (abs(nom - cvnom) > 1e-3)
            std::cout << "sim3 solver: " << abs(nom - cvnom) << std::endl
                      << nom << std::endl;
        Eigen::Array<float, 3, 3> aux_P3;
        aux_P3     = P3.array() * P3.array();
        double den = aux_P3.sum();

        ms12i = nom / den;
    }
    else
        ms12i = 1.0f;

    // Step 7: Translation
    mt12i = O1 - ms12i * mR12i * O2;

    // Step 8: Transformation

    // Step 8.1 T12
    mT12i.setIdentity();

    Eigen::Matrix3f sR      = ms12i * mR12i;
    mT12i.block<3, 3>(0, 0) = sR;
    mT12i.block<3, 1>(0, 3) = mt12i;

    // Step 8.2 T21
    mT21i.setIdentity();
    Eigen::Matrix3f sRinv = (1.0 / ms12i) * mR12i.transpose();

    // sRinv.copyTo(mT21i.rowRange(0,3).colRange(0,3));
    mT21i.block<3, 3>(0, 0) = sRinv;

    Eigen::Vector3f tinv    = -sRinv * mt12i;
    mT21i.block<3, 1>(0, 3) = tinv;
}

void Sim3Solver::checkInliers()
{
    vector<Eigen::Vector2f> vP1im2, vP2im1;
    project(points3Dc2, vP2im1, mT12i, pCamera1);
    project(points3Dc1, vP1im2, mT21i, pCamera2);

    inlierCount = 0;

    for (size_t i = 0; i < points1im1.size(); i++)
    {
        Eigen::Vector2f dist1 = points1im1[i] - vP2im1[i];
        Eigen::Vector2f dist2 = vP1im2[i] - points2im2[i];

        const float err1 = dist1.dot(dist1);
        const float err2 = dist2.dot(dist2);

        if (err1 < maxError1[i] && err2 < maxError2[i])
        {
            inlierFlags[i] = true;
            inlierCount++;
        }
        else
            inlierFlags[i] = false;
    }
}

Eigen::Matrix4f Sim3Solver::getEstimatedTransformation()
{
    return mBestT12;
}

Eigen::Matrix3f Sim3Solver::getEstimatedRotation()
{
    return mBestRotation;
}

Eigen::Vector3f Sim3Solver::getEstimatedTranslation()
{
    return mBestTranslation;
}

float Sim3Solver::getEstimatedScale()
{
    return mBestScale;
}

void Sim3Solver::project(const vector<Eigen::Vector3f>  &vP3Dw,
                         vector<Eigen::Vector2f>        &vP2D,
                         Eigen::Matrix4f                 Tcw,
                         camera_models::GeometricCamera *pCamera)
{
    Eigen::Matrix3f Rcw = Tcw.block<3, 3>(0, 0);
    Eigen::Vector3f tcw = Tcw.block<3, 1>(0, 3);

    vP2D.clear();
    vP2D.reserve(vP3Dw.size());

    for (size_t i = 0, iend = vP3Dw.size(); i < iend; i++)
    {
        Eigen::Vector3f P3Dc = Rcw * vP3Dw[i] + tcw;
        Eigen::Vector2f pt2D = pCamera->project(P3Dc);
        vP2D.push_back(pt2D);
    }
}

void Sim3Solver::fromCameraToImage(const vector<Eigen::Vector3f>  &vP3Dc,
                                   vector<Eigen::Vector2f>        &vP2D,
                                   camera_models::GeometricCamera *pCamera)
{
    vP2D.clear();
    vP2D.reserve(vP3Dc.size());

    for (size_t i = 0, iend = vP3Dc.size(); i < iend; i++)
    {
        Eigen::Vector2f pt2D = pCamera->project(vP3Dc[i]);
        vP2D.push_back(pt2D);
    }
}

} // namespace core
} // namespace vs_graphs
