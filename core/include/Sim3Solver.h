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

#ifndef SIM3SOLVER_H
#define SIM3SOLVER_H

#include <opencv2/core.hpp>
#include <vector>

#include "KeyFrame.h"
#include "MapPoint.h"

namespace vs_graphs
{
namespace core
{

class Sim3Solver
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Sim3Solver(
        KeyFrame                      *p_keyFrame1_inout,
        KeyFrame                      *p_keyFrame2_inout,
        const std::vector<MapPoint *> &matched12_in,
        const bool                     isScaleFixed_in = true,
        vector<KeyFrame *> keyFrameMatchedMapPoints_in = vector<KeyFrame *>()) :
        iterationCount(0),
        bestInlierCount(0),
        isScaleFixed(isScaleFixed_in),
        p_firstCamera(p_keyFrame1_inout->p_camera),
        p_secondCamera(p_keyFrame2_inout->p_camera)
    {
        bool areKeyFramesDifferent = false;
        if (keyFrameMatchedMapPoints_in.empty())
        {
            areKeyFramesDifferent = true;
            keyFrameMatchedMapPoints_in =
                vector<KeyFrame *>(matched12_in.size(), p_keyFrame2_inout);
        }

        p_keyFrame1 = p_keyFrame1_inout;
        p_keyFrame2 = p_keyFrame2_inout;

        vector<MapPoint *> keyFrameMapPoint1 =
            p_keyFrame1_inout->getMapPointMatches();

        firstMatchCount = matched12_in.size();

        mapPoints1.reserve(firstMatchCount);
        mapPoints2.reserve(firstMatchCount);
        mapPointMatches12 = matched12_in;
        indices1.reserve(firstMatchCount);
        points3Dc1.reserve(firstMatchCount);
        points3Dc2.reserve(firstMatchCount);

        Eigen::Matrix3f Rcw1 = p_keyFrame1_inout->getRotation();
        Eigen::Vector3f tcw1 = p_keyFrame1_inout->getTranslation();
        Eigen::Matrix3f Rcw2 = p_keyFrame2_inout->getRotation();
        Eigen::Vector3f tcw2 = p_keyFrame2_inout->getTranslation();

        allIndices.reserve(firstMatchCount);

        size_t idx = 0;

        KeyFrame *pKFm = p_keyFrame2_inout; // Default variable
        for (int i1 = 0; i1 < firstMatchCount; i1++)
        {
            if (matched12_in[i1])
            {
                MapPoint *p_mapPoint1 = keyFrameMapPoint1[i1];
                MapPoint *p_mapPoint2 = matched12_in[i1];

                if (!p_mapPoint1)
                    continue;

                if (p_mapPoint1->isBad() || p_mapPoint2->isBad())
                    continue;

                if (areKeyFramesDifferent)
                    pKFm = keyFrameMatchedMapPoints_in[i1];

                int indexKeyFrame1 =
                    get<0>(p_mapPoint1->getIndexInKeyFrame(p_keyFrame1_inout));
                int indexKeyFrame2 =
                    get<0>(p_mapPoint2->getIndexInKeyFrame(pKFm));

                if (indexKeyFrame1 < 0 || indexKeyFrame2 < 0)
                    continue;

                const cv::KeyPoint &keyPoint1 =
                    p_keyFrame1_inout->keyPointsUndistorted[indexKeyFrame1];
                const cv::KeyPoint &keyPoint2 =
                    pKFm->keyPointsUndistorted[indexKeyFrame2];

                const float sigmaSquare1 =
                    p_keyFrame1_inout->levelSigmaSquared[keyPoint1.octave];
                const float sigmaSquare2 =
                    pKFm->levelSigmaSquared[keyPoint2.octave];

                maxError1.push_back(9.210 * sigmaSquare1);
                maxError2.push_back(9.210 * sigmaSquare2);

                mapPoints1.push_back(p_mapPoint1);
                mapPoints2.push_back(p_mapPoint2);
                indices1.push_back(i1);

                Eigen::Vector3f X3D1w = p_mapPoint1->getWorldPos();
                points3Dc1.push_back(Rcw1 * X3D1w + tcw1);

                Eigen::Vector3f X3D2w = p_mapPoint2->getWorldPos();
                points3Dc2.push_back(Rcw2 * X3D2w + tcw2);

                allIndices.push_back(idx);
                idx++;
            }
        }

        fromCameraToImage(points3Dc1, points1im1, p_firstCamera);
        fromCameraToImage(points3Dc2, points2im2, p_secondCamera);

        setRansacParameters();
    }

    void setRansacParameters(double probability_in       = 0.99,
                             int    minimumInliers_in    = 6,
                             int    maximumIterations_in = 300);

    Eigen::Matrix4f find(std::vector<bool> &inliers12Flags_inout,
                         int               &inlierCount_inout);

    Eigen::Matrix4f iterate(int                iterationCount_in,
                            bool              &areIterationsExhausted_out,
                            std::vector<bool> &inliersFlags_out,
                            int               &inlierCount_out);
    Eigen::Matrix4f iterate(int           iterationCount_in,
                            bool         &areIterationsExhausted_out,
                            vector<bool> &inliersFlags_out,
                            int          &inlierCount_out,
                            bool         &hasConverged_out);

    Eigen::Matrix4f getEstimatedTransformation();
    Eigen::Matrix3f getEstimatedRotation();
    Eigen::Vector3f getEstimatedTranslation();
    float           getEstimatedScale();

  protected:
    void computeCentroid(Eigen::Matrix3f &P_in,
                         Eigen::Matrix3f &Pr_inout,
                         Eigen::Vector3f &C_out);

    void computeSim3(Eigen::Matrix3f &P1_inout, Eigen::Matrix3f &P2_inout);

    void checkInliers();

    void project(
        const std::vector<Eigen::Vector3f>              &vP3Dw_in,
        std::vector<Eigen::Vector2f>                    &points2D_out,
        Eigen::Matrix4f                                  Tcw_in,
        camera_models::geometriccamera::GeometricCamera *p_camera_inout);
    void fromCameraToImage(
        const std::vector<Eigen::Vector3f>              &vP3Dc_in,
        std::vector<Eigen::Vector2f>                    &points2D_out,
        camera_models::geometriccamera::GeometricCamera *p_camera_inout);

  protected:
    // KeyFrames and matches
    KeyFrame *p_keyFrame1;
    KeyFrame *p_keyFrame2;

    std::vector<Eigen::Vector3f> points3Dc1;
    std::vector<Eigen::Vector3f> points3Dc2;
    std::vector<MapPoint *>      mapPoints1;
    std::vector<MapPoint *>      mapPoints2;
    std::vector<MapPoint *>      mapPointMatches12;
    std::vector<size_t>          indices1;
    std::vector<size_t>          sigmaSquared1;
    std::vector<size_t>          sigmaSquared2;
    std::vector<size_t>          maxError1;
    std::vector<size_t>          maxError2;

    int correspondenceCount;
    int firstMatchCount;

    // Current Estimation
    Eigen::Matrix3f   mR12i;
    Eigen::Vector3f   mt12i;
    float             ms12i;
    Eigen::Matrix4f   mT12i;
    Eigen::Matrix4f   mT21i;
    std::vector<bool> inlierFlags;
    int               inlierCount;

    // Current Ransac State
    int               iterationCount;
    std::vector<bool> bestInlierFlags;
    int               bestInlierCount;
    Eigen::Matrix4f   mBestT12;
    Eigen::Matrix3f   bestRotation;
    Eigen::Vector3f   bestTranslation;
    float             bestScale;

    // Scale is fixed to 1 in the stereo/RGBD case
    bool isScaleFixed;

    // Indices for random selection
    std::vector<size_t> allIndices;

    // Projections
    std::vector<Eigen::Vector2f> points1im1;
    std::vector<Eigen::Vector2f> points2im2;

    // RANSAC probability
    double ransacProb;

    // RANSAC min inliers
    int ransacMinInliers;

    // RANSAC max iterations
    int ransacMaxIterations;

    // Threshold inlier/outlier. e = dist(Pi,T_ij*Pj)^2 < 5.991*mSigma2
    float threshold;
    float sigmaSquared;

    // Calibration
    // cv::Mat mK1;
    // cv::Mat mK2;

    camera_models::geometriccamera::GeometricCamera *p_firstCamera,
        *p_secondCamera;
};

} // namespace core
} // namespace vs_graphs

#endif // SIM3SOLVER_H
