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

#include <opencv2/opencv.hpp>
#include <vector>

#include "KeyFrame.h"

namespace vs_graphs
{
namespace core
{

class Sim3Solver
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Sim3Solver(
        KeyFrame                      *pKF1,
        KeyFrame                      *pKF2,
        const std::vector<MapPoint *> &vpMatched12,
        const bool                     bFixScale     = true,
        const vector<KeyFrame *> vpKeyFrameMatchedMP = vector<KeyFrame *>());

    void setRansacParameters(double probability   = 0.99,
                             int    minInliers    = 6,
                             int    maxIterations = 300);

    Eigen::Matrix4f find(std::vector<bool> &vbInliers12, int &nInliers);

    Eigen::Matrix4f iterate(int                nIterations,
                            bool              &bNoMore,
                            std::vector<bool> &vbInliers,
                            int               &nInliers);
    Eigen::Matrix4f iterate(int           nIterations,
                            bool         &bNoMore,
                            vector<bool> &vbInliers,
                            int          &nInliers,
                            bool         &bConverge);

    Eigen::Matrix4f getEstimatedTransformation();
    Eigen::Matrix3f getEstimatedRotation();
    Eigen::Vector3f getEstimatedTranslation();
    float           getEstimatedScale();

  protected:
    void computeCentroid(Eigen::Matrix3f &P,
                         Eigen::Matrix3f &Pr,
                         Eigen::Vector3f &C);

    void computeSim3(Eigen::Matrix3f &P1, Eigen::Matrix3f &P2);

    void checkInliers();

    void project(const std::vector<Eigen::Vector3f> &vP3Dw,
                 std::vector<Eigen::Vector2f>       &vP2D,
                 Eigen::Matrix4f                     Tcw,
                 camera_models::GeometricCamera     *pCamera);
    void fromCameraToImage(const std::vector<Eigen::Vector3f> &vP3Dc,
                           std::vector<Eigen::Vector2f>       &vP2D,
                           camera_models::GeometricCamera     *pCamera);

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

    int N;
    int mN1;

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
    Eigen::Matrix3f   mBestRotation;
    Eigen::Vector3f   mBestTranslation;
    float             mBestScale;

    // Scale is fixed to 1 in the stereo/RGBD case
    bool fixScale;

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

    camera_models::GeometricCamera *pCamera1, *pCamera2;
};

} // namespace core
} // namespace vs_graphs

#endif // SIM3SOLVER_H
