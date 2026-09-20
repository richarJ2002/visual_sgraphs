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
 * @file         KannalaBrandt8.cc
 *
 * @brief        Implements KannalaBrandt8 declared in KannalaBrandt8.h.
 */

#include "KannalaBrandt8.h"
#include <boost/serialization/export.hpp>

namespace vs_graphs
{
namespace core
{
namespace camera_models
{

cv::Point2f KannalaBrandt8::project(const cv::Point3f &point3D_in)
{
    const float x2_plus_y2 =
        point3D_in.x * point3D_in.x + point3D_in.y * point3D_in.y;
    const float theta = atan2f(sqrtf(x2_plus_y2), point3D_in.z);
    const float psi   = atan2f(point3D_in.y, point3D_in.x);

    const float theta2 = theta * theta;
    const float theta3 = theta * theta2;
    const float theta5 = theta3 * theta2;
    const float theta7 = theta5 * theta2;
    const float theta9 = theta7 * theta2;
    const float r = theta + parameters[4] * theta3 + parameters[5] * theta5 +
                    parameters[6] * theta7 + parameters[7] * theta9;

    return cv::Point2f(parameters[0] * r * cos(psi) + parameters[2],
                       parameters[1] * r * sin(psi) + parameters[3]);
}

Eigen::Vector2d KannalaBrandt8::project(const Eigen::Vector3d &point3D_in)
{
    const double x2_plus_y2 =
        point3D_in[0] * point3D_in[0] + point3D_in[1] * point3D_in[1];
    const double theta = atan2f(sqrtf(x2_plus_y2), point3D_in[2]);
    const double psi   = atan2f(point3D_in[1], point3D_in[0]);

    const double theta2 = theta * theta;
    const double theta3 = theta * theta2;
    const double theta5 = theta3 * theta2;
    const double theta7 = theta5 * theta2;
    const double theta9 = theta7 * theta2;
    const double r = theta + parameters[4] * theta3 + parameters[5] * theta5 +
                     parameters[6] * theta7 + parameters[7] * theta9;

    Eigen::Vector2d res;
    res[0] = parameters[0] * r * cos(psi) + parameters[2];
    res[1] = parameters[1] * r * sin(psi) + parameters[3];

    return res;
}

Eigen::Vector2f KannalaBrandt8::project(const Eigen::Vector3f &point3D_in)
{
    const float x2_plus_y2 =
        point3D_in[0] * point3D_in[0] + point3D_in[1] * point3D_in[1];
    const float theta = atan2f(sqrtf(x2_plus_y2), point3D_in[2]);
    const float psi   = atan2f(point3D_in[1], point3D_in[0]);

    const float theta2 = theta * theta;
    const float theta3 = theta * theta2;
    const float theta5 = theta3 * theta2;
    const float theta7 = theta5 * theta2;
    const float theta9 = theta7 * theta2;
    const float r = theta + parameters[4] * theta3 + parameters[5] * theta5 +
                    parameters[6] * theta7 + parameters[7] * theta9;

    Eigen::Vector2f res;
    res[0] = parameters[0] * r * cos(psi) + parameters[2];
    res[1] = parameters[1] * r * sin(psi) + parameters[3];

    return res;
}

Eigen::Vector2f KannalaBrandt8::projectMat(const cv::Point3f &point3D_in)
{
    cv::Point2f point = this->project(point3D_in);
    return Eigen::Vector2f(point.x, point.y);
}

float KannalaBrandt8::uncertainty2(
    const Eigen::Matrix<double, 2, 1> &point2D_in)
{
    /*Eigen::Matrix<double,2,1> c;
    c << parameters[2], parameters[3];
    if ((point2D_in-c).squaredNorm()>57600) // 240*240 (256)
        return 100.f;
    else
        return 1.0f;*/
    return 1.f;
}

Eigen::Vector3f KannalaBrandt8::unprojectEig(const cv::Point2f &point2D_in)
{
    cv::Point3f ray = this->unproject(point2D_in);
    return Eigen::Vector3f(ray.x, ray.y, ray.z);
}

cv::Point3f KannalaBrandt8::unproject(const cv::Point2f &point2D_in)
{
    // Use Newthon method to solve for theta with good precision (err ~ e-6)
    cv::Point2f pw((point2D_in.x - parameters[2]) / parameters[0],
                   (point2D_in.y - parameters[3]) / parameters[1]);
    float       scale   = 1.f;
    float       theta_d = sqrtf(pw.x * pw.x + pw.y * pw.y);
    theta_d             = fminf(fmaxf(-CV_PI / 2.f, theta_d), CV_PI / 2.f);

    if (theta_d > 1e-8)
    {
        // Compensate distortion iteratively
        float theta = theta_d;

        for (int j = 0; j < 10; j++)
        {
            float theta2 = theta * theta, theta4 = theta2 * theta2,
                  theta6 = theta4 * theta2, theta8 = theta4 * theta4;
            float k0_theta2 = parameters[4] * theta2,
                  k1_theta4 = parameters[5] * theta4;
            float k2_theta6 = parameters[6] * theta6,
                  k3_theta8 = parameters[7] * theta8;
            float theta_fix =
                (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) -
                 theta_d) /
                (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 +
                 9 * k3_theta8);
            theta = theta - theta_fix;
            if (fabsf(theta_fix) < precision)
                break;
        }
        scale = std::tan(theta) / theta_d;
    }

    return cv::Point3f(pw.x * scale, pw.y * scale, 1.f);
}

Eigen::Matrix<double, 2, 3>
    KannalaBrandt8::computeProjectionJacobian(const Eigen::Vector3d &point3D_in)
{
    double x2    = point3D_in[0] * point3D_in[0],
           y2    = point3D_in[1] * point3D_in[1],
           z2    = point3D_in[2] * point3D_in[2];
    double r2    = x2 + y2;
    double r     = sqrt(r2);
    double r3    = r2 * r;
    double theta = atan2(r, point3D_in[2]);

    double theta2 = theta * theta, theta3 = theta2 * theta;
    double theta4 = theta2 * theta2, theta5 = theta4 * theta;
    double theta6 = theta2 * theta4, theta7 = theta6 * theta;
    double theta8 = theta4 * theta4, theta9 = theta8 * theta;

    double f = theta + theta3 * parameters[4] + theta5 * parameters[5] +
               theta7 * parameters[6] + theta9 * parameters[7];
    double fd = 1 + 3 * parameters[4] * theta2 + 5 * parameters[5] * theta4 +
                7 * parameters[6] * theta6 + 9 * parameters[7] * theta8;

    Eigen::Matrix<double, 2, 3> JacGood;
    JacGood(0, 0) = parameters[0] *
                    (fd * point3D_in[2] * x2 / (r2 * (r2 + z2)) + f * y2 / r3);
    JacGood(1, 0) = parameters[1] * (fd * point3D_in[2] * point3D_in[1] *
                                         point3D_in[0] / (r2 * (r2 + z2)) -
                                     f * point3D_in[1] * point3D_in[0] / r3);

    JacGood(0, 1) = parameters[0] * (fd * point3D_in[2] * point3D_in[1] *
                                         point3D_in[0] / (r2 * (r2 + z2)) -
                                     f * point3D_in[1] * point3D_in[0] / r3);
    JacGood(1, 1) = parameters[1] *
                    (fd * point3D_in[2] * y2 / (r2 * (r2 + z2)) + f * x2 / r3);

    JacGood(0, 2) = -parameters[0] * fd * point3D_in[0] / (r2 + z2);
    JacGood(1, 2) = -parameters[1] * fd * point3D_in[1] / (r2 + z2);

    return JacGood;
}

bool KannalaBrandt8::reconstructWithTwoViews(
    const std::vector<cv::KeyPoint> &keys1_in,
    const std::vector<cv::KeyPoint> &keys2_in,
    const std::vector<int>          &matches12_in,
    Sophus::SE3f                    &pose21_out,
    std::vector<cv::Point3f>        &points3D_out,
    std::vector<bool>               &triangulated_out)
{
    if (!p_twoViewReconstruction)
    {
        Eigen::Matrix3f K       = this->toK_();
        p_twoViewReconstruction = new TwoViewReconstruction(K);
    }

    // Correct FishEye distortion
    std::vector<cv::KeyPoint> vKeysUn1 = keys1_in, vKeysUn2 = keys2_in;
    std::vector<cv::Point2f>  vPts1(keys1_in.size()), vPts2(keys2_in.size());

    for (size_t i = 0; i < keys1_in.size(); i++)
        vPts1[i] = keys1_in[i].pt;
    for (size_t i = 0; i < keys2_in.size(); i++)
        vPts2[i] = keys2_in[i].pt;

    cv::Mat D = (cv::Mat_<float>(4, 1) << parameters[4],
                 parameters[5],
                 parameters[6],
                 parameters[7]);
    cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat K = this->toK();
    cv::fisheye::undistortPoints(vPts1, vPts1, K, D, R, K);
    cv::fisheye::undistortPoints(vPts2, vPts2, K, D, R, K);

    for (size_t i = 0; i < keys1_in.size(); i++)
        vKeysUn1[i].pt = vPts1[i];
    for (size_t i = 0; i < keys2_in.size(); i++)
        vKeysUn2[i].pt = vPts2[i];

    return p_twoViewReconstruction->Reconstruct(vKeysUn1,
                                                vKeysUn2,
                                                matches12_in,
                                                pose21_out,
                                                points3D_out,
                                                triangulated_out);
}

cv::Mat KannalaBrandt8::toK()
{
    cv::Mat K = (cv::Mat_<float>(3, 3) << parameters[0],
                 0.f,
                 parameters[2],
                 0.f,
                 parameters[1],
                 parameters[3],
                 0.f,
                 0.f,
                 1.f);
    return K;
}
Eigen::Matrix3f KannalaBrandt8::toK_()
{
    Eigen::Matrix3f K;
    K << parameters[0], 0.f, parameters[2], 0.f, parameters[1], parameters[3],
        0.f, 0.f, 1.f;
    return K;
}

bool KannalaBrandt8::epipolarConstrain(GeometricCamera       *p_otherCamera_in,
                                       const cv::KeyPoint    &keypoint1_in,
                                       const cv::KeyPoint    &keypoint2_in,
                                       const Eigen::Matrix3f &rotation12_in,
                                       const Eigen::Vector3f &translation12_in,
                                       const float            sigmaLevel_in,
                                       const float            uncertainty_in)
{
    Eigen::Vector3f point3D;
    return this->triangulateMatches(p_otherCamera_in,
                                    keypoint1_in,
                                    keypoint2_in,
                                    rotation12_in,
                                    translation12_in,
                                    sigmaLevel_in,
                                    uncertainty_in,
                                    point3D) > 0.0001f;
}

bool KannalaBrandt8::matchAndTriangulate(const cv::KeyPoint &keypoint1_in,
                                         const cv::KeyPoint &keypoint2_in,
                                         GeometricCamera    *p_otherCamera_in,
                                         Sophus::SE3f       &pose1_in,
                                         Sophus::SE3f       &pose2_in,
                                         const float         sigmaLevel1_in,
                                         const float         sigmaLevel2_in,
                                         Eigen::Vector3f    &point3D_out)
{
    Eigen::Matrix<float, 3, 4> eigTcw1 = pose1_in.matrix3x4();
    Eigen::Matrix3f            Rcw1    = eigTcw1.block<3, 3>(0, 0);
    Eigen::Matrix3f            Rwc1    = Rcw1.transpose();
    Eigen::Matrix<float, 3, 4> eigTcw2 = pose2_in.matrix3x4();
    Eigen::Matrix3f            Rcw2    = eigTcw2.block<3, 3>(0, 0);
    Eigen::Matrix3f            Rwc2    = Rcw2.transpose();

    cv::Point3f ray1c = this->unproject(keypoint1_in.pt);
    cv::Point3f ray2c = p_otherCamera_in->unproject(keypoint2_in.pt);

    Eigen::Vector3f r1(ray1c.x, ray1c.y, ray1c.z);
    Eigen::Vector3f r2(ray2c.x, ray2c.y, ray2c.z);

    // Check parallax between rays
    Eigen::Vector3f ray1 = Rwc1 * r1;
    Eigen::Vector3f ray2 = Rwc2 * r2;

    const float cosParallaxRays = ray1.dot(ray2) / (ray1.norm() * ray2.norm());

    // If parallax is lower than 0.9998, reject this match
    if (cosParallaxRays > 0.9998)
    {
        return false;
    }

    // Parallax is good, so we try to triangulate
    cv::Point2f p11, p22;

    p11.x = ray1c.x;
    p11.y = ray1c.y;

    p22.x = ray2c.x;
    p22.y = ray2c.y;

    Eigen::Vector3f x3D;

    triangulate(p11, p22, eigTcw1, eigTcw2, x3D);

    // Check triangulation in front of cameras
    float z1 = Rcw1.row(2).dot(x3D) + pose1_in.translation()(2);
    if (z1 <= 0)
    { // Point is not in front of the first camera
        return false;
    }

    float z2 = Rcw2.row(2).dot(x3D) + pose2_in.translation()(2);
    if (z2 <= 0)
    { // Point is not in front of the first camera
        return false;
    }

    // Check reprojection error in first keyframe
    //   -Transform point into camera reference system
    Eigen::Vector3f x3D1 = Rcw1 * x3D + pose1_in.translation();
    Eigen::Vector2f uv1  = this->project(x3D1);

    float errX1 = uv1(0) - keypoint1_in.pt.x;
    float errY1 = uv1(1) - keypoint1_in.pt.y;

    if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaLevel1_in)
    { // Reprojection error is high
        return false;
    }

    // Check reprojection error in second keyframe;
    //   -Transform point into camera reference system
    Eigen::Vector3f x3D2 = Rcw2 * x3D + pose2_in.translation(); // avoid using q
    Eigen::Vector2f uv2  = p_otherCamera_in->project(x3D2);

    float errX2 = uv2(0) - keypoint2_in.pt.x;
    float errY2 = uv2(1) - keypoint2_in.pt.y;

    if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaLevel2_in)
    { // Reprojection error is high
        return false;
    }

    // Since parallax is big enough and reprojection errors are low, this pair
    // of points can be considered as a match
    point3D_out = x3D;

    return true;
}

float KannalaBrandt8::triangulateMatches(
    GeometricCamera       *p_otherCamera_in,
    const cv::KeyPoint    &keypoint1_in,
    const cv::KeyPoint    &keypoint2_in,
    const Eigen::Matrix3f &rotation12_in,
    const Eigen::Vector3f &translation12_in,
    const float            sigmaLevel_in,
    const float            uncertainty_in,
    Eigen::Vector3f       &point3D_out)
{

    Eigen::Vector3f r1 = this->unprojectEig(keypoint1_in.pt);
    Eigen::Vector3f r2 = p_otherCamera_in->unprojectEig(keypoint2_in.pt);

    // Check parallax
    Eigen::Vector3f r21 = rotation12_in * r2;

    const float cosParallaxRays = r1.dot(r21) / (r1.norm() * r21.norm());

    if (cosParallaxRays > 0.9998)
    {
        return -1;
    }

    // Parallax is good, so we try to triangulate
    cv::Point2f p11, p22;

    p11.x = r1[0];
    p11.y = r1[1];

    p22.x = r2[0];
    p22.y = r2[1];

    Eigen::Vector3f            x3D;
    Eigen::Matrix<float, 3, 4> pose1_in;
    pose1_in << Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero();

    Eigen::Matrix<float, 3, 4> pose2_in;

    Eigen::Matrix3f R21 = rotation12_in.transpose();
    pose2_in << R21, -R21 * translation12_in;

    triangulate(p11, p22, pose1_in, pose2_in, x3D);
    // cv::Mat x3Dt = x3D.t();

    float z1 = x3D(2);
    if (z1 <= 0)
    {
        return -2;
    }

    float z2 = R21.row(2).dot(x3D) + pose2_in(2, 3);
    if (z2 <= 0)
    {
        return -3;
    }

    // Check reprojection error
    Eigen::Vector2f uv1 = this->project(x3D);

    float errX1 = uv1(0) - keypoint1_in.pt.x;
    float errY1 = uv1(1) - keypoint1_in.pt.y;

    if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaLevel_in)
    { // Reprojection error is high
        return -4;
    }

    Eigen::Vector3f x3D2 = R21 * x3D + pose2_in.col(3);
    Eigen::Vector2f uv2  = p_otherCamera_in->project(x3D2);

    float errX2 = uv2(0) - keypoint2_in.pt.x;
    float errY2 = uv2(1) - keypoint2_in.pt.y;

    if ((errX2 * errX2 + errY2 * errY2) > 5.991 * uncertainty_in)
    { // Reprojection error is high
        return -5;
    }

    point3D_out = x3D;

    return z1;
}

std::ostream &operator<<(std::ostream &os, const KannalaBrandt8 &kannala_in)
{
    os << kannala_in.parameters[0] << " " << kannala_in.parameters[1] << " "
       << kannala_in.parameters[2] << " " << kannala_in.parameters[3] << " "
       << kannala_in.parameters[4] << " " << kannala_in.parameters[5] << " "
       << kannala_in.parameters[6] << " " << kannala_in.parameters[7];
    return os;
}

std::istream &operator>>(std::istream &is, KannalaBrandt8 &kannala_inout)
{
    float nextParam;
    for (size_t i = 0; i < 8; i++)
    {
        assert(is.good()); // Make sure the input stream is good
        is >> nextParam;
        kannala_inout.parameters[i] = nextParam;
    }
    return is;
}

void KannalaBrandt8::triangulate(const cv::Point2f                &point1_in,
                                 const cv::Point2f                &point2_in,
                                 const Eigen::Matrix<float, 3, 4> &pose1_in,
                                 const Eigen::Matrix<float, 3, 4> &pose2_in,
                                 Eigen::Vector3f                  &point3D_out)
{
    Eigen::Matrix<float, 4, 4> A;
    A.row(0) = point1_in.x * pose1_in.row(2) - pose1_in.row(0);
    A.row(1) = point1_in.y * pose1_in.row(2) - pose1_in.row(1);
    A.row(2) = point2_in.x * pose2_in.row(2) - pose2_in.row(0);
    A.row(3) = point2_in.y * pose2_in.row(2) - pose2_in.row(1);

    Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);
    Eigen::Vector4f                   x3Dh = svd.matrixV().col(3);
    point3D_out                            = x3Dh.head(3) / x3Dh(3);
}

bool KannalaBrandt8::isEqual(GeometricCamera *p_camera_in)
{
    if (p_camera_in->getType() != GeometricCamera::CAM_FISHEYE)
        return false;

    KannalaBrandt8 *p_kannala_in = (KannalaBrandt8 *)p_camera_in;

    if (abs(precision - p_kannala_in->getPrecision()) > 1e-6)
        return false;

    if (size() != p_kannala_in->size())
        return false;

    bool is_same_camera = true;
    for (size_t i = 0; i < size(); ++i)
    {
        if (abs(parameters[i] - p_kannala_in->getParameter(i)) > 1e-6)
        {
            is_same_camera = false;
            break;
        }
    }
    return is_same_camera;
}

} // namespace camera_models
} // namespace core
} // namespace vs_graphs