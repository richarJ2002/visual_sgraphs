/**
 * This file is part of ORB-SLAM3.
 * Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details: https://www.gnu.org/licenses/
*/

#include "Pinhole.h"

#include <boost/serialization/export.hpp>


namespace vs_graphs
{
namespace core
{
namespace camera_models {

    long unsigned int GeometricCamera::nextId=0;

    cv::Point2f Pinhole::project(const cv::Point3f &point3D_in) {
        return cv::Point2f(parameters[0] * point3D_in.x / point3D_in.z + parameters[2],
                           parameters[1] * point3D_in.y / point3D_in.z + parameters[3]);
    }

    Eigen::Vector2d Pinhole::project(const Eigen::Vector3d &point3D_in) {
        Eigen::Vector2d res;
        res[0] = parameters[0] * point3D_in[0] / point3D_in[2] + parameters[2];
        res[1] = parameters[1] * point3D_in[1] / point3D_in[2] + parameters[3];

        return res;
    }

    Eigen::Vector2f Pinhole::project(const Eigen::Vector3f &point3D_in) {
        Eigen::Vector2f res;
        res[0] = parameters[0] * point3D_in[0] / point3D_in[2] + parameters[2];
        res[1] = parameters[1] * point3D_in[1] / point3D_in[2] + parameters[3];

        return res;
    }

    Eigen::Vector2f Pinhole::projectMat(const cv::Point3f &point3D_in) {
        cv::Point2f point = this->project(point3D_in);
        return Eigen::Vector2f(point.x, point.y);
    }

    float Pinhole::uncertainty2(const Eigen::Matrix<double,2,1> &point2D_in)
    {
        return 1.0;
    }

    Eigen::Vector3f Pinhole::unprojectEig(const cv::Point2f &point2D_in) {
        return Eigen::Vector3f((point2D_in.x - parameters[2]) / parameters[0], (point2D_in.y - parameters[3]) / parameters[1],
                           1.f);
    }

    cv::Point3f Pinhole::unproject(const cv::Point2f &point2D_in) {
        return cv::Point3f((point2D_in.x - parameters[2]) / parameters[0], (point2D_in.y - parameters[3]) / parameters[1],
                           1.f);
    }

    Eigen::Matrix<double, 2, 3> Pinhole::projectJac(const Eigen::Vector3d &point3D_in) {
        Eigen::Matrix<double, 2, 3> Jac;
        Jac(0, 0) = parameters[0] / point3D_in[2];
        Jac(0, 1) = 0.f;
        Jac(0, 2) = -parameters[0] * point3D_in[0] / (point3D_in[2] * point3D_in[2]);
        Jac(1, 0) = 0.f;
        Jac(1, 1) = parameters[1] / point3D_in[2];
        Jac(1, 2) = -parameters[1] * point3D_in[1] / (point3D_in[2] * point3D_in[2]);

        return Jac;
    }

    bool Pinhole::ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& keys1_in, const std::vector<cv::KeyPoint>& keys2_in, const std::vector<int> &matches12_in,
                                 Sophus::SE3f &pose21_out, std::vector<cv::Point3f> &points3D_out, std::vector<bool> &triangulated_out){
        if(!p_twoViewReconstruction){
            Eigen::Matrix3f K = this->toK_();
            p_twoViewReconstruction = new TwoViewReconstruction(K);
        }

        return p_twoViewReconstruction->Reconstruct(keys1_in,keys2_in,matches12_in,pose21_out,points3D_out,triangulated_out);
    }


    cv::Mat Pinhole::toK() {
        cv::Mat K = (cv::Mat_<float>(3, 3)
                << parameters[0], 0.f, parameters[2], 0.f, parameters[1], parameters[3], 0.f, 0.f, 1.f);
        return K;
    }

    Eigen::Matrix3f Pinhole::toK_() {
        Eigen::Matrix3f K;
        K << parameters[0], 0.f, parameters[2], 0.f, parameters[1], parameters[3], 0.f, 0.f, 1.f;
        return K;
    }


    bool Pinhole::epipolarConstrain(GeometricCamera* p_otherCamera_in,  const cv::KeyPoint &keypoint1_in, const cv::KeyPoint &keypoint2_in, const Eigen::Matrix3f& rotation12_in, const Eigen::Vector3f& translation12_in, const float sigmaLevel_in, const float uncertainty_in) {
        //Compute Fundamental Matrix
        Eigen::Matrix3f t12x = Sophus::SO3f::hat(translation12_in);
        Eigen::Matrix3f K1 = this->toK_();
        Eigen::Matrix3f K2 = p_otherCamera_in->toK_();
        Eigen::Matrix3f F12 = K1.transpose().inverse() * t12x * rotation12_in * K2.inverse();
        
        // Epipolar line in second image l = x1'F12 = [a b c]
        const float a = keypoint1_in.pt.x*F12(0,0)+keypoint1_in.pt.y*F12(1,0)+F12(2,0);
        const float b = keypoint1_in.pt.x*F12(0,1)+keypoint1_in.pt.y*F12(1,1)+F12(2,1);
        const float c = keypoint1_in.pt.x*F12(0,2)+keypoint1_in.pt.y*F12(1,2)+F12(2,2);

        const float num = a*keypoint2_in.pt.x+b*keypoint2_in.pt.y+c;

        const float den = a*a+b*b;

        if(den==0)
            return false;

        const float dsqr = num*num/den;

        return dsqr<3.84*uncertainty_in;
    }

    std::ostream & operator<<(std::ostream &os, const Pinhole &pinhole_in) {
        os << pinhole_in.parameters[0] << " " << pinhole_in.parameters[1] << " " << pinhole_in.parameters[2] << " " << pinhole_in.parameters[3];
        return os;
    }

    std::istream & operator>>(std::istream &is, Pinhole &pinhole_inout) {
        float nextParam;
        for(size_t i = 0; i < 4; i++){
            assert(is.good());  //Make sure the input stream is good
            is >> nextParam;
            pinhole_inout.parameters[i] = nextParam;

        }
        return is;
    }

    bool Pinhole::isEqual(GeometricCamera* p_camera_in)
    {
        if(p_camera_in->getType() != GeometricCamera::CAM_PINHOLE)
            return false;

        Pinhole* p_pinhole_in = (Pinhole*) p_camera_in;

        if(size() != p_pinhole_in->size())
            return false;

        bool is_same_camera = true;
        for(size_t i=0; i<size(); ++i)
        {
            if(abs(parameters[i] - p_pinhole_in->getParameter(i)) > 1e-6)
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