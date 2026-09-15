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

#ifndef CAMERAMODELS_PINHOLE_H
#define CAMERAMODELS_PINHOLE_H

#include <assert.h>
#include "GeometricCamera.h"
#include "TwoViewReconstruction.h"

namespace vs_graphs
{
namespace core
{
namespace camera_models
{
    class Pinhole : public GeometricCamera
    {

        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar &boost::serialization::base_object<GeometricCamera>(*this);
        }

    public:
        Pinhole()
        {
            parameters.resize(4);
            id = nextId++;
            type = CAM_PINHOLE;
        }
        Pinhole(const std::vector<float> parameters_in) : GeometricCamera(parameters_in), p_twoViewReconstruction(nullptr)
        {
            assert(parameters.size() == 4);
            id = nextId++;
            type = CAM_PINHOLE;
        }

        Pinhole(Pinhole *p_pinhole_in) : GeometricCamera(p_pinhole_in->parameters), p_twoViewReconstruction(nullptr)
        {
            assert(parameters.size() == 4);
            id = nextId++;
            type = CAM_PINHOLE;
        }

        ~Pinhole()
        {
            if (p_twoViewReconstruction)
                delete p_twoViewReconstruction;
        }

        cv::Point2f project(const cv::Point3f &point3D_in);
        Eigen::Vector2d project(const Eigen::Vector3d &point3D_in);
        Eigen::Vector2f project(const Eigen::Vector3f &point3D_in);
        Eigen::Vector2f projectMat(const cv::Point3f &point3D_in);

        float uncertainty2(const Eigen::Matrix<double, 2, 1> &point2D_in);

        Eigen::Vector3f unprojectEig(const cv::Point2f &point2D_in);
        cv::Point3f unproject(const cv::Point2f &point2D_in);

        Eigen::Matrix<double, 2, 3> projectJac(const Eigen::Vector3d &point3D_in);

        bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint> &keys1_in, const std::vector<cv::KeyPoint> &keys2_in, const std::vector<int> &matches12_in,
                                     Sophus::SE3f &pose21_out, std::vector<cv::Point3f> &points3D_out, std::vector<bool> &triangulated_out);

        cv::Mat toK();
        Eigen::Matrix3f toK_();

        bool epipolarConstrain(GeometricCamera *p_otherCamera_in, const cv::KeyPoint &keypoint1_in, const cv::KeyPoint &keypoint2_in, const Eigen::Matrix3f &rotation12_in, const Eigen::Vector3f &translation12_in, const float sigmaLevel_in, const float uncertainty_in);

        bool matchAndtriangulate(const cv::KeyPoint &keypoint1_in, const cv::KeyPoint &keypoint2_in, GeometricCamera *p_otherCamera_in,
                                 Sophus::SE3f &pose1_in, Sophus::SE3f &pose2_in,
                                 const float sigmaLevel1_in, const float sigmaLevel2_in,
                                 Eigen::Vector3f &point3D_out) { return false; }

        friend std::ostream &operator<<(std::ostream &os, const Pinhole &pinhole_in);
        friend std::istream &operator>>(std::istream &is, Pinhole &pinhole_inout);

        bool isEqual(GeometricCamera *p_camera_in);

    private:
        TwoViewReconstruction *p_twoViewReconstruction;
    };
} // namespace camera_models
} // namespace core
} // namespace vs_graphs
#endif
