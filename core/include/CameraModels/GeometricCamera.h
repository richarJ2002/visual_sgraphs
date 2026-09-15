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

#ifndef CAMERAMODELS_GEOMETRICCAMERA_H
#define CAMERAMODELS_GEOMETRICCAMERA_H

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/assume_abstract.hpp>

#include <sophus/se3.hpp>

#include <Eigen/Geometry>

#include "Converter.h"
#include "GeometricTools.h"

namespace vs_graphs
{
namespace core
{
namespace camera_models
{
    class GeometricCamera
    {

        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar & id;
            ar & type;
            ar & parameters;
        }

    public:
        GeometricCamera() {}
        GeometricCamera(const std::vector<float> &parameters_in) : parameters(parameters_in) {}
        ~GeometricCamera() {}

        virtual cv::Point2f project(const cv::Point3f &point3D_in) = 0;
        virtual Eigen::Vector2d project(const Eigen::Vector3d &point3D_in) = 0;
        virtual Eigen::Vector2f project(const Eigen::Vector3f &point3D_in) = 0;
        virtual Eigen::Vector2f projectMat(const cv::Point3f &point3D_in) = 0;

        virtual float uncertainty2(const Eigen::Matrix<double, 2, 1> &point2D_in) = 0;

        virtual Eigen::Vector3f unprojectEig(const cv::Point2f &point2D_in) = 0;
        virtual cv::Point3f unproject(const cv::Point2f &point2D_in) = 0;

        virtual Eigen::Matrix<double, 2, 3> projectJac(const Eigen::Vector3d &point3D_in) = 0;

        virtual bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint> &keys1_in, const std::vector<cv::KeyPoint> &keys2_in, const std::vector<int> &matches12_in,
                                              Sophus::SE3f &pose21_out, std::vector<cv::Point3f> &points3D_out, std::vector<bool> &triangulated_out) = 0;

        virtual cv::Mat toK() = 0;
        virtual Eigen::Matrix3f toK_() = 0;

        virtual bool epipolarConstrain(GeometricCamera *p_otherCamera_in, const cv::KeyPoint &keypoint1_in, const cv::KeyPoint &keypoint2_in, const Eigen::Matrix3f &rotation12_in, const Eigen::Vector3f &translation12_in, const float sigmaLevel_in, const float uncertainty_in) = 0;

        float getParameter(const int index_in) { return parameters[index_in]; }
        void setParameter(const float value_in, const size_t index_in) { parameters[index_in] = value_in; }

        size_t size() { return parameters.size(); }

        virtual bool matchAndtriangulate(const cv::KeyPoint &keypoint1_in, const cv::KeyPoint &keypoint2_in, GeometricCamera *p_otherCamera_in,
                                         Sophus::SE3f &pose1_in, Sophus::SE3f &pose2_in,
                                         const float sigmaLevel1_in, const float sigmaLevel2_in,
                                         Eigen::Vector3f &point3D_out) = 0;

        unsigned int getId() { return id; }

        unsigned int getType() { return type; }

        const static unsigned int CAM_PINHOLE = 0;
        const static unsigned int CAM_FISHEYE = 1;

        static long unsigned int nextId;

    protected:
        std::vector<float> parameters;

        unsigned int id;

        unsigned int type;
    };
} // namespace camera_models
} // namespace core
} // namespace vs_graphs
#endif // CAMERAMODELS_GEOMETRICCAMERA_H
