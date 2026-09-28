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
 * @file         GeometricCamera.h
 *
 * @brief        Declares the abstract geometric camera interface.
 */

#ifndef CAMERAMODELS_GEOMETRICCAMERA_H
#define CAMERAMODELS_GEOMETRICCAMERA_H

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/serialization/access.hpp>
#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

#include <sophus/se3.hpp>

#include <Eigen/Geometry>

#include "GeometricTools.h"
#include "Utils/Converter/objects/Converter.h"

namespace vs_graphs
{
namespace core
{
namespace camera_models
{
namespace geometriccamera
{
/*!
 * @brief        Abstract interface for geometric camera models
 *               projecting camera-frame points into images.
 */
class GeometricCamera
{
    friend class boost::serialization::access;

    /*!
     * @brief        Serializes the camera identifier, type and
     *               parameters.
     *
     * @param[in,out] ar
     *                Archive receiving the stored fields.
     * @param[in]    version
     *               Archive version; currently unused.
     */
    template <class Archive>
    void serialize(Archive &ar, [[maybe_unused]] const unsigned int version)
    {
        ar & id;
        ar & type;
        ar & parameters;
    }

  public:
    /*!
     * @brief        Creates a camera with an empty parameter
     *               vector.
     */
    GeometricCamera() {}
    /*!
     * @brief        Creates a camera from calibration
     *               parameters.
     *
     * @param[in]    parameters_in
     *               Calibration entries; layout depends on the
     *               model.
     */
    GeometricCamera(const std::vector<float> &parameters_in) :
        parameters(parameters_in)
    {}
    /*!
     * @brief        Destroys the camera.
     */
    ~GeometricCamera() {}

    /*!
     * @brief        Projects a camera-frame point into the image.
     *
     * @param[in]    point3d_in
     *               Point expressed in the camera frame.
     *
     * @return       Pixel coordinates of the projection.
     */
    virtual cv::Point2f     project(const cv::Point3f &point3d_in) = 0;
    /*!
     * @brief        Projects a camera-frame point into the image.
     *
     * @param[in]    point3d_in
     *               Point expressed in the camera frame.
     *
     * @return       Pixel coordinates of the projection.
     */
    virtual Eigen::Vector2d project(const Eigen::Vector3d &point3d_in) = 0;
    /*!
     * @brief        Projects a camera-frame point into the image.
     *
     * @param[in]    point3d_in
     *               Point expressed in the camera frame.
     *
     * @return       Pixel coordinates of the projection.
     */
    virtual Eigen::Vector2f project(const Eigen::Vector3f &point3d_in) = 0;
    /*!
     * @brief        Projects a camera-frame point into the image.
     *
     * @param[in]    point3d_in
     *               Point expressed in the camera frame.
     *
     * @return       Pixel coordinates as an Eigen vector.
     */
    virtual Eigen::Vector2f projectMat(const cv::Point3f &point3d_in) = 0;

    /*!
     * @brief        Returns the squared uncertainty scale applied
     *               to observations at the given pixel.
     *
     * @param[in]    point2d_in
     *               Pixel whose scale is requested.
     *
     * @return       Squared scale factor; the shipped models
     *               return one for uniform weighting.
     */
    virtual float
        uncertainty2(const Eigen::Matrix<double, 2, 1> &point2d_in) = 0;

    /*!
     * @brief        Back-projects a pixel into a camera-frame
     *               ray.
     *
     * @param[in]    point2d_in
     *               Pixel to back-project.
     *
     * @return       Ray through the pixel in the camera frame.
     */
    virtual Eigen::Vector3f unprojectEig(const cv::Point2f &point2d_in) = 0;
    /*!
     * @brief        Back-projects a pixel into a camera-frame
     *               ray.
     *
     * @param[in]    point2d_in
     *               Pixel to back-project.
     *
     * @return       Ray through the pixel in the camera frame.
     */
    virtual cv::Point3f     unproject(const cv::Point2f &point2d_in) = 0;

    /*!
     * @brief        Returns the Jacobian of the projection at a
     *               camera-frame point.
     *
     * @param[in]    point3d_in
     *               Point expressed in the camera frame.
     *
     * @return       Two-by-three Jacobian with image-x then
     *               image-y rows.
     */
    virtual Eigen::Matrix<double, 2, 3>
        computeProjectionJacobian(const Eigen::Vector3d &point3d_in) = 0;

    /*!
     * @brief        Estimates the relative pose between two views
     *               and triangulates the matched keypoints.
     *
     * @param[in]    keys1_in
     *               Keypoints of the first view.
     * @param[in]    keys2_in
     *               Keypoints of the second view.
     * @param[in]    matches12_in
     *               Per-keypoint match indices from the first
     *               view into the second view.
     * @param[in,out] pose21_inout
     *               Estimated pose of the second view in the
     *               first view frame.
     * @param[in,out] points3d_inout
     *               Triangulated points.
     * @param[in,out] triangulated_inout
     *               Per-match flag reporting a valid
     *               triangulation.
     *
     * @return       True when the two-view reconstruction
     *               succeeds.
     */
    virtual bool
        reconstructWithTwoViews(const std::vector<cv::KeyPoint> &keys1_in,
                                const std::vector<cv::KeyPoint> &keys2_in,
                                const std::vector<int>          &matches12_in,
                                Sophus::SE3f                    &pose21_inout,
                                std::vector<cv::Point3f>        &points3d_inout,
                                std::vector<bool> &triangulated_inout) = 0;

    /*!
     * @brief        Returns the three-by-three calibration
     *               matrix.
     *
     * @return       Calibration matrix in single precision.
     */
    virtual cv::Mat         toK() = 0;
    /*!
     * @brief        Returns the three-by-three calibration
     *               matrix.
     *
     * @return       Calibration matrix in single precision.
     */
    virtual Eigen::Matrix3f toK_() = 0;

    /*!
     * @brief        Checks whether two keypoints satisfy the
     *               epipolar constraint between the cameras.
     *
     * @param[in,out] p_otherCamera_inout
     *               Non-owning pointer to the second camera;
     *               shall be non-null.
     * @param[in]    keypoint1_in
     *               Keypoint in this camera view.
     * @param[in]    keypoint2_in
     *               Keypoint in the second camera view.
     * @param[in]    rotation12_in
     *               Rotation from the first camera frame into
     *               the second.
     * @param[in]    translation12_in
     *               Translation from the first camera frame
     *               into the second, in metres.
     * @param[in]    sigmaLevel_in
     *               Scale variance of the keypoint level.
     * @param[in]    uncertainty_in
     *               Pixel uncertainty scale.
     *
     * @return       True when the pair passes the epipolar
     *               test.
     */
    virtual bool epipolarConstrain(GeometricCamera       *p_otherCamera_inout,
                                   const cv::KeyPoint    &keypoint1_in,
                                   const cv::KeyPoint    &keypoint2_in,
                                   const Eigen::Matrix3f &rotation12_in,
                                   const Eigen::Vector3f &translation12_in,
                                   const float            sigmaLevel_in,
                                   const float            uncertainty_in) = 0;

    /*!
     * @brief        Returns the calibration entry at the given
     *               index.
     *
     * @param[in]    index_in
     *               Entry to read.
     *
     * @return       Stored calibration value.
     */
    float getParameter(const int index_in)
    {
        return parameters[index_in];
    }
    /*!
     * @brief        Stores a calibration entry.
     *
     * @param[in]    value_in
     *               Value stored at the entry.
     * @param[in]    index_in
     *               Entry to update.
     */
    void setParameter(const float value_in, const size_t index_in)
    {
        parameters[index_in] = value_in;
    }

    /*!
     * @brief        Returns the number of calibration entries.
     *
     * @return       Size of the parameter vector.
     */
    size_t size()
    {
        return parameters.size();
    }

    /*!
     * @brief        Validates a keypoint pair and triangulates
     *               it.
     *
     * @param[in]    keypoint1_in
     *               Keypoint in this camera view.
     * @param[in]    keypoint2_in
     *               Keypoint in the second camera view.
     * @param[in,out] p_otherCamera_inout
     *               Non-owning pointer to the second camera;
     *               shall be non-null.
     * @param[in,out] pose1_inout
     *               Pose of this camera in the world frame.
     * @param[in,out] pose2_inout
     *               Pose of the second camera in the world
     *               frame.
     * @param[in]    sigmaLevel1_in
     *               Scale variance of the first keypoint level.
     * @param[in]    sigmaLevel2_in
     *               Scale variance of the second keypoint level.
     * @param[in,out] point3d_inout
     *               Triangulated point in the world frame.
     *
     * @return       True when the pair is accepted and
     *               point3D_out was set.
     */
    virtual bool matchAndTriangulate(const cv::KeyPoint &keypoint1_in,
                                     const cv::KeyPoint &keypoint2_in,
                                     GeometricCamera    *p_otherCamera_inout,
                                     Sophus::SE3f       &pose1_inout,
                                     Sophus::SE3f       &pose2_inout,
                                     const float         sigmaLevel1_in,
                                     const float         sigmaLevel2_in,
                                     Eigen::Vector3f    &point3d_inout) = 0;

    /*!
     * @brief        Returns the unique camera identifier.
     *
     * @return       Identifier assigned at construction.
     */
    unsigned int getId()
    {
        return id;
    }

    /*!
     * @brief        Returns the camera model type.
     *
     * @return       CAM_PINHOLE or CAM_FISHEYE.
     */
    unsigned int getType()
    {
        return type;
    }

    /*!
     * @brief        Model type tag for pinhole cameras.
     */
    const static unsigned int CAM_PINHOLE = 0;
    /*!
     * @brief        Model type tag for fisheye cameras.
     */
    const static unsigned int CAM_FISHEYE = 1;

    /*!
     * @brief        Source of unique camera identifiers.
     */
    static long unsigned int nextId;

  protected:
    /*!
     * @brief        Calibration entries; layout depends on the
     *               model.
     */
    std::vector<float> parameters;

    /*!
     * @brief        Unique camera identifier.
     */
    unsigned int id;

    /*!
     * @brief        Camera model type tag.
     */
    unsigned int type;
};
} // namespace geometriccamera
} // namespace camera_models
} // namespace core
} // namespace vs_graphs
#endif // CAMERAMODELS_GEOMETRICCAMERA_H
