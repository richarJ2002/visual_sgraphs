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
 * @file         Pinhole.h
 *
 * @brief        Declares the pinhole camera model.
 */

#ifndef CAMERAMODELS_PINHOLE_H
#define CAMERAMODELS_PINHOLE_H

#include "CameraModels/GeometricCamera/objects/GeometricCamera.h"
#include "CameraModels/Pinhole/objects/PinholeStatus.h"
#include "TwoViewReconstruction.h"
#include <assert.h>
#include <boost/serialization/access.hpp>

namespace vs_graphs
{
namespace core
{
namespace camera_models
{
namespace pinhole
{
/*!
 * @brief        Pinhole camera with four calibration entries.
 */
class Pinhole : public geometriccamera::GeometricCamera
{

    friend class boost::serialization::access;

    /*!
     * @brief        Serializes the base camera fields.
     *
     * @param[in,out] ar
     *                Archive receiving the stored fields.
     * @param[in]    version
     *               Archive version; currently unused.
     */
    template <class Archive>
    void serialize(Archive &ar, [[maybe_unused]] const unsigned int version);

  public:
    /*!
     * @brief        Creates a pinhole camera with four
     *               calibration entries.
     */
    Pinhole() :
        p_twoViewReconstruction(nullptr)
    {
        parameters.resize(4);
        id   = nextId++;
        type = CAM_PINHOLE;
    }
    /*!
     * @brief        Creates a pinhole camera from calibration
     *               parameters.
     *
     * @param[in]    parameters_in
     *               Four entries holding fx, fy, cx and cy; the
     *               size is asserted.
     */
    Pinhole(const std::vector<float> parameters_in) :
        geometriccamera::GeometricCamera(parameters_in),
        p_twoViewReconstruction(nullptr)
    {
        assert(parameters.size() == 4);
        id   = nextId++;
        type = CAM_PINHOLE;
    }

    /*!
     * @brief        Copies the calibration of another pinhole
     *               camera under a fresh identifier.
     *
     * @param[in,out] p_pinhole_inout
     *               Non-owning source camera; shall be non-null.
     */
    Pinhole(Pinhole *p_pinhole_inout) :
        geometriccamera::GeometricCamera(p_pinhole_inout->parameters),
        p_twoViewReconstruction(nullptr)
    {
        assert(parameters.size() == 4);
        id   = nextId++;
        type = CAM_PINHOLE;
    }

    /*!
     * @brief        Destroys the camera and its two-view helper.
     */
    ~Pinhole() override
    {
        if (p_twoViewReconstruction)
            delete p_twoViewReconstruction;
    }
    /*!
     * @brief        Copying is forbidden: a copy would share the two-view
     *               helper and both destructors would delete it. Clone the
     *               calibration with Pinhole(Pinhole *) instead.
     */
    Pinhole(const Pinhole &otherPinhole_in)            = delete;
    Pinhole &operator=(const Pinhole &otherPinhole_in) = delete;

    /*!
     * @brief        Projects a camera-frame point into the image.
     *
     * @param[in]    point3d_in
     *               Point expressed in the camera frame.
     *
     * @return       Pixel coordinates of the projection.
     */
    cv::Point2f     project(const cv::Point3f &point3d_in);
    /*!
     * @brief        Projects a camera-frame point into the image.
     *
     * @param[in]    point3d_in
     *               Point expressed in the camera frame.
     *
     * @return       Pixel coordinates of the projection.
     */
    Eigen::Vector2d project(const Eigen::Vector3d &point3d_in);
    /*!
     * @brief        Projects a camera-frame point into the image.
     *
     * @param[in]    point3d_in
     *               Point expressed in the camera frame.
     *
     * @return       Pixel coordinates of the projection.
     */
    Eigen::Vector2f project(const Eigen::Vector3f &point3d_in);
    /*!
     * @brief        Projects a camera-frame point into the image.
     *
     * @param[in]    point3d_in
     *               Point expressed in the camera frame.
     *
     * @return       Pixel coordinates as an Eigen vector.
     */
    Eigen::Vector2f projectMat(const cv::Point3f &point3d_in);

    /*!
     * @brief        Returns the squared uncertainty scale applied
     *               to observations at the given pixel.
     *
     * @param[in]    point2d_in
     *               Pixel whose scale is requested; currently
     *               unused.
     *
     * @return       One for uniform weighting.
     */
    float uncertainty2(const Eigen::Matrix<double, 2, 1> &point2d_in);

    /*!
     * @brief        Back-projects a pixel into a camera-frame
     *               ray.
     *
     * @param[in]    point2d_in
     *               Pixel to back-project.
     *
     * @return       Ray through the pixel in the camera frame.
     */
    Eigen::Vector3f unprojectEig(const cv::Point2f &point2d_in);
    /*!
     * @brief        Back-projects a pixel into a camera-frame
     *               ray.
     *
     * @param[in]    point2d_in
     *               Pixel to back-project.
     *
     * @return       Ray through the pixel in the camera frame.
     */
    cv::Point3f     unproject(const cv::Point2f &point2d_in);

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
    Eigen::Matrix<double, 2, 3>
        computeProjectionJacobian(const Eigen::Vector3d &point3d_in);

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
    bool reconstructWithTwoViews(const std::vector<cv::KeyPoint> &keys1_in,
                                 const std::vector<cv::KeyPoint> &keys2_in,
                                 const std::vector<int>          &matches12_in,
                                 Sophus::SE3f                    &pose21_inout,
                                 std::vector<cv::Point3f> &points3d_inout,
                                 std::vector<bool>        &triangulated_inout);

    /*!
     * @brief        Returns the three-by-three calibration
     *               matrix.
     *
     * @return       Calibration matrix in single precision.
     */
    cv::Mat         toK();
    /*!
     * @brief        Returns the three-by-three calibration
     *               matrix.
     *
     * @return       Calibration matrix in single precision.
     */
    Eigen::Matrix3f toK_();

    /*!
     * @brief        Checks whether two keypoints satisfy the
     *               epipolar constraint between the cameras.
     *
     *              The squared point-to-epipolar-line distance
     *              shall stay below 3.84 times the uncertainty.
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
     *               Scale variance of the keypoint level;
     *               currently unused.
     * @param[in]    uncertainty_in
     *               Pixel uncertainty scale.
     *
     * @return       True when the pair passes the epipolar
     *               test.
     */
    bool
        epipolarConstrain(geometriccamera::GeometricCamera *p_otherCamera_inout,
                          const cv::KeyPoint               &keypoint1_in,
                          const cv::KeyPoint               &keypoint2_in,
                          const Eigen::Matrix3f            &rotation12_in,
                          const Eigen::Vector3f            &translation12_in,
                          const float                       sigmaLevel_in,
                          const float                       uncertainty_in);

    /*!
     * @brief        Rejects every keypoint pair without
     *               triangulating.
     *
     * @param[in]    keypoint1_in
     *               Keypoint in this camera view.
     * @param[in]    keypoint2_in
     *               Keypoint in the second camera view.
     * @param[in,out] p_otherCamera_inout
     *               Non-owning pointer to the second camera.
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
     *               Left unchanged.
     *
     * @return       Always false.
     */
    bool matchAndTriangulate(
        [[maybe_unused]] const cv::KeyPoint               &keypoint1_in,
        [[maybe_unused]] const cv::KeyPoint               &keypoint2_in,
        [[maybe_unused]] geometriccamera::GeometricCamera *p_otherCamera_inout,
        [[maybe_unused]] Sophus::SE3f                     &pose1_inout,
        [[maybe_unused]] Sophus::SE3f                     &pose2_inout,
        [[maybe_unused]] const float                       sigmaLevel1_in,
        [[maybe_unused]] const float                       sigmaLevel2_in,
        [[maybe_unused]] Eigen::Vector3f                  &point3d_inout)
    {
        return false;
    }

    /*!
     * @brief        Appends the four calibration entries to the
     *               stream.
     *
     * @param[in,out] outputStream_inout
     *                Stream receiving the entries.
     * @param[in]    pinhole_in
     *               Camera whose entries are written.
     *
     * @return       The output stream.
     */
    friend std::ostream &operator<<(std::ostream  &outputStream_inout,
                                    const Pinhole &pinhole_in);
    /*!
     * @brief        Reads four calibration entries from the
     *               stream.
     *
     * @param[in,out] inputStream_inout
     *                Stream holding the entries; shall be good.
     * @param[in,out] pinhole_inout
     *                Camera receiving the entries.
     *
     * @return       The input stream.
     */
    friend std::istream &operator>>(std::istream &inputStream_inout,
                                    Pinhole      &pinhole_inout);

    /*!
     * @brief        Checks calibration equality with another
     *               pinhole camera.
     *
     *              Entries shall agree within 1e-6.
     *
     * @param[in]    p_camera_in
     *               Non-owning candidate camera; shall be
     *               non-null.
     *
     * @param[out] isEqual_out True when both cameras share the type and
     * calibration.
     * @return PINHOLE_STATUS_SUCCESS.
     */
    [[nodiscard]] PinholeStatus
        isEqual(geometriccamera::GeometricCamera *p_camera_in,
                bool                             &isEqual_out);

  private:
    /*!
     * @brief        Owned two-view helper created on first
     *               reconstruction and released at destruction.
     */
    TwoViewReconstruction *p_twoViewReconstruction;
};
} // namespace pinhole
} // namespace camera_models
} // namespace core
} // namespace vs_graphs
#endif
