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
 * @file         KannalaBrandt8.h
 *
 * @brief        Declares the Kannala-Brandt fisheye camera model.
 */

#ifndef CAMERAMODELS_KANNALABRANDT8_H
#define CAMERAMODELS_KANNALABRANDT8_H

#include <assert.h>
#include <boost/serialization/access.hpp>

#include "CameraModels/GeometricCamera/objects/GeometricCamera.h"
#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8Status.h"

#include "TwoViewReconstruction.h"

namespace vs_graphs::core::camera_models::kannalabrandt8
{
/*!
 * @brief        Fisheye camera following the Kannala-Brandt
 *               eight-parameter distortion model.
 */
class KannalaBrandt8 : public geometriccamera::GeometricCamera
{

    friend class boost::serialization::access;

    /*!
     * @brief        Serializes the base camera and the solver
     *               precision.
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
     * @brief        Creates a fisheye camera with default solver
     *               precision.
     */
    KannalaBrandt8() :
        precision(1e-6)
    {
        parameters.resize(8);
        id   = nextId++;
        type = CAM_FISHEYE;
    }
    /*!
     * @brief        Creates a fisheye camera from calibration
     *               parameters.
     *
     * @param[in]    parameters_in
     *               Eight entries holding fx, fy, cx, cy, k0,
     *               k1, k2 and k3; the size is asserted.
     */
    KannalaBrandt8(const std::vector<float> parameters_in) :
        geometriccamera::GeometricCamera(parameters_in),
        lappingArea(2, 0),
        precision(1e-6),
        p_twoViewReconstruction(nullptr)
    {
        assert(parameters.size() == 8);
        id   = nextId++;
        type = CAM_FISHEYE;
    }

    /*!
     * @brief        Creates a fisheye camera with custom solver
     *               precision.
     *
     * @param[in]    parameters_in
     *               Eight entries holding fx, fy, cx, cy, k0,
     *               k1, k2 and k3; the size is asserted.
     * @param[in]    precision_in
     *               Newton-solve tolerance for unprojection.
     */
    KannalaBrandt8(const std::vector<float> parameters_in,
                   const float              precision_in) :
        geometriccamera::GeometricCamera(parameters_in),
        lappingArea(2, 0),
        precision(precision_in),
        p_twoViewReconstruction(nullptr)
    {
        assert(parameters.size() == 8);
        id   = nextId++;
        type = CAM_FISHEYE;
    }
    /*!
     * @brief        Copies the calibration of another fisheye
     *               camera under a fresh identifier.
     *
     * @param[in,out] p_kannala_inout
     *               Non-owning source camera; shall be non-null.
     */
    KannalaBrandt8(KannalaBrandt8 *p_kannala_inout) :
        geometriccamera::GeometricCamera(p_kannala_inout->parameters),
        lappingArea(2, 0),
        precision(p_kannala_inout->precision),
        p_twoViewReconstruction(nullptr)
    {
        assert(parameters.size() == 8);
        id   = nextId++;
        type = CAM_FISHEYE;
    }

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
     *              Points are undistorted with the fisheye model
     *              before estimation.
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
     *              Acceptance requires a positive triangulated
     *              depth above 1e-4.
     *
     * @param[in]    p_otherCamera_in
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
    bool epipolarConstrain(geometriccamera::GeometricCamera *p_otherCamera_in,
                           const cv::KeyPoint               &keypoint1_in,
                           const cv::KeyPoint               &keypoint2_in,
                           const Eigen::Matrix3f            &rotation12_in,
                           const Eigen::Vector3f            &translation12_in,
                           const float                       sigmaLevel_in,
                           const float                       uncertainty_in);

    /*!
     * @brief        Triangulates a keypoint pair given the
     *               relative motion between the views.
     *
     *              Rejects pairs with low parallax, points
     *              behind either camera, or reprojection errors
     *              above the chi-squared bounds.
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
     * @param[out]   point3d_out
     *               Triangulated point in the world frame.
     *
     * @param[out] parallax_out Front depth of the triangulated point, or a
     * negative code when the pair is rejected.
     * @return KANNALA_BRANDT8_STATUS_SUCCESS.
     */
    [[nodiscard]] KannalaBrandt8Status triangulateMatches(
        geometriccamera::GeometricCamera *p_otherCamera_inout,
        const cv::KeyPoint               &keypoint1_in,
        const cv::KeyPoint               &keypoint2_in,
        const Eigen::Matrix3f            &rotation12_in,
        const Eigen::Vector3f            &translation12_in,
        const float                       sigmaLevel_in,
        const float                       uncertainty_in,
        Eigen::Vector3f                  &point3d_out,
        float                            &parallax_out);

    /*!
     * @brief        Image column bounds of the stereo overlap
     *               region.
     */
    std::vector<int> lappingArea;

    /*!
     * @brief        Validates a keypoint pair by parallax and
     *               reprojection checks and triangulates it.
     *
     * @param[in]    keypoint1_in
     *               Keypoint in this camera view.
     * @param[in]    keypoint2_in
     *               Keypoint in the second camera view.
     * @param[in,out] p_otherCamera_inout
     *               Non-owning pointer to the second camera;
     *               shall be non-null.
     * @param[in]    pose1_in
     *               Pose of this camera in the world frame.
     * @param[in]    pose2_in
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
    bool matchAndTriangulate(
        const cv::KeyPoint               &keypoint1_in,
        const cv::KeyPoint               &keypoint2_in,
        geometriccamera::GeometricCamera *p_otherCamera_inout,
        Sophus::SE3f                     &pose1_in,
        Sophus::SE3f                     &pose2_in,
        const float                       sigmaLevel1_in,
        const float                       sigmaLevel2_in,
        Eigen::Vector3f                  &point3d_inout);

    /*!
     * @brief        Appends the eight calibration entries to the
     *               stream.
     *
     * @param[in,out] outputStream_inout
     *                Stream receiving the entries.
     * @param[in]    kannala_in
     *               Camera whose entries are written.
     *
     * @return       The output stream.
     */
    friend std::ostream &operator<<(std::ostream         &outputStream_inout,
                                    const KannalaBrandt8 &kannala_in);
    /*!
     * @brief        Reads eight calibration entries from the
     *               stream.
     *
     * @param[in,out] inputStream_inout
     *                Stream holding the entries; shall be good.
     * @param[out]    kannala_out
     *                Camera receiving the entries.
     *
     * @return       The input stream.
     */
    friend std::istream &operator>>(std::istream   &inputStream_inout,
                                    KannalaBrandt8 &kannala_out);

    /*!
     * @brief        Returns the Newton-solve tolerance used for
     *               unprojection.
     *
     * @param[out] precision_out Solver precision.
     * @return KANNALA_BRANDT8_STATUS_SUCCESS.
     */
    [[nodiscard]] KannalaBrandt8Status getPrecision(float &precision_out) const
    {
        precision_out = precision;
        return KannalaBrandt8Status::KANNALA_BRANDT8_STATUS_SUCCESS;
    }

    /*!
     * @brief        Checks calibration equality with another
     *               fisheye camera.
     *
     *              Entries and precisions shall agree within
     *              1e-6.
     *
     * @param[in]    p_camera_in
     *               Non-owning candidate camera; shall be
     *               non-null.
     *
     * @param[out] isEqual_out True when both cameras share the type and
     * calibration.
     * @return KANNALA_BRANDT8_STATUS_SUCCESS.
     */
    [[nodiscard]] KannalaBrandt8Status
        isEqual(geometriccamera::GeometricCamera *p_camera_in,
                bool                             &isEqual_out);

  private:
    /*!
     * @brief        Newton-solve tolerance for unprojection.
     */
    const float precision;

    // Parameters vector corresponds to
    //[fx, fy, cx, cy, k0, k1, k2, k3]

    /*!
     * @brief        Two-view helper created on first
     *               reconstruction.
     */
    TwoViewReconstruction *p_twoViewReconstruction;

    /*!
     * @brief        Triangulates two normalized observations
     *               with a linear solve.
     *
     * @param[in]    point1_in
     *               Observation in the first view.
     * @param[in]    point2_in
     *               Observation in the second view.
     * @param[in]    pose1_in
     *               Three-by-four pose of the first view.
     * @param[in]    pose2_in
     *               Three-by-four pose of the second view.
     * @param[out]   point3d_out
     *               Triangulated point.
     */
    [[nodiscard]] KannalaBrandt8Status
        triangulate(const cv::Point2f                &point1_in,
                    const cv::Point2f                &point2_in,
                    const Eigen::Matrix<float, 3, 4> &pose1_in,
                    const Eigen::Matrix<float, 3, 4> &pose2_in,
                    Eigen::Vector3f                  &point3d_out);
};
} // namespace vs_graphs::core::camera_models::kannalabrandt8

#endif // CAMERAMODELS_KANNALABRANDT8_H
