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

#include "GeometricCamera.h"
#include "TwoViewReconstruction.h"
#include <assert.h>

namespace vs_graphs
{
namespace core
{
namespace camera_models
{
/*!
 * @brief        Pinhole camera with four calibration entries.
 */
class Pinhole : public GeometricCamera
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
    void serialize(Archive &ar, const unsigned int version)
    {
        ar &boost::serialization::base_object<GeometricCamera>(*this);
    }

  public:
    /*!
     * @brief        Creates a pinhole camera with four
     *               calibration entries.
     */
    Pinhole()
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
        GeometricCamera(parameters_in),
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
     * @param[in]    p_pinhole_in
     *               Non-owning source camera; shall be non-null.
     */
    Pinhole(Pinhole *p_pinhole_in) :
        GeometricCamera(p_pinhole_in->parameters),
        p_twoViewReconstruction(nullptr)
    {
        assert(parameters.size() == 4);
        id   = nextId++;
        type = CAM_PINHOLE;
    }

    /*!
     * @brief        Destroys the camera and its two-view helper.
     */
    ~Pinhole()
    {
        if (p_twoViewReconstruction)
            delete p_twoViewReconstruction;
    }

    /*!
     * @brief        Projects a camera-frame point into the image.
     *
     * @param[in]    point3D_in
     *               Point expressed in the camera frame.
     *
     * @return       Pixel coordinates of the projection.
     */
    cv::Point2f     project(const cv::Point3f &point3D_in);
    /*!
     * @brief        Projects a camera-frame point into the image.
     *
     * @param[in]    point3D_in
     *               Point expressed in the camera frame.
     *
     * @return       Pixel coordinates of the projection.
     */
    Eigen::Vector2d project(const Eigen::Vector3d &point3D_in);
    /*!
     * @brief        Projects a camera-frame point into the image.
     *
     * @param[in]    point3D_in
     *               Point expressed in the camera frame.
     *
     * @return       Pixel coordinates of the projection.
     */
    Eigen::Vector2f project(const Eigen::Vector3f &point3D_in);
    /*!
     * @brief        Projects a camera-frame point into the image.
     *
     * @param[in]    point3D_in
     *               Point expressed in the camera frame.
     *
     * @return       Pixel coordinates as an Eigen vector.
     */
    Eigen::Vector2f projectMat(const cv::Point3f &point3D_in);

    /*!
     * @brief        Returns the squared uncertainty scale applied
     *               to observations at the given pixel.
     *
     * @param[in]    point2D_in
     *               Pixel whose scale is requested; currently
     *               unused.
     *
     * @return       One for uniform weighting.
     */
    float uncertainty2(const Eigen::Matrix<double, 2, 1> &point2D_in);

    /*!
     * @brief        Back-projects a pixel into a camera-frame
     *               ray.
     *
     * @param[in]    point2D_in
     *               Pixel to back-project.
     *
     * @return       Ray through the pixel in the camera frame.
     */
    Eigen::Vector3f unprojectEig(const cv::Point2f &point2D_in);
    /*!
     * @brief        Back-projects a pixel into a camera-frame
     *               ray.
     *
     * @param[in]    point2D_in
     *               Pixel to back-project.
     *
     * @return       Ray through the pixel in the camera frame.
     */
    cv::Point3f     unproject(const cv::Point2f &point2D_in);

    /*!
     * @brief        Returns the Jacobian of the projection at a
     *               camera-frame point.
     *
     * @param[in]    point3D_in
     *               Point expressed in the camera frame.
     *
     * @return       Two-by-three Jacobian with image-x then
     *               image-y rows.
     */
    Eigen::Matrix<double, 2, 3>
        computeProjectionJacobian(const Eigen::Vector3d &point3D_in);

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
     * @param[out]   pose21_out
     *               Estimated pose of the second view in the
     *               first view frame.
     * @param[out]   points3D_out
     *               Triangulated points.
     * @param[out]   triangulated_out
     *               Per-match flag reporting a valid
     *               triangulation.
     *
     * @return       True when the two-view reconstruction
     *               succeeds.
     */
    bool reconstructWithTwoViews(const std::vector<cv::KeyPoint> &keys1_in,
                                 const std::vector<cv::KeyPoint> &keys2_in,
                                 const std::vector<int>          &matches12_in,
                                 Sophus::SE3f                    &pose21_out,
                                 std::vector<cv::Point3f>        &points3D_out,
                                 std::vector<bool> &triangulated_out);

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
     *               Scale variance of the keypoint level;
     *               currently unused.
     * @param[in]    uncertainty_in
     *               Pixel uncertainty scale.
     *
     * @return       True when the pair passes the epipolar
     *               test.
     */
    bool epipolarConstrain(GeometricCamera       *p_otherCamera_in,
                           const cv::KeyPoint    &keypoint1_in,
                           const cv::KeyPoint    &keypoint2_in,
                           const Eigen::Matrix3f &rotation12_in,
                           const Eigen::Vector3f &translation12_in,
                           const float            sigmaLevel_in,
                           const float            uncertainty_in);

    /*!
     * @brief        Rejects every keypoint pair without
     *               triangulating.
     *
     * @param[in]    keypoint1_in
     *               Keypoint in this camera view.
     * @param[in]    keypoint2_in
     *               Keypoint in the second camera view.
     * @param[in]    p_otherCamera_in
     *               Non-owning pointer to the second camera.
     * @param[in]    pose1_in
     *               Pose of this camera in the world frame.
     * @param[in]    pose2_in
     *               Pose of the second camera in the world
     *               frame.
     * @param[in]    sigmaLevel1_in
     *               Scale variance of the first keypoint level.
     * @param[in]    sigmaLevel2_in
     *               Scale variance of the second keypoint level.
     * @param[out]   point3D_out
     *               Left unchanged.
     *
     * @return       Always false.
     */
    bool matchAndTriangulate(const cv::KeyPoint &keypoint1_in,
                             const cv::KeyPoint &keypoint2_in,
                             GeometricCamera    *p_otherCamera_in,
                             Sophus::SE3f       &pose1_in,
                             Sophus::SE3f       &pose2_in,
                             const float         sigmaLevel1_in,
                             const float         sigmaLevel2_in,
                             Eigen::Vector3f    &point3D_out)
    {
        return false;
    }

    /*!
     * @brief        Appends the four calibration entries to the
     *               stream.
     *
     * @param[in,out] os
     *                Stream receiving the entries.
     * @param[in]    pinhole_in
     *               Camera whose entries are written.
     *
     * @return       The output stream.
     */
    friend std::ostream &operator<<(std::ostream  &os,
                                    const Pinhole &pinhole_in);
    /*!
     * @brief        Reads four calibration entries from the
     *               stream.
     *
     * @param[in,out] is
     *                Stream holding the entries; shall be good.
     * @param[in,out] pinhole_inout
     *                Camera receiving the entries.
     *
     * @return       The input stream.
     */
    friend std::istream &operator>>(std::istream &is, Pinhole &pinhole_inout);

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
     * @return       True when both cameras share the type and
     *               calibration.
     */
    bool isEqual(GeometricCamera *p_camera_in);

  private:
    /*!
     * @brief        Owned two-view helper created on first
     *               reconstruction and released at destruction.
     */
    TwoViewReconstruction *p_twoViewReconstruction;
};
} // namespace camera_models
} // namespace core
} // namespace vs_graphs
#endif
