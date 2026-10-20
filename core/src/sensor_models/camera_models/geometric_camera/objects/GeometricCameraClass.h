/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            GeometricCameraClass.h
 *
 * @brief           Declares the GeometricCamera abstract interface
 *                  for geometric camera models.
 */

#ifndef VS_GRAPHS_CORE_SRC_CAMERA_MODELS_GEOMETRIC_CAMERA_OBJECTS_GEOMETRIC_CAMERA_CLASS_H
#define VS_GRAPHS_CORE_SRC_CAMERA_MODELS_GEOMETRIC_CAMERA_OBJECTS_GEOMETRIC_CAMERA_CLASS_H

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/core/core.hpp>

#include "dependency_lineage/sophus/objects/se3.hpp"

#include <boost/serialization/access.hpp>

#include "sensor_models/camera_models/geometric_camera/objects/GeometricCameraStatusEnum.h"

namespace vs_graphs
{
namespace core
{
namespace camera_models
{
namespace geometric_camera
{
/*!
 * @brief           Abstract interface for geometric camera models
 *                  projecting camera-frame points into images.
 */
class GeometricCamera
{
    friend class boost::serialization::access;

    /*!
     * @brief           Serializes the camera identifier, type and
     *                  parameters.
     *
     * @param[in,out]   ar
     *                  Archive receiving the stored fields.
     *
     * @param[in]       version
     *                  Archive version; currently unused.
     */
    template <class Archive>
    void serialize(Archive &ar, [[maybe_unused]] const unsigned int version);

  public:
    /*!
     * @brief           Creates a camera with an empty parameter
     *                  vector.
     */
    GeometricCamera() {}
    /*!
     * @brief           Creates a camera from calibration
     *                  parameters.
     *
     * @param[in]       parameters_in
     *                  Calibration entries; layout depends on the
     *                  model.
     */
    GeometricCamera(const std::vector<float> &parameters_in) :
        parameters(parameters_in)
    {}
    /*!
     * @brief           Destroys the camera; virtual so that deleting a camera
     *                  through this base type runs the derived destructor.
     */
    virtual ~GeometricCamera() = default;

    /*!
     * @brief           Projects a camera-frame point into the image.
     *
     * @param[in]       point3d_in
     *                  Point expressed in the camera frame.
     *
     * @return          Pixel coordinates of the projection.
     */
    virtual cv::Point2f     project(const cv::Point3f &point3d_in) = 0;
    /*!
     * @brief           Projects a camera-frame point into the image.
     *
     * @param[in]       point3d_in
     *                  Point expressed in the camera frame.
     *
     * @return          Pixel coordinates of the projection.
     */
    virtual Eigen::Vector2d project(const Eigen::Vector3d &point3d_in) = 0;
    /*!
     * @brief           Projects a camera-frame point into the image.
     *
     * @param[in]       point3d_in
     *                  Point expressed in the camera frame.
     *
     * @return          Pixel coordinates of the projection.
     */
    virtual Eigen::Vector2f project(const Eigen::Vector3f &point3d_in) = 0;
    /*!
     * @brief           Projects a camera-frame point into the image.
     *
     * @param[in]       point3d_in
     *                  Point expressed in the camera frame.
     *
     * @return          Pixel coordinates as an Eigen vector.
     */
    virtual Eigen::Vector2f projectMat(const cv::Point3f &point3d_in) = 0;

    /*!
     * @brief           Returns the squared uncertainty scale applied
     *                  to observations at the given pixel.
     *
     * @param[in]       point2d_in
     *                  Pixel whose scale is requested.
     *
     * @return          Squared scale factor; the shipped models
     *                  return one for uniform weighting.
     */
    virtual float
        uncertainty2(const Eigen::Matrix<double, 2, 1> &point2d_in) = 0;

    /*!
     * @brief           Back-projects a pixel into a camera-frame
     *                  ray.
     *
     * @param[in]       point2d_in
     *                  Pixel to back-project.
     *
     * @return          Ray through the pixel in the camera frame.
     */
    virtual Eigen::Vector3f unprojectEig(const cv::Point2f &point2d_in) = 0;
    /*!
     * @brief           Back-projects a pixel into a camera-frame
     *                  ray.
     *
     * @param[in]       point2d_in
     *                  Pixel to back-project.
     *
     * @return          Ray through the pixel in the camera frame.
     */
    virtual cv::Point3f     unproject(const cv::Point2f &point2d_in) = 0;

    /*!
     * @brief           Returns the Jacobian of the projection at a
     *                  camera-frame point.
     *
     * @param[in]       point3d_in
     *                  Point expressed in the camera frame.
     *
     * @return          Two-by-three Jacobian with image-x then
     *                  image-y rows.
     */
    virtual Eigen::Matrix<double, 2, 3>
        computeProjectionJacobian(const Eigen::Vector3d &point3d_in) = 0;

    /*!
     * @brief           Estimates the relative pose between two views
     *                  and triangulates the matched keypoints.
     *
     * @param[in]       keys1_in
     *                  Keypoints of the first view.
     *
     * @param[in]       keys2_in
     *                  Keypoints of the second view.
     *
     * @param[in]       matches12_in
     *                  Per-keypoint match indices from the first
     *                  view into the second view.
     *
     * @param[in,out]   pose21_inout
     *                  Estimated pose of the second view in the
     *                  first view frame.
     *
     * @param[in,out]   points3d_inout
     *                  Triangulated points.
     *
     * @param[in,out]   triangulated_inout
     *                  Per-match flag reporting a valid
     *                  triangulation.
     *
     * @return          True when the two-view reconstruction
     *                  succeeds.
     */
    virtual bool
        reconstructWithTwoViews(const std::vector<cv::KeyPoint> &keys1_in,
                                const std::vector<cv::KeyPoint> &keys2_in,
                                const std::vector<int>          &matches12_in,
                                Sophus::SE3f                    &pose21_inout,
                                std::vector<cv::Point3f>        &points3d_inout,
                                std::vector<bool> &triangulated_inout) = 0;

    /*!
     * @brief           Returns the three-by-three calibration
     *                  matrix.
     *
     * @return          Calibration matrix in single precision.
     */
    virtual cv::Mat         toK() = 0;
    /*!
     * @brief           Returns the three-by-three calibration
     *                  matrix.
     *
     * @return          Calibration matrix in single precision.
     */
    virtual Eigen::Matrix3f toK_() = 0;

    /*!
     * @brief           Checks whether two keypoints satisfy the
     *                  epipolar constraint between the cameras.
     *
     * @param[in,out]   p_otherCamera_inout
     *                  Non-owning pointer to the second camera;
     *                  shall be non-null.
     *
     * @param[in]       keypoint1_in
     *                  Keypoint in this camera view.
     *
     * @param[in]       keypoint2_in
     *                  Keypoint in the second camera view.
     *
     * @param[in]       rotation12_in
     *                  Rotation from the first camera frame into
     *                  the second.
     *
     * @param[in]       translation12_in
     *                  Translation from the first camera frame
     *                  into the second, in metres.
     *
     * @param[in]       sigmaLevel_in
     *                  Scale variance of the keypoint level.
     *
     * @param[in]       uncertainty_in
     *                  Pixel uncertainty scale.
     *
     * @return          True when the pair passes the epipolar
     *                  test.
     */
    virtual bool epipolarConstrain(GeometricCamera       *p_otherCamera_inout,
                                   const cv::KeyPoint    &keypoint1_in,
                                   const cv::KeyPoint    &keypoint2_in,
                                   const Eigen::Matrix3f &rotation12_in,
                                   const Eigen::Vector3f &translation12_in,
                                   const float            sigmaLevel_in,
                                   const float            uncertainty_in) = 0;

    /*!
     * @brief           Returns the calibration entry at the given
     *                  index.
     *
     * @param[in]       index_in
     *                  Entry to read.
     *
     * @param[out]      parameter_out
     *                  Stored calibration value.
     *
     * @return          GEOMETRIC_CAMERA_STATUS_SUCCESS.
     */
    [[nodiscard]] GeometricCameraStatus getParameter(const int index_in,
                                                     float    &parameter_out);
    /*!
     * @brief           Stores a calibration entry.
     *
     * @param[in]       value_in
     *                  Value stored at the entry.
     *
     * @param[in]       index_in
     *                  Entry to update.
     */
    [[nodiscard]] GeometricCameraStatus setParameter(const float  value_in,
                                                     const size_t index_in);

    /*!
     * @brief           Returns the number of calibration entries.
     *
     * @param[out]      size_out
     *                  Size of the parameter vector.
     *
     * @return          GEOMETRIC_CAMERA_STATUS_SUCCESS.
     */
    [[nodiscard]] GeometricCameraStatus size(size_t &size_out);

    /*!
     * @brief           Validates a keypoint pair and triangulates
     *                  it.
     *
     * @param[in]       keypoint1_in
     *                  Keypoint in this camera view.
     *
     * @param[in]       keypoint2_in
     *                  Keypoint in the second camera view.
     *
     * @param[in,out]   p_otherCamera_inout
     *                  Non-owning pointer to the second camera;
     *                  shall be non-null.
     *
     * @param[in,out]   pose1_inout
     *                  Pose of this camera in the world frame.
     *
     * @param[in,out]   pose2_inout
     *                  Pose of the second camera in the world
     *                  frame.
     *
     * @param[in]       sigmaLevel1_in
     *                  Scale variance of the first keypoint level.
     *
     * @param[in]       sigmaLevel2_in
     *                  Scale variance of the second keypoint level.
     *
     * @param[in,out]   point3d_inout
     *                  Triangulated point in the world frame.
     *
     * @return          True when the pair is accepted and
     *                  point3D_out was set.
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
     * @brief           Returns the unique camera identifier.
     *
     * @param[out]      id_out
     *                  Identifier assigned at construction.
     *
     * @return          GEOMETRIC_CAMERA_STATUS_SUCCESS.
     */
    [[nodiscard]] GeometricCameraStatus getId(unsigned int &id_out) const;

    /*!
     * @brief           Returns the camera model type.
     *
     * @param[out]      type_out
     *                  CAM_PINHOLE or CAM_FISHEYE.
     *
     * @return          GEOMETRIC_CAMERA_STATUS_SUCCESS.
     */
    [[nodiscard]] GeometricCameraStatus getType(unsigned int &type_out) const;

    /*!
     * @brief           Model type tag for pinhole cameras.
     */
    const static unsigned int CAM_PINHOLE = 0;
    /*!
     * @brief           Model type tag for fisheye cameras.
     */
    const static unsigned int CAM_FISHEYE = 1;

    /*!
     * @brief           Source of unique camera identifiers.
     */
    static long unsigned int nextId;

  protected:
    /*!
     * @brief           Calibration entries; layout depends on the
     *                  model.
     */
    std::vector<float> parameters;

    /*!
     * @brief           Unique camera identifier.
     */
    unsigned int id;

    /*!
     * @brief           Camera model type tag.
     */
    unsigned int type;
};

} /* namespace geometric_camera */
} /* namespace camera_models */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_CAMERA_MODELS_GEOMETRIC_CAMERA_OBJECTS_GEOMETRIC_CAMERA_CLASS_H \
        */
