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
 * @file            VertexPoseClass.h
 *
 * @brief           Declares the VertexPose
 *                  Pose vertex holding the optimised IMU (body) pose with
 * derived camera poses.
 */

#ifndef VS_GRAPHS_CORE_SRC_G2O_TYPES_VERTEX_POSE_OBJECTS_VERTEX_POSE_CLASS_H
#define VS_GRAPHS_CORE_SRC_G2O_TYPES_VERTEX_POSE_OBJECTS_VERTEX_POSE_CLASS_H

#include <istream>
#include <ostream>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vs_graphs_g2o/core/base_vertex.h>

#include "g2o_types/imu_cam_pose/imu_cam_pose.h"

#include "frame/frame.h"
#include "keyframe/keyframe.h"

namespace vs_graphs
{
namespace core
{
namespace g2o_types
{
namespace vertex_pose
{

/*!
 * @brief        Pose vertex holding the optimised IMU (body) pose with derived
 * camera poses.
 */
class VertexPose : public g2o::BaseVertex<6, imu_cam_pose::ImuCamPose>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief        Creates an empty pose vertex with a default estimate.
     */
    VertexPose() {}
    /*!
     * @brief           Creates the vertex with the pose of a key frame as its
     *                  estimate.
     *
     * @param[in]       p_keyFrame_inout
     *                  Key frame to read the pose from; dereferenced without a
     *                  null check, not changed and not kept.
     */
    VertexPose(keyframe::KeyFrame *p_keyFrame_inout)
    {
        setEstimate(imu_cam_pose::ImuCamPose(p_keyFrame_inout));
    }
    /*!
     * @brief           Creates the vertex with the pose of a frame as its
     *                  estimate.
     *
     * @param[in]       p_pF_inout
     *                  Frame to read the pose from; dereferenced without a null
     *                  check, not changed and not kept.
     */
    VertexPose(frame::Frame *p_pF_inout)
    {
        setEstimate(imu_cam_pose::ImuCamPose(p_pF_inout));
    }
    /*!
     * @brief           Reads the camera poses, extrinsics, camera model
     *                  parameters and baseline-focal product from a stream into
     *                  the estimate, in the order write() produces them. The
     *                  number of cameras is that of the current estimate, whose
     *                  camera models receive the parameters. On a failed read
     *                  every value the stream could not give is taken as 0.
     *
     * @param[in,out]   inputStream_inout
     *                  Stream to read from.
     *
     * @return          true when every value was read (the stream has not
     *                  failed); false otherwise.
     */
    bool read(std::istream &inputStream_inout) override;
    /*!
     * @brief           Writes the camera poses, extrinsics, camera model
     *                  parameters and baseline-focal product to a stream,
     *                  separated by spaces.
     *
     * @param[out]      outputStream_out
     *                  Stream to write to.
     *
     * @return          true when the stream is still good after writing.
     */
    bool write(std::ostream &outputStream_out) const override;
    /*!
     * @brief           Does nothing; required by g2o, and the estimate keeps
     *                  its current value.
     */
    void setToOriginImpl() override;
    /*!
     * @brief           Applies an optimiser step to the pose in the body frame
     *                  and refreshes the g2o cache.
     *
     * @param[in]       p_update_in
     *                  Six values: rotation vector (radians) then translation
     *                  (metres), as for imu_cam_pose::ImuCamPose::update.
     */
    void oplusImpl(const double *p_update_in) override;
};

} /* namespace vertex_pose */
} /* namespace g2o_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_G2O_TYPES_VERTEX_POSE_OBJECTS_VERTEX_POSE_CLASS_H \
        */
