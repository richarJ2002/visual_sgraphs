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
 * @file            VertexVelocityClass.h
 *
 * @brief           Declares the VertexVelocity
 *                  Velocity vertex holding one world-frame velocity estimate.
 */

#ifndef VS_GRAPHS_CORE_SRC_G2O_TYPES_VERTEX_VELOCITY_OBJECTS_VERTEX_VELOCITY_CLASS_H
#define VS_GRAPHS_CORE_SRC_G2O_TYPES_VERTEX_VELOCITY_OBJECTS_VERTEX_VELOCITY_CLASS_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vs_graphs_g2o/core/base_vertex.h>

#include <rclcpp/logging.hpp>

#include "map_state/frame/frame.h"
#include "optimization/g2o_types/unsupported_graph_io/unsupported_graph_io.h"
#include "map_state/keyframe/keyframe.h"

namespace vs_graphs
{
namespace core
{
namespace g2o_types
{
namespace vertex_velocity
{

/*!
 * @brief           Velocity vertex holding one world-frame velocity estimate.
 */
class VertexVelocity : public unsupported_graph_io::UnsupportedGraphIo<
                           g2o::BaseVertex<3, Eigen::Vector3d>>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief           Creates a default velocity vertex.
     */
    VertexVelocity() {}
    /*!
     * @brief           Creates the vertex with the velocity of a key frame as
     *                  its estimate.
     *
     * @param[in]       p_keyFrame_inout
     *                  Key frame to read the velocity from; dereferenced
     *                  without a null check, not changed and not kept.
     */
    VertexVelocity(keyframe::KeyFrame *p_keyFrame_inout)
    {
        Eigen::Vector3f keyFrameVelocity{};
        if (p_keyFrame_inout->getVelocity(keyFrameVelocity) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getVelocity returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        setEstimate(keyFrameVelocity.cast<double>());
    }
    /*!
     * @brief           Creates the vertex with the velocity of a frame as its
     *                  estimate.
     *
     * @param[in]       p_pF_inout
     *                  Frame to read the velocity from; dereferenced without a
     *                  null check, not changed and not kept.
     */
    VertexVelocity(frame::Frame *p_pF_inout)
    {
        Eigen::Vector3f pFGetVelocity{};
        if (p_pF_inout->getVelocity(pFGetVelocity) !=
            frame::FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getVelocity returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        setEstimate(pFGetVelocity.cast<double>());
    }
    /*!
     * @brief           Does nothing; required by g2o, and the estimate keeps
     *                  its current value.
     */
    void setToOriginImpl() override;
    /*!
     * @brief           Adds an optimiser step to the velocity.
     *
     * @param[in]       p_update_in
     *                  Three values: change of the velocity in the world frame,
     *                  metres per second.
     */
    void oplusImpl(const double *p_update_in) override;
};

} /* namespace vertex_velocity */
} /* namespace g2o_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_G2O_TYPES_VERTEX_VELOCITY_OBJECTS_VERTEX_VELOCITY_CLASS_H \
        */
