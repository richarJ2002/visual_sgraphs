/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Ra\u00fal Mur-Artal, Carlos Campos, Richard Elvira, Juan J. G\u00f3mez
 * Rodr\u00edguez, Jos\u00e9 M.M. Montiel, and Juan D. Tard\u00f3s.
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
 * @file            VertexGravityDirectionClass.h
 *
 * @brief           Declares VertexGravityDirection, the g2o vertex holding a
 * gravity direction estimate.
 */

#ifndef VS_GRAPHS_CORE_SRC_G2O_TYPES_VERTEX_GRAVITY_DIRECTION_OBJECTS_VERTEX_GRAVITY_DIRECTION_CLASS_H
#define VS_GRAPHS_CORE_SRC_G2O_TYPES_VERTEX_GRAVITY_DIRECTION_OBJECTS_VERTEX_GRAVITY_DIRECTION_CLASS_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vs_graphs_g2o/core/base_vertex.h>

#include "g2o_types/gravity_direction/gravity_direction.h"
#include "g2o_types/unsupported_graph_io/unsupported_graph_io.h"

namespace vs_graphs
{
namespace core
{
namespace g2o_types
{
namespace vertex_gravity_direction
{

/*!
 * @brief           g2o vertex whose estimate is a GravityDirection, the
 * direction of gravity in the world, with two degrees of freedom.
 */
class VertexGravityDirection
    : public unsupported_graph_io::UnsupportedGraphIo<
          g2o::BaseVertex<2, gravity_direction::GravityDirection>>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexGravityDirection() {}
    /*!
     * @brief           Creates the vertex with a known gravity rotation as its
     *                  estimate.
     *
     * @param[in]       rwg_in
     *                  Rotation from the gravity-aligned frame to the world
     *                  frame.
     */
    VertexGravityDirection(Eigen::Matrix3d rwg_in)
    {
        setEstimate(gravity_direction::GravityDirection(rwg_in));
    }

    /*!
     * @brief           Does nothing; required by g2o, and the estimate keeps
     *                  its current value.
     */
    void setToOriginImpl() override;

    /*!
     * @brief           Applies an optimiser step to the gravity direction and
     *                  refreshes the g2o cache.
     *
     * @param[in]       p_update_in
     *                  Two values: rotation about the x and y axes of the
     *                  gravity-aligned frame, radians.
     */
    void oplusImpl(const double *p_update_in) override;
};

} /* namespace vertex_gravity_direction */
} /* namespace g2o_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_G2O_TYPES_VERTEX_GRAVITY_DIRECTION_OBJECTS_VERTEX_GRAVITY_DIRECTION_CLASS_H \
        */
