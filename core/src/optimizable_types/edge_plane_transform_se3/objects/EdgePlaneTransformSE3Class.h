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

#ifndef VS_GRAPHS_OPTIMIZABLE_TYPES_EDGE_PLANE_TRANSFORM_SE3_H
#define VS_GRAPHS_OPTIMIZABLE_TYPES_EDGE_PLANE_TRANSFORM_SE3_H

#include "g2o_types/unsupported_graph_io/unsupported_graph_io.h"
#include "optimizable_types/plane_pair_measurement/plane_pair_measurement.h"
#include <Eigen/Dense>
#include <vs_graphs_g2o/core/base_binary_edge.h>
#include <vs_graphs_g2o/core/base_unary_edge.h>
#include <vs_graphs_g2o/core/base_vertex.h>
#include <vs_graphs_g2o/types/types_six_dof_expmap.h>
#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs::core::optimizable_types::edge_plane_transform_se3
{

/*!
 * One unary factor on the inter-map SE(3) transform per accepted wall
 * correspondence. Residual is [orientation (2D tangent at n_B), offset
 * (1D)]. Only computeError() is provided: this codebase's
 * VertexSE3Expmap::oplusImpl perturbs on the LEFT
 * (setEstimate(SE3Quat::exp(update) * estimate())), so no analytical
 * Jacobian is derived under the opposite convention; this edge relies
 * on g2o::BaseUnaryEdge's own numerical linearizeOplus()
 * (base_unary_edge.hpp), which probes the vertex's actual oplus() and
 * is therefore correct regardless of the perturbation-side convention
 * (numerical differentiation with central differences is the explicit
 * fallback).
 */
class EdgePlaneTransformSE3
    : public g2o_types::unsupported_graph_io::UnsupportedGraphIo<
          g2o::BaseUnaryEdge<
              3,
              optimizable_types::plane_pair_measurement::PlanePairMeasurement,
              g2o::VertexSE3Expmap>>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgePlaneTransformSE3() {}

    /*!
     * @brief           Computes the 3D error: two orientation components in the
     *                  tangent plane at n_B, plus the offset difference, for
     *                  the source plane moved by the SE3 vertex.
     */
    void computeError() override;
};

} /* namespace vs_graphs::core::optimizable_types::edge_plane_transform_se3 */

#endif /* VS_GRAPHS_OPTIMIZABLE_TYPES_EDGE_PLANE_TRANSFORM_SE3_H */
