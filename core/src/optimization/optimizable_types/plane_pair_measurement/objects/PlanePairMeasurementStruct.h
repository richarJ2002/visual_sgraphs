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


#ifndef VS_GRAPHS_OPTIMIZABLE_TYPES_PLANE_PAIR_MEASUREMENT_H
#define VS_GRAPHS_OPTIMIZABLE_TYPES_PLANE_PAIR_MEASUREMENT_H

#include <Eigen/Dense>
#include <istream>
#include <ostream>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <vs_graphs_g2o/core/base_vertex.h>
#include <vs_graphs_g2o/core/base_binary_edge.h>
#include <vs_graphs_g2o/core/base_unary_edge.h>
#include <vs_graphs_g2o/types/types_six_dof_expmap.h>

namespace vs_graphs::core::optimizable_types::plane_pair_measurement
{

/*!
 * A fixed source/target plane-pair measurement for the inter-map
 * transform factor below. Both plane equations and the sign
 * hypothesis are fixed at construction; the surviving plane is not
 * a graph variable.
 */
struct PlanePairMeasurement
{
  public:
    /*!
     * @brief           Unit normal of the source plane, in the absorbed map
     *                  frame.
     */
    Eigen::Vector3d n_A;
    /*!
     * @brief           Offset of the source plane: n_A^T x + d_A = 0.
     */
    double          d_A;
    /*!
     * @brief           Unit normal of the target plane, in the surviving map
     *                  frame.
     */
    Eigen::Vector3d n_B;
    /*!
     * @brief           Offset of the target plane: n_B^T x + d_B = 0.
     */
    double          d_B;
    /*!
     * @brief           Fixed sign hypothesis for the source plane, +1 or -1.
     */
    int             sigma;

    PlanePairMeasurement() :
        n_A(Eigen::Vector3d::Zero()),
        d_A(0.0),
        n_B(Eigen::Vector3d::Zero()),
        d_B(0.0),
        sigma(1)
    {}
};

} /* namespace vs_graphs::core::optimizable_types::plane_pair_measurement */

#endif /* VS_GRAPHS_OPTIMIZABLE_TYPES_PLANE_PAIR_MEASUREMENT_H */
