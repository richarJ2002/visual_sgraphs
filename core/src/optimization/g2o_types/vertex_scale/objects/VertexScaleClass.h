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
 * @file            VertexScaleClass.h
 *
 * @brief           Declares VertexScale, the g2o vertex holding a scale
 *                  factor estimate.
 */

#ifndef VS_GRAPHS_CORE_SRC_G2O_TYPES_VERTEX_SCALE_OBJECTS_VERTEX_SCALE_CLASS_H
#define VS_GRAPHS_CORE_SRC_G2O_TYPES_VERTEX_SCALE_OBJECTS_VERTEX_SCALE_CLASS_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vs_graphs_g2o/core/base_vertex.h>

#include "optimization/g2o_types/unsupported_graph_io/unsupported_graph_io.h"
namespace vs_graphs
{
namespace core
{
namespace g2o_types
{
namespace vertex_scale
{

/*!
 * @brief           g2o vertex whose estimate is a scale factor, which the
 *                  inertial edges multiply onto map distances to compare them
 *                  with metric IMU measurements.
 */
class VertexScale : public unsupported_graph_io::UnsupportedGraphIo<
                        g2o::BaseVertex<1, double>>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexScale()
    {
        setEstimate(1.0);
    }
    /*!
     * @brief           Creates the vertex with an initial scale.
     *
     * @param[in]       ps_in
     *                  Initial scale factor.
     */
    VertexScale(double ps_in)
    {
        setEstimate(ps_in);
    }

    /*!
     * @brief           Resets the scale estimate to 1.0, meaning no scaling.
     */
    void setToOriginImpl() override;

    /*!
     * @brief           Applies an optimiser step to the scale by multiplying it
     *                  with the exponential of the step, which keeps a positive
     *                  scale positive.
     *
     * @param[in]       p_update_in
     *                  Pointer to one value, the logarithm of the factor to
     *                  multiply the scale by.
     */
    void oplusImpl(const double *p_update_in) override;
};

} /* namespace vertex_scale */
} /* namespace g2o_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_G2O_TYPES_VERTEX_SCALE_OBJECTS_VERTEX_SCALE_CLASS_H \
        */
