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
 * @file            oplusImpl.cc
 *
 * @brief           Implements VertexGravityDirection::oplusImpl().
 */

#include "optimization/g2o_types/vertex_gravity_direction/objects/VertexGravityDirectionClass.h"
#include "optimization/g2o_types/gravity_direction/objects/GravityDirectionStatusEnum.h"

#include <rclcpp/logging.hpp>

#include <Eigen/Dense>

namespace vs_graphs
{
namespace core
{
namespace g2o_types
{
namespace vertex_gravity_direction
{

void VertexGravityDirection::oplusImpl(const double *p_update_in)
{
    if (_estimate.update(p_update_in) !=
        gravity_direction::GravityDirectionStatus::
            GRAVITY_DIRECTION_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: update returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    updateCache();
}

} /* namespace vertex_gravity_direction */
} /* namespace g2o_types */
} /* namespace core */
} /* namespace vs_graphs */
