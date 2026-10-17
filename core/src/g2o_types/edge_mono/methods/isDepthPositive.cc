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
 * @file            isDepthPositive.cc
 *
 * @brief           Implements EdgeMono::isDepthPositive().
 */

#include "g2o_types/edge_mono/objects/EdgeMonoClass.h"

#include "g2o_types/vertex_pose/vertex_pose.h"

namespace vs_graphs
{
namespace core
{
namespace g2o_types
{
namespace edge_mono
{

bool EdgeMono::isDepthPositive()
{
    const g2o::VertexSBAPointXYZ *p_pointVertex =
        static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
    const vertex_pose::VertexPose *p_poseVertex =
        static_cast<const vertex_pose::VertexPose *>(_vertices[1]);
    return p_poseVertex->estimate().isDepthPositive(p_pointVertex->estimate(),
                                                    cam_idx);
}

} /* namespace edge_mono */
} /* namespace g2o_types */
} /* namespace core */
} /* namespace vs_graphs */
