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
 * @file            computeError.cc
 *
 * @brief           Implements EdgeAccRW::computeError().
 */

#include "g2o_types/edge_acc_rw/edge_acc_rw.h"
#include "g2o_types/vertex_acc_bias/vertex_acc_bias.h"

namespace vs_graphs::core::g2o_types::edge_acc_rw
{

void EdgeAccRW::computeError()
{
        const g2o_types::vertex_acc_bias::VertexAccBias *p_firstAccelerometerBiasVertex =
            static_cast<const g2o_types::vertex_acc_bias::VertexAccBias *>(_vertices[0]);
        const g2o_types::vertex_acc_bias::VertexAccBias *p_secondAccelerometerBiasVertex =
            static_cast<const g2o_types::vertex_acc_bias::VertexAccBias *>(_vertices[1]);
        _error = p_secondAccelerometerBiasVertex->estimate() -
                 p_firstAccelerometerBiasVertex->estimate();
    }

} /* namespace vs_graphs::core::g2o_types::edge_acc_rw */
