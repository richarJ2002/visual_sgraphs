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
 * @file            getHessian.cc
 *
 * @brief           Implements EdgeAccRW::getHessian().
 */

#include "g2o_types/edge_acc_rw/edge_acc_rw.h"

namespace vs_graphs::core::g2o_types::edge_acc_rw
{

EdgeAccRWStatus
        EdgeAccRW::getHessian(Eigen::Matrix<double, 6, 6> &hessian_out)
{
        linearizeOplus();
        Eigen::Matrix<double, 3, 6> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 3>(0, 3) = _jacobianOplusXj;
        hessian_out         = J.transpose() * information() * J;
        return EdgeAccRWStatus::EDGE_ACC_RWSTATUS_SUCCESS;
    }

} /* namespace vs_graphs::core::g2o_types::edge_acc_rw */
