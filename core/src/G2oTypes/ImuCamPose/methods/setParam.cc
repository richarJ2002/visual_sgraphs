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

#include "G2oTypes.h"
#include "ImuTypes.h"
#include "Utils/Converter/objects/Converter.h"

namespace vs_graphs
{
namespace core
{

void ImuCamPose::setParam(const std::vector<Eigen::Matrix3d> &_Rcw,
                          const std::vector<Eigen::Vector3d> &_tcw,
                          const std::vector<Eigen::Matrix3d> &_Rbc,
                          const std::vector<Eigen::Vector3d> &_tbc,
                          const double                       &_bf)
{
    Rbc                = _Rbc;
    tbc                = _tbc;
    Rcw                = _Rcw;
    tcw                = _tcw;
    const int num_cams = Rbc.size();
    Rcb.resize(num_cams);
    tcb.resize(num_cams);

    for (std::size_t i = 0; i < tcb.size(); i++)
    {
        Rcb[i] = Rbc[i].transpose();
        tcb[i] = -Rcb[i] * tbc[i];
    }
    Rwb = Rcw[0].transpose() * Rcb[0];
    twb = Rcw[0].transpose() * (tcb[0] - tcw[0]);

    bf = _bf;
}

} // namespace core
} // namespace vs_graphs
