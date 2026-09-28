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

bool VertexPose::write(std::ostream &outputStream_out) const
{
    std::vector<Eigen::Matrix<double, 3, 3>> Rcw = _estimate.Rcw;
    std::vector<Eigen::Matrix<double, 3, 1>> tcw = _estimate.tcw;

    std::vector<Eigen::Matrix<double, 3, 3>> Rbc = _estimate.Rbc;
    std::vector<Eigen::Matrix<double, 3, 1>> tbc = _estimate.tbc;

    const int cameraCount = tcw.size();

    for (int cameraIndex = 0; cameraIndex < cameraCount; cameraIndex++)
    {
        for (int componentIndex = 0; componentIndex < 3; componentIndex++)
        {
            for (int columnIndex = 0; columnIndex < 3; columnIndex++)
                outputStream_out
                    << Rcw[cameraIndex](componentIndex, columnIndex) << " ";
        }
        for (int componentIndex = 0; componentIndex < 3; componentIndex++)
        {
            outputStream_out << tcw[cameraIndex](componentIndex) << " ";
        }

        for (int componentIndex = 0; componentIndex < 3; componentIndex++)
        {
            for (int columnIndex = 0; columnIndex < 3; columnIndex++)
                outputStream_out
                    << Rbc[cameraIndex](componentIndex, columnIndex) << " ";
        }
        for (int componentIndex = 0; componentIndex < 3; componentIndex++)
        {
            outputStream_out << tbc[cameraIndex](componentIndex) << " ";
        }

        for (size_t componentIndex = 0;
             componentIndex < _estimate.pCamera[cameraIndex]->size();
             componentIndex++)
        {
            outputStream_out
                << _estimate.pCamera[cameraIndex]->getParameter(componentIndex)
                << " ";
        }
    }

    outputStream_out << _estimate.bf << " ";

    return outputStream_out.good();
}

} // namespace core
} // namespace vs_graphs
