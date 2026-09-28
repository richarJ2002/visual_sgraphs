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

#include "OptimizableTypes.h"

namespace vs_graphs
{
namespace core
{

bool VertexSim3Expmap::write(std::ostream &outputStream_inout) const
{
    g2o::Sim3     cam2world(estimate().inverse());
    g2o::Vector7d logVector = cam2world.log();
    for (int parameterIndex = 0; parameterIndex < 7; parameterIndex++)
        outputStream_inout << logVector[parameterIndex] << " ";
    for (size_t parameterIndex = 0; parameterIndex < p_firstCamera->size();
         parameterIndex++)
        outputStream_inout << p_firstCamera->getParameter(parameterIndex)
                           << " ";

    for (size_t parameterIndex = 0; parameterIndex < p_secondCamera->size();
         parameterIndex++)
        outputStream_inout << p_secondCamera->getParameter(parameterIndex)
                           << " ";
    return outputStream_inout.good();
}

} // namespace core
} // namespace vs_graphs
