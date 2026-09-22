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

bool VertexSim3Expmap::read(std::istream &is)
{
    g2o::Vector7d cam2world;
    for (int i = 0; i < 6; i++)
        is >> cam2world[i];

    is >> cam2world[6];

    float nextParam;
    for (size_t i = 0; i < pCamera1->size(); i++)
    {
        is >> nextParam;
        pCamera1->setParameter(nextParam, i);
    }

    for (size_t i = 0; i < pCamera2->size(); i++)
    {
        is >> nextParam;
        pCamera2->setParameter(nextParam, i);
    }

    setEstimate(g2o::Sim3(cam2world).inverse());
    return true;
}

} // namespace core
} // namespace vs_graphs
