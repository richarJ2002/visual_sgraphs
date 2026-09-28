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

bool EdgeSE3ProjectSE3::read(std::istream &inputStream_inout)
{
    g2o::Vector7D measurementVector;
    g2o::internal::readVector(inputStream_inout, measurementVector);
    g2o::Vector4D::MapType(measurementVector.data() + 3).normalize();
    setMeasurement(g2o::internal::fromVectorQT(measurementVector));
    if (inputStream_inout.bad())
        return false;
    readInformationMatrix(inputStream_inout);
    return inputStream_inout.good() || inputStream_inout.eof();
}

} // namespace core
} // namespace vs_graphs
