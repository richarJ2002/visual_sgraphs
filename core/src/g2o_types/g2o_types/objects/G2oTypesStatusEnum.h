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
 * @file            G2oTypesStatusEnum.h
 *
 * @brief           Declares the status returned by every g2o_types
 *                  free-function operation.
 */

#ifndef VS_GRAPHS_CORE_SRC_G2O_TYPES_G2O_TYPES_OBJECTS_G2O_TYPES_STATUS_ENUM_H
#define VS_GRAPHS_CORE_SRC_G2O_TYPES_G2O_TYPES_OBJECTS_G2O_TYPES_STATUS_ENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace g2o_types
{

/*!
 * @brief        Result of a g2o_types free-function operation.
 *               Values are fixed and never reordered.
 */
enum class G2oTypesStatus : std::uint8_t
{
    /*!
     * @brief        The operation completed and every output was written.
     */
    G2O_TYPES_STATUS_SUCCESS = 0U
};

} /* namespace g2o_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_G2O_TYPES_G2O_TYPES_OBJECTS_G2O_TYPES_STATUS_ENUM_H \
        */
