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
 * @file            getPrecision.cc
 *
 * @brief           Implements KannalaBrandt8::getPrecision().
 */

#include "sensor_models/camera_models/kannala_brandt8/objects/KannalaBrandt8Class.h"

namespace vs_graphs
{
namespace core
{
namespace camera_models
{
namespace kannala_brandt8
{

[[nodiscard]] KannalaBrandt8Status
    KannalaBrandt8::getPrecision(float &precision_out) const
{
    precision_out = precision;
    return kannala_brandt8::KannalaBrandt8Status::
        KANNALA_BRANDT8_STATUS_SUCCESS;
}

} /* namespace kannala_brandt8 */
} /* namespace camera_models */
} /* namespace core */
} /* namespace vs_graphs */
