/*!
 * This file is part of ORB-SLAM3.
 * Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details:
 * https://www.gnu.org/licenses/
 */

/*!
 * @file            TwoViewReconstructionStatusEnum.h
 *
 * @brief           Declares the status returned by every TwoViewReconstruction
 *                  operation.
 */

#ifndef VS_GRAPHS_CORE_SRC_TWO_VIEW_RECONSTRUCTION_OBJECTS_TWOVIEWRECONSTRUCTION_STATUS_ENUM_H
#define VS_GRAPHS_CORE_SRC_TWO_VIEW_RECONSTRUCTION_OBJECTS_TWOVIEWRECONSTRUCTION_STATUS_ENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace two_view_reconstruction
{

/*!
 * @brief           Result of a TwoViewReconstruction operation. Values are
 *                  fixed and never reordered.
 */
enum class TwoViewReconstructionStatus : std::uint8_t
{
    /*!
     * @brief           The operation completed and every output was written.
     */
    TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS = 0U
};

} /* namespace two_view_reconstruction */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_TWO_VIEW_RECONSTRUCTION_OBJECTS_TWOVIEWRECONSTRUCTION_STATUS_ENUM_H \
        */
