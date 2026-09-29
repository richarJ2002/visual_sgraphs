/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            TrackingStatus.h
 *
 * @brief           Declares the status returned by every Tracking operation.
 */

#ifndef TRACKING_STATUS_H
#define TRACKING_STATUS_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief       Result of a Tracking operation. Values are fixed and never
 *              reordered.
 */
enum class TrackingStatus : std::uint8_t
{
    /*! @brief The operation completed and every output was written. */
    TRACKING_STATUS_SUCCESS = 0U
};

} // namespace core
} // namespace vs_graphs

#endif // TRACKING_STATUS_H
