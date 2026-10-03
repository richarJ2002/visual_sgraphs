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
 * @file            UtilsStatus.h
 *
 * @brief           Declares the status returned by every Utils operation.
 */

#ifndef UTILS_STATUS_H
#define UTILS_STATUS_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

/*!
 * @brief           Result of a Utils operation. Values are fixed and never
 *                  reordered.
 */
enum class UtilsStatus : std::uint8_t
{
    /*!
     * @brief           The operation completed and every output was written.
     */
    UTILS_STATUS_SUCCESS = 0U
};

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs

#endif // UTILS_STATUS_H
