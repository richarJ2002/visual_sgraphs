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
 * @file            ORBmatcherStatus.h
 *
 * @brief           Declares the status returned by every ORBmatcher operation.
 */

#ifndef ORBMATCHER_STATUS_H
#define ORBMATCHER_STATUS_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Result of a ORBmatcher operation. Values are fixed and never
 *                  reordered.
 */
enum class ORBmatcherStatus : std::uint8_t
{
    /*!
     * @brief           The operation completed and every output was written.
     */
    ORBMATCHER_STATUS_SUCCESS = 0U
};

} // namespace core
} // namespace vs_graphs

#endif // ORBMATCHER_STATUS_H
