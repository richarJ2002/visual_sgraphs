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
 * @file            LoopClosingStatusEnum.h
 *
 * @brief           Declares the status returned by every LoopClosing operation.
 */

#ifndef VS_GRAPHS_CORE_SRC_LOOP_CLOSING_OBJECTS_LOOP_CLOSING_STATUS_ENUM_H
#define VS_GRAPHS_CORE_SRC_LOOP_CLOSING_OBJECTS_LOOP_CLOSING_STATUS_ENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace loop_closing
{

/*!
 * @brief           Result of a LoopClosing operation. Values are fixed and
 *                  never reordered.
 */
enum class LoopClosingStatus : std::uint8_t
{
    /*!
     * @brief           The operation completed and every output was written.
     */
    LOOP_CLOSING_STATUS_SUCCESS = 0U
};

} /* namespace loop_closing */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_LOOP_CLOSING_OBJECTS_LOOP_CLOSING_STATUS_ENUM_H   \
        */
