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
 * @file            SystemStatus.h
 *
 * @brief           Declares the status returned by every System operation.
 */

#ifndef SYSTEM_STATUS_H
#define SYSTEM_STATUS_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Result of a System operation. Values are fixed and never
 *                  reordered.
 */
enum class SystemStatus : std::uint8_t
{
    /*!
     * @brief           The operation completed and every output was written.
     */
    SYSTEM_STATUS_SUCCESS = 0U,

    /*!
     * @brief           The settings file could not be opened.
     */
    SYSTEM_STATUS_SETTINGS_UNREADABLE = 1U,

    /*!
     * @brief           The ORB vocabulary file could not be loaded.
     */
    SYSTEM_STATUS_VOCABULARY_UNREADABLE = 2U,

    /*!
     * @brief           The saved map (Atlas) file could not be loaded.
     */
    SYSTEM_STATUS_ATLAS_UNREADABLE = 3U
};

} // namespace core
} // namespace vs_graphs

#endif // SYSTEM_STATUS_H
