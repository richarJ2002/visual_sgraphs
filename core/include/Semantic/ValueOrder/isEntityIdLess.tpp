/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
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
 * @file            isEntityIdLess.tpp
 *
 * @brief           Implements isEntityIdLess(), declared in
 *                  Semantic/ValueOrder.h.
 */

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

template <typename EntityT>
bool isEntityIdLess(const EntityT *p_first_in, const EntityT *p_second_in)
{
    if (p_first_in == nullptr)
    {
        return false;
    }
    if (p_second_in == nullptr)
    {
        return true;
    }

    /* Every entity status enumeration has SUCCESS = 0. */
    int firstId{};
    if (p_first_in->getId(firstId) != decltype(p_first_in->getId(firstId)){})
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    int secondId{};
    if (p_second_in->getId(secondId) !=
        decltype(p_second_in->getId(secondId)){})
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    return firstId < secondId;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
