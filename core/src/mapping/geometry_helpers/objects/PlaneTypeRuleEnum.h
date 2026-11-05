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
 * @file            PlaneTypeRuleEnum.h
 *
 * @brief           Declares how Utils::findMatchingPlane() compares the
 *                  observed label with a mapped plane's type.
 */

#ifndef VS_GRAPHS_CORE_SRC_UTILS_UTILS_OBJECTS_PLANETYPERULEENUM_H
#define VS_GRAPHS_CORE_SRC_UTILS_UTILS_OBJECTS_PLANETYPERULEENUM_H

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
 * @brief           Semantic compatibility rule between the observed label and
 *                  a mapped plane. Values are fixed and never reordered.
 */
enum class PlaneTypeRule : std::uint8_t
{
    /*!
     * @brief           The mapped plane's expected (leading-vote) type equals
     *                  the label, or either one is UNDEFINED. An UNDEFINED
     *                  plane can then gather votes until it is confirmed.
     */
    EXPECTED_TYPE_OR_UNDEFINED = 0U,

    /*!
     * @brief           The mapped plane's confirmed type equals the label, and
     *                  the EXPECTED_TYPE_OR_UNDEFINED rule also holds.
     */
    CONFIRMED_TYPE_STRICT = 1U
};

} /* namespace utils */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_UTILS_UTILS_OBJECTS_PLANETYPERULEENUM_H */
