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
 * @file            ORBmatcherStatusEnum.h
 *
 * @brief           Declares the status returned by every ORBmatcher operation.
 */

#ifndef VS_GRAPHS_CORE_SRC_ORB_MATCHER_OBJECTS_ORBMATCHER_STATUS_ENUM_H
#define VS_GRAPHS_CORE_SRC_ORB_MATCHER_OBJECTS_ORBMATCHER_STATUS_ENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace orb_matcher
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

} /* namespace orb_matcher */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_ORB_MATCHER_OBJECTS_ORBMATCHER_STATUS_ENUM_H */
