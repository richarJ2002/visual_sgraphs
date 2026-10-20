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
 * @file            ExtractorNodeStatusEnum.h
 *
 * @brief           Declares the status returned by every ExtractorNode
 *                  operation.
 */

#ifndef VS_GRAPHS_CORE_SRC_ORB_EXTRACTOR_OBJECTS_EXTRACTORNODE_STATUS_ENUM_H
#define VS_GRAPHS_CORE_SRC_ORB_EXTRACTOR_OBJECTS_EXTRACTORNODE_STATUS_ENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace orb_extractor
{

/*!
 * @brief           Result of a ExtractorNode operation. Values are fixed and
 *                  never reordered.
 */
enum class ExtractorNodeStatus : std::uint8_t
{
    /*!
     * @brief           The operation completed and every output was written.
     */
    EXTRACTOR_NODE_STATUS_SUCCESS = 0U
};

} /* namespace orb_extractor */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_ORB_EXTRACTOR_OBJECTS_EXTRACTORNODE_STATUS_ENUM_H \
        */
