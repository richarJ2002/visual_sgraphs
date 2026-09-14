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
 * @file            AtlasCurrentMapStatus.h
 *
 * @brief           Declares the truthful status of
 * Atlas::GetCoherentMapView()'s reported current-map id relative to its own
 * reported active map set.
 */

#ifndef ATLAS_CURRENT_MAP_STATUS_H
#define ATLAS_CURRENT_MAP_STATUS_H

#include <cstdint>

namespace ORB_SLAM3
{

/*!
 * @brief       Status of Atlas::GetCoherentMapView()'s reported current-map
 *              id relative to the active map vector it returns in the same
 *              call.
 *
 *              Introduced 2026-09-06 because Atlas::SetMapBad() erases a map
 *              from the active set (Atlas::mspMaps) and marks it bad without
 *              clearing Atlas::mpCurrentMap; a later Atlas::ChangeMap() call
 *              is what eventually installs a new current map. Between those
 *              two calls, a truthful coherent read must be able to report
 *              that the current map id names a map genuinely absent from the
 *              active set, rather than silently claiming an invariant that
 *              does not hold at that instant.
 */
enum class AtlasCurrentMapStatus : std::uint8_t
{
    /*! @brief Atlas::mpCurrentMap was nullptr at the moment of the read. */
    NO_CURRENT_MAP = 0U,

    /*! @brief Atlas::mpCurrentMap was non-null and present in the returned
     *  active map vector. */
    CURRENT_MAP_ACTIVE = 1U,

    /*! @brief Atlas::mpCurrentMap was non-null but absent from the returned
     *  active map vector -- reachable via Atlas::SetMapBad(currentMap)
     *  followed by no Atlas::ChangeMap() call yet. */
    CURRENT_MAP_NOT_ACTIVE = 2U
};

} // namespace ORB_SLAM3

#endif // ATLAS_CURRENT_MAP_STATUS_H
