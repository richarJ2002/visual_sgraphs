/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            transferSemanticEntities.cc
 *
 * @brief           Implements transferSemanticEntities(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"

namespace vs_graphs
{
namespace core
{

void transferSemanticEntities(
    map::Map                             *p_currentMap_inout,
    map::Map                             *p_mergeMap_inout,
    const loop_closing::MergeMapContents &mergeMapContents_in,
    bool                                  skipBadMarkerBasedRooms_in)
{
    transferPlanes(p_currentMap_inout,
                   p_mergeMap_inout,
                   mergeMapContents_in.planes);
    transferMarkers(p_currentMap_inout,
                    p_mergeMap_inout,
                    mergeMapContents_in.markers);
    transferPassages(p_currentMap_inout,
                     p_mergeMap_inout,
                     mergeMapContents_in.passages,
                     mergeMapContents_in.detectedRooms,
                     mergeMapContents_in.markerBasedRooms);
    transferRooms(p_currentMap_inout,
                  p_mergeMap_inout,
                  mergeMapContents_in.detectedRooms,
                  mergeMapContents_in.markerBasedRooms,
                  skipBadMarkerBasedRooms_in);
    transferFloors(p_currentMap_inout,
                   p_mergeMap_inout,
                   mergeMapContents_in.floors);
}

} /* namespace core */
} /* namespace vs_graphs */
